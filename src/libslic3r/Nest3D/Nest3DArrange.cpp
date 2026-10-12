#include "Nest3DArrange.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <tbb/parallel_for.h>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r { namespace nest3d {

using arrangement::ArrangePolygon;
using arrangement::ArrangePolygons;

namespace {

constexpr double EFFORT_CELL_MM[]   = {1.0, 0.6, 0.4};
constexpr int    EFFORT_ORDERINGS[] = {1, 3, 6};
constexpr double MAX_BED_CELLS      = 600.;  // per side; bounds the cost of one placement scan
constexpr size_t MAX_LAY_FLAT       = 5;
constexpr double OVERHANG_TOLERANCE = 1.1;   // a lay-flat pose may add up to 10% overhang area
constexpr double BED_CONTACT_MM     = 0.1;   // facets this close to the bottom rest on the bed
constexpr double SAME_NORMAL_DOT    = 0.9998; // ~1 degree

// An item posed like ModelInstance::get_arrange_polygon() poses it: the instance transform
// without its XY offset and Z rotation. A Z rotation and an XY translation applied afterwards
// then mean what ArrangePolygon::rotation and ::translation mean.
struct ItemGeometry
{
    std::vector<Vec3d> vertices; // three per facet, counter-clockwise seen from outside
    Matrix3d           rotation0 = Matrix3d::Identity(); // rotation of that pose
    bool               fill_to_bed  = false;             // supports fill everything below the part
    double             overhang_cos = 0.;                // facets with normal z below -overhang_cos need support
};

// One way to rest the item on the bed, before its Z rotation.
struct Pose
{
    Matrix3d rotation = Matrix3d::Identity(); // applied to ItemGeometry::vertices
    Vec3d    euler    = Vec3d::Zero();        // instance X/Y angles in this pose
    double   lift     = 0.;                   // instance Z offset that puts the pose on the bed
    double   penalty  = 0.;
};

// The instance transform without its XY offset and Z rotation.
Geometry::Transformation arrange_pose(const ModelInstance &mi)
{
    Geometry::Transformation t(mi.get_transformation());
    Vec3d rotation = t.get_rotation();
    rotation.z()   = 0.;
    t.set_rotation(rotation);
    t.reset_offset();
    return t;
}

ItemGeometry item_geometry(const ArrangePolygon &ap, const DynamicPrintConfig &config, double prism_height)
{
    ItemGeometry g;
    if (ap.instance == nullptr) {
        // Wipe tower, exclusion areas and the like: a prism over the convex hull of the polygon.
        const Points hull = Geometry::convex_hull(ap.poly.contour.points).points;
        for (size_t i = 1; i + 1 < hull.size(); ++i)
            for (double z : {0., prism_height})
                for (const Point &pt : {hull[0], hull[i], hull[i + 1]})
                    g.vertices.emplace_back(to_3d(unscaled(pt), z));
        return g;
    }

    const ModelInstance           &mi = *ap.instance;
    ModelObject                   &mo = *mi.get_object();
    const Geometry::Transformation t  = arrange_pose(mi);
    g.rotation0 = t.get_rotation_matrix().linear();

    for (const ModelVolume *volume : mo.volumes) {
        if (!volume->is_model_part())
            continue;
        const Transform3d           m    = t.get_matrix() * volume->get_matrix();
        const bool                  flip = m.linear().determinant() < 0.; // mirroring reverses the winding
        const indexed_triangle_set &its  = volume->mesh().its;
        for (const stl_triangle_vertex_indices &f : its.indices)
            for (int i = 0; i < 3; ++i)
                g.vertices.emplace_back(m * its.vertices[f[flip ? 2 - i : i]].cast<double>());
    }

    // Both options are FFF only; without them the part is taken as printing without support.
    const auto *support   = mo.get_config_value<ConfigOptionBool>(config, "enable_support");
    const auto *threshold = mo.get_config_value<ConfigOptionInt>(config, "support_threshold_angle");
    g.fill_to_bed         = support && support->value;
    // 0 leaves the threshold to the support generator; assume the usual 30 degrees then.
    g.overhang_cos = std::cos(Geometry::deg2rad(double(threshold && threshold->value > 0 ? threshold->value : 30)));
    return g;
}

double lowest_z(const ItemGeometry &g, const Matrix3d &rotation)
{
    double z = std::numeric_limits<double>::max();
    for (const Vec3d &v : g.vertices)
        z = std::min(z, rotation.row(2).dot(v));
    return g.vertices.empty() ? 0. : z;
}

// Area of the facets that would print as unsupported overhangs, not counting those on the bed.
double overhang_area(const ItemGeometry &g, const Matrix3d &rotation, double bottom)
{
    double area = 0.;
    for (size_t i = 0; i + 2 < g.vertices.size(); i += 3) {
        const Vec3d  a = rotation * g.vertices[i], b = rotation * g.vertices[i + 1], c = rotation * g.vertices[i + 2];
        const Vec3d  n = (b - a).cross(c - a);
        const double len = n.norm();
        if (n.z() < -g.overhang_cos * len && std::max({a.z(), b.z(), c.z()}) > bottom + BED_CONTACT_MM)
            area += 0.5 * len;
    }
    return area;
}

// The pose as given, then up to MAX_LAY_FLAT poses resting on the largest faces of the convex
// hull, kept only when they do not add much overhang.
std::vector<Pose> item_poses(const ItemGeometry &g, bool reorient)
{
    std::vector<Pose> poses(1);
    poses[0].lift = -lowest_z(g, Matrix3d::Identity());
    if (!reorient || g.vertices.empty())
        return poses;

    std::vector<Vec3f> points;
    points.reserve(g.vertices.size());
    for (const Vec3d &v : g.vertices)
        points.emplace_back(v.cast<float>());
    const indexed_triangle_set hull = its_convex_hull(points);

    // Coplanar hull triangles form one face.
    std::vector<std::pair<Vec3d, double>> faces; // outward normal, area
    for (const stl_triangle_vertex_indices &f : hull.indices) {
        const Vec3d a = hull.vertices[f[0]].cast<double>();
        Vec3d       n = (hull.vertices[f[1]].cast<double>() - a).cross(hull.vertices[f[2]].cast<double>() - a);
        const double len = n.norm();
        if (len < EPSILON)
            continue;
        n /= len;
        auto it = std::find_if(faces.begin(), faces.end(), [&n](const auto &face) { return face.first.dot(n) > SAME_NORMAL_DOT; });
        if (it == faces.end())
            faces.emplace_back(n, 0.5 * len);
        else
            it->second += 0.5 * len;
    }
    std::sort(faces.begin(), faces.end(), [](const auto &l, const auto &r) { return l.second > r.second; });
    faces.resize(std::min(faces.size(), MAX_LAY_FLAT));

    const double base = overhang_area(g, Matrix3d::Identity(), -poses[0].lift);
    for (const auto &face : faces) {
        if (face.first.z() < -SAME_NORMAL_DOT)
            continue; // the face it rests on already
        const Matrix3d lay = Eigen::Quaterniond::FromTwoVectors(face.first, -Vec3d::UnitZ()).toRotationMatrix();
        Pose pose;
        pose.euler     = Geometry::extract_euler_angles(Matrix3d(lay * g.rotation0));
        pose.euler.z() = 0.; // the candidates supply the Z angle, as for the pose as given
        // Exactly the rotation the instance gets from these angles, so that what is
        // rasterized is what finalize applies.
        pose.rotation = Geometry::rotation_transform(pose.euler).linear() * g.rotation0.transpose();
        const double bottom   = lowest_z(g, pose.rotation);
        const double overhang = overhang_area(g, pose.rotation, bottom);
        if (overhang > OVERHANG_TOLERANCE * base + EPSILON)
            continue;
        pose.lift    = -bottom;
        pose.penalty = base > EPSILON ? std::max(0., overhang - base) / base : 0.;
        poses.push_back(pose);
    }
    return poses;
}

std::vector<double> z_rotations(const ArrangePolygon &ap, const arrangement::ArrangeParams &params, int steps)
{
    if (!params.allow_rotations)
        return {ap.rotation};
    std::vector<double> out;
    // Every item carries the default {0.}, which the legacy arranger ignores too; a longer
    // list restricts the item to those rotations.
    if (ap.allowed_rotations.size() > 1)
        for (double r : ap.allowed_rotations)
            out.push_back(ap.rotation + r);
    else
        for (int i = 0; i < steps; ++i)
            out.push_back(ap.rotation + 2. * PI * i / steps);
    return out;
}

// Column grid of the item in `pose`, turned by `angle` about the reference point and moved by
// `shift`, grown by `inflation` in XY.
HeightMap footprint(const ItemGeometry &g, const Pose &pose, double angle, const Vec2d &shift, double cell, double inflation)
{
    const Matrix3d r = Eigen::AngleAxisd(angle, Vec3d::UnitZ()).toRotationMatrix() * pose.rotation;
    const Vec3d    t(shift.x(), shift.y(), pose.lift);
    std::vector<Triangle> tris(g.vertices.size() / 3);
    for (size_t i = 0; i < tris.size() * 3; ++i) {
        const Vec3d v = r * g.vertices[i] + t;
        for (int k = 0; k < 3; ++k)
            tris[i / 3].v[i % 3][k] = float(v[k]);
    }
    return dilate(rasterize(tris, cell, 0., g.fill_to_bed), inflation);
}

} // namespace

Stats arrange_polygons(ArrangePolygons                  &selected,
                       const ArrangePolygons            &fixed,
                       const Points                     &bed,
                       const arrangement::ArrangeParams &params,
                       const ArrangeSettings            &settings,
                       const DynamicPrintConfig         &config,
                       int                               max_plates)
{
    const int         effort = std::clamp(settings.effort, 0, 2);
    const BoundingBox bed_bb(bed);
    const Vec2d       bed_min = unscaled(bed_bb.min);

    Params p;
    p.bed_w          = unscaled(bed_bb.size().x());
    p.bed_h          = unscaled(bed_bb.size().y());
    p.cell_mm        = std::max(EFFORT_CELL_MM[effort], std::max(p.bed_w, p.bed_h) / MAX_BED_CELLS);
    p.max_height     = params.printable_height;
    p.vertical_gap   = settings.vertical_gap_mm;
    p.allow_stacking = settings.allow_stacking;
    p.mix_groups     = false;
    p.orderings      = EFFORT_ORDERINGS[effort];
    p.max_plates     = max_plates;
    p.align_x        = params.do_final_align ? params.align_center.x() : -1.;
    p.align_y        = params.do_final_align ? params.align_center.y() : -1.;
    p.parallel_for   = [](size_t n, const std::function<void(size_t)> &fn) { tbb::parallel_for(size_t(0), n, fn); };
    p.should_stop    = params.stopcondition;
    if (params.progressind)
        p.progress = [&params, n = selected.size()](int done, int total) {
            params.progressind(unsigned(n * size_t(std::max(done, 0)) / size_t(std::max(total, 1))), "");
        };

    // Non-rectangular beds (circles, polygons): mask the cells whose centre lies outside.
    const Polygon bed_poly(bed);
    if (std::abs(bed_poly.area()) < (1. - 1e-3) * double(bed_bb.size().x()) * double(bed_bb.size().y())) {
        // Same grid size as the engine's bed.
        const int cols = int(std::floor(p.bed_w / p.cell_mm + 1e-6)), rows = int(std::floor(p.bed_h / p.cell_mm + 1e-6));
        p.bed_mask.assign(size_t(cols) * size_t(rows), 0);
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < cols; ++x)
                p.bed_mask[size_t(y) * cols + x] = !bed_poly.contains(
                    bed_bb.min + Point(scaled((x + 0.5) * p.cell_mm), scaled((y + 0.5) * p.cell_mm)));
    }

    // The legacy arranger never puts high- and low-temperature filaments on one plate and,
    // unless allowed, keeps parts of different filament sets apart. Groups are a partition, so
    // parts compatible with both temperatures join the low-temperature ones, and each distinct
    // filament set is its own group, which is stricter than the legacy subset rule. Fixed parts
    // (not virtual objects) give their plate a group too, as packed items do in the legacy scoring.
    auto has_temp = [&selected, &fixed](int type) {
        auto is = [type](const ArrangePolygon &ap) { return !ap.is_virt_object && ap.filament_temp_type == type; };
        return std::any_of(selected.begin(), selected.end(), is) || std::any_of(fixed.begin(), fixed.end(), is);
    };
    const bool split_temps = has_temp(FilamentTempType::HighTemp) && has_temp(FilamentTempType::LowTemp);
    std::map<std::pair<bool, std::vector<int>>, int> groups;
    auto group_of = [&](const ArrangePolygon &ap) {
        std::vector<int> filaments;
        if (!params.allow_multi_materials_on_same_plate) {
            filaments = ap.extrude_ids;
            std::sort(filaments.begin(), filaments.end());
            filaments.erase(std::unique(filaments.begin(), filaments.end()), filaments.end());
        }
        const auto key = std::make_pair(split_temps && ap.filament_temp_type == FilamentTempType::HighTemp, std::move(filaments));
        return groups.emplace(key, int(groups.size())).first->second;
    };

    std::vector<Part>              parts(selected.size());
    std::vector<std::vector<Pose>> poses(selected.size());
    for (size_t i = 0; i < selected.size(); ++i) {
        parts[i].id           = int(i);
        parts[i].priority     = selected[i].priority;
        parts[i].compat_group = group_of(selected[i]);
    }
    // Instances of one object posed alike, at the same Z angle and spacing, get identical
    // candidates, so the engine may pack them as copies. A key is the index of the first such item.
    std::vector<Transform3d> pose_of(selected.size(), Transform3d::Identity());
    std::vector<size_t>      firsts;
    for (size_t i = 0; i < selected.size(); ++i) {
        const ArrangePolygon &ap = selected[i];
        if (ap.instance == nullptr)
            continue;
        pose_of[i]   = arrange_pose(*ap.instance).get_matrix();
        const auto it = std::find_if(firsts.begin(), firsts.end(), [&](size_t j) {
            const ArrangePolygon &o = selected[j];
            return o.instance->get_object() == ap.instance->get_object() && o.rotation == ap.rotation && o.inflation == ap.inflation &&
                   o.allowed_rotations == ap.allowed_rotations && pose_of[j].isApprox(pose_of[i], 1e-9);
        });
        parts[i].shape_key = int(it - firsts.begin());
        if (it == firsts.end())
            firsts.push_back(i);
    }
    tbb::parallel_for(size_t(0), selected.size(), [&](size_t i) {
        const ArrangePolygon &ap = selected[i];
        const ItemGeometry    g  = item_geometry(ap, config, ap.height);
        poses[i] = item_poses(g, settings.reorient && ap.instance != nullptr);
        const std::vector<double> angles = z_rotations(ap, params, std::max(settings.rotation_steps, 1));
        for (size_t k = 0; k < poses[i].size(); ++k)
            for (double angle : angles)
                parts[i].candidates.push_back(
                    {footprint(g, poses[i][k], angle, Vec2d::Zero(), p.cell_mm, unscaled(ap.inflation)), angle, int(k), poses[i][k].penalty});
    });

    // Fixed items stand on their own plate, the bed's excluded regions on every plate (-1). All
    // of them are rasterized with the real mesh when they are instances, else as full columns.
    std::vector<std::pair<const ArrangePolygon *, int>> sources; // item, plate (-1: every plate)
    for (const ArrangePolygon &ap : fixed)
        if (ap.bed_idx >= 0 && ap.bed_idx < max_plates)
            sources.emplace_back(&ap, ap.bed_idx);
    for (const ArrangePolygons *regions : {&params.excluded_regions, &params.nonprefered_regions})
        for (const ArrangePolygon &ap : *regions)
            sources.emplace_back(&ap, -1);

    std::vector<HeightMap> maps(sources.size());
    tbb::parallel_for(size_t(0), sources.size(), [&](size_t i) {
        const ArrangePolygon &ap = *sources[i].first;
        const ItemGeometry    g  = item_geometry(ap, config, params.printable_height);
        maps[i] = footprint(g, item_poses(g, false).front(), ap.rotation, unscaled(ap.translation) - bed_min, p.cell_mm, unscaled(ap.inflation));
    });
    std::vector<Obstacle> obstacles;
    for (size_t i = 0; i < sources.size(); ++i) {
        const ArrangePolygon &ap = *sources[i].first;
        obstacles.push_back({sources[i].second, std::move(maps[i]), sources[i].second >= 0 && !ap.is_virt_object ? group_of(ap) : -1});
    }

    Stats stats;
    const std::vector<Placement> placements = nest3d::arrange(parts, obstacles, p, &stats);

    for (size_t i = 0; i < selected.size(); ++i) {
        ArrangePolygon  &ap = selected[i];
        const Placement &pl = placements[i];
        ap.bed_idx = pl.plate < 0 ? arrangement::UNARRANGED : pl.plate;
        if (pl.plate < 0)
            continue;
        ap.translation = scaled(Vec2d(bed_min + Vec2d(pl.x, pl.y)));
        ap.rotation    = pl.rotation_z;
        if (pl.orientation <= 0)
            continue;
        // The base setter moves the part and sets its Z angle; then rest it on the chosen face.
        const Pose &pose = poses[i][size_t(pl.orientation)];
        ap.setter = [instance = ap.instance, euler = pose.euler, lift = pose.lift, base = std::move(ap.setter)](const ArrangePolygon &arranged) {
            if (base)
                base(arranged);
            if (!arranged.is_arranged())
                return;
            instance->set_rotation(Vec3d(euler.x(), euler.y(), arranged.rotation));
            instance->set_offset(Z, lift);
            instance->get_object()->invalidate_bounding_box();
        };
    }
    return stats;
}

}} // namespace Slic3r::nest3d

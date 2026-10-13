#include "Nest3DArrange.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <boost/log/trivial.hpp>
#include <Eigen/Geometry>
#include <tbb/parallel_for.h>

#include "Nest3D.hpp"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r { namespace nest3d {

using arrangement::ArrangePolygon;
using arrangement::ArrangePolygons;

namespace {

constexpr size_t MAX_LAY_FLAT       = 5;
constexpr double OVERHANG_TOLERANCE = 1.1;   // a lay-flat pose may add up to 10% overhang area
constexpr double BED_CONTACT_MM     = 0.1;   // facets this close to the bottom rest on the bed
constexpr double SAME_NORMAL_DOT    = 0.9998; // ~1 degree

// An item in the pose arrange works in (ModelInstance::get_arrange_transformation()), so that a Z
// rotation and an XY translation applied afterwards mean what ArrangePolygon::rotation and
// ::translation mean.
struct ItemGeometry
{
    indexed_triangle_set its;                                // counter-clockwise seen from outside
    Matrix3d             rotation0 = Matrix3d::Identity();   // rotation of that pose
    bool                 fill_to_bed  = false;               // supports fill everything below the part
    double               overhang_cos = 0.;                  // facets with normal z below -overhang_cos need support
};

// One way to rest the item on the bed, before its Z rotation.
struct Pose
{
    Matrix3d rotation = Matrix3d::Identity(); // applied to ItemGeometry::its
    Vec3d    euler    = Vec3d::Zero();        // instance X/Y angles in this pose
    double   lift     = 0.;                   // instance Z offset that puts the pose on the bed
    double   penalty  = 0.;
};

ItemGeometry item_geometry(const ArrangePolygon &ap, const DynamicPrintConfig &config, double prism_height)
{
    ItemGeometry g;
    if (ap.instance == nullptr) {
        // Wipe tower, exclusion areas and the like: the top and bottom of a prism over the convex
        // hull of the polygon.
        const Points hull = Geometry::convex_hull(ap.poly.contour.points).points;
        const int    n    = int(hull.size());
        for (double z : {0., prism_height})
            for (const Point &pt : hull)
                g.its.vertices.emplace_back(to_3d(unscaled(pt), z).cast<float>());
        for (int i = 1; i + 1 < n; ++i)
            for (int base : {0, n})
                g.its.indices.emplace_back(base, base + i, base + i + 1);
        return g;
    }

    const ModelInstance           &mi = *ap.instance;
    ModelObject                   &mo = *mi.get_object(); // get_config_value() is not const
    const Geometry::Transformation t  = mi.get_arrange_transformation();
    g.rotation0 = t.get_rotation_matrix().linear();
    g.its       = mo.raw_indexed_triangle_set();
    its_transform(g.its, t.get_matrix(), /*fix_left_handed=*/true);

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
    for (const Vec3f &v : g.its.vertices)
        z = std::min(z, rotation.row(2).dot(v.cast<double>()));
    return g.its.vertices.empty() ? 0. : z;
}

// Area of the facets that would print as unsupported overhangs, not counting those on the bed.
double overhang_area(const ItemGeometry &g, const Matrix3d &rotation, double bottom)
{
    double area = 0.;
    for (const stl_triangle_vertex_indices &f : g.its.indices) {
        const Vec3d  a = rotation * g.its.vertices[f[0]].cast<double>(), b = rotation * g.its.vertices[f[1]].cast<double>(),
                    c = rotation * g.its.vertices[f[2]].cast<double>();
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
    if (!reorient || g.its.vertices.empty())
        return poses;

    const indexed_triangle_set hull = its_convex_hull(g.its);

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

// Candidate Z angles in canonical order (see candidate_angles()).
std::vector<double> z_rotations(const ArrangePolygon &ap, const arrangement::ArrangeParams &params, int steps)
{
    std::vector<double> offsets{0.};
    // Every item carries the default {0.}, which the legacy arranger ignores too; a longer
    // list restricts the item to those rotations.
    if (params.allow_rotations && ap.allowed_rotations.size() > 1)
        offsets = ap.allowed_rotations;
    else if (params.allow_rotations)
        for (int i = 1; i < steps; ++i)
            offsets.push_back(2. * PI * i / steps);
    return candidate_angles(ap.rotation, offsets);
}

bool same_angles(const std::vector<double> &a, const std::vector<double> &b)
{
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](double x, double y) { return std::abs(x - y) < 1e-6; });
}

// Column grid of the item in `pose`, turned by `angle` about the reference point and moved by
// `shift`, grown by `inflation` in XY.
HeightMap footprint(const ItemGeometry &g, const Pose &pose, double angle, const Vec2d &shift, double cell, double inflation)
{
    const Matrix3d r = Eigen::AngleAxisd(angle, Vec3d::UnitZ()).toRotationMatrix() * pose.rotation;
    const Vec3d    t(shift.x(), shift.y(), pose.lift);
    std::vector<Triangle> tris(g.its.indices.size());
    for (size_t i = 0; i < tris.size(); ++i)
        for (int k = 0; k < 3; ++k) {
            const Vec3d v = r * g.its.vertices[g.its.indices[i][k]].cast<double>() + t;
            for (int c = 0; c < 3; ++c)
                tris[i].v[k][c] = float(v[c]);
        }
    return rasterize(tris, cell, inflation, g.fill_to_bed);
}

Params make_params(const Points &bed, const arrangement::ArrangeParams &params, const ArrangeSettings &settings, size_t n_items)
{
    const BoundingBox bed_bb(bed);
    Params            p;
    p.bed_w             = unscaled(bed_bb.size().x());
    p.bed_h             = unscaled(bed_bb.size().y());
    const Effort effort = effort_settings(settings.effort, p.bed_w, p.bed_h);
    p.cell_mm           = effort.cell_mm;
    p.max_height        = params.printable_height;
    p.vertical_gap      = settings.vertical_gap_mm;
    p.allow_stacking    = settings.allow_stacking;
    p.orderings         = effort.orderings;
    p.align_x           = params.do_final_align ? params.align_center.x() : -1.;
    p.align_y           = params.do_final_align ? params.align_center.y() : -1.;
    p.parallel_for      = [](size_t n, const std::function<void(size_t)> &fn) { tbb::parallel_for(size_t(0), n, fn); };
    p.should_stop       = params.stopcondition;
    if (params.progressind)
        p.progress = [&params, n_items](int done, int total) {
            params.progressind(unsigned(n_items * size_t(std::max(done, 0)) / size_t(std::max(total, 1))), "");
        };

    // Non-rectangular beds (circles, polygons): mask the cells whose centre lies outside.
    const Polygon bed_poly(bed);
    if (std::abs(bed_poly.area()) < (1. - 1e-3) * double(bed_bb.size().x()) * double(bed_bb.size().y())) {
        const int cols = p.grid_cols(), rows = p.grid_rows();
        p.bed_mask.assign(size_t(cols) * size_t(rows), 0);
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < cols; ++x)
                p.bed_mask[size_t(y) * cols + x] = !bed_poly.contains(
                    bed_bb.min + Point(scaled((x + 0.5) * p.cell_mm), scaled((y + 0.5) * p.cell_mm)));
    }
    return p;
}

// Compat groups as the legacy arranger draws them: high- and low-temperature filaments never
// share a plate and, unless allowed, neither do different filament sets. Groups partition the
// parts, so those compatible with both temperatures join the low-temperature ones, and each
// filament set is a group of its own, stricter than the legacy subset rule.
class CompatGroups
{
public:
    CompatGroups(const ArrangePolygons &selected, const ArrangePolygons &fixed, bool split_filaments) : m_split_filaments(split_filaments)
    {
        auto has_temp = [&selected, &fixed](int type) {
            auto is = [type](const ArrangePolygon &ap) { return !ap.is_virt_object && ap.filament_temp_type == type; };
            return std::any_of(selected.begin(), selected.end(), is) || std::any_of(fixed.begin(), fixed.end(), is);
        };
        m_split_temps = has_temp(FilamentTempType::HighTemp) && has_temp(FilamentTempType::LowTemp);
    }

    // Numbered in order of first appearance.
    int of(const ArrangePolygon &ap)
    {
        std::vector<int> filaments;
        if (m_split_filaments) {
            filaments = ap.extrude_ids;
            std::sort(filaments.begin(), filaments.end());
            filaments.erase(std::unique(filaments.begin(), filaments.end()), filaments.end());
        }
        const auto key = std::make_pair(m_split_temps && ap.filament_temp_type == FilamentTempType::HighTemp, std::move(filaments));
        return m_groups.emplace(key, int(m_groups.size())).first->second;
    }

private:
    bool                                             m_split_filaments = false;
    bool                                             m_split_temps     = false;
    std::map<std::pair<bool, std::vector<int>>, int> m_groups;
};

// One part per selected item, with its poses per part for apply_placements().
std::vector<Part> build_parts(const ArrangePolygons            &selected,
                              const arrangement::ArrangeParams &params,
                              const ArrangeSettings            &settings,
                              const DynamicPrintConfig         &config,
                              double                            cell,
                              CompatGroups                     &groups,
                              std::vector<std::vector<Pose>>   &poses)
{
    std::vector<Part> parts(selected.size());
    poses.assign(selected.size(), {});
    for (size_t i = 0; i < selected.size(); ++i) {
        parts[i].id           = int(i);
        parts[i].priority     = selected[i].priority;
        parts[i].compat_group = groups.of(selected[i]);
    }
    // Instances of one object posed alike, with the same candidate Z angles and spacing, get
    // identical candidates, so the engine may pack them as copies. A key's items all take its
    // first item's angles, so that rounding never splits them; a key is the index of that item.
    std::vector<Transform3d>         pose_of(selected.size(), Transform3d::Identity());
    std::vector<std::vector<double>> angles(selected.size());
    std::vector<size_t>              firsts;
    for (size_t i = 0; i < selected.size(); ++i) {
        const ArrangePolygon &ap = selected[i];
        angles[i] = z_rotations(ap, params, settings.rotation_steps);
        if (ap.instance == nullptr)
            continue;
        pose_of[i]    = ap.instance->get_arrange_transformation().get_matrix();
        const auto it = std::find_if(firsts.begin(), firsts.end(), [&](size_t j) {
            const ArrangePolygon &o = selected[j];
            return o.instance->get_object() == ap.instance->get_object() && o.inflation == ap.inflation &&
                   same_angles(angles[j], angles[i]) && pose_of[j].isApprox(pose_of[i], 1e-9);
        });
        parts[i].shape_key = int(it - firsts.begin());
        if (it == firsts.end())
            firsts.push_back(i);
        else
            angles[i] = angles[*it];
    }
    // Copies share their key's first item's candidates, so only that item is rasterized, each of its
    // candidates as a task of its own: one large mesh would otherwise take all its angles serially.
    tbb::parallel_for(size_t(0), selected.size(), [&](size_t i) {
        if (parts[i].shape_key >= 0 && firsts[size_t(parts[i].shape_key)] != i)
            return;
        const ArrangePolygon &ap = selected[i];
        const ItemGeometry    g  = item_geometry(ap, config, ap.height);
        poses[i]                 = item_poses(g, settings.reorient && ap.instance != nullptr);
        const size_t n_angles    = angles[i].size();
        parts[i].candidates.resize(poses[i].size() * n_angles);
        tbb::parallel_for(size_t(0), parts[i].candidates.size(), [&](size_t c) {
            const Pose  &pose  = poses[i][c / n_angles];
            const double angle = angles[i][c % n_angles];
            parts[i].candidates[c] = {footprint(g, pose, angle, Vec2d::Zero(), cell, unscaled(ap.inflation)), angle, int(c / n_angles), pose.penalty};
        });
    });
    for (size_t i = 0; i < selected.size(); ++i)
        if (const int key = parts[i].shape_key; key >= 0 && firsts[size_t(key)] != i) {
            parts[i].candidates = parts[firsts[size_t(key)]].candidates;
            poses[i]            = poses[firsts[size_t(key)]];
        }
    return parts;
}

// Fixed items stand on their own plate, the bed's excluded regions on every plate (-1). All of
// them are rasterized with the real mesh when they are instances, else as full columns. A plate
// with fixed parts (not virtual objects) takes their group, as packed items do in the legacy scoring.
std::vector<Obstacle> build_obstacles(const ArrangePolygons            &fixed,
                                      const arrangement::ArrangeParams &params,
                                      const DynamicPrintConfig         &config,
                                      const Vec2d                      &bed_min,
                                      double                            cell,
                                      int                               max_plates,
                                      CompatGroups                     &groups)
{
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
        maps[i] = footprint(g, item_poses(g, false).front(), ap.rotation, unscaled(ap.translation) - bed_min, cell, unscaled(ap.inflation));
    });
    std::vector<Obstacle> obstacles;
    for (size_t i = 0; i < sources.size(); ++i) {
        const auto [ap, plate] = sources[i];
        obstacles.push_back({plate, std::move(maps[i]), plate >= 0 && !ap->is_virt_object ? groups.of(*ap) : -1});
    }
    return obstacles;
}

// Writes the placements back as arrangement::arrange() does; a part laid on another face gets a
// setter that also rests it on that face.
void apply_placements(ArrangePolygons &selected, const std::vector<Placement> &placements, const std::vector<std::vector<Pose>> &poses, const Vec2d &bed_min)
{
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
}

// What the guard in arrange_polygons() ranks a result by: fewer unplaced items first, then fewer plates.
std::pair<int, int> cost(const ArrangePolygons &items)
{
    int unplaced = 0, plates = 0;
    for (const ArrangePolygon &ap : items) {
        unplaced += !ap.is_arranged();
        plates = std::max(plates, ap.bed_idx + 1);
    }
    return {unplaced, plates};
}

} // namespace

std::vector<double> candidate_angles(double current, const std::vector<double> &offsets)
{
    std::vector<double> out;
    for (double offset : offsets) {
        const double a = Geometry::angle_to_0_2PI(current + offset);
        // Angles this close count as equal, so ones a hair below a full turn wrap to 0.
        out.push_back(a > 2. * PI - 1e-9 ? 0. : a);
    }
    std::sort(out.begin(), out.end());
    return out;
}

void arrange_polygons(ArrangePolygons                  &selected,
                      const ArrangePolygons            &fixed,
                      const Points                     &bed,
                      const arrangement::ArrangeParams &params,
                      const ArrangeSettings            &settings,
                      const DynamicPrintConfig         &config,
                      int                               max_plates)
{
    Params p     = make_params(bed, params, settings, selected.size());
    p.max_plates = max_plates;
    const Vec2d                    bed_min = unscaled(BoundingBox(bed).min);
    CompatGroups                   groups(selected, fixed, !params.allow_multi_materials_on_same_plate);
    std::vector<std::vector<Pose>> poses;
    const std::vector<Part>        parts     = build_parts(selected, params, settings, config, p.cell_mm, groups, poses);
    const std::vector<Obstacle>    obstacles = build_obstacles(fixed, params, config, bed_min, p.cell_mm, max_plates, groups);
    const ArrangePolygons original = selected;
    apply_placements(selected, nest3d::arrange(parts, obstacles, p), poses, bed_min);

    // Shape-true packing is a heuristic that can lose to the outline arranger on some inputs, and a
    // result needing more plates or leaving more parts out must never replace it. A single full
    // plate cannot be beaten, so the outline arranger only runs when something is off plate 0.
    const auto [unplaced, plates] = cost(selected);
    if (unplaced == 0 && plates <= 1)
        return;
    ArrangePolygons stock = original;
    arrangement::arrange(stock, fixed, bed, params);
    const bool stock_wins = cost(stock) < std::make_pair(unplaced, plates);
    BOOST_LOG_TRIVIAL(info) << "arrange: nest3d used " << plates << " plates with " << unplaced << " unplaced, "
                            << (stock_wins ? "the outline arranger did better and is kept" : "kept over the outline arranger");
    if (stock_wins)
        selected = std::move(stock);
}

}} // namespace Slic3r::nest3d

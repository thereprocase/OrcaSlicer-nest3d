#include "Nest3DArrange.hpp"
#include "Nest3D.hpp"

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include <Eigen/Geometry>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {

using arrangement::ArrangeParams;
using arrangement::ArrangePolygon;
using arrangement::ArrangePolygons;

namespace {

// Cells of 0.5 mm divide whole and half millimetres, so an axis-parallel edge at such a size and spacing
// falls on a cell boundary and keeps its spacing exactly. Past MaxBedCells per side the cell grows with
// the bed, which bounds the cost of a scan.
constexpr double CellMM        = 0.5;
constexpr double MaxBedCells   = 720.;
constexpr int    RotationSteps = 8; // over a full turn, which also covers parts that tile at 45 degrees

void parallel(size_t n, const std::function<void(size_t)> &fn) { tbb::parallel_for(size_t(0), n, fn); }

// The pose ArrangePolygon::rotation and ::translation apply to (the instance transform without its offset
// and Euler Z angle, as ModelInstance::get_arrange_polygon() uses it), less the turn about Z that an Euler
// decomposition can leave in it as half turns about X and Y. That turn goes to `yaw`, and the item's angles
// take it over, so instances turned alike share one shape.
Transform3d arrange_pose(const ModelInstance &mi, double &yaw)
{
    Vec3d rotation = mi.get_rotation();
    rotation.z()   = 0.;
    Geometry::Transformation t = mi.get_transformation();
    t.set_rotation(rotation);
    t.reset_offset();
    yaw = std::atan2(t.get_matrix()(1, 0), t.get_matrix()(0, 0));
    return Eigen::AngleAxisd(-yaw, Vec3d::UnitZ()) * t.get_matrix();
}

// An item's triangles in its arrange pose, resting on the bed: the instance's mesh, or for an item without
// one (the wipe tower, an exclusion zone) the caps of a prism `height` tall over its polygon's hull. `yaw`
// is the Z turn arrange_pose() took out of the pose: the item's rotation is short by that much.
struct ArrangeMesh
{
    std::vector<Triangle> tris;
    double                yaw = 0.;
};

ArrangeMesh arrange_mesh(const ArrangePolygon &ap, double height)
{
    indexed_triangle_set its;
    double               yaw = 0.;
    if (ap.instance) {
        its = ap.instance->get_object()->raw_indexed_triangle_set();
        its_transform(its, arrange_pose(*ap.instance, yaw), /*fix_left_handed=*/true);
    } else {
        const Points hull = Geometry::convex_hull(ap.poly.contour.points).points;
        for (double z : {0., height})
            for (const Point &pt : hull)
                its.vertices.emplace_back(to_3d(unscaled(pt), z).cast<float>());
        for (int i = 1; i + 1 < int(hull.size()); ++i)
            for (int base : {0, int(hull.size())})
                its.indices.emplace_back(base, base + i, base + i + 1);
    }
    const float           bottom = its.vertices.empty() ? 0.f : float(bounding_box(its).min.z());
    std::vector<Triangle> tris(its.indices.size());
    for (size_t i = 0; i < tris.size(); ++i)
        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c)
                tris[i].v[k][c] = its.vertices[size_t(its.indices[i][k])][c] - (c == 2 ? bottom : 0.f);
    return {tris, yaw};
}

// The height map of `tris` turned by `angle` about the reference point and moved by `shift`, grown by
// `inflation` in XY. Parts that print with support fill their columns down to the bed: the support takes
// that room.
HeightMap height_map(std::vector<Triangle> tris, double angle, const Vec2d &shift, double cell,
                     const ArrangePolygon &ap, const DynamicPrintConfig &config, bool generated_supports = false)
{
    const double c = std::cos(angle), s = std::sin(angle);
    for (Triangle &t : tris)
        for (auto &v : t.v) {
            const double x = v[0], y = v[1];
            v[0]           = float(c * x - s * y + shift.x());
            v[1]           = float(s * x + c * y + shift.y());
        }
    const ConfigOptionBool *support = nullptr;
    if (ap.instance && !generated_supports)
        support = ap.instance->get_object()->get_config_value<ConfigOptionBool>(config, "enable_support");
    return rasterize(tris, cell, unscaled(ap.inflation), support && support->value);
}

void check_canceled(const ArrangeParams &params)
{
    if (params.stopcondition && params.stopcondition())
        throw CanceledException();
}

// The slicer disables compensation when used filaments disagree. Isolated parts could then acquire
// a different scale than the final plate. Reject that case instead of packing incompatible envelopes;
// unused filament slots with different shrinkage remain harmless and are handled by Print::apply.
void validate_support_shrinkage(const ArrangePolygons &selected, const ArrangePolygons &fixed,
                               const DynamicPrintConfig &config, const ArrangeParams &params)
{
    Model model;
    std::set<const ModelObject *> seen;
    for (const ArrangePolygons *items : {&selected, &fixed})
        for (const ArrangePolygon &ap : *items)
            if (ap.instance && seen.insert(ap.instance->get_object()).second) {
                check_canceled(params);
                ModelObject *object = model.add_object(*ap.instance->get_object());
                object->clear_instances();
                ModelInstance *instance = object->add_instance(*ap.instance);
                object->printable = true;
                instance->printable = true;
                instance->print_volume_state = ModelInstancePVS_Inside;
            }
    if (model.objects.empty())
        return;
    DynamicPrintConfig full_config = DynamicPrintConfig::full_print_config();
    full_config.apply(config);
    Print print;
    print.set_status_silent();
    for (ModelObject *object : model.objects)
        print.auto_assign_extruders(object);
    print.apply(model, std::move(full_config));
    if (!print.has_same_shrinkage_compensations())
        throw SlicingError(L("Generated-support arrangement requires matching XY and Z shrinkage compensation for all used filaments."));
}

// Print's inner loops poll its atomic cancellation flag, rather than an external callback. Keep that
// flag connected during the preliminary slice as well, including long intervals between status events.
class SupportCancellation
{
public:
    SupportCancellation(Print &print, const ArrangeParams &params)
    {
        if (params.stopcondition)
            m_thread = std::thread([this, &print, &params] {
                std::unique_lock<std::mutex> lock(m_mutex);
                while (!m_done) {
                    if (params.stopcondition()) {
                        print.cancel();
                        return;
                    }
                    m_condition.wait_for(lock, std::chrono::milliseconds(50), [this] { return m_done; });
                }
            });
    }
    ~SupportCancellation()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_done = true;
        }
        m_condition.notify_one();
        if (m_thread.joinable())
            m_thread.join();
    }

private:
    std::mutex              m_mutex;
    std::condition_variable m_condition;
    bool                    m_done = false;
    std::thread             m_thread;
};

// Caps suffice for the height map: it keeps the full interval between the lowest and highest
// occupied Z in every XY cell. The polygon union keeps gaps between separate support beads open.
void append_layer_geometry(std::vector<Triangle> &tris, const ExPolygons &area, const Vec2d &center,
                           double bottom, double top)
{
    const std::vector<Vec2d> vertices = triangulate_expolygons_2d(area);
    for (double z : {bottom, top})
        for (size_t i = 0; i + 2 < vertices.size(); i += 3) {
            Triangle t;
            for (size_t k = 0; k < 3; ++k) {
                t.v[k][0] = float(vertices[i + k].x() + center.x());
                t.v[k][1] = float(vertices[i + k].y() + center.y());
                t.v[k][2] = float(z);
            }
            tris.push_back(t);
        }
}

// Generate in the candidate's final Z orientation: rotating one support result would also rotate
// world-aligned support patterns. The clone retains modifiers, blockers/enforcers, painted facets,
// variable layers and their configuration without invalidating the live project's slicing state.
// A generous temporary bed avoids clipping desired tree branches before their position is known;
// the real bed and exclusions are checked against the resulting footprint by the nesting engine.
HeightMap supported_height_map(const ArrangePolygon &ap, const std::vector<Triangle> &mesh, double angle,
                               const Vec2d &shift, double cell, const DynamicPrintConfig &config,
                               const ArrangeParams &params)
{
    check_canceled(params);
    if (!ap.instance)
        return height_map(mesh, angle, shift, cell, ap, config, true);

    Model model;
    ModelObject *object = model.add_object(*ap.instance->get_object());
    object->clear_instances();
    ModelInstance *instance = object->add_instance(*ap.instance);
    double yaw = 0.;
    instance->set_transformation(Geometry::Transformation(Eigen::AngleAxisd(angle, Vec3d::UnitZ()) *
                                                        arrange_pose(*ap.instance, yaw)));
    object->printable = true;
    instance->printable = true;
    instance->print_volume_state = ModelInstancePVS_Inside;
    // ArrangeMesh also normalizes the lowest vertex to the bed, including objects whose GUI auto-drop
    // switch is off. Only move this temporary instance, using the same normalization.
    object->translate_instance(0, Vec3d(0., 0., -object->instance_bounding_box(0).min.z()));

    DynamicPrintConfig slice_config = DynamicPrintConfig::full_print_config();
    slice_config.apply(config);
    const BoundingBoxf3 bounds = object->instance_bounding_box(0);
    const double margin = std::max(1000., bounds.size().norm() * 2.);
    slice_config.set_key_value("printable_area", new ConfigOptionPoints({
        Vec2d(bounds.min.x() - margin, bounds.min.y() - margin),
        Vec2d(bounds.max.x() + margin, bounds.min.y() - margin),
        Vec2d(bounds.max.x() + margin, bounds.max.y() + margin),
        Vec2d(bounds.min.x() - margin, bounds.max.y() + margin)}));
    slice_config.set_key_value("bed_exclude_area", new ConfigOptionPoints());

    Print print;
    print.set_status_silent();
    print.set_no_check_flag(true);
    print.auto_assign_extruders(object);
    print.apply(model, std::move(slice_config));
    SupportCancellation cancellation(print, params);
    // Use the public native pipeline, including its support prerequisites (bridge fills), model
    // compensation, tree/raft generation, and auto/painted brims. No G-code is exported.
    print.process();
    std::vector<Triangle> tris;

    for (PrintObject *po : print.objects()) {
        check_canceled(params);
        const Vec2d center = unscaled(po->center_offset());
        for (const Layer *layer : po->layers()) {
            check_canceled(params);
            // Use native sliced geometry rather than the unscaled model mesh. This includes
            // filament shrinkage, XY compensation, modifiers, raft lift and variable layer heights.
            Polygons area = to_polygons(layer->lslices);
            for (const LayerRegion *region : layer->regions())
                region->perimeters.polygons_covered_by_width(area, float(SCALED_EPSILON));
            append_layer_geometry(tris, union_ex(area), center, layer->print_z - layer->height, layer->print_z);
        }
        for (const SupportLayer *layer : po->support_layers()) {
            check_canceled(params);
            // Include the physical width of every support extrusion (base, interface, tree brim and
            // raft), not its centerline or a polygon enclosing unused space between sparse lines.
            const ExPolygons area = union_ex(layer->support_fills.polygons_covered_by_width(float(SCALED_EPSILON)));
            append_layer_geometry(tris, area, center, layer->print_z - layer->height, layer->print_z);
        }
    }
    for (const auto &group : print.skirt_brim_groups())
        for (const auto &brim : group.brims) {
            check_canceled(params);
            // The brim generator returns plate coordinates, unlike the centered per-object layers.
            append_layer_geometry(tris, union_ex(brim.brim.polygons_covered_by_width(float(SCALED_EPSILON))),
                                  Vec2d::Zero(), 0., print.objects().front()->layers().front()->print_z);
        }
    check_canceled(params);
    ArrangePolygon footprint = ap;
    // Auto spacing encodes guessed support growth in inflation (up to 12 mm for every item).
    // Actual support/brim beads already occupy that space. Keep the no-support automatic gap, or
    // the requested explicit gap, and retain stricter sequential-print head clearances.
    if (!params.is_seq_print)
        footprint.inflation = params.min_obj_distance != 0 ? params.min_obj_distance / 2 :
                              std::min(ap.inflation, scaled(1.));
    return height_map(std::move(tris), 0., shift, cell, footprint, config, true);
}

// The item's Z angles from `current`: that one, and with rotation allowed RotationSteps over a full turn or
// the item's allowed_rotations when it lists more than the default {0}.
std::vector<double> z_angles(const ArrangePolygon &ap, double current, const ArrangeParams &params)
{
    std::vector<double> offsets{0.};
    if (params.allow_rotations && ap.allowed_rotations.size() > 1)
        offsets = ap.allowed_rotations;
    else if (params.allow_rotations)
        for (int i = 1; i < RotationSteps; ++i)
            offsets.push_back(2. * PI * i / RotationSteps);
    return candidate_angles(current, offsets);
}

// Compat groups as the outline arranger draws them: high- and low-temperature filaments never share a
// plate and, unless allowed, neither do different filament sets. Groups partition the parts, so those
// compatible with both temperatures join the low-temperature ones, and each filament set is a group of
// its own. Numbered in order of first appearance.
struct CompatGroups
{
    bool                                           split_temps, split_filaments;
    std::map<std::pair<bool, std::vector<int>>, int> ids;

    int operator()(const ArrangePolygon &ap)
    {
        std::vector<int> filaments = split_filaments ? ap.extrude_ids : std::vector<int>{};
        std::sort(filaments.begin(), filaments.end());
        filaments.erase(std::unique(filaments.begin(), filaments.end()), filaments.end());
        const bool high_temp = split_temps && ap.filament_temp_type == FilamentTempType::HighTemp;
        const auto key       = std::make_pair(high_temp, std::move(filaments));
        return ids.emplace(key, int(ids.size())).first->second;
    }
};

CompatGroups compat_groups(const ArrangePolygons &selected, const ArrangePolygons &fixed,
                           const ArrangeParams &params)
{
    std::vector<int> types;
    for (const ArrangePolygons *items : {&selected, &fixed})
        for (const ArrangePolygon &ap : *items)
            if (!ap.is_virt_object)
                types.push_back(ap.filament_temp_type);
    return {!Print::is_filaments_compatible(types), !params.allow_multi_materials_on_same_plate, {}};
}

// The engine's parameters for a bed and the dialog's settings; progress is reported as a count of the
// `items` selected items.
Params engine_params(const BoundingBox &bed_bb, const ArrangeParams &params, const ArrangeSettings &settings,
                     int max_plates, size_t items)
{
    Params p;
    p.bed_w        = unscaled(bed_bb.size().x());
    p.bed_h        = unscaled(bed_bb.size().y());
    p.cell_mm      = std::max(CellMM, std::max(p.bed_w, p.bed_h) / MaxBedCells);
    p.max_height   = params.printable_height;
    p.vertical_gap = settings.allow_stacking ? settings.vertical_gap_mm : Inf;
    p.max_plates   = max_plates;
    p.align_x      = params.do_final_align ? params.align_center.x() : -1.;
    p.align_y      = params.do_final_align ? params.align_center.y() : -1.;
    p.parallel_for = parallel;
    p.should_stop  = params.stopcondition;
    if (params.progressind)
        p.progress = [&params, items](int done, int total) {
            params.progressind(unsigned(items * size_t(done) / size_t(std::max(total, 1))), "");
        };
    return p;
}

// The selected items as the engine's parts and shapes. Instances of one object posed alike, with the same
// angles and spacing, share a shape and are packed as copies. `yaw` is the Z turn arrange_pose() took out
// of each item's pose, which its result must give back.
struct Shapes
{
    std::vector<Part>                   parts;      // per item
    std::vector<std::vector<Candidate>> candidates; // per shape
    std::vector<double>                 yaw;        // per item
};

Shapes build_shapes(const ArrangePolygons &selected, const ArrangeParams &params, const DynamicPrintConfig &config,
                    const ArrangeSettings &settings, CompatGroups &group, double cell_mm)
{
    Shapes                           shapes;
    std::vector<size_t>              source; // per shape, the item it is built from
    std::vector<std::vector<double>> angles; // per shape
    std::vector<Transform3d>         pose(selected.size(), Transform3d::Identity());
    shapes.yaw.assign(selected.size(), 0.);
    for (size_t i = 0; i < selected.size(); ++i) {
        const ArrangePolygon &ap = selected[i];
        if (ap.instance)
            pose[i] = arrange_pose(*ap.instance, shapes.yaw[i]);
        const std::vector<double> mine = z_angles(ap, ap.rotation + shapes.yaw[i], params);
        const auto                same_shape = [&](size_t s) {
            const ArrangePolygon &o = selected[source[s]];
            if (!ap.instance || !o.instance || o.instance->get_object() != ap.instance->get_object())
                return false;
            const auto same_angle = [](double a, double b) { return std::abs(a - b) < 1e-6; };
            const auto &theirs    = angles[s];
            if (!std::equal(mine.begin(), mine.end(), theirs.begin(), theirs.end(), same_angle))
                return false;
            return o.inflation == ap.inflation && pose[source[s]].isApprox(pose[i], 1e-9);
        };
        size_t shape = 0;
        while (shape < source.size() && !same_shape(shape))
            ++shape;
        shapes.parts.push_back({int(i), int(shape), double(ap.priority), group(ap)});
        if (shape == source.size()) {
            source.push_back(i);
            angles.push_back(mine);
        }
    }
    shapes.candidates.resize(source.size());
    const auto build_shape = [&](size_t s) {
        const ArrangePolygon       &ap   = selected[source[s]];
        if (settings.generate_supports && params.progressind)
            params.progressind(0, ap.name);
        // The mesh's yaw is the item's, already folded into its angles.
        const std::vector<Triangle> mesh = arrange_mesh(ap, ap.height).tris;
        shapes.candidates[s].resize(angles[s].size());
        const auto build_candidate = [&](size_t k) {
            const double angle      = angles[s][k];
            shapes.candidates[s][k] = {
                settings.generate_supports ? supported_height_map(ap, mesh, angle, Vec2d::Zero(), cell_mm, config, params) :
                                             height_map(mesh, angle, Vec2d::Zero(), cell_mm, ap, config), angle};
        };
        if (settings.generate_supports)
            for (size_t k = 0; k < angles[s].size(); ++k)
                build_candidate(k);
        else
            parallel(angles[s].size(), build_candidate);
    };
    // Each native slice already parallelizes its own work. Keeping candidates sequential bounds the
    // memory consumed by their temporary Print state, especially for tree supports at eight rotations.
    if (settings.generate_supports)
        for (size_t s = 0; s < source.size(); ++s)
            build_shape(s);
    else
        parallel(source.size(), build_shape);
    return shapes;
}

// Fixed items on their plate, the bed's excluded regions on every plate, and for a bed that is not a
// rectangle, its outline: the cells whose centre lies outside it.
std::vector<Obstacle> build_obstacles(const ArrangePolygons &fixed, const Points &bed, const ArrangeParams &params,
                                      const DynamicPrintConfig &config, const ArrangeSettings &settings,
                                      CompatGroups &group, int max_plates,
                                      double cell_mm)
{
    const BoundingBox                   bed_bb(bed);
    const Vec2d                         bed_min = unscaled(bed_bb.min);
    std::vector<Obstacle>               obstacles;
    std::vector<const ArrangePolygon *> items;
    for (const ArrangePolygon &ap : fixed)
        if (ap.bed_idx >= 0 && ap.bed_idx < max_plates) {
            obstacles.push_back({ap.bed_idx, {}, ap.is_virt_object ? -1 : group(ap)});
            items.push_back(&ap);
        }
    for (const ArrangePolygons *regions : {&params.excluded_regions, &params.nonprefered_regions})
        for (const ArrangePolygon &ap : *regions) {
            obstacles.push_back({-1, {}, -1});
            items.push_back(&ap);
        }
    const auto build_obstacle = [&](size_t i) {
        const ArrangePolygon &ap   = *items[i];
        if (settings.generate_supports && ap.instance && params.progressind)
            params.progressind(0, ap.name);
        const ArrangeMesh     mesh = arrange_mesh(ap, params.printable_height);
        // A fixed part sits at its rotation plus the yaw its arrange pose gave up, like an arranged part does.
        const Vec2d shift = unscaled(ap.translation) - bed_min;
        obstacles[i].map = settings.generate_supports ?
            supported_height_map(ap, mesh.tris, ap.rotation + mesh.yaw, shift, cell_mm, config, params) :
            height_map(mesh.tris, ap.rotation + mesh.yaw, shift, cell_mm, ap, config);
    };
    if (settings.generate_supports)
        for (size_t i = 0; i < items.size(); ++i)
            build_obstacle(i);
    else
        parallel(items.size(), build_obstacle);
    const Polygon outline(bed);
    const double  bed_w = unscaled(bed_bb.size().x()), bed_h = unscaled(bed_bb.size().y());
    const int     cells_x = int(std::ceil(bed_w / cell_mm)), cells_y = int(std::ceil(bed_h / cell_mm));
    Obstacle     &mask    = obstacles.emplace_back(Obstacle{-1, HeightMap(cells_x, cells_y, cell_mm), -1});
    for (int y = 0; y < mask.map.height; ++y)
        for (int x = 0; x < mask.map.width; ++x)
            if (!outline.contains(bed_bb.min + Point(scaled((x + 0.5) * cell_mm), scaled((y + 0.5) * cell_mm))))
                mask.map.at(x, y).add(-Inf, Inf);
    return obstacles;
}

void apply_placements(const std::vector<Placement> &placements, const std::vector<double> &yaw, const Vec2d &bed_min,
                      ArrangePolygons &selected)
{
    for (const Placement &pl : placements) {
        ArrangePolygon &ap = selected[size_t(pl.id)];
        ap.bed_idx         = pl.plate < 0 ? arrangement::UNARRANGED : pl.plate;
        ap.translation     = scaled(Vec2d(bed_min + Vec2d(pl.x, pl.y)));
        ap.rotation        = pl.rotation_z - yaw[size_t(pl.id)];
    }
}

// Parts left out, then plates used: lower is better.
std::pair<int, int> unplaced_and_plates(const ArrangePolygons &items)
{
    std::pair<int, int> cost{0, 0};
    for (const ArrangePolygon &ap : items) {
        cost.first += !ap.is_arranged();
        cost.second = std::max(cost.second, ap.bed_idx + 1);
    }
    return cost;
}

// Shape-true packing is a heuristic, and a result that needs more plates or leaves more parts out than
// the outline arranger's must never replace it. That arranger packs every item's convex hull without
// overlap inside the bed, so it needs at least as many plates as the hulls' total area fills beds: a
// result already there is kept without running it. A cancelled arrange keeps what it placed: run under
// the same stop condition, the outline arranger returns every part "arranged" on top of the others.
void outline_arranger_guard(ArrangePolygons &selected, const ArrangePolygons &input, const ArrangePolygons &fixed,
                            const Points &bed, const ArrangeParams &params)
{
    const bool stopped = params.stopcondition && params.stopcondition();
    if (stopped)
        return;
    double hulls = 0.;
    for (const ArrangePolygon &ap : input)
        hulls += ap.poly.area();
    const int plates_needed = std::max(1, int(std::ceil(hulls / BoundingBox(bed).area() - 1e-9)));
    if (unplaced_and_plates(selected) <= std::make_pair(0, plates_needed))
        return;
    ArrangePolygons outline_result = input;
    arrangement::arrange(outline_result, fixed, bed, params);
    if (unplaced_and_plates(outline_result) < unplaced_and_plates(selected))
        selected = std::move(outline_result);
}

} // namespace

std::vector<double> candidate_angles(double current, const std::vector<double> &offsets)
{
    std::vector<double> out;
    for (double offset : offsets) {
        const double a = Geometry::angle_to_0_2PI(current + offset);
        out.push_back(a > 2. * PI - 1e-9 ? 0. : a); // a hair below a full turn is 0
    }
    std::sort(out.begin(), out.end());
    return out;
}

void arrange_polygons(ArrangePolygons          &selected,
                      const ArrangePolygons    &fixed,
                      const Points             &bed,
                      const ArrangeParams      &params,
                      const ArrangeSettings    &settings,
                      const DynamicPrintConfig &config,
                      int                       max_plates)
{
    const BoundingBox bed_bb(bed);
    const Params      p     = engine_params(bed_bb, params, settings, max_plates, selected.size());
    if (settings.generate_supports)
        validate_support_shrinkage(selected, fixed, config, params);
    CompatGroups      group = compat_groups(selected, fixed, params);
    const Shapes      shapes    = build_shapes(selected, params, config, settings, group, p.cell_mm);
    const auto        obstacles = build_obstacles(fixed, bed, params, config, settings, group, max_plates, p.cell_mm);

    const ArrangePolygons input = selected; // for the outline arranger below
    const std::vector<Placement> placements = arrange(shapes.candidates, shapes.parts, obstacles, p);
    if (settings.generate_supports)
        check_canceled(params);
    apply_placements(placements, shapes.yaw, unscaled(bed_bb.min), selected);
    if (!settings.generate_supports)
        outline_arranger_guard(selected, input, fixed, bed, params);
}

}} // namespace Slic3r::nest3d

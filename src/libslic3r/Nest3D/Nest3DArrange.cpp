#include "Nest3DArrange.hpp"
#include "Nest3D.hpp"
#include "Footprint.hpp"
#include "Report.hpp"

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCode/StartGCodeExtrusion.hpp"
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
#include <boost/log/trivial.hpp>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cstdio>
#include <cstddef>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string>
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
                     const ArrangePolygon &ap, coord_t inflation, const DynamicPrintConfig &config,
                     bool generated_supports = false)
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
    return rasterize(tris, cell, unscaled(inflation), support && support->value);
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

void extend_extrusion_z(const ExtrusionEntityCollection &extrusions, double print_z, double &bottom, double &top)
{
    for_each_extrusion_path(extrusions, [&](const ExtrusionPath &path) {
        bottom = std::min(bottom, print_z - path.height);
        if (path.z_contoured)
            for (const Point3 &point : path.polyline.points) {
                // ContourZ stores scaled offsets from the nominal layer Z in these points;
                // GCode emits nominal_z + unscale(point.z()). Keep both extrema, including the
                // full nominal bead height below a lowered path as a conservative bound.
                const double z = print_z + unscale<double>(point.z());
                bottom = std::min(bottom, z - path.height);
                top = std::max(top, z);
            }
    });
}

// Generate in the candidate's final Z orientation: rotating one support result would also rotate
// world-aligned support patterns. The clone retains modifiers, blockers/enforcers, painted facets,
// variable layers and their configuration without invalidating the live project's slicing state.
// A generous temporary bed avoids clipping desired tree branches before their position is known;
// the real bed and exclusions are checked against the resulting footprint by the nesting engine.
HeightMap supported_height_map(const ArrangePolygon &ap, const std::vector<Triangle> &mesh, double angle,
                               const Vec2d &shift, double cell, const DynamicPrintConfig &config,
                               const ArrangeParams &params, const std::vector<Triangle> &extra)
{
    check_canceled(params);
    // Extras (the per-object skirt) are made in the arrange pose and turn with the candidate.
    const auto with_extra = [&](std::vector<Triangle> tris, double turn) {
        const double c = std::cos(turn), s = std::sin(turn);
        for (Triangle t : extra) {
            for (auto &v : t.v) {
                const double x = v[0], y = v[1];
                v[0]           = float(c * x - s * y);
                v[1]           = float(s * x + c * y);
            }
            tris.push_back(t);
        }
        return tris;
    };
    if (!ap.instance)
        return height_map(with_extra(mesh, 0.), angle, shift, cell, ap, ap.inflation, config, true);

    double yaw = 0.;
    Print  print;
    apply_scratch_print(print, *ap.instance, Eigen::AngleAxisd(angle, Vec3d::UnitZ()) * arrange_pose(*ap.instance, yaw), config);
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
            double bottom = layer->print_z - layer->height, top = layer->print_z;
            for (const LayerRegion *region : layer->regions()) {
                region->perimeters.polygons_covered_by_width(area, float(SCALED_EPSILON));
                region->fills.polygons_covered_by_width(area, float(SCALED_EPSILON));
                extend_extrusion_z(region->perimeters, layer->print_z, bottom, top);
                extend_extrusion_z(region->fills, layer->print_z, bottom, top);
            }
            // Thick bridge beads and combined infill can extend farther below the nozzle than the
            // nominal layer thickness. Reserve their full height when checking vertical clearance.
            append_layer_geometry(tris, union_ex(area), center, bottom, top);
        }
        for (const SupportLayer *layer : po->support_layers()) {
            check_canceled(params);
            // Include the physical width of every support extrusion (base, interface, tree brim and
            // raft), not its centerline or a polygon enclosing unused space between sparse lines.
            const ExPolygons area = union_ex(layer->support_fills.polygons_covered_by_width(float(SCALED_EPSILON)));
            double bottom = layer->print_z - layer->height, top = layer->print_z;
            extend_extrusion_z(layer->support_fills, layer->print_z, bottom, top);
            append_layer_geometry(tris, area, center, bottom, top);
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
    return height_map(with_extra(std::move(tris), angle), 0., shift, cell, footprint, footprint.inflation, config, true);
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

// The convex hull in the plane of a shape's triangles and, if it has one, its brim.
Polygon shape_hull(const std::vector<Triangle> &mesh, const ExPolygons &brim)
{
    Points points;
    points.reserve(mesh.size() * 3);
    for (const Triangle &t : mesh)
        for (const auto &v : t.v)
            points.emplace_back(scaled<coord_t>(double(v[0])), scaled<coord_t>(double(v[1])));
    for (const ExPolygon &area : brim)
        append(points, area.contour.points);
    return Geometry::convex_hull(std::move(points));
}

// What a shape prints around its geometry besides the geometry, found once per shape and arrange, in the
// shape's arrange pose: the brim as Orca generates it and the skirt of an object that has its own, as
// triangles that join the shape's before it is rasterized at each angle.
struct Extras
{
    ExPolygons            brim;
    Polygon               hull;      // geometry and brim; empty unless a skirt needs it
    std::vector<Triangle> tris;      // the brim and the object's own skirt, as caps
    double                brim_mm = 0.;
    // Whether the brim is in the footprint as Orca makes it (an object with none counts). When it is not,
    // because the brim could not be made or the generated support's own slice carries it, the footprint
    // falls back on the caller's uniform brim allowance in `inflation` and in the bed margin.
    bool                  brim_exact = true;
    coord_t               inflation  = 0;
};

// Orca's brim for an object in a pose is the same for every instance posed alike, whatever its
// position, so each is made once per arrange.
class BrimCache
{
public:
    const ObjectBrim *get(const ModelInstance &instance, const Transform3d &pose, const DynamicPrintConfig &config,
                          const ArrangeParams &params, bool rethrow_cancel)
    {
        for (const Entry &entry : m_entries)
            if (entry.object == instance.get_object() && entry.pose.isApprox(pose, 1e-9))
                return entry.ok ? &entry.brim : nullptr;
        Entry entry{instance.get_object(), pose, {}, true};
        try {
            entry.brim = object_brim(instance, pose, config, params.stopcondition);
        } catch (const CanceledException &) {
            // Generated supports leave the placements untouched when cancelled; otherwise the arrange goes
            // on with the caller's allowance for this brim and stops at its next poll.
            if (rethrow_cancel)
                throw;
            entry.ok = false;
        } catch (const std::exception &e) {
            BOOST_LOG_TRIVIAL(warning) << "Nest3D: no brim shape for " << instance.get_object()->name
                                       << ", using the spacing allowance: " << e.what();
            entry.ok = false;
        }
        m_entries.push_back(std::move(entry));
        return m_entries.back().ok ? &m_entries.back().brim : nullptr;
    }

private:
    struct Entry
    {
        const ModelObject *object;
        Transform3d        pose;
        ObjectBrim         brim;
        bool               ok;
    };
    std::deque<Entry> m_entries; // references stay valid as it grows
};

struct ExtrasContext
{
    const ArrangeParams       &params;
    const ArrangeSettings     &settings;
    const DynamicPrintConfig  &config;
    SkirtPlan                  skirt;
    BrimCache                  brims;
};

Extras make_extras(const ArrangePolygon &ap, const std::vector<Triangle> &mesh, ExtrasContext &ctx, bool keep_inflation)
{
    Extras extras;
    extras.inflation = ap.inflation;
    if (!ap.instance || ap.is_virt_object)
        return extras;
    ModelObject &object = *ap.instance->get_object();
    const auto *support = object.get_config_value<ConfigOptionBool>(ctx.config, "enable_support");
    const bool  supports = (support && support->value) || ap.has_tree_support;
    const bool  brim     = prints_brim(object, ctx.config);

    if (brim) {
        // Generated supports slice the object with its brim already; its width stands in for the hull below.
        if (!ctx.settings.generate_supports) {
            double yaw = 0.;
            if (const ObjectBrim *shape = ctx.brims.get(*ap.instance, arrange_pose(*ap.instance, yaw), ctx.config, ctx.params, ctx.settings.generate_supports)) {
                extras.brim    = shape->areas;
                extras.brim_mm = shape->reach;
            } else
                extras.brim_exact = false;
        } else
            extras.brim_exact = false;
        if (!extras.brim_exact) {
            const auto *width = object.get_config_value<ConfigOptionFloat>(ctx.config, "brim_width");
            extras.brim_mm    = width ? width->value : 0.;
        }
    }
    // With the brim in the footprint, the spacing no longer has to stand in for it: a part without a
    // supporting allowance keeps the automatic gap of one millimetre, as the generated-support path does.
    if (extras.brim_exact && !keep_inflation && !supports && !ctx.settings.generate_supports && ctx.params.min_obj_distance == 0)
        extras.inflation = std::min(ap.inflation, scaled<coord_t>(1.));

    const double first_layer = ctx.config.opt_float("initial_layer_print_height");
    append_layer_geometry(extras.tris, extras.brim, Vec2d::Zero(), 0., first_layer);

    if (ctx.skirt.enabled) {
        extras.hull = shape_hull(mesh, extras.brim);
        if (!extras.brim_exact && extras.brim_mm > 0.) {
            const Polygons grown = offset(extras.hull, float(scaled(extras.brim_mm)));
            if (!grown.empty())
                extras.hull = grown.front();
        }
        if (ctx.skirt.per_object && extras.hull.size() >= 3) {
            const ExPolygons ring = diff_ex(offset_ex(extras.hull, float(scaled(ctx.skirt.extent))),
                                            offset_ex(extras.hull, float(scaled(ctx.skirt.distance))));
            const double     top  = first_layer + std::max(0, ctx.config.opt_int("skirt_height") - 1) *
                                                      ctx.config.opt_float("layer_height");
            append_layer_geometry(extras.tris, ring, Vec2d::Zero(), 0., top);
        }
    }
    return extras;
}

// The selected items as the engine's parts and shapes. Instances of one object posed alike, with the same
// angles and spacing, share a shape and are packed as copies. `yaw` is the Z turn arrange_pose() took out
// of each item's pose, which its result must give back.
struct Shapes
{
    std::vector<Part>                   parts;      // per item
    std::vector<std::vector<Candidate>> candidates; // per shape
    std::vector<double>                 yaw;        // per item
    std::vector<size_t>                 source;     // per shape, the item it is built from
    std::vector<std::vector<double>>    angles;     // per shape
    std::vector<Extras>                 extras;     // per shape
};

// The caller's spacing differs from item to item by its own brim, so two instances of one object share a
// shape only when their spacing agrees as well.
Shapes plan_shapes(const ArrangePolygons &selected, const ArrangeParams &params, CompatGroups &group)
{
    Shapes                   shapes;
    std::vector<Transform3d> pose(selected.size(), Transform3d::Identity());
    shapes.yaw.assign(selected.size(), 0.);
    for (size_t i = 0; i < selected.size(); ++i) {
        const ArrangePolygon &ap = selected[i];
        if (ap.instance)
            pose[i] = arrange_pose(*ap.instance, shapes.yaw[i]);
        const std::vector<double> mine = z_angles(ap, ap.rotation + shapes.yaw[i], params);
        const auto                same_shape = [&](size_t s) {
            const ArrangePolygon &o = selected[shapes.source[s]];
            if (!ap.instance || !o.instance || o.instance->get_object() != ap.instance->get_object())
                return false;
            const auto same_angle = [](double a, double b) { return std::abs(a - b) < 1e-6; };
            const auto &theirs    = shapes.angles[s];
            if (!std::equal(mine.begin(), mine.end(), theirs.begin(), theirs.end(), same_angle))
                return false;
            return o.inflation == ap.inflation && pose[shapes.source[s]].isApprox(pose[i], 1e-9);
        };
        size_t shape = 0;
        while (shape < shapes.source.size() && !same_shape(shape))
            ++shape;
        shapes.parts.push_back({int(i), int(shape), double(ap.priority), group(ap)});
        if (shape == shapes.source.size()) {
            shapes.source.push_back(i);
            shapes.angles.push_back(mine);
        }
    }
    return shapes;
}

// The shapes' candidates at `cell_mm`, from the meshes and extras planned for them.
void build_candidates(Shapes &shapes, const ArrangePolygons &selected, const ArrangeParams &params,
                      const DynamicPrintConfig &config, const ArrangeSettings &settings, double cell_mm,
                      std::vector<std::vector<Triangle>> &meshes)
{
    shapes.candidates.resize(shapes.source.size());
    const auto build_shape = [&](size_t s) {
        const ArrangePolygon &ap = selected[shapes.source[s]];
        if (settings.generate_supports && params.progressind)
            params.progressind(0, ap.name);
        const Extras &extras = shapes.extras[s];
        shapes.candidates[s].resize(shapes.angles[s].size());
        // The mesh's yaw is the item's, already folded into its angles.
        const auto build_candidate = [&](size_t k) {
            const double angle = shapes.angles[s][k];
            if (settings.generate_supports) {
                shapes.candidates[s][k] = {supported_height_map(ap, meshes[s], angle, Vec2d::Zero(), cell_mm, config, params,
                                                                extras.tris), angle};
                return;
            }
            std::vector<Triangle> tris = meshes[s];
            tris.insert(tris.end(), extras.tris.begin(), extras.tris.end());
            shapes.candidates[s][k] = {height_map(std::move(tris), angle, Vec2d::Zero(), cell_mm, ap, extras.inflation, config), angle};
        };
        if (settings.generate_supports)
            for (size_t k = 0; k < shapes.angles[s].size(); ++k)
                build_candidate(k);
        else
            parallel(shapes.angles[s].size(), build_candidate);
        meshes[s] = {};
    };
    // Each native slice already parallelizes its own work. Keeping candidates sequential bounds the
    // memory consumed by their temporary Print state, especially for tree supports at eight rotations.
    if (settings.generate_supports)
        for (size_t s = 0; s < shapes.source.size(); ++s)
            build_shape(s);
    else
        parallel(shapes.source.size(), build_shape);
}

// The obstacles, fixed items first, then the regions of the caller; the bed's outline mask comes last.
struct Obstacles
{
    std::vector<Obstacle> list;
    size_t                regions = 0; // the caller's excluded and non-preferred regions, just before the mask
    size_t                mask    = 0; // the bed outline's place in the list
};

// Fixed items on their plate, the bed's excluded regions on every plate, and for a bed that is not a
// rectangle, its outline: the cells whose centre lies outside it.
Obstacles build_obstacles(const ArrangePolygons &fixed, const Points &bed, const ArrangeParams &params,
                          const DynamicPrintConfig &config, const ArrangeSettings &settings,
                          CompatGroups &group, int max_plates, double cell_mm, ExtrasContext &ctx)
{
    const BoundingBox                   bed_bb(bed);
    const Vec2d                         bed_min = unscaled(bed_bb.min);
    Obstacles                           result;
    std::vector<Obstacle>              &obstacles = result.list;
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
            ++result.regions;
        }
    // A fixed part's brim is as real as a selected part's, and its shape is made one at a time: each
    // slices with all the threads.
    std::vector<ArrangeMesh> meshes(items.size());
    std::vector<Extras>      extras(items.size());
    for (size_t i = 0; i < items.size(); ++i) {
        if (settings.generate_supports)
            check_canceled(params);
        meshes[i] = arrange_mesh(*items[i], params.printable_height);
        extras[i] = make_extras(*items[i], meshes[i].tris, ctx, false);
    }
    const auto build_obstacle = [&](size_t i) {
        const ArrangePolygon &ap   = *items[i];
        if (settings.generate_supports && ap.instance && params.progressind)
            params.progressind(0, ap.name);
        // A fixed part sits at its rotation plus the yaw its arrange pose gave up, like an arranged part does.
        const Vec2d shift = unscaled(ap.translation) - bed_min;
        if (settings.generate_supports) {
            obstacles[i].map = supported_height_map(ap, meshes[i].tris, ap.rotation + meshes[i].yaw, shift, cell_mm, config, params,
                                                    extras[i].tris);
            return;
        }
        std::vector<Triangle> tris = std::move(meshes[i].tris);
        tris.insert(tris.end(), extras[i].tris.begin(), extras[i].tris.end());
        obstacles[i].map = height_map(std::move(tris), ap.rotation + meshes[i].yaw, shift, cell_mm, ap, extras[i].inflation, config);
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
    result.mask           = obstacles.size() - 1;
    for (int y = 0; y < mask.map.height; ++y)
        for (int x = 0; x < mask.map.width; ++x)
            if (!outline.contains(bed_bb.min + Point(scaled((x + 0.5) * cell_mm), scaled((y + 0.5) * cell_mm))))
                mask.map.at(x, y).add(-Inf, Inf);
    return result;
}

// The plate area no part may take besides the bed outline and the regions the caller passes. `all` is
// both parts together.
struct Reserved
{
    ExPolygons excluded; // the bed's excluded area as one polygon
    ExPolygons purge;    // what the start G-code lays on the plate
    ExPolygons all;
};

// The bed's excluded area as one polygon, as the plate draws it and the arrange report measures it
// (callers pass it as the bounding boxes of every four points, which loses the rest of a polygon with more
// corners), and the material the machine start G-code lays on the plate before the first layer, a purge line.
Reserved reserved_area(const ArrangePolygons &selected, const ArrangePolygons &fixed, const Points &bed,
                       const DynamicPrintConfig &config, coord_t inflation)
{
    Reserved reserved;
    if (const auto *excluded = config.option<ConfigOptionPoints>("bed_exclude_area"); excluded && excluded->values.size() >= 3) {
        Polygon polygon;
        for (const Vec2d &pt : excluded->values)
            polygon.points.push_back(scaled(pt));
        reserved.excluded = union_ex(Polygons{polygon});
    }
    // The start G-code primes the filament the plate starts with; its moves rarely depend on which.
    int first = INT_MAX;
    for (const ArrangePolygons *items : {&selected, &fixed})
        for (const ArrangePolygon &ap : *items)
            if (ap.instance)
                for (int id : ap.extrude_ids)
                    first = std::min(first, id);
    // Parts lie inside the bed, so only what comes within both spacings of it matters.
    coord_t reach = inflation + scaled(1.);
    for (const ArrangePolygon &ap : selected)
        reach = std::max(reach, inflation + ap.inflation + scaled(1.));
    const Polygons   near_bed    = offset(Polygon(bed), float(reach));
    reserved.purge = intersection_ex(
        start_gcode_extrusion_area(config, first == INT_MAX || first < 1 ? 0u : unsigned(first - 1)), near_bed);
    if (!reserved.purge.empty()) {
        const BoundingBox bb = get_extents(reserved.purge);
        BOOST_LOG_TRIVIAL(info) << "Nest3D: keeping clear of the start G-code extrusions within ["
                                << unscaled(bb.min).transpose() << "] - [" << unscaled(bb.max).transpose() << "]";
    }
    reserved.excluded = intersection_ex(reserved.excluded, near_bed);
    reserved.all      = union_ex(reserved.excluded, reserved.purge);
    return reserved;
}

// The plate and the bed's shrink, as the caller handed them over; empty when the printable area does
// not match the bed, as for a bed the caller reshaped.
Points printable_of(const Points &bed, const ArrangeParams &params, const DynamicPrintConfig &config)
{
    const Points printable = get_bed_shape(config);
    if (printable.size() < 3 || bed.size() < 3)
        return {};
    // The caller's bed is the printable area shrunk by bed_shrink on each axis.
    const BoundingBox printable_bb(printable), bed_bb(bed);
    const coord_t     tolerance = scaled<coord_t>(0.01);
    if (std::abs(printable_bb.size().x() - bed_bb.size().x() - 2 * scaled<coord_t>(params.bed_shrink_x)) > tolerance ||
        std::abs(printable_bb.size().y() - bed_bb.size().y() - 2 * scaled<coord_t>(params.bed_shrink_y)) > tolerance)
        return {};
    return printable;
}

constexpr coord_t NoMargin = std::numeric_limits<coord_t>::max();

// The bed the engine packs on. Callers hand over the printable area pulled in by a bed shrink, and every
// part is grown by half the spacing besides, which keeps parts apart but would count a second time at the
// bed's edge. Here the printable area is the bed and a part's footprint may reach its boundary; only what
// prints beyond the footprint stays inside. `margins` holds, for each selected part, the spacing it is grown
// by less that extent (NoMargin: the part sets none). The bed moves by the least of them, so the part with
// the widest extent keeps it whole inside and the others lie at least as far in as they must. Falls
// back to the caller's bed when the printable area does not match it.
Points engine_bed(const Points &bed, const std::vector<coord_t> &margins, const ArrangeParams &params,
                  const DynamicPrintConfig &config)
{
    const Points printable = printable_of(bed, params, config);
    if (printable.empty() || margins.empty())
        return bed;
    const coord_t grow = *std::min_element(margins.begin(), margins.end());
    if (grow == NoMargin)
        return bed;
    const Polygons moved = offset(Polygon(printable), float(grow));
    return moved.size() == 1 ? moved.front().points : bed;
}

// The margin of the uniform allowance: the spacing, less the brim width and the skirt's distance and loops.
std::vector<coord_t> uniform_margins(const ArrangePolygons &selected, const ArrangeParams &params)
{
    std::vector<coord_t> margins;
    for (const ArrangePolygon &ap : selected)
        margins.push_back(ap.is_virt_object ? NoMargin :
                          ap.inflation - scaled<coord_t>(ap.brim_width + params.brim_skirt_distance));
    return margins;
}

// The margins with each part's own brim in its footprint: only a skirt that encloses the whole plate
// stays outside the footprints, since an object's own skirt is part of its footprint.
std::vector<coord_t> footprint_margins(const ArrangePolygons &selected, const Shapes &shapes, const ArrangeParams &params,
                                       const SkirtPlan &skirt)
{
    std::vector<coord_t> margins = uniform_margins(selected, params);
    const coord_t        plate_skirt = skirt.enabled && !skirt.per_object ? scaled<coord_t>(params.brim_skirt_distance) : 0;
    for (size_t i = 0; i < selected.size(); ++i) {
        const Extras &extras = shapes.extras[size_t(shapes.parts[i].shape)];
        if (!selected[i].is_virt_object && extras.brim_exact)
            margins[i] = extras.inflation - plate_skirt;
    }
    return margins;
}

// The spacing a part keeps from the bed's excluded area: what the caller gave its excluded regions
// (1 mm, or half of an explicit gap, which the part's own inflation completes).
coord_t exclusion_inflation(const ArrangeParams &params)
{
    coord_t inflation = 0;
    for (const ArrangePolygon &ap : params.excluded_regions)
        inflation = std::max(inflation, ap.inflation);
    return params.excluded_regions.empty() ? std::max<coord_t>(scaled(1.), params.min_obj_distance / 2) : inflation;
}

Obstacle reserved_obstacle(const ExPolygons &area, coord_t inflation, const Points &bed, const ArrangeParams &params,
                           const DynamicPrintConfig &config, double cell_mm)
{
    std::vector<Triangle> tris;
    append_layer_geometry(tris, area, Vec2d::Zero(), 0., params.printable_height);
    ArrangePolygon region;
    region.inflation = inflation;
    return {-1, height_map(std::move(tris), 0., -unscaled(BoundingBox(bed).min), cell_mm, region, inflation, config, true), -1};
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

// Parts left out, then plates used, of an engine result: lower is better.
std::pair<int, int> placement_cost(const std::vector<Placement> &placements)
{
    std::pair<int, int> cost{0, 0};
    for (const Placement &pl : placements) {
        cost.first += pl.plate < 0;
        cost.second = std::max(cost.second, pl.plate + 1);
    }
    return cost;
}

// Which limit a skirt around everything on a plate crosses.
enum class SkirtHit { None, ExcludedArea, PurgeStrip, Bed };

// The skirt that encloses a whole plate is a ring around the convex hull of everything on it, brims
// included, which no single part's footprint can see. It must lie on the printable area and clear of the
// excluded area and the purge strip. Checked on the placements; fixed parts count with their outline and
// brim width.
SkirtHit combined_skirt_hit(const std::vector<Placement> &placements, const Shapes &shapes, const ArrangePolygons &fixed,
                            const Vec2d &bed_min, const Points &printable, const ExPolygons &excluded, const ExPolygons &purge,
                            const SkirtPlan &skirt)
{
    std::map<int, Points> plates;
    for (const Placement &pl : placements) {
        if (pl.plate < 0)
            continue;
        const Polygon &hull = shapes.extras[size_t(shapes.parts[size_t(pl.id)].shape)].hull;
        const double   c = std::cos(pl.rotation_z), s = std::sin(pl.rotation_z);
        for (const Point &pt : hull.points) {
            const Vec2d p = unscaled(pt);
            plates[pl.plate].push_back(scaled(Vec2d(bed_min.x() + pl.x + c * p.x() - s * p.y(), bed_min.y() + pl.y + s * p.x() + c * p.y())));
        }
    }
    for (const ArrangePolygon &ap : fixed) {
        if (ap.bed_idx < 0 || ap.is_virt_object)
            continue;
        Polygons grown = offset(ap.transformed_poly().contour, float(scaled(ap.brim_width)));
        for (const Polygon &poly : grown)
            append(plates[ap.bed_idx], poly.points);
    }
    // Overlap thinner than a bead of the skirt's own tolerance is not a crossing.
    const double tolerance = sqr(scaled(0.7));
    for (auto &[plate, points] : plates) {
        if (points.size() < 3)
            continue;
        const Polygon    hull = Geometry::convex_hull(std::move(points));
        const ExPolygons ring = diff_ex(offset_ex(hull, float(scaled(skirt.extent))), offset_ex(hull, float(scaled(skirt.distance))));
        if (area(intersection_ex(ring, excluded)) > tolerance)
            return SkirtHit::ExcludedArea;
        if (area(intersection_ex(ring, purge)) > tolerance)
            return SkirtHit::PurgeStrip;
        if (!printable.empty() && area(diff_ex(ring, ExPolygons{ExPolygon(Polygon(printable))})) > tolerance)
            return SkirtHit::Bed;
    }
    return SkirtHit::None;
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
// result already there is kept without running it. `footprints` are the items as that arranger should see
// them, hulls grown by their brim and skirt and spaced as the shapes are. A cancelled arrange keeps what
// it placed: run under the same stop condition, the outline arranger returns every part "arranged" on top
// of the others.
void outline_arranger_guard(ArrangePolygons &selected, const ArrangePolygons &input, const ArrangePolygons &footprints,
                            const ArrangePolygons &fixed, const Points &bed, const ArrangeParams &params,
                            const ExPolygons &reserved, coord_t reserved_inflation)
{
    const bool stopped = params.stopcondition && params.stopcondition();
    if (stopped)
        return;
    double hulls = 0.;
    for (const ArrangePolygon &ap : footprints)
        hulls += ap.poly.area();
    const int plates_needed = std::max(1, int(std::ceil(hulls / BoundingBox(bed).area() - 1e-9)));
    if (unplaced_and_plates(selected) <= std::make_pair(0, plates_needed))
        return;
    // The outline arranger only knows the regions its caller passed, so it gets the reserved area too.
    ArrangeParams outline_params = params;
    for (const ExPolygon &region : reserved) {
        ArrangePolygon ap;
        ap.poly      = region;
        ap.inflation = reserved_inflation;
        outline_params.excluded_regions.push_back(std::move(ap));
    }
    ArrangePolygons outline_result = footprints;
    arrangement::arrange(outline_result, fixed, bed, outline_params);
    if (unplaced_and_plates(outline_result) < unplaced_and_plates(selected)) {
        // Only the places are taken over; the items stay what the caller handed in.
        for (size_t i = 0; i < outline_result.size(); ++i) {
            outline_result[i].poly      = input[i].poly;
            outline_result[i].inflation = input[i].inflation;
        }
        selected = std::move(outline_result);
    }
}


// The footprint's convex hull, in the shape's pose: geometry and brim, and the skirt that goes around the
// object alone.
Polygon footprint_hull(const ArrangePolygon &ap, const Extras &extras, const SkirtPlan &skirt)
{
    Polygon hull = extras.hull;
    if (hull.empty()) {
        hull = shape_hull(arrange_mesh(ap, ap.height).tris, extras.brim);
        if (!extras.brim_exact && extras.brim_mm > 0.) {
            const Polygons grown = offset(hull, float(scaled(extras.brim_mm)));
            if (!grown.empty())
                hull = grown.front();
        }
    }
    if (skirt.enabled && skirt.per_object && hull.size() >= 3) {
        const Polygons grown = offset(hull, float(scaled(skirt.extent)));
        if (!grown.empty())
            hull = grown.front();
    }
    return hull;
}

// The width and height in mm of `hull` turned by the angle that leaves it the smallest box, the longer
// side first compared, among `angles`.
std::pair<double, double> smallest_box(const Polygon &hull, const std::vector<double> &angles)
{
    std::pair<double, double> best{Inf, Inf};
    for (double angle : angles) {
        const double c = std::cos(angle), s = std::sin(angle);
        double       x0 = Inf, x1 = -Inf, y0 = Inf, y1 = -Inf;
        for (const Point &pt : hull.points) {
            const Vec2d p = unscaled(pt);
            const double x = c * p.x() - s * p.y(), y = s * p.x() + c * p.y();
            x0 = std::min(x0, x);
            x1 = std::max(x1, x);
            y0 = std::min(y0, y);
            y1 = std::max(y1, y);
        }
        const std::pair<double, double> box{x1 - x0, y1 - y0};
        if (std::make_pair(std::max(box.first, box.second), std::min(box.first, box.second)) <
            std::make_pair(std::max(best.first, best.second), std::min(best.first, best.second)))
            best = box;
    }
    return best;
}

// Everything the explanations of a result need.
struct Explain
{
    const ArrangePolygons    &selected;
    const Shapes             &shapes;
    const Obstacles          &obstacles;
    const Reserved           &reserved;
    const ArrangeParams      &params;
    const DynamicPrintConfig &config;
    const SkirtPlan          &skirt;
    const Points             &bed;
    const Params             &engine;
    coord_t                   reserved_inflation;
    int                       max_plates;
};

// Whether the shape fits on an empty plate that has only `obstacles`.
bool fits_alone(const Explain &e, size_t shape, int group, const std::vector<Obstacle> &obstacles)
{
    Params p     = e.engine;
    p.max_plates = 1;
    p.progress   = nullptr;
    const std::vector<std::vector<Candidate>> one{e.shapes.candidates[shape]};
    const std::vector<Placement>              placed = arrange(one, {Part{0, 0, 0., group}}, obstacles, p);
    return !placed.empty() && placed.front().plate >= 0;
}

// The sentence that gives a part's size with what prints around it.
Sentence measures(const std::string &name, std::pair<double, double> size, double brim_mm, bool own_skirt)
{
    std::vector<std::string> args{name, length_text(size.first), length_text(size.second)};
    if (brim_mm >= 0.05) {
        args.push_back(length_text(brim_mm));
        // TRN %1% is the object's name, %2% and %3% its width and depth, %4% the width of its brim.
        return {own_skirt ? L("%1% measures %2% × %3% mm with its %4% mm brim and its skirt.") :
                            L("%1% measures %2% × %3% mm with its %4% mm brim."), std::move(args)};
    }
    // TRN %1% is the object's name, %2% and %3% its width and depth.
    return {own_skirt ? L("%1% measures %2% × %3% mm with its skirt.") : L("%1% measures %2% × %3% mm."), std::move(args)};
}

Sentence size_hint(double brim_mm, bool skirt, bool rotates)
{
    if (brim_mm >= 0.05)
        return {rotates ? L("Reduce the brim or make the part smaller.") : L("Reduce the brim or rotate the part."), {}};
    if (skirt)
        return {L("Reduce the skirt distance or loops, or make the part smaller."), {}};
    return {rotates ? L("Make the part smaller.") : L("Turn on Auto rotate or make the part smaller."), {}};
}

// Why the parts that did not get a place on the first plate they could have asked for are where they are:
// the part left out, or moved to a later plate. A part that fits no empty plate is told which limit stops
// it, with the room that limit leaves; one that would fit is told the others take its room.
std::vector<Issue> explain_result(const Explain &e)
{
    std::vector<Issue> issues;
    int  last_plate = -1;
    for (const ArrangePolygon &ap : e.selected)
        last_plate = std::max(last_plate, ap.bed_idx);
    const bool plates_full = last_plate + 1 >= e.max_plates;
    for (size_t i = 0; i < e.selected.size(); ++i) {
        const ArrangePolygon &ap = e.selected[i];
        if (ap.is_virt_object)
            continue;
        const Part   &part = e.shapes.parts[i];
        const Extras &extras = e.shapes.extras[size_t(part.shape)];
        bool          spilled = false;
        if (ap.is_arranged()) {
            for (size_t j = 0; j < e.selected.size() && !spilled; ++j)
                spilled = e.shapes.parts[j].group == part.group && e.selected[j].is_arranged() && e.selected[j].bed_idx < ap.bed_idx;
            if (!spilled)
                continue;
        }
        Issue issue;
        issue.part = ap.name;
        const Polygon hull = footprint_hull(ap, extras, e.skirt);
        const auto    size = smallest_box(hull, e.shapes.angles[size_t(part.shape)]);
        const bool    own_skirt = e.skirt.enabled && e.skirt.per_object;
        const double  brim_mm = extras.brim_mm;
        issue.sentences.push_back(measures(ap.name, size, brim_mm, own_skirt));
        const bool rotates = e.params.allow_rotations;
        if (spilled) {
            issue.kind  = Issue::Kind::Spilled;
            issue.limit = Issue::Limit::OtherParts;
            issue.sentences.push_back({L("It was moved to the next plate because the parts already placed leave no room for it."), {}});
            issue.sentences.push_back({L("Reduce the spacing or the brim, or move a part to another plate."), {}});
            issues.push_back(std::move(issue));
            continue;
        }
        issue.kind = Issue::Kind::Unplaced;
        // The first limit the part fails against, adding them one at a time to an empty plate.
        const std::vector<Obstacle> &list = e.obstacles.list;
        const size_t                 regions_begin = e.obstacles.mask - e.obstacles.regions;
        std::vector<Obstacle>        scene(list.begin() + std::ptrdiff_t(regions_begin), list.begin() + std::ptrdiff_t(e.obstacles.mask) + 1); // regions, then the outline
        const double                 cell = e.engine.cell_mm;
        const coord_t                inflation = extras.inflation;
        ExPolygons                   blocked;
        for (const ArrangePolygons *regions : {&e.params.excluded_regions, &e.params.nonprefered_regions})
            for (const ArrangePolygon &region : *regions)
                expolygons_append(blocked, offset_ex(region.transformed_poly(), float(region.inflation)));
        const Polygon     bed_outline(e.bed);
        const BoundingBox bed_bb(e.bed);
        // The room the footprint has: the bed pulled in by the spacing, less what is blocked and the spacing around it.
        const auto room = [&](const ExPolygons &blockers) {
            const ExPolygons free_area = diff_ex(offset_ex(bed_outline, -float(inflation)), offset_ex(blockers, float(inflation)));
            return largest_free_rectangle(free_area, bed_bb, 1.);
        };
        const auto add_room = [&](Issue::Limit limit, const char *msgid, const ExPolygons &blockers) {
            issue.limit = limit;
            const auto rect = room(blockers);
            issue.sentences.push_back({msgid, {length_text(rect.first), length_text(rect.second)}});
            issue.sentences.push_back(size_hint(brim_mm, e.skirt.enabled, rotates));
        };
        // The scene without the reserved areas first: only the bed and the caller's regions.
        std::vector<Obstacle> only_bed{list[e.obstacles.mask]};
        if (!fits_alone(e, size_t(part.shape), part.group, only_bed))
            // TRN %1% and %2% are the width and depth of the largest free area, in mm.
            add_room(Issue::Limit::Bed, L("The room left on the bed is %1% × %2% mm."), {});
        else if (!fits_alone(e, size_t(part.shape), part.group, scene))
            // TRN %1% and %2% are the width and depth of the largest free area, in mm.
            add_room(Issue::Limit::ExcludedArea, L("The room left beside the excluded area is %1% × %2% mm."), blocked);
        else {
            ExPolygons with_excluded = blocked;
            expolygons_append(with_excluded, offset_ex(e.reserved.excluded, float(e.reserved_inflation)));
            std::vector<Obstacle> with_reserved_excluded = scene;
            if (!e.reserved.excluded.empty())
                with_reserved_excluded.push_back(reserved_obstacle(e.reserved.excluded, e.reserved_inflation, e.bed, e.params, e.config, cell));
            std::vector<Obstacle> everything = scene;
            if (!e.reserved.all.empty())
                everything.push_back(reserved_obstacle(e.reserved.all, e.reserved_inflation, e.bed, e.params, e.config, cell));
            if (!e.reserved.excluded.empty() && !fits_alone(e, size_t(part.shape), part.group, with_reserved_excluded))
                add_room(Issue::Limit::ExcludedArea, L("The room left beside the excluded area is %1% × %2% mm."), with_excluded);
            else if (!fits_alone(e, size_t(part.shape), part.group, everything)) {
                ExPolygons with_purge = with_excluded;
                expolygons_append(with_purge, offset_ex(e.reserved.purge, float(e.reserved_inflation)));
                // TRN %1% and %2% are the width and depth of the largest free area, in mm.
                add_room(Issue::Limit::PurgeStrip, L("The room left beside the start G-code purge line is %1% × %2% mm."), with_purge);
            } else if (plates_full) {
                issue.limit = Issue::Limit::Plates;
                issue.sentences.push_back({L("Every plate is full."), {}});
                issue.sentences.push_back({L("Remove a part to make room."), {}});
            } else {
                issue.limit = Issue::Limit::OtherParts;
                issue.sentences.push_back({L("The parts already placed leave no room for it."), {}});
                issue.sentences.push_back({L("Reduce the spacing or the brim, or move a part to another plate."), {}});
            }
        }
        issues.push_back(std::move(issue));
    }
    return issues;
}

Issue skirt_issue(SkirtHit hit)
{
    Issue issue;
    issue.kind = Issue::Kind::Skirt;
    switch (hit) {
    case SkirtHit::ExcludedArea:
        issue.limit = Issue::Limit::ExcludedArea;
        issue.sentences.push_back({L("The skirt around all objects would cross the excluded area."), {}});
        break;
    case SkirtHit::PurgeStrip:
        issue.limit = Issue::Limit::PurgeStrip;
        issue.sentences.push_back({L("The skirt around all objects would cross the start G-code purge line."), {}});
        break;
    default:
        issue.limit = Issue::Limit::Bed;
        issue.sentences.push_back({L("The skirt around all objects would cross the edge of the bed."), {}});
        break;
    }
    issue.sentences.push_back({L("Reduce the skirt distance or loops, or move the objects inward."), {}});
    return issue;
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
                      const Points             &caller_bed,
                      const ArrangeParams      &params,
                      const ArrangeSettings    &settings,
                      const DynamicPrintConfig &config,
                      int                       max_plates,
                      std::vector<Issue>       *issues)
{
    if (settings.generate_supports)
        validate_support_shrinkage(selected, fixed, config, params);
    CompatGroups group = compat_groups(selected, fixed, params);
    Shapes       shapes = plan_shapes(selected, params, group);

    // Brim and skirt shapes first: the bed they leave depends on them, and the cell size on the bed.
    ExtrasContext ctx{params, settings, config, skirt_plan(config), {}};
    const bool    keep_inflation = std::any_of(selected.begin(), selected.end(), [](const ArrangePolygon &ap) { return ap.has_tree_support; });
    std::vector<std::vector<Triangle>> meshes(shapes.source.size());
    parallel(meshes.size(), [&](size_t s) { meshes[s] = arrange_mesh(selected[shapes.source[s]], selected[shapes.source[s]].height).tris; });
    for (size_t s = 0; s < meshes.size(); ++s) {
        if (settings.generate_supports)
            check_canceled(params);
        shapes.extras.push_back(make_extras(selected[shapes.source[s]], meshes[s], ctx, keep_inflation));
    }

    const Points      bed = engine_bed(caller_bed, footprint_margins(selected, shapes, params, ctx.skirt), params, config);
    const BoundingBox bed_bb(bed);
    const Params      p     = engine_params(bed_bb, params, settings, max_plates, selected.size());
    build_candidates(shapes, selected, params, config, settings, p.cell_mm, meshes);
    Obstacles         obstacles = build_obstacles(fixed, bed, params, config, settings, group, max_plates, p.cell_mm, ctx);
    const coord_t     reserved_inflation = exclusion_inflation(params);
    const Reserved    reserved  = reserved_area(selected, fixed, bed, config, reserved_inflation);
    // Where the reserved area sits in the list, to widen it for a skirt; past the end when there is none.
    size_t reserved_index = std::numeric_limits<size_t>::max();
    if (!reserved.all.empty()) {
        reserved_index = obstacles.list.size();
        obstacles.list.push_back(reserved_obstacle(reserved.all, reserved_inflation, bed, params, config, p.cell_mm));
    }

    const ArrangePolygons input = selected; // for the outline arranger below
    std::vector<Placement> placements = arrange(shapes.candidates, shapes.parts, obstacles.list, p);

    // One skirt around everything on the plate, if the print makes one that way, has to clear the
    // excluded area and the purge strip as well. Nothing above could know where it falls, so the result
    // is checked and, if it crosses one, the parts are packed again with the skirt's width added to the room
    // they keep from those areas.
    std::vector<Issue> found;
    if (ctx.skirt.enabled && !ctx.skirt.per_object && !(params.stopcondition && params.stopcondition())) {
        const Points printable = printable_of(caller_bed, params, config);
        const Vec2d  bed_min   = unscaled(bed_bb.min);
        // The caller's excluded regions are the bed's excluded area as the plate lists it.
        ExPolygons excluded = reserved.excluded;
        for (const ArrangePolygon &region : params.excluded_regions)
            expolygons_append(excluded, ExPolygons{region.transformed_poly()});
        excluded = union_ex(excluded);
        SkirtHit hit = combined_skirt_hit(placements, shapes, fixed, bed_min, printable, excluded, reserved.purge, ctx.skirt);
        if (hit == SkirtHit::ExcludedArea || hit == SkirtHit::PurgeStrip) {
            BOOST_LOG_TRIVIAL(info) << "Nest3D: the skirt around all objects crosses the excluded area or the purge strip; packing again with room for it";
            Obstacle wider = reserved_obstacle(union_ex(excluded, reserved.purge), reserved_inflation + scaled<coord_t>(ctx.skirt.extent), bed,
                                               params, config, p.cell_mm);
            if (reserved_index < obstacles.list.size())
                obstacles.list[reserved_index] = std::move(wider);
            else
                obstacles.list.push_back(std::move(wider));
            std::vector<Placement> wider_placements = arrange(shapes.candidates, shapes.parts, obstacles.list, p);
            const SkirtHit         wider_hit = combined_skirt_hit(wider_placements, shapes, fixed, bed_min, printable, excluded, reserved.purge, ctx.skirt);
            // Packing with more room must not cost parts or plates: then the skirt is reported instead.
            if (placement_cost(wider_placements) <= placement_cost(placements)) {
                placements = std::move(wider_placements);
                hit        = wider_hit;
            }
        }
        if (hit != SkirtHit::None)
            found.push_back(skirt_issue(hit));
    }
    if (settings.generate_supports)
        check_canceled(params);
    apply_placements(placements, shapes.yaw, unscaled(bed_bb.min), selected);
    if (!settings.generate_supports) {
        ArrangePolygons footprints = input;
        for (size_t i = 0; i < footprints.size(); ++i) {
            const Extras &extras = shapes.extras[size_t(shapes.parts[i].shape)];
            if (footprints[i].is_virt_object || !extras.brim_exact)
                continue;
            footprints[i].inflation = extras.inflation;
            const double grow = extras.brim_mm + (ctx.skirt.enabled && ctx.skirt.per_object ? ctx.skirt.extent : 0.);
            if (grow > 0.) {
                const ExPolygons bigger = offset_ex(footprints[i].poly, float(scaled(grow)));
                if (!bigger.empty())
                    footprints[i].poly = bigger.front();
            }
        }
        outline_arranger_guard(selected, input, footprints, fixed, bed, params, reserved.all, reserved_inflation);
    }
    if (issues) {
        std::vector<Issue> explained = explain_result(Explain{selected, shapes, obstacles, reserved, params, config, ctx.skirt, bed, p,
                                                              reserved_inflation, max_plates});
        issues->insert(issues->end(), std::make_move_iterator(found.begin()), std::make_move_iterator(found.end()));
        issues->insert(issues->end(), std::make_move_iterator(explained.begin()), std::make_move_iterator(explained.end()));
    }
}

}} // namespace Slic3r::nest3d

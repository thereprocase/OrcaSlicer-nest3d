#include "Footprint.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Brim.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {

namespace {

// PrintBase hands out its cancellation check to its own subclasses only.
struct ScratchPrint : Print
{
    PrintTryCancel try_cancel() const { return make_try_cancel(); }
};

} // namespace

SkirtPlan skirt_plan(const DynamicPrintConfig &config)
{
    SkirtPlan plan;
    plan.enabled = has_skirt(config);
    if (!plan.enabled)
        return plan;
    plan.per_object = config.opt_enum<SkirtType>("skirt_type") == stPerObject;
    plan.distance   = config.opt_float("skirt_distance");
    plan.extent     = get_real_skirt_dist(config); // the distance and the width of every loop
    return plan;
}

bool prints_brim(ModelObject &object, const DynamicPrintConfig &config)
{
    // The test PrintObject::has_brim() makes, on the object's own settings.
    const auto *type = object.get_config_value<ConfigOptionEnum<BrimType>>(config, "brim_type");
    if (!type || type->value == btNoBrim)
        return false;
    if (type->value == btAutoBrim)
        return true;
    if (type->value == btPainted)
        return !object.brim_points.empty();
    const auto *width = object.get_config_value<ConfigOptionFloat>(config, "brim_width");
    return width && width->value > 0.;
}

void apply_scratch_print(Print &print, const ModelInstance &instance, const Transform3d &transform,
                         const DynamicPrintConfig &config)
{
    Model        model;
    ModelObject *object = model.add_object(*instance.get_object());
    object->clear_instances();
    ModelInstance *copy = object->add_instance(instance);
    copy->set_transformation(Geometry::Transformation(transform));
    object->printable                = true;
    copy->printable                  = true;
    copy->print_volume_state         = ModelInstancePVS_Inside;
    // ArrangeMesh also normalizes the lowest vertex to the bed, including objects whose GUI auto-drop
    // switch is off. Only move this temporary instance, using the same normalization.
    object->translate_instance(0, Vec3d(0., 0., -object->instance_bounding_box(0).min.z()));

    DynamicPrintConfig slice_config = DynamicPrintConfig::full_print_config();
    slice_config.apply(config);
    const BoundingBoxf3 bounds = object->instance_bounding_box(0);
    const double        margin = std::max(1000., bounds.size().norm() * 2.);
    slice_config.set_key_value("printable_area", new ConfigOptionPoints({
        Vec2d(bounds.min.x() - margin, bounds.min.y() - margin),
        Vec2d(bounds.max.x() + margin, bounds.min.y() - margin),
        Vec2d(bounds.max.x() + margin, bounds.max.y() + margin),
        Vec2d(bounds.min.x() - margin, bounds.max.y() + margin)}));
    slice_config.set_key_value("bed_exclude_area", new ConfigOptionPoints());

    print.set_status_silent();
    print.set_no_check_flag(true);
    print.auto_assign_extruders(object);
    print.apply(model, std::move(slice_config));
}

ObjectBrim object_brim(const ModelInstance &instance, const Transform3d &pose, const DynamicPrintConfig &config,
                       const std::function<bool()> &canceled)
{
    const auto check_canceled = [&canceled] {
        if (canceled && canceled())
            throw CanceledException();
    };
    check_canceled();
    ScratchPrint print;
    apply_scratch_print(print, instance, pose, config);
    if (print.objects().empty())
        return {};
    // The brim generator needs the sliced first layer and nothing after it: no perimeters, infill or
    // support, which is most of what process() costs.
    PrintObject *object = print.objects_mutable().front();
    object->slice();
    check_canceled();

    // One object on one extruder, so every brim of the print is this object's.
    Polygons                                       islands;
    std::map<ObjectID, ExtrusionEntityCollection>  by_object;
    std::map<ObjectInstanceID, ExtrusionEntityCollection> by_instance;
    std::vector<std::pair<ObjectID, unsigned int>> objects{{object->id(), 1u}};
    std::vector<unsigned int>                      extruders{0u};
    std::map<ObjectInstanceID, ExPolygons>         areas;
    make_brim(print, print.try_cancel(), islands, by_object, by_instance, objects, extruders, &areas);

    // The areas are in plate coordinates, where the instance sits at its shift (its offset, here zero, and
    // the object's centring); the slices are about the centre.
    ObjectBrim result;
    for (auto &[id, brim] : areas)
        expolygons_append(result.areas, std::move(brim));
    if (result.areas.empty())
        return result;
    const BoundingBox first_layer = get_extents(object->layers().front()->lslices);
    const BoundingBox brim_box    = get_extents(result.areas);
    const Point       shift       = object->instances().front().shift_without_plate_offset();
    const BoundingBox moved(first_layer.min + shift, first_layer.max + shift);
    const coord_t     reach = std::max({moved.min.x() - brim_box.min.x(), moved.min.y() - brim_box.min.y(),
                                        brim_box.max.x() - moved.max.x(), brim_box.max.y() - moved.max.y(), coord_t(0)});
    result.reach = unscaled(reach);
    return result;
}

}} // namespace Slic3r::nest3d

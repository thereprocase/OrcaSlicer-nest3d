#include "FillMorphingSpiral.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include <boost/log/trivial.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace Slic3r {
namespace {
constexpr int samples = 720;
constexpr double tau = 6.2831853071795864769;
constexpr double phase = .000123;
double cross(const Vec2d& a, const Vec2d& b) { return a.x()*b.y()-a.y()*b.x(); }
Vec2d mm(const Point& p) { return Vec2d(unscale<double>(p.x()),unscale<double>(p.y())); }
Point point(const Vec2d& p) { return Point::new_scale(p.x(),p.y()); }

bool radii(const Polygon& polygon, const Vec2d& center, std::vector<double>& result)
{
    if (polygon.points.size()<3 || polygon.points.size()>10000) return false;
    result.resize(samples);
    for (int k=0;k<samples;++k) {
        const double angle=phase+tau*k/samples;
        const Vec2d ray(std::cos(angle),std::sin(angle));
        int hits=0;
        for (size_t i=0;i<polygon.points.size();++i) {
            const Vec2d a=mm(polygon.points[i])-center;
            const Vec2d edge=mm(polygon.points[(i+1)%polygon.points.size()])-center-a;
            const double den=cross(ray,edge);
            if (std::abs(den)<1e-12) continue;
            const double distance=cross(a,edge)/den;
            const double t=cross(a,ray)/den;
            if (distance>0 && t>=-1e-10 && t<1.-1e-10) { result[k]=distance; ++hits; }
        }
        if (hits!=1) return false;
    }
    return true;
}
std::vector<double> derivative(const std::vector<double>& r)
{
    std::vector<double> d(samples);
    for (int i=0;i<samples;++i) d[i]=(r[(i+1)%samples]-r[(i+samples-1)%samples])/(2*tau/samples);
    return d;
}
void append_point(ThickPolyline& path, const Vec2d& p, double width)
{
    const double scaled_width=scale_(width);
    if (!path.points.empty()) {
        path.width.emplace_back(path.width.empty()?scaled_width:path.width.back());
        path.width.emplace_back(scaled_width);
    }
    path.points.emplace_back(point(p));
}
// Polygonal swept-strip audit using the sampled path's vertex normals. Building
// one strip per path avoids an expensive union of a cap at every vertex.
// This is geometric coverage, not a molten-bead or local-height simulation.
Polygons footprint(const ThickPolylines& paths, double& strip_area)
{
    Polygons strips;
    strip_area=0;
    for (const ThickPolyline& path:paths) {
        const size_t n=path.points.size();
        Polygon strip;
        Points right;
        for (size_t i=0;i<n;++i) {
            const Vec2d p=mm(path.points[i]);
            Vec2d tangent=mm(path.points[std::min(i+1,n-1)])-mm(path.points[i==0?0:i-1]);
            const double length=tangent.norm();
            if (length<1e-9) continue;
            const Vec2d normal(-tangent.y()/length,tangent.x()/length);
            const double width=unscale<double>(i==0?path.width.front():path.width[2*i-1]);
            strip.points.emplace_back(point(p+normal*width/2));
            right.emplace_back(point(p-normal*width/2));
            if (i+1<n) strip_area+=(mm(path.points[i+1])-p).norm()*
                unscale<double>(path.width[2*i]+path.width[2*i+1])/2;
        }
        std::reverse(right.begin(),right.end());
        append(strip.points,std::move(right));
        strip.make_counter_clockwise();
        strips.emplace_back(std::move(strip));
        for (size_t i : {size_t(0),n-1}) {
            const double width=unscale<double>(i==0?path.width.front():path.width.back());
            Polygon cap;
            for (int j=0;j<16;++j) cap.points.emplace_back(point(mm(path.points[i])+Vec2d(std::cos(tau*j/16),std::sin(tau*j/16))*width/2));
            strips.emplace_back(std::move(cap));
        }
    }
    return union_(strips);
}
double area_mm2(const ExPolygons& polygons)
{
    double area=0;for (const ExPolygon& p:polygons) area+=unscale<double>(unscale<double>(p.area()));return area;
}
}

MorphingSpiralResult morphing_spiral(const ExPolygon& domain,double spacing,double min_spacing,double max_spacing)
{
    MorphingSpiralResult result;
    auto reject=[&](const char* reason) { result.paths.clear();result.fallback_reason=reason;return result; };
    if (!std::isfinite(spacing)||!std::isfinite(min_spacing)||!std::isfinite(max_spacing)||
        min_spacing<=0||spacing<min_spacing||spacing>max_spacing) return reject("invalid spacing limits");
    if (domain.holes.size()!=1) return reject("requires a single-hole annulus; disk centre not supported yet");
    if (domain.holes.front().points.size()<3) return reject("degenerate hole");
    const Vec2d center=mm(domain.holes.front().centroid());
    std::vector<double> outer,inner;
    if (!radii(domain.contour,center,outer)||!radii(domain.holes.front(),center,inner))
        return reject("boundaries are not single-valued around the hole centre");
    double mean=0;
    for (int i=0;i<samples;++i) {
        if (outer[i]-inner[i]<3*spacing) return reject("annulus too narrow");
        mean+=(outer[i]-inner[i])/samples;
    }
    if (mean/spacing>301) return reject("turn budget exceeded");
    const int turns=std::max(1,int(std::round(mean/spacing))-1);
    auto dout=derivative(outer),din=derivative(inner);
    for (int i=0;i<samples;++i) {
        outer[i]-=spacing*.5*std::hypot(outer[i],dout[i])/outer[i];
        inner[i]+=spacing*.5*std::hypot(inner[i],din[i])/inner[i];
        if (outer[i]<=inner[i]) return reject("offset boundaries cross");
    }
    dout=derivative(outer);din=derivative(inner);
    ThickPolylines candidate;
    for (const auto* r:{&outer,&inner}) {
        ThickPolyline boundary;
        for (int i=0;i<=samples;++i) {
            const double angle=phase+tau*(i%samples)/samples;
            append_point(boundary,center+Vec2d(std::cos(angle),std::sin(angle))*(*r)[i%samples],spacing);
        }
        candidate.emplace_back(std::move(boundary));
    }
    ThickPolyline spiral;
    double previous_width=spacing;
    Vec2d previous_point;
    for (int i=0;i<=turns*samples;++i) {
        const int k=i%samples;
        const double t=double(i)/(turns*samples),angle=phase+tau*i/samples;
        const double r=outer[k]*(1-t)+inner[k]*t;
        const double slope=dout[k]*(1-t)+din[k]*t;
        const double width=(outer[k]-inner[k])/turns*r/std::hypot(r,slope);
        if (!std::isfinite(width)||width<min_spacing||width>max_spacing) return reject("normal bead spacing exceeds nozzle bounds");
        if (i>0 && std::abs(width-previous_width)>spacing*.15) return reject("unresolved width transition");
        const Vec2d current=center+Vec2d(std::cos(angle),std::sin(angle))*r;
        if (i>0 && std::abs(width-previous_width)>0.20*(current-previous_point).norm())
            return reject("width change per millimetre exceeds limit");
        append_point(spiral,current,width);
        previous_point=current;
        previous_width=width;
    }
    // append_point cannot infer the first endpoint width until the second point.
    // Its first pair must use the actual first width, not the second width.
    const double first=(outer[0]-inner[0])/turns*outer[0]/std::hypot(outer[0],dout[0]);
    spiral.width.front()=scale_(first);
    candidate.emplace_back(std::move(spiral));
    double strip_area=0;
    const Polygons covered=footprint(candidate,strip_area);
    const double area=unscale<double>(unscale<double>(domain.area()));
    if (area<=0) return reject("invalid domain area");
    const ExPolygons gaps=diff_ex(ExPolygons{domain},covered);
    const ExPolygons spills=diff_ex(covered,ExPolygons{domain});
    result.uncovered_fraction=area_mm2(gaps)/area;
    result.outside_fraction=area_mm2(spills)/area;
    for (const auto& gap:gaps) result.largest_gap_mm2=std::max(result.largest_gap_mm2,area_mm2(ExPolygons{gap}));
    for (const auto& spill:spills) result.largest_spill_mm2=std::max(result.largest_spill_mm2,area_mm2(ExPolygons{spill}));
    if (result.largest_gap_mm2>.25 || result.largest_spill_mm2>.10 ||
        !offset_ex(gaps,-float(scale_(spacing*.15))).empty() ||
        !offset_ex(spills,-float(scale_(spacing*.15))).empty())
        return reject("local gap or spill exceeds area/inscribed-width limit");
    result.excess_fraction=strip_area/area-1;
    if (result.uncovered_fraction>.002 || result.outside_fraction>.002 || std::abs(result.excess_fraction)>.03)
        return reject("bead footprint coverage or deposition budget failed");
    result.paths=std::move(candidate);
    return result;
}

void FillMorphingSpiral::_fill_surface_single(const FillParams& params,unsigned int layers,
    const std::pair<float,Point>& direction,ExPolygon expolygon,ThickPolylines& out)
{
    if (params.density<=.9999f || params.dont_adjust || params.flow.bridge()) {
        FillSpiralInset::_fill_surface_single(params,layers,direction,std::move(expolygon),out);return;
    }
    // FillBase contracted by half the line spacing. Recover the bead domain;
    // retain the normal writer's variable-width flow, acceleration and speed caps.
    const ExPolygons domains=offset_ex(expolygon,float(scale_(spacing*.5)));
    if (domains.size()==1) {
        const double nozzle=params.flow.nozzle_diameter();
        const double rounded_correction=params.layer_height*(1.-.25*3.14159265358979323846);
        auto result=morphing_spiral(domains.front(),spacing,std::max(.01,.75*nozzle-rounded_correction),1.5*nozzle-rounded_correction);
        if (!result.paths.empty()) {
            if (params.fill_order!=SurfaceFillOrder::Inward) {
                for (auto& path:result.paths) path.reverse();
                std::reverse(result.paths.begin(),result.paths.end());
            }
            append(out,std::move(result.paths));return;
        }
        BOOST_LOG_TRIVIAL(debug) << "Morphing spiral fallback: " << result.fallback_reason;
    } else BOOST_LOG_TRIVIAL(debug) << "Morphing spiral fallback: offset topology";
    FillSpiralInset::_fill_surface_single(params,layers,direction,std::move(expolygon),out);
}
}

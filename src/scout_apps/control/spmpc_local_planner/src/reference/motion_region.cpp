#include "spmpc_local_planner/reference/motion_region.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace spmpc_local_planner {
namespace {
const double kEps = 1e-9;
struct H { double x; double y; double b; };
bool finite(double v) { return std::isfinite(v); }
double cross(const RegionVertex& a, const RegionVertex& b, const RegionVertex& c) {
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
double area2(const std::vector<RegionVertex>& p) {
  double r=0.0;
  for (std::size_t i=0; i<p.size(); ++i) {
    const RegionVertex& a=p[i]; const RegionVertex& b=p[(i+1)%p.size()];
    r += a.x*b.y-a.y*b.x;
  }
  return r;
}
bool inside(const H& h, const RegionVertex& p) { return h.x*p.x+h.y*p.y <= h.b+1e-8; }
bool intersection(const H& a, const H& b, RegionVertex* p) {
  const double d=a.x*b.y-a.y*b.x;
  if (std::abs(d)<kEps) return false;
  p->x=(a.b*b.y-a.y*b.b)/d; p->y=(a.x*b.b-a.b*b.x)/d;
  return finite(p->x)&&finite(p->y);
}
bool has_area(const std::vector<H>& hs) {
  std::vector<RegionVertex> points;
  for (const H& a: hs) for (const H& b: hs) {
    RegionVertex p;
    if (!intersection(a,b,&p)) continue;
    bool ok=true;
    for (const H& h: hs) if (!inside(h,p)) { ok=false; break; }
    if (ok) points.push_back(p);
  }
  if (points.size()<3) return false;
  double cx=0, cy=0;
  for (const RegionVertex& p:points) { cx+=p.x; cy+=p.y; }
  cx/=points.size(); cy/=points.size();
  std::sort(points.begin(),points.end(),[cx,cy](const RegionVertex& a,const RegionVertex& b) {
    return std::atan2(a.y-cy,a.x-cx)<std::atan2(b.y-cy,b.x-cx);
  });
  return std::abs(area2(points))>kEps;
}
bool fail(std::string* reason, const std::string& message) {
  if (reason) *reason=message;
  return false;
}
std::vector<H> polygon_halfspaces(const MotionRegionCell& c, double inflation) {
  const double orientation=area2(c.vertices)>=0 ? 1.0 : -1.0;
  std::vector<H> result; result.reserve(c.vertices.size());
  for (std::size_t i=0;i<c.vertices.size();++i) {
    const RegionVertex& a=c.vertices[i]; const RegionVertex& b=c.vertices[(i+1)%c.vertices.size()];
    double nx=orientation*(b.y-a.y), ny=orientation*(a.x-b.x);
    const double norm=std::hypot(nx,ny); nx/=norm; ny/=norm;
    result.push_back({nx,ny,nx*a.x+ny*a.y-inflation});
  }
  return result;
}
const MotionRegionCell* find_cell(const MotionRegionConfig& c, double s) {
  for (const MotionRegionCell& cell:c.cells)
    if (s>=cell.s_begin-kProgressTolerance && s<=cell.s_end+kProgressTolerance) return &cell;
  return nullptr;
}
}

bool MotionRegion::validate(const MotionRegionConfig& c, std::string* reason) {
  if (!finite(c.footprint_radius)||!finite(c.margin)||c.footprint_radius<0||c.margin<0)
    return fail(reason,"footprint_radius and margin must be finite and nonnegative");
  if (c.enabled && c.footprint_radius<=0) return fail(reason,"enabled region requires positive footprint_radius");
  if (!c.enabled) { if (reason) reason->clear(); return true; }
  if (c.id.empty()||c.frame_id.empty()) return fail(reason,"enabled region requires id and frame_id");
  if (c.cells.empty()) return fail(reason,"enabled region requires cells");
  std::unordered_set<std::string> ids; double previous_begin=0,previous_end=0;
  for (std::size_t i=0;i<c.cells.size();++i) {
    const MotionRegionCell& cell=c.cells[i];
    if (cell.id.empty()||!ids.insert(cell.id).second) return fail(reason,"cell ids must be nonempty and unique");
    if (!finite(cell.s_begin)||!finite(cell.s_end)||cell.s_end<=cell.s_begin) return fail(reason,"cell progress interval is invalid");
    if (i && (cell.s_begin+1e-8<previous_begin||cell.s_end+1e-8<previous_end)) return fail(reason,"cells are not ordered by progress");
    if (i && cell.s_begin>previous_end+1e-8) return fail(reason,"cell progress domain has a gap");
    previous_begin=cell.s_begin; previous_end=cell.s_end;
    if (cell.vertices.size()<3||cell.vertices.size()>kMaxRegionFaces) return fail(reason,"cell must have 3..8 vertices");
    for (const RegionVertex& p:cell.vertices) if (!finite(p.x)||!finite(p.y)) return fail(reason,"vertex is nonfinite");
    for (std::size_t j=0;j<cell.vertices.size();++j) for (std::size_t k=j+1;k<cell.vertices.size();++k)
      if (std::hypot(cell.vertices[j].x-cell.vertices[k].x,cell.vertices[j].y-cell.vertices[k].y)<kEps) return fail(reason,"duplicate vertex");
    const double signed_area=area2(cell.vertices);
    if (std::abs(signed_area)<=kEps) return fail(reason,"cell area is degenerate");
    const double orientation=signed_area>0 ? 1.0 : -1.0;
    for (std::size_t j=0;j<cell.vertices.size();++j) {
      const RegionVertex& a=cell.vertices[j]; const RegionVertex& b=cell.vertices[(j+1)%cell.vertices.size()];
      for (const RegionVertex& p:cell.vertices) if (orientation*cross(a,b,p)<-kEps) return fail(reason,"cell is not a convex simple polygon");
    }
    const std::vector<H> shrunken=polygon_halfspaces(cell,c.footprint_radius+c.margin);
    if (!has_area(shrunken)) return fail(reason,"inflated footprint leaves no interior in cell");
    if (i) {
      std::vector<H> overlap=polygon_halfspaces(c.cells[i-1],c.footprint_radius+c.margin);
      overlap.insert(overlap.end(),shrunken.begin(),shrunken.end());
      if (!has_area(overlap)) return fail(reason,"adjacent cells have no traversable spatial overlap");
    }
  }
  if (reason) reason->clear();
  return true;
}

MotionRegion::MotionRegion(MotionRegionConfig config):config_(std::move(config)) {
  std::string reason; if (!validate(config_,&reason)) throw std::invalid_argument(reason);
}
RegionStageData MotionRegion::stage(double route_s,double sweep_distance) const {
  if (!finite(route_s)||!finite(sweep_distance)||sweep_distance<0) throw std::invalid_argument("stage arguments must be finite; sweep_distance nonnegative");
  RegionStageData out; out.enabled=config_.enabled;
  for (RegionHalfspace& f:out.faces) f=RegionHalfspace{};
  if (!config_.enabled) return out;
  const MotionRegionCell* cell=find_cell(config_,route_s);
  if (!cell) throw std::out_of_range("route_s is outside motion-region cells");
  return stageForCell(static_cast<std::size_t>(cell-config_.cells.data()),sweep_distance);
}
RegionStageData MotionRegion::stageForCell(std::size_t index,double sweep_distance) const {
  if (!finite(sweep_distance)||sweep_distance<0||index>=config_.cells.size())
    throw std::invalid_argument("invalid region cell/sweep");
  RegionStageData out; out.enabled=config_.enabled;
  if (!config_.enabled) return out;
  const auto* cell=&config_.cells[index];
  const std::vector<H> hs=polygon_halfspaces(*cell,config_.footprint_radius+config_.margin+sweep_distance);
  if (!has_area(hs)) throw std::out_of_range("sweep leaves no interior in motion-region cell");
  out.cell_id=cell->id; out.s_begin=cell->s_begin; out.s_end=cell->s_end;
  for (std::size_t i=0;i<hs.size();++i) out.faces[i]={hs[i].x,hs[i].y,hs[i].b};
  return out;
}
double MotionRegion::clearance(double x,double y,double route_s) const {
  if (!finite(x)||!finite(y)||!finite(route_s)) throw std::invalid_argument("clearance arguments must be finite");
  if (!config_.enabled) return std::numeric_limits<double>::infinity();
  const MotionRegionCell* cell=find_cell(config_,route_s);
  if (!cell) throw std::out_of_range("route_s is outside motion-region cells");
  const std::vector<H> hs=polygon_halfspaces(*cell,config_.footprint_radius+config_.margin);
  double result=std::numeric_limits<double>::infinity();
  for (const H& h:hs) result=std::min(result,h.b-h.x*x-h.y*y);
  return result;
}
}

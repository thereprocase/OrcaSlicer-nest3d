"""Offline morphing-spiral experiment. Millimetres; never edits printer G-code."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from matplotlib.path import Path as PolygonPath

class UnsupportedGeometry(ValueError):
    pass


def ring_radii(poly, center, angles):
    p=np.asarray(poly,dtype=float)-center
    if p.ndim != 2 or p.shape[1] != 2 or len(p)<3 or not np.isfinite(p).all():
        raise UnsupportedGeometry('invalid polygon coordinates')
    if np.linalg.norm(p[0]-p[-1])<1e-9: p=p[:-1]
    e=np.roll(p,-1,axis=0)-p
    u=np.column_stack((np.cos(angles),np.sin(angles)))
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    den=cross(u[:,None,:],e[None,:,:])
    valid=np.abs(den)>1e-12
    t=np.divide(cross(p,e)[None,:],den,out=np.full_like(den,np.nan),where=valid)
    q=np.divide(cross(p[None,:,:],u[:,None,:]),den,out=np.full_like(den,np.nan),where=valid)
    # Half-open segment convention avoids double-counting a polygon vertex.
    hit=valid & (t>0) & (q>=-1e-10) & (q<1-1e-10)
    if not np.all(hit.sum(axis=1)==1):
        raise UnsupportedGeometry('boundary is not single-valued around the chosen centre')
    return np.sum(np.where(hit,t,0),axis=1)


def demo(lobes=8):
    a=np.linspace(0,2*np.pi,720,endpoint=False)
    def ring(r):return np.column_stack((r*np.cos(a),r*np.sin(a))).tolist()
    return {'outer':ring(40+3*np.cos(lobes*a) if lobes else np.full_like(a,40)),
            'holes':[ring(np.full_like(a,14))], 'center':[0,0]}


def generate(region, target=.6, minimum=.45, maximum=.9, samples_per_turn=720):
    if not 0<minimum<=target<=maximum or not all(map(math.isfinite,[target,minimum,maximum])):
        raise ValueError('require 0 < minimum <= target <= maximum')
    holes=region.get('holes',[])
    if len(holes)!=1:
        raise UnsupportedGeometry('initial prototype supports one-hole annuli; disk centre and multiple holes use stock fill')
    if samples_per_turn<180 or samples_per_turn>2880:
        raise ValueError('samples_per_turn must be between 180 and 2880')
    center=np.asarray(region.get('center',np.mean(holes[0],axis=0)),dtype=float)
    a=np.linspace(0,2*np.pi,samples_per_turn,endpoint=False)+.000123
    outer=ring_radii(region['outer'],center,a)
    inner=ring_radii(holes[0],center,a)
    thickness=outer-inner
    if np.min(thickness)<3*target:raise UnsupportedGeometry('annulus too narrow for separate boundary paths')
    turns=max(1,round(float(np.mean(thickness))/target)-1)
    if turns>300:raise UnsupportedGeometry('turn budget exceeded')
    theta=np.linspace(a[0],a[0]+2*np.pi*turns,turns*samples_per_turn+1)
    # Radial boundary offsets account for the local normal, rather than treating
    # radial thickness as normal thickness on a lobed contour.
    da=2*np.pi/samples_per_turn
    normal=lambda r:r/np.sqrt(r*r+((np.roll(r,-1)-np.roll(r,1))/(2*da))**2)
    outer_center=outer-target/(2*normal(outer))
    inner_center=inner+target/(2*normal(inner))
    fraction=(theta-theta[0])/(2*np.pi*turns)
    interp=lambda values:np.interp((theta-a[0])%(2*np.pi),np.arange(samples_per_turn+1)*da,np.r_[values,values[0]])
    ro,ri=interp(outer_center),interp(inner_center)
    r=ro*(1-fraction)+ri*fraction
    xy=center+np.column_stack((r*np.cos(theta),r*np.sin(theta)))
    # Width follows perpendicular spacing of neighbouring turns. dr/dtheta of
    # the interpolated contour excludes the spiral's slow inward advance.
    derivatives=lambda values:(np.roll(values,-1)-np.roll(values,1))/(2*da)
    slope=interp(derivatives(outer_center))*(1-fraction)+interp(derivatives(inner_center))*fraction
    widths=(ro-ri)/turns*r/np.sqrt(r*r+slope*slope)
    if not np.isfinite(widths).all() or widths.min()<minimum or widths.max()>maximum:
        raise UnsupportedGeometry(f'normal widths {widths.min():.4f}..{widths.max():.4f} outside {minimum}..{maximum} mm')
    paths=[]
    for radius in [outer_center,inner_center]:
        points=center+np.column_stack((radius*np.cos(a),radius*np.sin(a)))
        points=np.vstack((points,points[0]))
        paths.append({'role':'boundary','xy':points,'width':np.full(len(points),target)})
    paths.append({'role':'morphing_spiral','xy':xy,'width':widths})
    return {'paths':paths,'turns':turns,'width_min':float(widths.min()),'width_max':float(widths.max()),
            'boundary_paths':2,'spiral_paths':1,'center':center.tolist()}


def coverage(region,result,step=.12):
    """Sampled bead-union audit, not an exact geometric or physical guarantee.

    Nearest segment is searched in all segments whose midpoint can possibly
    cover the sample: global half-width plus half longest segment. No nearest-k
    shortcut can silently miss a wider bead. Outside spill is measured too.
    """
    paths=result['paths'];starts=np.concatenate([p['xy'][:-1] for p in paths]);ends=np.concatenate([p['xy'][1:] for p in paths])
    w0=np.concatenate([p['width'][:-1] for p in paths]);w1=np.concatenate([p['width'][1:] for p in paths])
    vec=ends-starts;len2=np.sum(vec*vec,axis=1)
    if np.any(len2<1e-16):raise UnsupportedGeometry('zero-length path segment')
    mids=(starts+ends)/2;tree=cKDTree(mids)
    search=float(np.max(np.maximum(w0,w1))/2+np.sqrt(len2).max()/2)
    outer=np.asarray(region['outer']);margin=max(w0.max(),w1.max())
    lo=outer.min(axis=0)-margin;hi=outer.max(axis=0)+margin
    axes=[np.arange(lo[i]+step/2,hi[i],step) for i in range(2)]
    if len(axes[0])*len(axes[1])>3000000:raise UnsupportedGeometry('coverage grid budget exceeded')
    xx,yy=np.meshgrid(*axes);points=np.column_stack((xx.ravel(),yy.ravel()))
    inside=PolygonPath(outer).contains_points(points)
    for hole in region.get('holes',[]):inside &= ~PolygonPath(hole).contains_points(points)
    covered=np.zeros(len(points),dtype=bool)
    for offset in range(0,len(points),20000):
        batch=points[offset:offset+20000];lists=tree.query_ball_point(batch,search)
        lengths=np.array([len(v) for v in lists]);rows=np.repeat(np.arange(len(batch)),lengths)
        if not len(rows):continue
        ids=np.concatenate([v for v in lists if len(v)]).astype(int)
        q=batch[rows];t=np.clip(np.sum((q-starts[ids])*vec[ids],axis=1)/len2[ids],0,1)
        distances=np.linalg.norm(q-starts[ids]-t[:,None]*vec[ids],axis=1)
        width=w0[ids]*(1-t)+w1[ids]*t
        covered[offset+np.unique(rows[distances<=width/2])]=True
    area=float(inside.sum()*step*step)
    uncovered=inside & ~covered;spill=~inside & covered
    # Integrated strip area reveals excess deposition hidden by union coverage.
    nominal=float(np.sum(np.sqrt(len2)*(w0+w1)/2))
    report={'grid_mm':step,'region_area_mm2_sampled':area,'uncovered_fraction':float(uncovered.sum()/max(1,inside.sum())),
            'outside_fraction':float(spill.sum()/max(1,inside.sum())),
            'nominal_strip_area_mm2':nominal,'nominal_area_excess_fraction':nominal/area-1,
            'scope':'sampled circular bead footprint; not exact union, extrusion-volume model or print qualification'}
    return report,points[uncovered],points[spill]


def write_preview(region,result,audit,missing,spill,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig,ax=plt.subplots(figsize=(10,10));fig.patch.set_facecolor('#f3efe6');ax.set_facecolor('#f3efe6')
    for p in result['paths']:
        lines=np.stack((p['xy'][:-1],p['xy'][1:]),axis=1)
        lc=LineCollection(lines,cmap='viridis',linewidths=.45,clim=(.45,.9));lc.set_array((p['width'][:-1]+p['width'][1:])/2);ax.add_collection(lc)
    for poly in [region['outer']]+region.get('holes',[]):
        p=np.asarray(poly);ax.plot(*np.vstack((p,p[0])).T,color='#243c43',linewidth=.5)
    if len(missing):ax.scatter(*missing.T,s=.4,color='#d84722',label='Uncovered samples')
    if len(spill):ax.scatter(*spill.T,s=.4,color='#e180d5',label='Outside samples')
    ax.autoscale();ax.set_aspect('equal');ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)')
    ax.set_title('Morphing spiral — offline annulus prototype\n'+f"{result['turns']} turns · {result['width_min']:.3f}–{result['width_max']:.3f} mm normal width")
    fig.colorbar(lc,ax=ax,label='Normal bead width (mm)',shrink=.7)
    fig.text(.08,.02,f"Sampled gaps {audit['uncovered_fraction']:.2%}; spill {audit['outside_fraction']:.2%}; strip area excess {audit['nominal_area_excess_fraction']:.2%}\nTwo separate boundary paths. Experimental geometry; no G-code output.",fontsize=9)
    fig.savefig(path,dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path);parser.add_argument('--demo',choices=['lobed','circle'],default='lobed');parser.add_argument('--output',type=Path,required=True);parser.add_argument('--target-width',type=float,default=.6);parser.add_argument('--min-width',type=float,default=.45);parser.add_argument('--max-width',type=float,default=.9);parser.add_argument('--grid',type=float,default=.12)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    region=json.loads(args.input.read_text()) if args.input else demo(8 if args.demo=='lobed' else 0)
    try:
        if not math.isfinite(args.grid) or args.grid<=0:raise ValueError('grid must be positive and finite')
        result=generate(region,args.target_width,args.min_width,args.max_width)
        audit,missing,spill=coverage(region,result,args.grid)
        # Experimental acceptance, deliberately explicit. Do not hide a failed
        # coverage test by silently emitting the candidate as a stock fallback.
        accepted=audit['uncovered_fraction']<=.002 and audit['outside_fraction']<=.002 and abs(audit['nominal_area_excess_fraction'])<=.025
        report={k:v for k,v in result.items() if k!='paths'};report.update(accepted=accepted,action='candidate_only' if accepted else 'stock_fallback',coverage=audit)
        write_preview(region,result,audit,missing,spill,args.output/'preview.png')
        data=[{'role':p['role'],'xy_mm':p['xy'].tolist(),'width_mm':p['width'].tolist()} for p in result['paths']]
        (args.output/'candidate-paths.json').write_text(json.dumps(data))
    except UnsupportedGeometry as error:
        report={'accepted':False,'action':'stock_fallback','reason':str(error)}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');(args.output/'input.json').write_text(json.dumps(region));print(json.dumps(report,indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())

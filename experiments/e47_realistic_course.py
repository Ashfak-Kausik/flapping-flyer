"""
e47 — realistic containment course GENERATOR (geometry only, this file).

Builds a rectilinear (Manhattan) passage that TURNS 90 degrees between legs and whose WIDTH
varies leg-to-leg, with clean corner-trimmed walls and BREACHES (wall openings) punched into
chosen segments — including on the hard parts (near turns, in narrow sections). This replaces
the single straight tube so the mission can be shown on a passage that looks like a damaged
containment, not a demo lane.

A leg = (direction in {E,N,W,S}, length m, width m). Consecutive legs are a 90-deg turn.
Left/right walls are each a continuous polyline; each leg's wall line is the centreline offset
by w/2, and corner vertices are the intersections of adjacent legs' wall lines (so inner corners
tuck in and outer corners open out correctly — the turn stays passable).
A breach = (side +1 left/-1 right, leg index, frac_start, frac_end) -> a gap in that wall segment.

This module exposes geometry + model build + a layout renderer + a numeric self-check. Flight/
mission logic lives in the mission runner (next file); kept separate so the geometry is trusted
before anything flies on it.
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e36_reactive_course as e
import experiments.e39_branching as b39

DIRS={'E':(1.0,0.0),'N':(0.0,1.0),'W':(-1.0,0.0),'S':(0.0,-1.0)}
def _left(d): return np.array([-d[1], d[0]])          # rotate +90 (left of travel)

def centerline(legs):
    P=[np.array([0.0,0.0])]; dirs=[]; W=[]
    for (dc,L,w) in legs:
        d=np.array(DIRS[dc]); P.append(P[-1]+d*L); dirs.append(d); W.append(w)
    return P, dirs, W

def _isect(pt1,d1,pt2,d2):
    """intersection of two axis-aligned perpendicular lines: line k passes pt_k along dir d_k."""
    if abs(d1[0])>abs(d1[1]):                         # line1 horizontal (fixed y=pt1[1]), line2 vertical (fixed x=pt2[0])
        return np.array([pt2[0], pt1[1]])
    return np.array([pt1[0], pt2[1]])                 # line1 vertical, line2 horizontal

def wall_vertices(P, dirs, W, side):
    """continuous wall polyline (list of vertices) for side=+1 (left) / -1 (right)."""
    n=len(dirs); off=[_left(dirs[i])*side*W[i]/2 for i in range(n)]
    verts=[P[0]+off[0]]
    for i in range(1,n):
        verts.append(_isect(P[i]+off[i-1], dirs[i-1], P[i]+off[i], dirs[i]))
    verts.append(P[n]+off[n-1])
    return verts

def _seg_blocks_with_gaps(a, b, gaps):
    """box-chain from a->b (2D), skipping [f0,f1] fractional spans in `gaps`."""
    a=np.array(a,float); b=np.array(b,float); out=[]; segs=[]
    cur=0.0
    for (f0,f1) in sorted(gaps):
        if f0>cur+1e-9: segs.append((cur,f0))
        cur=f1
    if cur<1.0-1e-9: segs.append((cur,1.0))
    if not gaps: segs=[(0.0,1.0)]
    for (f0,f1) in segs:
        out+=b39.blocks([a+(b-a)*f0, a+(b-a)*f1])
    return out

def build_geometry(legs, breaches):
    P,dirs,W=centerline(legs)
    Lv=wall_vertices(P,dirs,W,+1); Rv=wall_vertices(P,dirs,W,-1)
    n=len(dirs); bx=[]
    for i in range(n):
        gL=[(f0,f1) for (s,li,f0,f1) in breaches if s==+1 and li==i]
        gR=[(f0,f1) for (s,li,f0,f1) in breaches if s==-1 and li==i]
        bx+=_seg_blocks_with_gaps(Lv[i],Lv[i+1],gL)
        bx+=_seg_blocks_with_gaps(Rv[i],Rv[i+1],gR)
    return dict(P=P,dirs=dirs,W=W,Lv=Lv,Rv=Rv,blocks=bx,finish=P[-1])

def build_model(geo, path="models/_realistic.xml"):
    xml=open("models/flyer.xml").read(); fin=geo['finish']
    g="".join(f'    <geom name="bw{i}" type="box" pos="{cx:.4f} {cy:.4f} {e.WALL_H/2:.4f}" '
              f'size="{hs:.4f} {hs:.4f} {e.WALL_H/2:.4f}" group="3" rgba="0.62 0.66 0.72 1" '
              f'contype="0" conaffinity="0"/>\n' for i,(cx,cy,hs) in enumerate(geo['blocks']))
    g+=(f'    <geom name="fin0" type="cylinder" pos="{fin[0]:.4f} {fin[1]:.4f} 0.001" '
        f'size="{e.FIN_PAD:.4f} 0.001" rgba="0.2 0.85 0.35 0.85" group="2" contype="0" conaffinity="0"/>\n')
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path

def selfcheck(geo, legs):
    """verify each leg's two walls are exactly width apart (perpendicular) and corners connect."""
    P,dirs,W,Lv,Rv=geo['P'],geo['dirs'],geo['W'],geo['Lv'],geo['Rv']; ok=True
    print(" leg | dir | len(mm) | width(mm) | perp separation(mm)")
    for i,(dc,L,w) in enumerate(legs):
        nrm=_left(dirs[i]); sep=abs(float(np.dot(Lv[i]-Rv[i], nrm)))   # perpendicular distance
        flag="" if abs(sep-w)<1e-3 else "  <-- MISMATCH"; ok=ok and not flag
        cont = np.allclose(Lv[i+1] if i+1<len(Lv) else Lv[i], Lv[i+1]) # trivially true; corner continuity below
        print(f"  {i}  |  {dc}  | {L*1e3:6.0f}  |  {w*1e3:5.0f}   |  {sep*1e3:6.1f}{flag}")
    # corner continuity: each wall polyline is a single connected chain by construction (shared vertices)
    print(f"  finish at {np.round(geo['finish']*1e3).astype(int)} mm ; {len(geo['blocks'])} wall blocks ; corridor {'OK' if ok else 'CHECK'}")
    return ok

def layout_figure(geo, breaches, path="outputs/e47_layout.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,6))
    for (cx,cy,hs) in geo['blocks']: ax.add_patch(plt.Rectangle((cx-hs,cy-hs),2*hs,2*hs,color="#9aa4b0",lw=0))
    P=np.array(geo['P']); ax.plot(P[:,0],P[:,1],'--',color="#94a3b8",lw=1,label="centerline")
    for (s,li,f0,f1) in breaches:                     # ground-truth breach spans
        V=geo['Lv'] if s==+1 else geo['Rv']; a,b=V[li],V[li+1]
        p0=a+(b-a)*f0; p1=a+(b-a)*f1; ax.plot([p0[0],p1[0]],[p0[1],p1[1]],color="#dc2626",lw=6,alpha=0.55)
    fin=geo['finish']; ax.add_patch(plt.Circle((fin[0],fin[1]),e.FIN_PAD,color="#2fbf52",alpha=0.6))
    ax.plot([],[],color="#dc2626",lw=6,alpha=0.55,label="breach (truth)")
    ax.set_aspect('equal'); ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set_title("e47 realistic containment course (turns + variable width + breaches)")
    fig.savefig(path,dpi=120,bbox_inches='tight'); print("saved ->",path)

# realistic instance: 4 legs, two turns, width breathes 64->58->56->60mm (noise-robust, >48mm floor),
# breaches on the hard parts. (narrow-corridor floor itself is characterized in e46.)
# final mission instance: 3 legs, two turns (left E->N, right N->E), width breathes 72->66->70mm
# (>=64mm noise-robust). breaches spaced MID-leg, away from the post-turn settling window.
LEGS=[('E',0.24,0.072), ('N',0.20,0.066), ('E',0.22,0.070)]
BREACHES=[(+1,0,0.45,0.62),      # left wall, leg0 (small)
          (-1,1,0.55,0.80),      # right wall, leg1 (late, after post-turn settle)
          (+1,2,0.45,0.72)]      # left wall, leg2 (mid)

if __name__=="__main__":
    geo=build_geometry(LEGS,BREACHES); build_model(geo)
    selfcheck(geo,LEGS); layout_figure(geo,BREACHES)
"""
e45 — MISSION: autonomous containment-breach inspection.

Scenario (nuclear-containment framing): a low-downwash flapping MAV flies a confined passage
it cannot see inside, and must DETECT, LOCALIZE, and SIZE structural breaches (wall openings)
in the walls — while STAYING ON ITS INSPECTION ROUTE. A through-breach defeats containment, so
finding openings is the mission; being pulled OUT through a large one (the open-seeking lurch,
e40) is mission failure. The antenna vetoes the wing-wash so the flyer flies PAST each breach,
logging it, instead of being sucked through.

Everything is done with the flyer's own aerodynamics + antenna feelers: no vision, no GPS, no
external localization, no learning. Reactive.

Detection rule: a breach = a 90-deg side feeler reading OPEN (>GAP_THRESH) while the forward
path is CLEAR (fwd>CLEAR: distinguishes a breach-to-pass from a junction-to-follow). Onset =
wall->open crossing, offset = open->wall. Per breach we log side, center-x, length.

Deliverable: ONE combined map — the flown trajectory (traversed geometry) + the breach overlay
(detected openings on the walls) vs ground truth — plus a metrics table (detection rate,
localization error, size error) and a with/without-veto comparison (the veto is mission-critical).

Run:  python experiments/e45_breach_inspection.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e36_reactive_course as e
import experiments.e39_branching as b39
import experiments.e40_side_gap as e40
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna
from src.safety import Clearance, WINGREACH
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe

hw=e.Wd/2.0; Wd=e.Wd
CRUISE=e.CRUISE; FIN_ZONE=e.FIN_ZONE; WALL_H=e.WALL_H
GAP_THRESH=e40.GAP_THRESH; KFOLLOW=e40.KFOLLOW; STANDOFF=e40.STANDOFF
SAFE_BUF=e.SAFE_BUF; KVEER=e.KVEER; CONTROL_RATE=1000

# ground-truth breaches: (side 'L'=+y wall / 'R'=-y wall, x_start, x_end) in metres; sizes vary
LTOT=0.80
BREACHES=[('L',0.100,0.148),      # 0.75 x Wd  (48mm)  small
          ('R',0.260,0.356),      # 1.5  x Wd  (96mm)  medium
          ('L',0.500,0.660)]      # 2.5  x Wd (160mm)  large (would pull flyer out w/o veto)

def wall_segments(side):
    yy = hw if side=='L' else -hw
    br=sorted([(x0,x1) for (s,x0,x1) in BREACHES if s==side]); segs=[]; cur=0.0
    for x0,x1 in br:
        if x0>cur+1e-6: segs.append([np.array([cur,yy]),np.array([x0,yy])])
        cur=x1
    if cur<LTOT-1e-6: segs.append([np.array([cur,yy]),np.array([LTOT,yy])])
    return segs

def geometry(): return wall_segments('L')+wall_segments('R'), np.array([LTOT,0.0])

def build_model(path="models/_breach_course.xml"):
    xml=open("models/flyer.xml").read(); walls,fin=geometry(); bx=[]
    for w in walls: bx+=b39.blocks(w)
    g="".join(f'    <geom name="bw{i}" type="box" pos="{cx:.4f} {cy:.4f} {WALL_H/2:.4f}" '
              f'size="{hs:.4f} {hs:.4f} {WALL_H/2:.4f}" group="3" rgba="0.62 0.66 0.72 1" '
              f'contype="0" conaffinity="0"/>\n' for i,(cx,cy,hs) in enumerate(bx))
    g+=(f'    <geom name="fin0" type="cylinder" pos="{fin[0]:.4f} {fin[1]:.4f} 0.001" '
        f'size="{e.FIN_PAD:.4f} 0.001" rgba="0.2 0.85 0.35 0.85" group="2" contype="0" conaffinity="0"/>\n')
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path

def run(level=0.0, seed=0, rate=CONTROL_RATE, fuse=True, tmax=None):
    walls,fin=geometry(); nm=NoiseModel(level,seed); fly=Flyer(build_model())
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,
                         Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/rate)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly,n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=CRUISE); ctrl.h_ref=CRUISE
    N=max(1,round((1.0/rate)/fly.dt)); dt_c=N*fly.dt
    t=0.0; stepi=0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; Iy=0.0
    minc=1e3; crashed=False; reached=None; tmax=(LTOT/e.Vc*2.2+14 if tmax is None else tmax)
    floor=dict(axis=2,sign=1,pos=0.0); pl=[]; traj=[]
    # breach logger state
    openL=openR=False; onL=onR=0.0; detected=[]   # detected: (side, x0, x1)
    while t<tmax:
        if stepi%N==0:
            s=nm.sense(fly.sense()); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
            b=bodyframe(s); v_lat=b['vy']
            nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*dt_c/0.04; nrate=(nose_f-nose_prev)/dt_c; nose_prev=nose_f
            f0,fLp,fRp,f50p,f50m,dL,dR=nm.feel(ant.feel([0,30,-30,50,-50,90,-90])); fwd,fL,fR=f0,fLp,fRp
            rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*dt_c/0.10; pl=e.planes(psi,fly.x_com,dL,dR)
            gapL=dL>GAP_THRESH; gapR=dR>GAP_THRESH; fclear=fwd>e.CLEAR
            # ---- BREACH LOGGER: side open while forward clear = breach; log onset/offset x ----
            if fclear:
                if gapL and not openL: openL=True; onL=x
                if (not gapL) and openL: openL=False; detected.append(('L',onL,x))
                if gapR and not openR: openR=True; onR=x
                if (not gapR) and openR: openR=False; detected.append(('R',onR,x))
            # ---- NAV: base centering + antenna veto (hold lane past a breach) ----
            ln=min(f50p,dL); rn=min(f50m,dR); safe=0.0
            if rn<SAFE_BUF: safe+=KVEER*(SAFE_BUF-rn)/SAFE_BUF
            if ln<SAFE_BUF: safe-=KVEER*(SAFE_BUF-ln)/SAFE_BUF
            slow=0.35 if min(ln,rn)<SAFE_BUF else 1.0
            Vcmd=e.Vc*np.clip((fwd-e.STOP)/0.04,0.0,1.0)*slow
            steer=np.clip(e.Ksteer*(min(fL,e.FMAX)-min(fR,e.FMAX)),-0.5,0.5)
            if fuse and fclear and (gapL!=gapR):
                if gapL: roll_ref=np.clip(-KFOLLOW*(dR-STANDOFF)-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                else:    roll_ref=np.clip(+KFOLLOW*(dL-STANDOFF)-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                pref+=(-pref/0.3+safe)*dt_c
            else:
                roll_ref=np.clip(-e.Kc*rd_f-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                pref+=(steer+safe)*dt_c
            er=Vcmd-b['vx']; I_s=np.clip(I_s+er*dt_c,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
            eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*dt_c,-1.0,1.0)
            uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
            u=ctrl.update(b,dt_c,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1]-e.ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[floor])); x,y,z=fly.x_com
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            c=clr.min_clearance(); minc=min(minc,c)
            if c<WINGREACH: crashed=True
            traj.append((x,y))
        if np.hypot(fin[0]-x,fin[1]-y)<FIN_ZONE: reached='finish'; break
        if y>hw+0.030 or y<-hw-0.030: reached='exited'; break     # pulled out through a breach
        if z<0.0 or z>0.2: break
        t+=fly.dt; stepi+=1
    # close any breach still open at end
    if openL: detected.append(('L',onL,x))
    if openR: detected.append(('R',onR,x))
    return dict(reached=reached, crashed=crashed, min_clear_mm=minc*1e3, t=t,
                traj=traj, detected=detected, end=(float(x),float(y)))

def score(detected):
    """Match detected breaches to ground truth; report localization + size errors."""
    print(" breach (truth)         | detected center | loc err | size(true/det) mm")
    rows=[]
    for (s,x0,x1) in BREACHES:
        cen=(x0+x1)/2; ln=(x1-x0)
        cand=[d for d in detected if d[0]==s and abs((d[1]+d[2])/2-cen)<0.06]
        if cand:
            d=min(cand,key=lambda d:abs((d[1]+d[2])/2-cen)); dc=(d[1]+d[2])/2; dl=d[2]-d[1]
            print(f"  {s} {x0*1e3:.0f}-{x1*1e3:.0f}mm ({ln*1e3:.0f}mm) | {dc*1e3:6.0f}mm       | {abs(dc-cen)*1e3:5.1f}mm | {ln*1e3:.0f} / {dl*1e3:.0f}")
            rows.append((True,abs(dc-cen)*1e3,ln*1e3,dl*1e3))
        else:
            print(f"  {s} {x0*1e3:.0f}-{x1*1e3:.0f}mm ({ln*1e3:.0f}mm) | MISSED          |   -     |  -")
            rows.append((False,None,ln*1e3,None))
    det=sum(1 for r in rows if r[0]); print(f"  => detected {det}/{len(BREACHES)}")
    return rows

def inspection_map(res, path="outputs/e45_breach_map.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    walls,fin=geometry(); fig,ax=plt.subplots(figsize=(11,3.4))
    for w in walls:
        for (cx,cy,hs) in b39.blocks(w): ax.add_patch(plt.Rectangle((cx-hs,cy-hs),2*hs,2*hs,color="#9aa4b0",lw=0))
    for (s,x0,x1) in BREACHES:              # ground-truth breaches (red spans on wall line)
        yy=hw if s=='L' else -hw; ax.plot([x0,x1],[yy,yy],color="#dc2626",lw=6,alpha=0.5,solid_capstyle='butt')
    for (s,x0,x1) in res['detected']:       # detected breaches (blue markers)
        yy=(hw if s=='L' else -hw)*1.15; ax.plot([(x0+x1)/2],[yy],'v',color="#2563eb",ms=9)
        ax.plot([x0,x1],[yy,yy],color="#2563eb",lw=2)
    tj=np.array(res['traj']); ax.plot(tj[:,0],tj[:,1],'k-',lw=1.2,label="flight")
    ax.add_patch(plt.Circle((fin[0],fin[1]),e.FIN_PAD,color="#2fbf52",alpha=0.6))
    ax.plot([],[],color="#dc2626",lw=6,alpha=0.5,label="breach (truth)")
    ax.plot([],[],'v',color="#2563eb",label="breach (detected)")
    ax.set_xlim(-0.02,LTOT+0.03); ax.set_ylim(-hw-0.05,hw+0.05); ax.set_aspect('equal')
    ax.set_title("e45 containment inspection — flight path + breach map"); ax.legend(fontsize=8,loc="lower right"); ax.grid(alpha=0.2)
    fig.savefig(path,dpi=120,bbox_inches='tight'); print("saved ->",path)

if __name__=="__main__":
    print(f"containment passage {LTOT*1e3:.0f}mm, {len(BREACHES)} breaches; corridor {Wd*1e3:.0f}mm\n")
    print("##### WITH antenna-veto (mission mode) #####")
    r=run(fuse=True)
    print(f"outcome: {r['reached']}  minClear {r['min_clear_mm']:.1f}mm  {'CRASH' if r['crashed'] else 'ok'}  t={r['t']:.1f}s")
    score(r['detected']); inspection_map(r)
    print("\n##### WITHOUT veto (baseline: pulled out through a breach = inspection fails) #####")
    r0=run(fuse=False)
    print(f"outcome: {r0['reached']}  (exited={r0['reached']=='exited'})  detected {len(r0['detected'])} before failure")
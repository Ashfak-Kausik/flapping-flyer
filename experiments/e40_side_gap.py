"""
e40 — SIDE-GAP (phase 2, branching Case B): the asymmetric "one wall drops away" test.
A straight corridor whose LEFT wall has a gap of length g while the RIGHT wall continues
unbroken and the corridor runs straight to a single finish. This is the case where the
wing-wash can actively MISLEAD: with the left wall gone, the right wall's push is
unopposed, so the centering (roll_dist) goes one-sided and commands roll toward the void,
*and* the left feelers read open through the gap -> both effects pull the flyer LEFT, into
the opening. Question: does it hold its line past the gap, or get sucked out?

We sweep the gap length g (in multiples of corridor width Wd) to find the critical opening
beyond which the open-seeking lurch carries the flyer out of the corridor.

Run:  python experiments/e40_side_gap.py      (layout+trajectory pngs, gap-length sweep)
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e36_reactive_course as e
import experiments.e39_branching as b39          # reuse blocks() + model-build pattern
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna
from src.safety import Clearance, WINGREACH
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe

S=e.SCALE; hw=e.Wd/2.0
XG=0.10*S                                          # gap starts here (after a centered approach)
LPOST=0.14*S                                       # corridor continues past the gap to the finish
WALL_H=e.WALL_H; CRUISE=e.CRUISE; FIN_ZONE=e.FIN_ZONE
SAFE_BUF=e.SAFE_BUF; KVEER=e.KVEER; CONTROL_RATE=1000
GAP_THRESH=0.10; KFOLLOW=2.0; STANDOFF=0.030       # antenna-veto: side feeler>GAP_THRESH => wall gone; follow present wall to STANDOFF

def geometry(gap):
    """Build wall polylines + finish for a left-wall gap of length `gap` (m)."""
    Ltot=XG+gap+LPOST
    right =[np.array([0.0,-hw]), np.array([Ltot,-hw])]            # continuous right wall
    left1 =[np.array([0.0, hw]), np.array([XG,  hw])]            # left wall before gap
    left2 =[np.array([XG+gap, hw]), np.array([Ltot, hw])]        # left wall resumes after gap
    walls=[right,left1,left2]; fin=np.array([Ltot,0.0])
    return walls, fin, Ltot

def build_model(gap, path="models/_side_gap.xml"):
    xml=open("models/flyer.xml").read(); walls,fin,Ltot=geometry(gap); bx=[]
    for w in walls: bx+=b39.blocks(w)
    g="".join(f'    <geom name="bw{i}" type="box" pos="{cx:.4f} {cy:.4f} {WALL_H/2:.4f}" '
              f'size="{hs:.4f} {hs:.4f} {WALL_H/2:.4f}" group="3" rgba="0.62 0.66 0.72 1" '
              f'contype="0" conaffinity="0"/>\n' for i,(cx,cy,hs) in enumerate(bx))
    g+=(f'    <geom name="fin0" type="cylinder" pos="{fin[0]:.4f} {fin[1]:.4f} 0.001" '
        f'size="{e.FIN_PAD:.4f} 0.001" rgba="0.2 0.85 0.35 0.85" group="2" contype="0" conaffinity="0"/>\n')
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path

def run(gap, level=0.0, seed=0, rate=CONTROL_RATE, rec=True, fuse=False, tmax=None):
    walls,fin,Ltot=geometry(gap)
    nm=NoiseModel(level,seed); fly=Flyer(build_model(gap))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,
                         Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/rate)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly,n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=CRUISE); ctrl.h_ref=CRUISE
    N=max(1,round((1.0/rate)/fly.dt)); dt_c=N*fly.dt
    t=0.0; stepi=0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; state="CRUISE"; tdir=0; turn0=None
    minc=1e3; crashed=False; reached=None; tmax=((Ltot)/e.Vc*2.2+14 if tmax is None else tmax); floor=dict(axis=2,sign=1,pos=0.0); pl=[]; traj=[]
    ymax=-1e9                                                     # peak LEFTWARD (+y) excursion = lurch magnitude
    while t<tmax:
        if stepi%N==0:
            s=nm.sense(fly.sense()); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
            bf=bodyframe(s); v_lat=bf['vy']
            nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*dt_c/0.04; nrate=(nose_f-nose_prev)/dt_c; nose_prev=nose_f
            f0,fLp,fRp,f50p,f50m,dL,dR=nm.feel(ant.feel([0,30,-30,50,-50,90,-90])); fwd,fL,fR=f0,fLp,fRp
            rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*dt_c/0.10; pl=e.planes(psi,fly.x_com,dL,dR)
            left_near=min(f50p,dL); right_near=min(f50m,dR); safe=0.0
            if right_near<SAFE_BUF: safe+=KVEER*(SAFE_BUF-right_near)/SAFE_BUF
            if left_near <SAFE_BUF: safe-=KVEER*(SAFE_BUF-left_near)/SAFE_BUF
            slow=0.35 if min(left_near,right_near)<SAFE_BUF else 1.0
            if state=="CRUISE":                                  # BASE policy, + optional antenna veto (fuse)
                Vcmd=e.Vc*np.clip((fwd-e.STOP)/0.04,0.0,1.0)*slow
                steer=np.clip(e.Ksteer*(min(fL,e.FMAX)-min(fR,e.FMAX)),-0.5,0.5)
                gapL=dL>GAP_THRESH; gapR=dR>GAP_THRESH
                if fuse and fwd>e.CLEAR and (gapL!=gapR):         # ANTENNA VETO: one wall gone but path ahead clear -> side-gap, not a fork
                    if gapL:                                      # left wall gone -> follow RIGHT wall
                        roll_ref=np.clip(-KFOLLOW*(dR-STANDOFF)-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                    else:                                         # right wall gone -> follow LEFT wall
                        roll_ref=np.clip(+KFOLLOW*(dL-STANDOFF)-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                    pref+=(-pref/0.3+safe)*dt_c                   # re-straighten heading down the corridor (don't follow the void)
                else:                                             # both walls present (or a fork): wing-wash centering
                    roll_ref=np.clip(-e.Kc*rd_f-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                    pref+=(steer+safe)*dt_c
                if fwd<e.STOP and min(fL,fR)<e.STOP and spd<0.03: tdir=+1 if fL>=fR else -1; state="TURN"; turn0=nose_f; I_y=0.0
            else:
                Vcmd=0.0; pref+=tdir*e.YAWRATE*dt_c; roll_ref=np.clip(-e.KLAT*v_lat,-np.radians(2),np.radians(2))
                turned=abs(((nose_f-turn0+np.pi)%(2*np.pi)-np.pi))
                if fwd>e.CLEAR and turned>np.radians(20): state="CRUISE"; I_s=0.0; rd_f=0.0
                elif turned>np.radians(175): tdir=-tdir; turn0=nose_f
            er=Vcmd-bf['vx']; I_s=np.clip(I_s+er*dt_c,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
            eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*dt_c,-1.0,1.0)
            uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
            u=ctrl.update(bf,dt_c,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1]-e.ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[floor])); x,y,z=fly.x_com; ymax=max(ymax,y)
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            c=clr.min_clearance(); minc=min(minc,c)
            if c<WINGREACH: crashed=True
            if rec: traj.append((x,y))
        if np.hypot(fin[0]-x,fin[1]-y)<FIN_ZONE: reached='finish'; break
        if y>hw+0.030: reached='exited'; break                   # carried out through the gap (well past wall line)
        if z<0.0 or z>0.2: break
        t+=fly.dt; stepi+=1
    return dict(reached=reached, lurch_mm=(ymax)*1e3, wall_mm=hw*1e3, min_clear_mm=minc*1e3,
                crashed=crashed, t=t, end=(float(x),float(y)), traj=traj, Ltot=Ltot, gap_mm=gap*1e3)

def layout_plot(gap, traj=None, path="outputs/e40_side_gap.png", title=None):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    walls,fin,Ltot=geometry(gap); fig,ax=plt.subplots(figsize=(9,3.6)); allpts=[]
    for w in walls:
        for (cx,cy,hs) in b39.blocks(w): ax.add_patch(plt.Rectangle((cx-hs,cy-hs),2*hs,2*hs,color="#9aa4b0",lw=0)); allpts.append((cx,cy))
    ax.add_patch(plt.Circle((fin[0],fin[1]),e.FIN_PAD,color="#2fbf52",alpha=0.65)); allpts.append(tuple(fin))
    ax.plot([0],[0],'ko',ms=6,label="start")
    if traj: tj=np.array(traj); ax.plot(tj[:,0],tj[:,1],'r-',lw=1.8,label="flight")
    ap=np.array(allpts); ax.set_xlim(-0.02,Ltot+0.03); ax.set_ylim(-hw-0.05,hw+0.06)
    ax.set_aspect('equal'); ax.set_title(title or f"e40 side-gap g={gap*1e3:.0f}mm"); ax.legend(fontsize=8,loc="upper left"); ax.grid(alpha=0.2)
    fig.savefig(path,dpi=120,bbox_inches='tight'); print("saved ->",path)

def sweep(gmults=(0.5,1.0,1.5,2.0,3.0), level=0.0, seed=0, fuse=False):
    tag="ANTENNA-VETO (fused)" if fuse else "base policy"
    print(f"side-gap sweep (SCALE={S}, 1kHz, aero-on, {tag}); gap = {list(gmults)} x Wd({e.Wd*1e3:.0f}mm), level {level}")
    rows=[]
    for gm in gmults:
        r=run(gm*e.Wd, level=level, seed=seed, rec=False, fuse=fuse)
        rows.append((gm, r['gap_mm'], r['reached'], r['lurch_mm'], r['min_clear_mm'], r['crashed']))
        print(f"  gap {gm:>3}xWd ({r['gap_mm']:3.0f}mm): {str(r['reached']):7}  peak +y excursion {r['lurch_mm']:5.1f}mm "
              f"(wall at {r['wall_mm']:.0f}mm)  minClear {r['min_clear_mm']:4.1f}mm  {'CRASH' if r['crashed'] else ''}")
    print(" gap(xWd) | outcome | peak +y (mm) | wall line | minClear")
    for gm,gmm,out,lz,mc,cr in rows:
        print(f"   {gm:>4} | {str(out):7} | {lz:6.1f}      |   {hw*1e3:.0f}mm   | {mc:4.1f}mm {'CRASH' if cr else ''}")
    return rows

if __name__=="__main__":
    print(f"side-gap (Case-B) on SCALE={S}: approach {XG*1e3:.0f}mm, corridor width {e.Wd*1e3:.0f}mm, wall line at +-{hw*1e3:.0f}mm\n")
    print("##### BASE policy (wing-wash centering only) — open-seeking failure #####")
    base=sweep(fuse=False)
    print("\n##### ANTENNA-VETO fusion (feeler overrides wash when a wall vanishes) #####")
    fused=sweep(fuse=True)
    # illustrative fused trajectory at the gap that exited under base policy
    gm=2.0; r=run(gm*e.Wd, level=0.0, seed=0, rec=True, fuse=True)
    layout_plot(gm*e.Wd, r['traj'], path="outputs/e40_fused.png",
                title=f"e40 side-gap g={gm}xWd FUSED: {r['reached']}, peak +y {r['lurch_mm']:.1f}mm")
"""
e48 — MISSION on the realistic course: navigate + inspect a turning, variable-width containment
passage under realistic sensor noise, by feel only.

Consolidates the reactive nav we built across e36-e45 into ONE policy:
  * corner-following  (e36 CRUISE/TURN state machine: pivot when boxed-in, resume when clear)
  * wing-wash centering + clearance veer (e36)
  * breach-veto        (e40: forward clear + one side open => follow the present wall, don't get
                        pulled out; hold the LATCHED pre-breach heading, not a global axis)
  * breach logger      (e45: log onset/offset of a side opening while forward clear & cruising;
                        report center, and size with the +16mm feeler-edge calibration from e45)
Runs at the 1kHz operating point, with 1x IMU+range noise. Output: combined map (flight path +
detected breaches vs ground truth) + score table. fuse=False drops the veto (baseline).

Run:  python experiments/e48_mission.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e36_reactive_course as e
import experiments.e40_side_gap as e40
import experiments.e47_realistic_course as e47
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna
from src.safety import Clearance, WINGREACH
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe

CRUISE=e.CRUISE; Vc=e.Vc; Kc=e.Kc; Kd=e.Kd; KLAT=e.KLAT; Ksteer=e.Ksteer; FMAX=e.FMAX
STOP=e.STOP; CLEAR=e.CLEAR; SAFE_BUF=e.SAFE_BUF; KVEER=e.KVEER; YAWRATE=e.YAWRATE
ROLL_FF=e.ROLL_FF; FIN_ZONE=e.FIN_ZONE
GAP_THRESH=e40.GAP_THRESH; KFOLLOW=e40.KFOLLOW; STANDOFF=e40.STANDOFF
MIN_SPAN=0.003            # open-span floor: filters single-tick glitches but passes real openings
                          # down to ~15mm (raw span ~5mm); adrift FPs are killed by the opposite-wall gate, not this.
SIZE_OFFSET_MM=16.0                      # e45 feeler-edge calibration: true ~ detected + 16mm
_TRACE=None                              # set to a list to capture per-control-step diagnostics
def _wrap(a): return (a+np.pi)%(2*np.pi)-np.pi

def run(geo, level=1.0, seed=0, rate=1000, fuse=True, tmax=None,
        use_wingwash=True, use_feelers=True, open_loop=False, model_path="models/_realistic.xml"):
    """Ablation switches (all default to the original behaviour, bit-for-bit):
      use_wingwash=False -> drop the -Kc*rd_f wing-wash centring term from roll_ref (keep -Kd*v_lat damping).
      use_feelers=False  -> drop the feeler-based steer/safe contributions to pref, disable the
                             breach veto AND the breach logger (no feelers -> can't detect openings
                             either). The forward-feeler speed law (Vcmd/slow) and corner-turn trigger
                             are left alone -- basic collision/speed handling, not the steering/detection
                             channel being ablated.
      open_loop=True     -> ignore all sensing: constant Vcmd=Vc, roll_ref=0.0, pref never updated
                             (holds the initial heading), state machine never leaves CRUISE (no corner
                             turns), breach logger disabled. Only the stabilizing attitude/pitch/yaw-rate
                             loop runs.
      model_path         -> where the built MJCF is written; e47.build_model()'s default path is a
                             SHARED fixed file, so concurrent callers (e.g. parallel course runs)
                             MUST pass distinct paths to avoid racing on the same file.
    """
    nm=NoiseModel(level,seed); fly=Flyer(e47.build_model(geo, path=model_path)); fin=geo['finish']
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,
                         Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/rate)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly,n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=CRUISE); ctrl.h_ref=CRUISE
    N=max(1,round((1.0/rate)/fly.dt)); dt_c=N*fly.dt
    # path length for a time budget
    P=np.array(geo['P']); plen=np.sum(np.linalg.norm(np.diff(P,axis=0),axis=1))
    tmax=(plen/Vc*3.0+16 if tmax is None else tmax)
    t=0.0; si=0; state="CRUISE"; tdir=0; turn0=None; pref=0.0; nose_f=nose_prev=0.0; I_s=I_y=0.0; rd_f=0.0
    minc=1e3; crashed=False; reached=None; traj=[]; pl=[]; floor=dict(axis=2,sign=1,pos=0.0)
    openL=openR=False; onL=onR=None; onLx=onRx=None; detected=[]; latch=None
    while t<tmax:
        if si%N==0:
            s_true=fly.sense(); s=nm.sense(s_true); psi=s['yaw']; psi_true=s_true['yaw']; x,y,z=fly.x_com; b=bodyframe(s); v_lat=b['vy']
            spd=np.hypot(s['vx'],s['vy'])
            nose_f+=_wrap(psi-nose_f)*dt_c/0.04; nrate=(nose_f-nose_prev)/dt_c; nose_prev=nose_f
            raw=ant.feel([0,30,-30,50,-50,90,-90]); f0,fLp,fRp,f50p,f50m,dL,dR=nm.feel(raw); fwd,fL,fR=f0,fLp,fRp
            rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*dt_c/0.10; pl=e.planes(psi_true,fly.x_com,raw[5],raw[6])
            ln=min(f50p,dL); rn=min(f50m,dR); safe=0.0
            if use_feelers:
                if rn<SAFE_BUF: safe+=KVEER*(SAFE_BUF-rn)/SAFE_BUF
                if ln<SAFE_BUF: safe-=KVEER*(SAFE_BUF-ln)/SAFE_BUF
            slow=0.35 if min(ln,rn)<SAFE_BUF else 1.0
            gapL=dL>GAP_THRESH; gapR=dR>GAP_THRESH; fclear=fwd>CLEAR
            # ---- breach logger: project onto the open-side wall (perp offset = closed-side feeler) ----
            perp=np.array([-np.sin(nose_f), np.cos(nose_f)])   # left normal of heading
            def _proj(sign, dclosed): return (np.array([x,y])+sign*min(dclosed,0.05)*perp)
            # a breach = ONE wall open while the OPPOSITE wall is intact (a gap in a real corridor);
            # both-open (adrift/open space) or transient toggles at turns must not log.
            Lb = gapL and (not gapR); Rb = gapR and (not gapL)
            if use_feelers and (not open_loop) and state=="CRUISE" and fclear:
                if Lb and not openL: openL=True; onL=_proj(+1,dR); onLx=np.array([x,y])
                if (not Lb) and openL:
                    openL=False
                    if np.linalg.norm(np.array([x,y])-onLx)>MIN_SPAN: detected.append(('L',onL,_proj(+1,dR) if not gapR else (x,y)))
                if Rb and not openR: openR=True; onR=_proj(-1,dL); onRx=np.array([x,y])
                if (not Rb) and openR:
                    openR=False
                    if np.linalg.norm(np.array([x,y])-onRx)>MIN_SPAN: detected.append(('R',onR,_proj(-1,dL) if not gapL else (x,y)))
            else:                                          # left CRUISE (turn) or forward blocked: close cleanly, no logging
                openL=openR=False
            # ---- state machine ----
            if open_loop:
                Vcmd=Vc; roll_ref=0.0                       # dead-reckoning: constant speed, wings level, hold initial heading
            elif state=="CRUISE":
                Vcmd=Vc*np.clip((fwd-STOP)/0.04,0.0,1.0)*slow
                breach = fuse and use_feelers and fclear and (gapL!=gapR)
                if breach:
                    if latch is None: latch=nose_f                    # hold heading at breach onset
                    if gapL: roll_ref=np.clip(-KFOLLOW*(dR-STANDOFF)-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                    else:    roll_ref=np.clip(+KFOLLOW*(dL-STANDOFF)-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                    pref+=_wrap(latch-pref)*dt_c/0.2
                else:
                    latch=None
                    wingwash = -Kc*rd_f if use_wingwash else 0.0
                    roll_ref=np.clip(wingwash-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                    steer = np.clip(Ksteer*(min(fL,FMAX)-min(fR,FMAX)),-0.5,0.5) if use_feelers else 0.0
                    pref+=(steer+safe)*dt_c
                if fwd<STOP and min(fL,fR)<STOP and spd<0.03:
                    tdir=+1 if fL>=fR else -1; state="TURN"; turn0=nose_f; I_y=0.0; latch=None
            else:                                                     # TURN (pivot in place)
                Vcmd=0.0; pref+=tdir*YAWRATE*dt_c; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
                turned=abs(_wrap(nose_f-turn0))
                if fwd>CLEAR and turned>np.radians(20): state="CRUISE"; I_s=0.0
                elif turned>np.radians(175): tdir=-tdir; turn0=nose_f
            er=Vcmd-b['vx']; I_s=np.clip(I_s+er*dt_c,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
            eyaw=_wrap(nose_f-pref); I_y=np.clip(I_y+eyaw*dt_c,-1.0,1.0); uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
            u=ctrl.update(b,dt_c,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy)
            if _TRACE is not None and si%50==0:
                _TRACE.append((t,x,y,np.degrees(nose_f),state,fwd,fL,fR,fclear,gapL,gapR))
        fly.step(kin,t,surface=(pl+[floor])); x,y,z=fly.x_com
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            dfin=np.hypot(fin[0]-x,fin[1]-y); c=clr.min_clearance(); minc=min(minc,c)
            if c<WINGREACH and dfin>0.05: crashed=True
            traj.append((x,y))
        if np.hypot(fin[0]-x,fin[1]-y)<FIN_ZONE: reached='finish'; break
        if z<0.0 or z>0.2: reached='fell'; break
        t+=fly.dt; si+=1
    if openL and onLx is not None and np.linalg.norm(np.array([x,y])-onLx)>MIN_SPAN: detected.append(('L',onL,(x,y)))
    if openR and onRx is not None and np.linalg.norm(np.array([x,y])-onRx)>MIN_SPAN: detected.append(('R',onR,(x,y)))
    return dict(reached=reached, crashed=crashed, min_clear_mm=minc*1e3, t=t, traj=traj, detected=detected)

def _truth_spans(geo, breaches):
    out=[]
    for (s,li,f0,f1) in breaches:
        V=geo['Lv'] if s==+1 else geo['Rv']; a,b=V[li],V[li+1]
        out.append((s,li,a+(b-a)*f0,a+(b-a)*f1))
    return out

def score(geo, breaches, detected):
    truth=_truth_spans(geo,breaches)
    print(" breach (truth)                | detected center(mm) | loc err | size true/det+16")
    n=0
    for (s,li,p0,p1) in truth:
        cen=(p0+p1)/2; L=np.linalg.norm(p1-p0)
        cand=[d for d in detected if (d[0]=='L')==(s==+1) and np.linalg.norm((np.array(d[1])+np.array(d[2]))/2-cen)<0.06]
        if cand:
            d=min(cand,key=lambda d:np.linalg.norm((np.array(d[1])+np.array(d[2]))/2-cen))
            dc=(np.array(d[1])+np.array(d[2]))/2; dl=np.linalg.norm(np.array(d[2])-np.array(d[1]))
            loc=np.linalg.norm(dc-cen); n+=1
            print(f"  {'L' if s==+1 else 'R'} leg{li} @({cen[0]*1e3:.0f},{cen[1]*1e3:.0f}) {L*1e3:.0f}mm | ({dc[0]*1e3:.0f},{dc[1]*1e3:.0f})        | {loc*1e3:5.1f}mm | {L*1e3:.0f} / {dl*1e3+SIZE_OFFSET_MM:.0f}")
        else:
            print(f"  {'L' if s==+1 else 'R'} leg{li} @({cen[0]*1e3:.0f},{cen[1]*1e3:.0f}) {L*1e3:.0f}mm | MISSED")
    print(f"  => detected {n}/{len(truth)}")
    return n

def mission_map(geo, breaches, res, path="outputs/e48_mission_map.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,6.5))
    for (cx,cy,hs) in geo['blocks']: ax.add_patch(plt.Rectangle((cx-hs,cy-hs),2*hs,2*hs,color="#9aa4b0",lw=0))
    for (s,li,p0,p1) in _truth_spans(geo,breaches):
        ax.plot([p0[0],p1[0]],[p0[1],p1[1]],color="#dc2626",lw=6,alpha=0.5)
    for (side,a,b) in res['detected']:
        a=np.array(a); b=np.array(b); m=(a+b)/2
        ax.plot([a[0],b[0]],[a[1],b[1]],color="#2563eb",lw=2); ax.plot([m[0]],[m[1]],'v',color="#2563eb",ms=9)
    tj=np.array(res['traj']); ax.plot(tj[:,0],tj[:,1],'k-',lw=1.1,label="flight")
    fin=geo['finish']; ax.add_patch(plt.Circle((fin[0],fin[1]),e.FIN_PAD,color="#2fbf52",alpha=0.6))
    ax.plot([],[],color="#dc2626",lw=6,alpha=0.5,label="breach (truth)"); ax.plot([],[],'v',color="#2563eb",label="breach (detected)")
    ax.set_aspect('equal'); ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set_title("e48 mission: navigate + inspect (turns + variable width + breaches, 1x noise)")
    fig.savefig(path,dpi=120,bbox_inches='tight'); print("saved ->",path)

if __name__=="__main__":
    geo=e47.build_geometry(e47.LEGS,e47.BREACHES)
    print("##### MISSION (fused, 1x noise) #####")
    r=run(geo,level=1.0,seed=1,fuse=True)
    print(f"outcome={r['reached']} crash={r['crashed']} minClear={r['min_clear_mm']:.1f}mm t={r['t']:.1f}s")
    score(geo,e47.BREACHES,r['detected']); mission_map(geo,e47.BREACHES,r)
    print("\n##### BASELINE (no veto) #####")
    r0=run(geo,level=1.0,seed=1,fuse=False)
    print(f"outcome={r0['reached']} crash={r0['crashed']} detected {len(r0['detected'])}")
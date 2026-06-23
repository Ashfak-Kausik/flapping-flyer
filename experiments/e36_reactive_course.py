"""
e36 — Full REACTIVE COURSE: the flyer feels its way through a winding corridor
(START -> L90 -> R90 -> 30 LEFT -> sharp RIGHT -> FINISH) and lands. NO memorized route,
NO homing/shortcut: it navigates the WHOLE corridor purely by sensing, and lands when it
reaches the finish pad.
  ANTENNA (forward feeler rays): detect walls ahead, steer toward the opening (rounds bends;
    stop-and-pivot only if fully boxed in).
  WING-WASH (ctrl.roll_dist): aerodynamic self-effect off the side walls keeps it centered.


To make the course LONGER for a harder test, just change SCALE (e.g. SCALE=3.0 tripls every
segment). Walls, finish pad, and the run timeout all scale automatically.
Run:  python experiments/e36_reactive_course.py      (saves outputs/e36_reactive_course.png)
"""
import sys; from pathlib import Path; import numpy as np, mujoco
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna, BIG
from experiments.e31_corner import bodyframe


Vc=0.05; Kc=1.5e-5; Kd=3.0; KLAT=3.0; ROLL_FF=175.0/10399.0
YAWRATE=0.5; Ksteer=5.0; STOP=0.042; CLEAR=0.090; FMAX=0.11; SAFE=0.013
PAD=0.006; CRUISE=0.05; DESC_RATE=0.014; WALL_H=0.080; Wd=0.064
FIN_PAD=0.028          # finish pad radius (visible green disk)
FIN_ZONE=0.034         # land when the flyer reactively reaches within this of the finish


SCALE=3.0              # <-- set to 3.0 for a course with every segment tripled
CPTS=np.array([[0,0],[0.055,0],[0.055,0.065],[0.120,0.065],[0.176,0.098],[0.206,0.046]])*SCALE
FINISH=CPTS[-1]


def path_length(pts=None):
    p=CPTS if pts is None else pts
    return float(sum(np.linalg.norm(p[i+1]-p[i]) for i in range(len(p)-1)))


def _miter(pts,W):
    d=[(pts[i+1]-pts[i])/np.linalg.norm(pts[i+1]-pts[i]) for i in range(len(pts)-1)]
    ln=lambda v:np.array([-v[1],v[0]]); nrm=[]
    for i in range(len(pts)):
        if i==0: v=d[0]
        elif i==len(pts)-1: v=d[-1]
        else: v=d[i-1]+d[i]; v=v/np.linalg.norm(v)
        n=ln(v)
        if 0<i<len(pts)-1:
            c=np.clip(d[i-1]@d[i],-1,1); h=np.arccos(c)/2; n=n/max(np.cos(h),0.45)
        nrm.append(n)
    return pts+(W/2)*np.array(nrm), pts-(W/2)*np.array(nrm)


def wall_boxes(pts=None,W=Wd,step=0.007,hs=0.005):
    """Dense overlapping blocks along each offset wall line -> guaranteed gap-free."""
    pts=CPTS if pts is None else pts
    L,R=_miter(pts,W); boxes=[]
    for edge in (L,R):
        for i in range(len(edge)-1):
            a,b=edge[i],edge[i+1]; v=b-a; ln=np.linalg.norm(v)
            if ln<1e-6: continue
            n=max(1,int(np.ceil(ln/step)))
            for k in range(n+1):
                p=a+(k/n)*v; boxes.append((p[0],p[1],hs))
    return boxes


def build_model(path="models/_reactive_course.xml"):
    xml=open("models/flyer.xml").read()
    g="".join(f'    <geom name="cw{i}" type="box" pos="{cx:.4f} {cy:.4f} {WALL_H/2:.4f}" '
              f'size="{hs:.4f} {hs:.4f} {WALL_H/2:.4f}" group="3" '
              f'rgba="0.62 0.66 0.72 1" contype="0" conaffinity="0"/>\n'
              for i,(cx,cy,hs) in enumerate(wall_boxes()))
    g+=(f'    <geom name="fin" type="cylinder" pos="{FINISH[0]:.4f} {FINISH[1]:.4f} 0.001" '
        f'size="{FIN_PAD:.4f} 0.001" rgba="0.2 0.85 0.35 0.85" group="2" contype="0" conaffinity="0"/>\n')
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path


def planes(psi,pos,dL,dR):
    lat=np.array([-np.sin(psi),np.cos(psi)]); P=[]
    if dL<BIG: q=pos[:2]+dL*lat; P.append(dict(normal=[-lat[0],-lat[1],0],point=[q[0],q[1],0]))
    if dR<BIG: q=pos[:2]-dR*lat; P.append(dict(normal=[ lat[0], lat[1],0],point=[q[0],q[1],0]))
    return P


def run_timeout():
    return path_length()/Vc*2.2 + 14.0     # generous; scales with course length


def run(do_takeoff=True, tmax=None):
    if tmax is None: tmax=run_timeout()
    fly=Flyer(Path(build_model()))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    h0=PAD if do_takeoff else CRUISE
    ctrl.reset(); fly.reset(kin=kin,height=h0); ctrl.h_ref=h0
    ant=Antenna(fly)
    t=0.0; I_s=0.0; href=h0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; thr_f=None
    phase="TAKEOFF" if do_takeoff else "NAV"; state="CRUISE"; tdir=0; turn0=None; log=[]; ev=[]
    while t<tmax:
        s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
        b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
        nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(nose_f-nose_prev)/fly.dt; nose_prev=nose_f
        fwd,fL,fR,dL,dR=ant.feel([0,30,-30,90,-90]); rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*fly.dt/0.10
        pl=planes(psi,fly.x_com,dL,dR); thrust_extra=0.0; dist_fin=np.hypot(FINISH[0]-x,FINISH[1]-y)
        safe=0.0
        if dR<SAFE: safe+=0.6*(SAFE-dR)/SAFE
        if dL<SAFE: safe-=0.6*(SAFE-dL)/SAFE
        if phase=="TAKEOFF":
            href=min(href+0.025*fly.dt,CRUISE); Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            if z>=CRUISE-0.003 and t>1.0: phase="NAV"; ev.append((round(t,1),"airborne -> navigating"))
        elif phase=="NAV":
            href=CRUISE
            if state=="CRUISE":
                Vcmd=Vc*np.clip((fwd-STOP)/0.04,0.0,1.0)*(0.4 if abs(safe)>0 else 1.0)
                pref+= (np.clip(Ksteer*(min(fL,FMAX)-min(fR,FMAX)),-0.5,0.5)+safe)*fly.dt
                roll_ref=np.clip(-Kc*rd_f-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                if fwd<STOP and min(fL,fR)<STOP and spd<0.03:
                    tdir=+1 if fL>=fR else -1; state="TURN"; turn0=nose_f; I_y=0.0; ev.append((round(t,1),f"boxed in -> pivot {'L' if tdir>0 else 'R'}"))
            else:
                Vcmd=0.0; pref+=tdir*YAWRATE*fly.dt; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
                turned=abs(((nose_f-turn0+np.pi)%(2*np.pi)-np.pi))
                if fwd>CLEAR and turned>np.radians(20): state="CRUISE"; I_s=0.0; rd_f=0.0; ev.append((round(t,1),"opening -> go"))
                elif turned>np.radians(175): tdir=-tdir; turn0=nose_f
            if dist_fin<FIN_ZONE: phase="LAND"; ev.append((round(t,1),f"reached finish pad ({x*1e3:.0f},{y*1e3:.0f}) -> landing"))
        else:
            Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            closeness=np.clip((-thr_f-0.012)/0.020,0.0,1.0) if thr_f is not None else 0.0
            href=max(href-DESC_RATE*(1.0-0.8*closeness)*fly.dt,0.003)
            if closeness>0.5 and abs(s['vz'])<0.02: thrust_extra=-0.012*closeness
        ctrl.h_ref=href
        er=Vcmd-u_fwd; I_s=np.clip(I_s+er*fly.dt,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
        eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*fly.dt,-1.0,1.0)
        uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
        u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        thr_f=u[0] if thr_f is None else thr_f+(u[0]-thr_f)*fly.dt/0.15
        kin.set_control(thrust=u[0]+thrust_extra,roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[dict(axis=2,sign=1,pos=0.0)]))
        log.append([t,x*1e3,y*1e3,z*1e3,np.degrees(psi),0 if state=="CRUISE" else 1,{"TAKEOFF":0,"NAV":1,"LAND":2}.get(phase,1)]); t+=fly.dt
        if phase=="LAND" and z<0.012 and abs(s['vz'])<0.01 and t>ev[-1][0]+0.8: break
    return np.array(log), ev


def make_figure(L, path="outputs/e36_reactive_course.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    Lw,Rw=_miter(CPTS,Wd)
    fig,ax=plt.subplots(figsize=(9,7)); fig.patch.set_facecolor("white")
    ax.plot(Lw[:,0]*1e3,Lw[:,1]*1e3,color="#6f747d",lw=4); ax.plot(Rw[:,0]*1e3,Rw[:,1]*1e3,color="#6f747d",lw=4)
    cr=L[L[:,5]==0]; tn=L[L[:,5]==1]
    ax.plot(cr[:,1],cr[:,2],".",ms=2,color="#1d6fb8",label="cruise (feel + steer + center)")
    if len(tn): ax.plot(tn[:,1],tn[:,2],".",ms=3.5,color="#c0392b",label="stop-pivot")
    ax.plot(L[0,1],L[0,2],"go",ms=11,label="start")
    ax.add_patch(plt.Circle((FINISH[0]*1e3,FINISH[1]*1e3),FIN_PAD*1e3,color="#2e9e4f",alpha=0.5,label="finish pad"))
    ax.plot(L[-1,1],L[-1,2],"ks",ms=8,label="landed")
    ax.set_aspect("equal"); ax.legend(fontsize=9,loc="upper left"); ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.grid(alpha=0.2)
    ax.set_title(f"Reactive course (SCALE={SCALE}): felt its way through, no memorized route, no shortcut")
    fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)


if __name__=="__main__":
    print(f"SCALE={SCALE}  path length={path_length()*1e3:.0f}mm  timeout={run_timeout():.0f}s")
    L,ev=run(do_takeoff=True)
    print("what it felt and did:")
    for tt,msg in ev: print(f"  {tt:.1f}s  {msg}")
    print(f"final ({L[-1,1]:.0f},{L[-1,2]:.0f})mm z {L[-1,3]:.1f}mm; finish ({FINISH[0]*1e3:.0f},{FINISH[1]*1e3:.0f})mm")
    np.savez("outputs/e36_reactive_course.npz",L=L); make_figure(L)
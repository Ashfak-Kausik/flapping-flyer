"""
e35 — REACTIVE navigation: the flyer feels its way through a corridor with NO memorized
route. Two senses cooperate:
  * ANTENNA (src/antenna.py): forward feeler rays detect a wall ahead and which way is
    open. The nose STEERS toward the most-open direction (rounds bends), and if the way
    is fully blocked it STOPS and pivots until a feeler clears (sharp turns / dead-ends).
  * WING-WASH (ctrl.roll_dist): the aerodynamic self-effect off the side walls keeps it
    CENTERED in the corridor (the project's novel sensing).
Nothing about the turn is scripted: the flyer turns because it FELT a wall and felt where
the opening was. Here the course is an L (90 deg bend); it rounds it by steering, ending
aligned and centered in the second corridor.

Run:  python experiments/e35_reactive_nav.py   ->  saves outputs/e35_reactive_nav.png
"""
import sys; from pathlib import Path; import numpy as np, mujoco
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna, BIG
from experiments.e31_corner import bodyframe

Vc=0.06; Kc=1.5e-5; Kd=3.0; KLAT=3.0; ROLL_FF=175.0/10399.0
YAWRATE=0.5; Ksteer=4.0; STOP=0.045; CLEAR=0.095; FMAX=0.12

# L-corridor (solid walls, collision group 3 so the antenna rays see them)
WALLS=[("bot","0.060 -0.030 0.04","0.080 0.002 0.04"),
       ("rgt","0.130  0.060 0.04","0.002 0.092 0.04"),
       ("topi","0.030 0.030 0.04","0.042 0.002 0.04"),
       ("lfti","0.070 0.090 0.04","0.002 0.062 0.04")]

def build_model(path="models/_reactive_lcorner.xml"):
    xml=open("models/flyer.xml").read()
    g="".join(f'    <geom name="{n}" type="box" pos="{p}" size="{s}" group="3" '
              f'rgba="0.62 0.66 0.72 1" contype="0" conaffinity="0"/>\n' for n,p,s in WALLS)
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path

def planes(psi,pos,dL,dR):
    lat=np.array([-np.sin(psi),np.cos(psi)]); P=[]
    if dL<BIG: q=pos[:2]+dL*lat; P.append(dict(normal=[-lat[0],-lat[1],0],point=[q[0],q[1],0]))
    if dR<BIG: q=pos[:2]-dR*lat; P.append(dict(normal=[ lat[0], lat[1],0],point=[q[0],q[1],0]))
    return P

def run():
    fly=Flyer(Path(build_model()))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    ctrl.reset(); fly.reset(kin=kin,height=0.05); ctrl.h_ref=0.05
    ant=Antenna(fly)
    t=0.0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; state="CRUISE"; tdir=0; turn0=None; log=[]; ev=[]
    while t<16.0:
        s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
        b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
        nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(nose_f-nose_prev)/fly.dt; nose_prev=nose_f
        fwd,fL,fR,dL,dR=ant.feel([0,30,-30,90,-90])
        rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*fly.dt/0.10; pl=planes(psi,fly.x_com,dL,dR)
        if state=="CRUISE":
            Vcmd=Vc*np.clip((fwd-STOP)/0.04,0.0,1.0)
            pref=pref+np.clip(Ksteer*(min(fL,FMAX)-min(fR,FMAX)),-0.5,0.5)*fly.dt
            roll_ref=np.clip(-Kc*rd_f-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
            if fwd<STOP and spd<0.03:
                tdir=+1 if fL>=fR else -1; state="TURN"; turn0=nose_f; I_y=0.0
                ev.append((round(t,1),f"wall {fwd*1e3:.0f}mm -> turn {'L' if tdir>0 else 'R'}"))
        else:
            Vcmd=0.0; pref=pref+tdir*YAWRATE*fly.dt
            roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            turned=abs(((nose_f-turn0+np.pi)%(2*np.pi)-np.pi))
            if fwd>CLEAR and turned>np.radians(20):
                state="CRUISE"; I_s=0.0; rd_f=0.0; ev.append((round(t,1),f"opening found -> go"))
            elif turned>np.radians(175): tdir=-tdir; turn0=nose_f
        e=Vcmd-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*I_s,-np.radians(8),np.radians(8))
        eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*fly.dt,-1.0,1.0)
        uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
        u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        kin.set_control(thrust=u[0],roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[dict(axis=2,sign=1,pos=0.0)]))
        log.append([t,x*1e3,y*1e3,np.degrees(psi),fwd*1e3,spd*1e3,0 if state=="CRUISE" else 1]); t+=fly.dt
    return np.array(log), ev

def make_figure(L, path="outputs/e35_reactive_nav.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(7,7)); fig.patch.set_facecolor('white')
    for n,p,s in WALLS:
        cx,cy,_=[float(v)*1e3 for v in p.split()]; sx,sy,_=[float(v)*1e3 for v in s.split()]
        ax.add_patch(plt.Rectangle((cx-sx,cy-sy),2*sx,2*sy,color='#888c95'))
    cr=L[L[:,6]==0]; tn=L[L[:,6]==1]
    ax.plot(cr[:,1],cr[:,2],'.',ms=2,color='#1d6fb8',label='cruise (steer+center)')
    if len(tn): ax.plot(tn[:,1],tn[:,2],'.',ms=3,color='#c0392b',label='pivot')
    ax.plot(L[0,1],L[0,2],'go',ms=9,label='start'); ax.plot(L[-1,1],L[-1,2],'ks',ms=8,label='end')
    ax.set_aspect('equal'); ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)'); ax.legend(fontsize=9,loc='lower right')
    ax.set_title('Reactive navigation: feels its way around the bend (no memorized route)'); ax.grid(alpha=0.2)
    fig.tight_layout(); fig.savefig(path,dpi=110); print("saved ->",path)

if __name__=="__main__":
    L,ev=run()
    print("what it felt and did:")
    for tt,msg in ev: print(f"  {tt:.1f}s  {msg}")
    g=L[L[:,2]>70]
    print(f"final ({L[-1,1]:.0f},{L[-1,2]:.0f})mm heading {L[-1,3]:.0f}deg")
    if len(g): print(f"after the bend: heading {g[:,3].mean():.0f}deg (corridor is 90), centered x={g[:,1].mean():.0f}mm (center 100)")
    np.savez("outputs/e35_reactive_nav.npz",L=L); make_figure(L)
"""
e31 — Stop-turn-go corner (the sharp-turn maneuver for tight corridors).

A coordinated banked turn needs ~390 mm radius; a corridor is tens of mm wide, so
sharp corners must be taken by stopping, pivoting in place, and re-accelerating:
  CRUISE -> STOP (hover-hold) -> YAW in place to the new heading -> GO down it.

KEY ENABLER (heading-agnostic control)
  Our actuators are body-frame but pitch/roll were sensed in the WORLD frame, so the
  controller silently assumed a fixed heading. After a 90 deg pivot, body-forward points
  along world-y, a forward-pitch command reads as world-roll, and the axes swap -> it
  flies off diagonally and can't hold the new heading. Fix: feed the controller
  HEADING-RELATIVE pitch/roll (and rates) -- rotate the world tilt by the nose angle.
  At heading 0 this is identity; at any heading the controller works. Combined with the
  body-frame velocity feed, the controller is now fully body-referenced.

RESULT
  In-place yaw runs at ~0.6 rad/s (34 deg/s, ~3x the coordinated-turn ceiling, since a
  stopped pivot has no coordination to respect). Pivot drift ~4 mm (won't clip a wall).
  Go leg holds the new heading to <1 deg and flies straight down it at cruise speed.
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

Vc=0.07; KLAT=3.0; ROLL_FF=175.0/10399.0; YAWRATE=0.6


def bodyframe(s):
    """Return a sense dict with velocity and attitude rotated into the nose (body) frame."""
    psi=s['yaw']; cp, sp=np.cos(psi), np.sin(psi)
    b=dict(s)
    b['vx']= s['vx']*cp+s['vy']*sp
    b['vy']=-s['vx']*sp+s['vy']*cp
    b['pitch']= s['pitch']*cp+s['roll']*sp
    b['roll'] =-s['pitch']*sp+s['roll']*cp
    b['wx']= s['wx']*cp+s['wy']*sp
    b['wy']=-s['wx']*sp+s['wy']*cp
    return b


def run(turn_deg=90.0):
    fly=Flyer(Path("models/flyer.xml"))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    ctrl.reset(); fly.reset(kin=kin,height=0.05); ctrl.h_ref=0.05
    t=0.0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; log=[]
    TURN=np.radians(turn_deg); state="CRUISE"
    while t<14.0:
        s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
        b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
        nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(nose_f-nose_prev)/fly.dt; nose_prev=nose_f
        if state=="CRUISE":
            Vcmd=Vc; target=0.0
            if t>2.0: state="STOP"
        elif state=="STOP":
            Vcmd=0.0; target=0.0
            if spd<0.012: state="YAW"; I_y=0.0
        elif state=="YAW":
            Vcmd=0.0; target=min(pref+YAWRATE*fly.dt,TURN)
            if nose_f>TURN-np.radians(2) and abs(nrate)<0.25: state="GO"; I_s=0.0; I_y=0.0
        else:
            Vcmd=Vc; target=TURN
        pref=target
        e=Vcmd-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*I_s,-np.radians(8),np.radians(8))
        roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*fly.dt,-1.0,1.0)
        uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
        u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        kin.set_control(thrust=u[0],roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy); fly.step(kin,t)
        log.append([t,x*1e3,y*1e3,z*1e3,np.degrees(psi),spd*1e3,{"CRUISE":0,"STOP":1,"YAW":2,"GO":3}[state]]); t+=fly.dt
    return np.array(log)


def make_figure(L, path="outputs/e31_corner.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t,x,y,psi,spd,ph=L[:,0],L[:,1],L[:,2],L[:,4],L[:,5],L[:,6]
    fig,ax=plt.subplots(1,2,figsize=(12,5)); fig.patch.set_facecolor('white')
    cols={0:'#1d6fb8',1:'#e08a1e',2:'#c0392b',3:'#127a3d'}; labs={0:'cruise',1:'stop',2:'yaw',3:'go'}
    for p in [0,1,2,3]:
        seg=L[ph==p]
        if len(seg): ax[0].plot(seg[:,1],seg[:,2],'.',ms=2,color=cols[p],label=labs[p])
    ax[0].set_aspect('equal'); ax[0].set_xlabel('x (mm)'); ax[0].set_ylabel('y (mm)'); ax[0].legend(fontsize=9)
    ax[0].set_title('Stop-turn-go: cruise in x -> pivot -> go in y'); ax[0].grid(alpha=0.3)
    ax2=ax[1]; ax2.plot(t,psi,color='#6a3d9a',label='nose heading (deg)'); ax2.set_ylabel('heading (deg)',color='#6a3d9a')
    ax3=ax2.twinx(); ax3.plot(t,spd,color='#1d6fb8',alpha=0.6,label='speed (mm/s)'); ax3.set_ylabel('speed (mm/s)',color='#1d6fb8')
    for p,c in [(1,'#e08a1e'),(2,'#c0392b'),(3,'#127a3d')]:
        seg=t[ph==p]
        if len(seg): ax2.axvspan(seg.min(),seg.max(),color=c,alpha=0.08)
    ax2.set_xlabel('time (s)'); ax2.set_title('Heading pivots 0->90 deg; speed stops and resumes'); ax2.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path,dpi=110); print("saved ->",path)


if __name__ == "__main__":
    L=run(); yaw=L[L[:,6]==2]; go=L[L[:,6]==3]; g2=go[go[:,0]>go[0,0]+1.0]
    print(f"pivot {yaw[-1,0]-yaw[0,0]:.1f}s, drift {np.hypot(yaw[-1,1]-yaw[0,1],yaw[-1,2]-yaw[0,2]):.1f} mm; "
          f"go heading {g2[:,4].mean():.0f}+-{g2[:,4].std():.1f} deg, speed {g2[:,5].mean():.0f} mm/s")
    np.savez("outputs/e31_corner.npz", L=L); make_figure(L)
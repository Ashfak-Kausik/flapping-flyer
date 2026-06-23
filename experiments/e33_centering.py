"""
e33 — Corridor lateral centering from the flyer's own wing-wash asymmetry.

A corridor is two vertical side walls (passed to ground_effect as two plane surfaces;
per-strip enhancements add). Off-center, the nearer wall enhances its wing's lift more
-> an asymmetric roll torque the disturbance observer estimates as ctrl.roll_dist
(rad/s^2, sign = which wall is closer). NO dedicated proximity sensor: the cue is the
flyer's own aerodynamic self-effect. This is the project's central idea, in a corridor.

CALIBRATION (held at known offsets; ctrl.roll_dist, rad/s^2):
   y=-10mm: -1317   y=-5mm: -408   y=0: 0   y=+5mm:+408   y=+10mm:+1317
  Monotonic, zero at center, superlinear (~1/d^2 wall law), sign = side.

CENTERING LAW (uses ONLY the proximity signal, no position knowledge):
   roll_ref = -Kc * lowpass(roll_dist) - Kd * v_lat
Drives roll_dist -> 0, i.e. flyer -> centerline. (Sign matters: +offset -> +roll_dist
-> need -roll_ref to move back; getting it backwards runs it into the wall.)

RESULT: from +10mm and -12mm offsets, centers to y=0.0mm within ~0.4 m of corridor,
cruise undisturbed. Works at any corridor heading because the controller and roll_dist
are body-referenced (e31 body-frame feed).
"""
import sys; from pathlib import Path; import numpy as np, mujoco
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design
from experiments.e31_corner import bodyframe

Vc=0.07; W=0.060; ALT=0.05; Kc=1.5e-5; Kd=3.0


def corridor_walls(p0, heading, width=W):
    """Two wall planes for a corridor centerline through p0 (x,y) at `heading` (rad)."""
    lat=np.array([-np.sin(heading), np.cos(heading)])      # left (+) / right (-) of travel
    pL=np.array([p0[0],p0[1]])+ (width/2)*lat; pR=np.array([p0[0],p0[1]])-(width/2)*lat
    return [dict(normal=[-lat[0],-lat[1],0.0], point=[pL[0],pL[1],0.0]),
            dict(normal=[ lat[0], lat[1],0.0], point=[pR[0],pR[1],0.0])]


def run(y0=0.010):
    fly=Flyer(Path("models/flyer.xml"))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    WALLS=corridor_walls((0.0,0.0),0.0)
    ctrl.reset(); fly.reset(kin=kin,height=ALT)
    fly.data.qpos[1]=y0; mujoco.mj_forward(fly.model,fly.data); ctrl.h_ref=ALT
    t=0.0; I_s=0.0; rd_f=0.0; log=[]
    while t<8.0:
        s=fly.sense(); x,y,z=fly.x_com; b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
        rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*fly.dt/0.10
        roll_ref=np.clip(-Kc*rd_f-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
        e=Vc-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*I_s,-np.radians(8),np.radians(8))
        u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        kin.set_control(thrust=u[0],roll=u[1],pitch=u[2]); fly.step(kin,t,surface=WALLS)
        log.append([t,x*1e3,y*1e3,z*1e3,rd_f]); t+=fly.dt
    return np.array(log)


def make_figure(L, path="outputs/e33_centering.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t,x,y,rd=L[:,0],L[:,1],L[:,2],L[:,4]
    fig,ax=plt.subplots(2,1,figsize=(11,7)); fig.patch.set_facecolor('white')
    ax[0].axhspan(W/2*1e3,W/2*1e3+6,color='#555',alpha=0.7); ax[0].axhspan(-W/2*1e3-6,-W/2*1e3,color='#555',alpha=0.7)
    ax[0].axhline(0,color='k',ls=':',lw=1,label='centerline')
    ax[0].plot(x,y,color='#1d6fb8',lw=2,label='flyer path')
    ax[0].plot(x[0],y[0],'ro',ms=8,label=f'start (off-center {y[0]:+.0f}mm)')
    ax[0].set_xlabel('x along corridor (mm)'); ax[0].set_ylabel('y (mm)'); ax[0].set_ylim(-W/2*1e3-8,W/2*1e3+8)
    ax[0].legend(fontsize=9,loc='upper right'); ax[0].set_title('Centering: rides back to the centerline from its own wing-wash signal'); ax[0].grid(alpha=0.3)
    ax[1].plot(t,rd,color='#127a3d'); ax[1].axhline(0,color='k',lw=0.5)
    ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('roll_dist (rad/s$^2$)'); ax[1].grid(alpha=0.3)
    ax[1].set_title('Proximity signal (wall asymmetry) driven to zero = centered')
    fig.tight_layout(); fig.savefig(path,dpi=110); print("saved ->",path)


if __name__ == "__main__":
    L=run(0.010); s=L[L[:,0]>5.0]
    print(f"start y=+10mm -> settled y={s[:,2].mean():+.1f}mm (range [{s[:,2].min():+.1f},{s[:,2].max():+.1f}]); cruised x={L[-1,1]:.0f}mm")
    np.savez("outputs/e33_centering.npz", L=L, W=W); make_figure(L)
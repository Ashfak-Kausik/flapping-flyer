"""
e41 — PHASE 3: characterize the WING-WASH as a side-proximity SENSOR.

The novelty is that a nearby wall is felt with NO dedicated side sensor: the wall blocks
each wing's induced wake (a ground-effect-like enhancement, kappa(d)=min(1+K_GE*(R/4d)^2,
KAPPA_MAX); Cheeseman-Bennett 1955), and because the near wing is enhanced more than the
far wing the asymmetry is a ROLL TORQUE — the directional "which-way-is-the-wall" signal
the disturbance observer reports as roll_dist.

This experiment measures the SENSOR TRANSFER FUNCTION directly from the aero model: hold the
flyer level at a fixed pose, place a wall at lateral distance d, and read the cycle-averaged
roll torque it induces (with-wall minus free-air). No control loop — pure force evaluation —
so it isolates the sensor physics from the navigation/observer. We then report range,
saturation, sensitivity and (with IMU noise through the observer) resolution.

HONEST CAVEAT: this characterizes the sensor GIVEN our aero model. K_GE (=1.0) is the
ground-effect coefficient that ground_effect.py flags as the wind-tunnel number to pin down;
every range/resolution figure here scales with that model being right. That validation is
future experimental/CFD work.

Run:  python experiments/e41_wingwash_sensor.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import mujoco
from src.flyer import Flyer
from src.controller import design
from src.ground_effect import wing_aero_ge, kappa
import src.aero as aero

CRUISE=0.05; FREQ=80.0; PERIOD=1.0/FREQ            # hover flap frequency

def _aero_torque(fly, kin, t, surface):
    """Replicate flyer.step's aero (body-frame torque + force) WITHOUT integrating the body,
    so the pose stays fixed and we read the instantaneous wall-induced load."""
    d=fly.data; fly._set_body_state(); fly._prescribe_wings(kin,t); mujoco.mj_forward(fly.model,d)
    R=d.xmat[fly.thorax].reshape(3,3); F_w=np.zeros(3); T_w=np.zeros(3)
    for s,bid in fly.wings.items():
        F,T,fly.vn[s],_=wing_aero_ge(fly.model,d,bid,fly.strips[s],fly.vn[s],fly.dt,surface,fly.R)
        F_w+=F; T_w+=T+np.cross(d.xipos[bid]-fly.x_com,F)
    return (R.T@T_w), F_w                            # body-frame torque, world force

def cycle_avg_roll(fly, kin, surface, cycles=3):
    """Cycle-averaged body-roll torque (Nm). Runs `cycles` flap periods; averages the last one
    (lets the added-mass state vn settle)."""
    dt=fly.dt; nper=int(round(PERIOD/dt)); Tx=[]
    for i in range(cycles*nper):
        tb,_=_aero_torque(fly,kin,i*dt,surface)
        if i>=(cycles-1)*nper: Tx.append(tb[0])      # body x-axis = roll
    return float(np.mean(Tx))

def measure(d_wall, thrust=0.5, h=CRUISE):
    """Wall-induced cycle-averaged roll torque (and roll accel) at lateral wall distance d_wall (m).
    Wall on +y side; signal = (roll torque with wall) - (roll torque in free air)."""
    fly=Flyer("models/flyer.xml"); ctrl,kin,info=design(fly,control_dt=1e-3)
    kin.set_control(thrust=thrust,roll=0.0,pitch=0.0,yaw=0.0)
    fly.reset(kin=kin,height=h)                      # level, at origin (resets added-mass state vn)
    wall=dict(normal=[0.0,-1.0,0.0], point=[0.0,d_wall,0.0])   # plane at y=d_wall, normal toward flyer
    Tx_wall=cycle_avg_roll(fly,kin,wall)
    fly.reset(kin=kin,height=h)
    Tx_free=cycle_avg_roll(fly,kin,None)
    Ixx=float(fly.I[0,0]); dT=Tx_wall-Tx_free
    return dict(d_mm=d_wall*1e3, T_wall_nNm=dT*1e9, roll_acc=dT/Ixx, Ixx=Ixx)

def sweep(dmin_mm=2.0, dmax_mm=40.0, n=20, thrust=0.5):
    ds=np.linspace(dmin_mm,dmax_mm,n)/1e3
    R=Flyer('models/flyer.xml').R
    print(f"wing-wash sensor transfer function (hover thrust={thrust}, {FREQ:.0f}Hz flap, R={R*1e3:.1f}mm, sat d<R/4={R/4*1e3:.1f}mm)")
    print(" d(mm) | wall roll torque (nN·m) | roll accel (rad/s^2) | kappa(d)")
    rows=[]
    for d in ds:
        r=measure(d, thrust=thrust)
        kp=float(kappa(d, R))
        rows.append((r['d_mm'], r['T_wall_nNm'], r['roll_acc'], kp))
        print(f" {r['d_mm']:5.1f} | {r['T_wall_nNm']:11.2f}            | {r['roll_acc']:9.1f}            | {kp:.3f}")
    return rows

def make_figure(rows, path="outputs/e41_wingwash_sensor.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    d=np.array([r[0] for r in rows]); T=np.array([r[1] for r in rows]); ra=np.array([r[2] for r in rows])
    fig,ax=plt.subplots(1,2,figsize=(11,4.2))
    ax[0].plot(d,np.abs(T),'o-',color="#2563eb"); ax[0].set_xlabel("wall distance d (mm)")
    ax[0].set_ylabel("|wall roll torque| (nN·m)"); ax[0].set_title("wing-wash sensor signal vs wall distance")
    ax[0].grid(alpha=0.3)
    ax[1].loglog(d,np.abs(ra),'o-',color="#7c3aed"); ax[1].set_xlabel("wall distance d (mm)")
    ax[1].set_ylabel("|roll accel| (rad/s²)"); ax[1].set_title("log-log (slope -> falloff power)")
    ax[1].grid(alpha=0.3,which='both')
    fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)

if __name__=="__main__":
    rows=sweep()
    make_figure(rows)
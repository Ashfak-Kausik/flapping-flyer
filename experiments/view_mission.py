"""
view_mission.py — watch the integrated mission LIVE in the MuJoCo viewer.

Run WITH a display:   python experiments/view_mission.py
Same controller/orchestration as e30_mission.py (TAKEOFF -> guided TRANSIT with a
coordinated turn -> ARRIVE -> LAND), but stepped inside mujoco.viewer (slow-mo, loops).
A green beacon marks the goal. Close the window to quit.
"""
import sys, time
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design

Vc=0.07; PSIDOT=0.3; KFF=60.0; KFB=98.0; KP_BANK=1.5; KD_BANK=0.5; KLAT=3.0; ROLL_FF=175.0/10399.0
PAD=0.006; CRUISE=0.05; GOAL=np.array([0.35,0.12]); CAPTURE=0.030; CLIMB_RATE=0.018; DESC_RATE=0.014
FLOOR=dict(axis=2,sign=1,pos=0.0); SLOWMO=0.10

# build a model with a goal beacon on the floor
xml=(ROOT/"models"/"flyer.xml").read_text()
gx,gy=GOAL[0],GOAL[1]
beacon=(f'    <geom name="goal" type="cylinder" pos="{gx:.4f} {gy:.4f} 0.001" size="0.010 0.001" '
        f'rgba="0.20 0.85 0.35 0.7" contype="0" conaffinity="0"/>\n'
        f'    <geom name="goalpost" type="cylinder" pos="{gx:.4f} {gy:.4f} 0.020" size="0.0015 0.020" '
        f'rgba="0.20 0.85 0.35 0.5" contype="0" conaffinity="0"/>\n')
key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
(ROOT/"models"/"flyer_mission.xml").write_text(xml.replace(key,key+beacon))

fly=Flyer(ROOT/"models"/"flyer_mission.xml")
ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0

class M:  # mission state, resettable
    pass
def reset_mission():
    ctrl.reset(); fly.reset(kin=kin,height=PAD); ctrl.h_ref=PAD
    m=M(); m.t=0.0; m.I_s=0.0; m.href=PAD; m.pref=0.0
    m.psiv_f=m.psiv_prev=m.nose_f=m.nose_prev=m.I_y=0.0; m.thr_f=None
    m.phase="TAKEOFF"; m.hold_head=0.0; m.t_arrive=None
    return m
m=reset_mission()

def step():
    s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
    u_fwd=s['vx']*np.cos(psi)+s['vy']*np.sin(psi); v_lat=-s['vx']*np.sin(psi)+s['vy']*np.cos(psi)
    psi_vel=np.arctan2(s['vy'],s['vx'])
    m.psiv_f+=((psi_vel-m.psiv_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.05
    vrate=((m.psiv_f-m.psiv_prev+np.pi)%(2*np.pi)-np.pi)/fly.dt; m.psiv_prev=m.psiv_f
    m.nose_f+=((psi-m.nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(m.nose_f-m.nose_prev)/fly.dt; m.nose_prev=m.nose_f
    dx,dy=GOAL[0]-x,GOAL[1]-y; dist=np.hypot(dx,dy); bearing=np.arctan2(dy,dx); thrust_extra=0.0
    if m.phase=="TAKEOFF":
        m.href=min(m.href+CLIMB_RATE*fly.dt,CRUISE); Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        eyaw=((m.nose_f+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*m.I_y,-0.3,0.3); m.I_y=np.clip(m.I_y+eyaw*fly.dt,-1.5,1.5)
        if z>=CRUISE-0.003 and m.t>1.2: m.phase="TRANSIT"
    elif m.phase=="TRANSIT":
        m.href=CRUISE
        if spd<0.04:
            Vcmd=Vc; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            eyaw=((m.nose_f+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*m.I_y,-0.3,0.3); m.I_y=np.clip(m.I_y+eyaw*fly.dt,-1.5,1.5)
        else:
            Vcmd=Vc*np.clip(dist/0.12,0.2,1.0)
            stp=np.clip(((bearing-m.pref+np.pi)%(2*np.pi)-np.pi),-PSIDOT*fly.dt,PSIDOT*fly.dt); m.pref+=stp; pdot=stp/fly.dt
            eb=((m.pref-m.psiv_f+np.pi)%(2*np.pi)-np.pi)
            roll_ref=np.clip(pdot/KFF+(KP_BANK*eb+KD_BANK*(pdot-vrate))/KFB,-np.radians(1.2),np.radians(1.2))
            eyaw=((m.nose_f-m.psiv_f+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.15*eyaw+0.02*nrate+0.15*m.I_y,-0.3,0.3); m.I_y=np.clip(m.I_y+eyaw*fly.dt,-1.5,1.5)
        if dist<CAPTURE: m.phase="ARRIVE"; m.hold_head=m.nose_f; m.t_arrive=m.t
    elif m.phase=="ARRIVE":
        m.href=CRUISE; Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        eyaw=((m.nose_f-m.hold_head+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*m.I_y,-0.3,0.3); m.I_y=np.clip(m.I_y+eyaw*fly.dt,-1.5,1.5)
        if spd<0.02 and m.t>m.t_arrive+1.5: m.phase="LAND"
    else:
        Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        eyaw=((m.nose_f-m.hold_head+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*m.I_y,-0.3,0.3); m.I_y=np.clip(m.I_y+eyaw*fly.dt,-1.5,1.5)
        closeness=np.clip((-m.thr_f-0.012)/0.020,0.0,1.0) if m.thr_f is not None else 0.0
        m.href=max(m.href-DESC_RATE*(1.0-0.8*closeness)*fly.dt,0.003)
        if closeness>0.5 and abs(s['vz'])<0.02: thrust_extra=-0.012*closeness
        if z<0.012 and abs(s['vz'])<0.01 and m.t>2.0: return True   # landed -> signal loop reset
    ctrl.h_ref=m.href
    e=Vcmd-u_fwd; m.I_s=np.clip(m.I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*m.I_s,-np.radians(8),np.radians(8))
    u=ctrl.update(s,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
    m.thr_f=u[0] if m.thr_f is None else m.thr_f+(u[0]-m.thr_f)*fly.dt/0.15
    kin.set_control(thrust=u[0]+thrust_extra,roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy); fly.step(kin,m.t,surface=FLOOR)
    m.t+=fly.dt
    return False

with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.azimuth=215; viewer.cam.elevation=-18; viewer.cam.distance=0.16
    while viewer.is_running():
        f0=time.time()
        for _ in range(int(0.01/fly.dt)):
            done=step()
            if done or m.t>24.0:
                # brief pause on landing, then loop the mission
                time.sleep(0.5); m=reset_mission(); break
        c=fly.x_com
        viewer.cam.lookat[:]=[c[0]+0.02, c[1], max(c[2],0.02)]
        viewer.sync()
        dtw=0.01/SLOWMO-(time.time()-f0)
        if dtw>0: time.sleep(dtw)
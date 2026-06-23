"""
view_corridor_mission.py — watch the WINDING-CORRIDOR mission LIVE in the MuJoCo viewer.

Run WITH a display:  python experiments/view_corridor_mission.py

Route (your spec): straight -> 90 deg LEFT -> 90 deg RIGHT -> 40 deg -> 10 deg ->
>90 deg U-turn -> land. The flyer threads each corridor by CENTERING on its walls
(ctrl.roll_dist), takes each bend by STOP-TURN-GO, and lands at the green exit.
Walls are 80 mm tall and the flyer cruises at 50 mm, so it flies BETWEEN the walls
(down inside the corridor), not over them. Loops on landing; close window to quit.
"""
import sys, time
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design
from experiments.e31_corner import bodyframe
from experiments.e33_centering import corridor_walls

Vc=0.07; Kc=1.5e-5; Kd=3.0; KLAT=3.0; ROLL_FF=175.0/10399.0; YAWRATE=0.9
PAD=0.006; CRUISE=0.05; W=0.060; DESC_RATE=0.014; FLOOR=dict(axis=2,sign=1,pos=0.0)
WALL_H=0.080; SLOWMO=0.12

# route as (heading_deg, length_m)
LEGS=[(0,0.10),(90,0.09),(0,0.09),(40,0.08),(50,0.08),(185,0.09)]
WP=[np.array([0.0,0.0])]
for h,L in LEGS: WP.append(WP[-1]+L*np.array([np.cos(np.radians(h)),np.sin(np.radians(h))]))
HEAD=[h for h,_ in LEGS]; SEGSTART=[WP[i] for i in range(len(LEGS))]; TARGET=[WP[i+1] for i in range(len(LEGS))]
EXIT=WP[-1]

# build model: flyer + tall wall boxes (two per segment, oriented along heading) + exit beacon
def wallbox(i, cx, cy, hL, hth, h_rad):
    return (f'    <geom name="w{i}" type="box" pos="{cx:.4f} {cy:.4f} {WALL_H/2:.4f}" '
            f'size="{hL:.4f} {hth:.4f} {WALL_H/2:.4f}" euler="0 0 {h_rad:.5f}" '
            f'rgba="0.62 0.66 0.72 0.5" contype="0" conaffinity="0"/>\n')
geoms=""; gi=0
for i,(h,L) in enumerate(LEGS):
    hr=np.radians(h); lat=np.array([-np.sin(hr),np.cos(hr)]); mid=(WP[i]+WP[i+1])/2
    for sgn in (+1,-1):
        c=mid+sgn*(W/2)*lat
        geoms+=wallbox(gi, c[0], c[1], L/2+0.004, 0.002, hr); gi+=1
gx,gy=EXIT
geoms+=(f'    <geom name="goal" type="cylinder" pos="{gx:.4f} {gy:.4f} 0.001" size="0.012 0.001" rgba="0.2 0.85 0.35 0.8" contype="0" conaffinity="0"/>\n'
        f'    <geom name="goalpost" type="cylinder" pos="{gx:.4f} {gy:.4f} 0.030" size="0.0015 0.030" rgba="0.2 0.85 0.35 0.5" contype="0" conaffinity="0"/>\n')
xml=(ROOT/"models"/"flyer.xml").read_text()
key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
(ROOT/"models"/"flyer_corridor.xml").write_text(xml.replace(key,key+geoms))

fly=Flyer(ROOT/"models"/"flyer_corridor.xml")
ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0

class M: pass
def reset_mission():
    ctrl.reset(); fly.reset(kin=kin,height=PAD); ctrl.h_ref=PAD
    m=M(); m.t=0.0; m.I_s=0.0; m.href=PAD; m.pref=0.0
    m.nose_f=m.nose_prev=m.I_y=0.0; m.rd_f=0.0; m.thr_f=None; m.phase="TAKEOFF"; m.seg=0
    return m
m=reset_mission()

def step():
    s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
    b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
    m.nose_f+=((psi-m.nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(m.nose_f-m.nose_prev)/fly.dt; m.nose_prev=m.nose_f
    rd=ctrl.roll_dist; m.rd_f+=(rd-m.rd_f)*fly.dt/0.10; walls=None; thrust_extra=0.0
    if m.phase=="TAKEOFF":
        m.href=min(m.href+0.025*fly.dt,CRUISE); Vcmd=0.0; m.pref=0.0
        roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        if z>=CRUISE-0.003 and m.t>1.0: m.phase="CRUISE"
    elif m.phase=="CRUISE":
        head=np.radians(HEAD[m.seg]); walls=corridor_walls(SEGSTART[m.seg],head); m.pref=head; m.href=CRUISE
        T=TARGET[m.seg]; dist=np.hypot(T[0]-x,T[1]-y); Vcmd=Vc*np.clip(dist/0.07,0.0,1.0)
        roll_ref=np.clip(-Kc*m.rd_f-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
        if dist<0.018 and spd<0.02:
            m.phase="PIVOT" if m.seg<len(TARGET)-1 else "LAND"
            if m.phase=="PIVOT": m.I_y=0.0
    elif m.phase=="PIVOT":
        Vcmd=0.0; nxt=np.radians(HEAD[m.seg+1]); m.href=CRUISE
        m.pref=m.pref+np.clip(((nxt-m.pref+np.pi)%(2*np.pi)-np.pi),-YAWRATE*fly.dt,YAWRATE*fly.dt)
        roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        if abs(((m.nose_f-nxt+np.pi)%(2*np.pi)-np.pi))<np.radians(2.5) and abs(nrate)<0.3:
            m.seg+=1; m.phase="CRUISE"; m.I_s=0.0; m.I_y=0.0; m.rd_f=0.0
    else:
        Vcmd=0.0; m.pref=np.radians(HEAD[m.seg]); roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        closeness=np.clip((-m.thr_f-0.012)/0.020,0.0,1.0) if m.thr_f is not None else 0.0
        m.href=max(m.href-DESC_RATE*(1.0-0.8*closeness)*fly.dt,0.003)
        if closeness>0.5 and abs(s['vz'])<0.02: thrust_extra=-0.012*closeness
        if z<0.012 and abs(s['vz'])<0.01 and m.t>2.0: return True
    ctrl.h_ref=m.href
    e=Vcmd-u_fwd; m.I_s=np.clip(m.I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*m.I_s,-np.radians(8),np.radians(8))
    eyaw=((m.nose_f-m.pref+np.pi)%(2*np.pi)-np.pi); m.I_y=np.clip(m.I_y+eyaw*fly.dt,-1.0,1.0)
    uy=np.clip(0.14*eyaw+0.03*nrate+0.10*m.I_y,-0.3,0.3)
    u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
    m.thr_f=u[0] if m.thr_f is None else m.thr_f+(u[0]-m.thr_f)*fly.dt/0.15
    kin.set_control(thrust=u[0]+thrust_extra,roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy)
    fly.step(kin,m.t,surface=(([] if walls is None else list(walls))+[FLOOR]))
    m.t+=fly.dt
    return False

with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.azimuth=215; viewer.cam.elevation=-28; viewer.cam.distance=0.16
    while viewer.is_running():
        f0=time.time()
        for _ in range(int(0.01/fly.dt)):
            if step() or m.t>55.0:
                time.sleep(0.6); m=reset_mission(); break
        c=fly.x_com
        viewer.cam.lookat[:]=[c[0],c[1],max(c[2],0.02)]
        viewer.sync()
        dtw=0.01/SLOWMO-(time.time()-f0)
        if dtw>0: time.sleep(dtw)
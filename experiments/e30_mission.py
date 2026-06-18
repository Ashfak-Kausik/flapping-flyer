"""
e30 — Integrated mission on the unified controller (e29) of the flapping flyer:
  TAKEOFF -> guided TRANSIT (coordinated turn to the goal) -> ARRIVE (stop) -> LAND.
One controller throughout (K[:,0]=K[:,1]=0); an outer state machine sets the goals.
Flat floor (first end-to-end pass); tunnel/ramp richness layered on later.
Run: python experiments/e30_mission.py  [saves outputs/_mission_states.npz for rendering]
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

Vc=0.07; PSIDOT=0.3; KFF=60.0; KFB=98.0; KP_BANK=1.5; KD_BANK=0.5; KLAT=3.0; ROLL_FF=175.0/10399.0
PAD=0.006; CRUISE=0.05; GOAL=np.array([0.35,0.12]); CAPTURE=0.030; CLIMB_RATE=0.018; DESC_RATE=0.014
FLOOR=dict(axis=2,sign=1,pos=0.0)

def run(save_states=True):
    fly=Flyer(Path("models/flyer.xml"))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    ctrl.reset(); fly.reset(kin=kin,height=PAD); ctrl.h_ref=PAD
    t=0.0; I_s=0.0; href=PAD; pref=0.0; psiv_f=psiv_prev=nose_f=nose_prev=I_y=0.0; thr_f=None
    phase="TAKEOFF"; hold_head=0.0; t_arrive=None; log=[]; events=[]; QP=[]; PH=[]; cap=int(0.02/fly.dt)
    pmap={"TAKEOFF":0,"TRANSIT":1,"ARRIVE":2,"LAND":3}; i=0
    while t<22.0:
        s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
        u_fwd=s['vx']*np.cos(psi)+s['vy']*np.sin(psi); v_lat=-s['vx']*np.sin(psi)+s['vy']*np.cos(psi)
        psi_vel=np.arctan2(s['vy'],s['vx'])
        psiv_f+=((psi_vel-psiv_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.05
        vrate=((psiv_f-psiv_prev+np.pi)%(2*np.pi)-np.pi)/fly.dt; psiv_prev=psiv_f
        nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(nose_f-nose_prev)/fly.dt; nose_prev=nose_f
        dx,dy=GOAL[0]-x,GOAL[1]-y; dist=np.hypot(dx,dy); bearing=np.arctan2(dy,dx); thrust_extra=0.0
        if phase=="TAKEOFF":
            href=min(href+CLIMB_RATE*fly.dt,CRUISE); Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            eyaw=((nose_f+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*I_y,-0.3,0.3); I_y=np.clip(I_y+eyaw*fly.dt,-1.5,1.5)
            if z>=CRUISE-0.003 and t>1.2: phase="TRANSIT"; events.append((round(t,1),"TRANSIT"))
        elif phase=="TRANSIT":
            href=CRUISE
            if spd<0.04:
                Vcmd=Vc; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
                eyaw=((nose_f+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*I_y,-0.3,0.3); I_y=np.clip(I_y+eyaw*fly.dt,-1.5,1.5)
            else:
                Vcmd=Vc*np.clip(dist/0.12,0.2,1.0)
                step=np.clip(((bearing-pref+np.pi)%(2*np.pi)-np.pi),-PSIDOT*fly.dt,PSIDOT*fly.dt); pref+=step; pdot=step/fly.dt
                eb=((pref-psiv_f+np.pi)%(2*np.pi)-np.pi)
                roll_ref=np.clip(pdot/KFF+(KP_BANK*eb+KD_BANK*(pdot-vrate))/KFB,-np.radians(1.2),np.radians(1.2))
                eyaw=((nose_f-psiv_f+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.15*eyaw+0.02*nrate+0.15*I_y,-0.3,0.3); I_y=np.clip(I_y+eyaw*fly.dt,-1.5,1.5)
            if dist<CAPTURE: phase="ARRIVE"; hold_head=nose_f; t_arrive=t; events.append((round(t,1),"ARRIVE"))
        elif phase=="ARRIVE":
            href=CRUISE; Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            eyaw=((nose_f-hold_head+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*I_y,-0.3,0.3); I_y=np.clip(I_y+eyaw*fly.dt,-1.5,1.5)
            if spd<0.02 and t>t_arrive+1.5: phase="LAND"; events.append((round(t,1),"LAND"))
        else:
            Vcmd=0.0; roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            eyaw=((nose_f-hold_head+np.pi)%(2*np.pi)-np.pi); uy=np.clip(0.10*eyaw+0.02*nrate+0.12*I_y,-0.3,0.3); I_y=np.clip(I_y+eyaw*fly.dt,-1.5,1.5)
            closeness=np.clip((-thr_f-0.012)/0.020,0.0,1.0) if thr_f is not None else 0.0
            href=max(href-DESC_RATE*(1.0-0.8*closeness)*fly.dt,0.003)
            if closeness>0.5 and abs(s['vz'])<0.02: thrust_extra=-0.012*closeness
        ctrl.h_ref=href
        e=Vcmd-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*I_s,-np.radians(8),np.radians(8))
        u=ctrl.update(s,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        thr_f=u[0] if thr_f is None else thr_f+(u[0]-thr_f)*fly.dt/0.15
        kin.set_control(thrust=u[0]+thrust_extra,roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy); fly.step(kin,t)
        beta=(np.degrees(psi-psi_vel)+180)%360-180 if spd>0.03 else 0.0
        log.append([t,x*1e3,y*1e3,z*1e3,np.degrees(psi),beta,spd*1e3,dist*1e3,pmap[phase]])
        if save_states and i%cap==0: QP.append(fly.data.qpos.copy()); PH.append(pmap[phase])
        i+=1; t+=fly.dt
        if phase=="LAND" and z<0.012 and abs(s['vz'])<0.01 and t>events[-1][0]+1.0: break
    L=np.array(log)
    if save_states:
        np.savez("outputs/_mission_states.npz", qp=np.array(QP), ph=np.array(PH), goal=GOAL*1e3)
    return L, events

if __name__=="__main__":
    L,events=run()
    print("events:", events)
    tr=(L[:,8]==1)&(L[:,6]>40)
    print(f"transit sideslip mean {np.mean(np.abs(L[tr,5])):.1f} peak {np.max(np.abs(L[tr,5])):.1f} deg")
    print(f"final pos ({L[-1,1]:.0f},{L[-1,2]:.0f}) mm, dist_to_goal {L[-1,7]:.1f} mm, z {L[-1,3]:.1f} mm")
    np.savez("outputs/e30_mission.npz", L=L)
    print("saved states for render -> outputs/_mission_states.npz")
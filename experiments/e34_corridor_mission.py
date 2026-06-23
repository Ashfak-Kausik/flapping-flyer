"""
e34 — Integrated WINDING-CORRIDOR mission (the narrow-space demo).

TAKEOFF -> for each corridor segment: thread it holding the segment heading while
CENTERING on that segment's two walls (ctrl.roll_dist, e33) -> STOP-TURN-GO pivot at
the junction (e31; centering off, hover-hold in the open corner) -> next segment ...
-> LAND at the exit. One body-frame unified controller throughout (e29 + e31).

Honest framing: a KNOWN winding route (forward sensing is blind, so corners are
triggered by position on the known map), threaded by proximity wall-centering and
sharp stop-turn-go corners. Not a maze solver — a confined-route navigator that plays
to the flyer's real sensing strength.

Default route is a 3-segment staircase. Runs end-to-end on a normal machine; the
sandbox is slower, so validate_short() exercises the same logic in less sim time.
Saves outputs/_corridor_states.npz (qpos per frame + wall geometry) for rendering.
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design
from experiments.e31_corner import bodyframe
from experiments.e33_centering import corridor_walls

Vc=0.07; Kc=1.5e-5; Kd=3.0; KLAT=3.0; ROLL_FF=175.0/10399.0; YAWRATE=0.7
PAD=0.006; CRUISE=0.05; W=0.060; DESC_RATE=0.014; FLOOR=dict(axis=2,sign=1,pos=0.0)

# Route as (heading_deg, length_m): straight, 90 LEFT, 90 RIGHT, 40, 10, >90 U-turn, land.
# Walls are tall (WALL_H) in the render so the flyer threads BETWEEN them at CRUISE=50mm.
WALL_H=0.080
def _route(legs):
    wp=[np.array([0.0,0.0])]
    for h,L in legs: wp.append(wp[-1]+L*np.array([np.cos(np.radians(h)),np.sin(np.radians(h))]))
    return dict(HEAD=[h for h,_ in legs], SEGSTART=[wp[i] for i in range(len(legs))],
                TARGET=[wp[i+1] for i in range(len(legs))])
FULL=_route([(0,0.10),(90,0.09),(0,0.09),(40,0.08),(50,0.08),(185,0.09)])
SHORT=_route([(0,0.06),(90,0.06)])


def run(route=FULL, save_states=True, tmax=40.0):
    HEAD, SEGSTART, TARGET = route['HEAD'], route['SEGSTART'], route['TARGET']
    fly=Flyer(Path("models/flyer.xml"))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    ctrl.reset(); fly.reset(kin=kin,height=PAD); ctrl.h_ref=PAD
    t=0.0; I_s=0.0; href=PAD; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; thr_f=None
    phase="TAKEOFF"; seg=0; log=[]; events=[]; QP=[]; PH=[]; SG=[]; cap=int(0.02/fly.dt); i=0
    segdev={k:[] for k in range(len(HEAD))}
    def latdev(x,y,k):
        h=np.radians(HEAD[k]); lat=np.array([-np.sin(h),np.cos(h)]); return (np.array([x,y])-np.array(SEGSTART[k]))@lat
    pm={"TAKEOFF":0,"CRUISE":1,"PIVOT":2,"LAND":3}
    while t<tmax:
        s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
        b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
        nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(nose_f-nose_prev)/fly.dt; nose_prev=nose_f
        rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*fly.dt/0.10; walls=None; thrust_extra=0.0
        if phase=="TAKEOFF":
            href=min(href+0.025*fly.dt,CRUISE); Vcmd=0.0; pref=0.0
            roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            if z>=CRUISE-0.003 and t>1.0: phase="CRUISE"; events.append((round(t,1),"seg0",round(x*1e3),round(y*1e3)))
        elif phase=="CRUISE":
            head=np.radians(HEAD[seg]); walls=corridor_walls(SEGSTART[seg],head); pref=head; href=CRUISE
            T=TARGET[seg]; dist=np.hypot(T[0]-x,T[1]-y); Vcmd=Vc*np.clip(dist/0.08,0.0,1.0)
            roll_ref=np.clip(-Kc*rd_f-Kd*v_lat,-np.radians(2.5),np.radians(2.5))
            segdev[seg].append(latdev(x,y,seg)*1e3)
            if dist<0.020 and spd<0.02:
                if seg<len(TARGET)-1: phase="PIVOT"; I_y=0.0; events.append((round(t,1),f"corner{seg}",round(x*1e3),round(y*1e3)))
                else: phase="LAND"; events.append((round(t,1),"land",round(x*1e3),round(y*1e3)))
        elif phase=="PIVOT":
            Vcmd=0.0; nxt=np.radians(HEAD[seg+1]); href=CRUISE
            pref=pref+np.clip(((nxt-pref+np.pi)%(2*np.pi)-np.pi),-YAWRATE*fly.dt,YAWRATE*fly.dt)
            roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            if abs(((nose_f-nxt+np.pi)%(2*np.pi)-np.pi))<np.radians(2) and abs(nrate)<0.25:
                seg+=1; phase="CRUISE"; I_s=0.0; I_y=0.0; rd_f=0.0
        else:
            Vcmd=0.0; pref=np.radians(HEAD[seg]); roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
            closeness=np.clip((-thr_f-0.012)/0.020,0.0,1.0) if thr_f is not None else 0.0
            href=max(href-DESC_RATE*(1.0-0.8*closeness)*fly.dt,0.003)
            if closeness>0.5 and abs(s['vz'])<0.02: thrust_extra=-0.012*closeness
        ctrl.h_ref=href
        e=Vcmd-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*I_s,-np.radians(8),np.radians(8))
        eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*fly.dt,-1.0,1.0)
        uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
        u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        thr_f=u[0] if thr_f is None else thr_f+(u[0]-thr_f)*fly.dt/0.15
        kin.set_control(thrust=u[0]+thrust_extra,roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(([] if walls is None else list(walls))+[FLOOR]))
        log.append([t,x*1e3,y*1e3,z*1e3,np.degrees(psi),spd*1e3,seg,pm[phase]])
        if save_states and i%cap==0: QP.append(fly.data.qpos.copy()); PH.append(pm[phase]); SG.append(seg)
        i+=1; t+=fly.dt
        if phase=="LAND" and z<0.012 and abs(s['vz'])<0.01 and t>events[-1][0]+1.0: break
    L=np.array(log)
    if save_states:
        np.savez("outputs/_corridor_states.npz", qp=np.array(QP), ph=np.array(PH), sg=np.array(SG),
                 segstart=np.array(SEGSTART), head=np.array(HEAD), target=np.array(TARGET), W=W,
                 start=np.array(SEGSTART[0]), exit=np.array(TARGET[-1]))
    return L, events, segdev, route


if __name__ == "__main__":
    import os
    route = SHORT if os.environ.get("SHORT") else FULL
    L, events, segdev, route = run(route=route)
    print("events:", events)
    for k in range(len(route['HEAD'])):
        d=np.array(segdev[k])
        if len(d): print(f"  seg{k} heading {route['HEAD'][k]:.0f}: centering mean|dev| {np.mean(np.abs(d)):.1f} max {np.max(np.abs(d)):.1f} mm")
    print(f"final ({L[-1,1]:.0f},{L[-1,2]:.0f}) mm z {L[-1,3]:.1f} mm; exit ({int(route['TARGET'][-1][0]*1e3)},{int(route['TARGET'][-1][1]*1e3)})")
    np.savez("outputs/e34_corridor_mission.npz", L=L)
    print("saved render states -> outputs/_corridor_states.npz")
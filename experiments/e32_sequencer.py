"""
e32 — Segment sequencer: fly a KNOWN winding route of corridor segments joined by
90 deg corners, then land at the exit.

route = headings[i] (segment directions) + junctions[i] (corner positions) + exit.
Per segment: hold the segment heading, cruise, decelerate to a stop at the (known)
junction -> stop-turn-go pivot to the next heading -> accelerate into the next segment.
Forward sensing is blind, so cornering is triggered by POSITION on a known route
(honest framing: navigate a known confined route, not discover/solve a maze). Lateral
centering (added with the walled scene) keeps it off the side walls during each cruise.
All on the body-frame unified controller (e29 + e31 bodyframe feed).
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design
from experiments.e31_corner import bodyframe

Vc=0.07; KLAT=3.0; ROLL_FF=175.0/10399.0; YAWRATE=0.6; CRUISE=0.05; DESC_RATE=0.014
FLOOR=dict(axis=2,sign=1,pos=0.0)


def run(headings=(0.0,90.0,0.0), junct=((0.20,0.0),(0.20,0.15)), exit=(0.35,0.15)):
    fly=Flyer(Path("models/flyer.xml"))
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4))
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0
    ctrl.reset(); fly.reset(kin=kin,height=CRUISE); ctrl.h_ref=CRUISE
    t=0.0; I_s=0.0; pref=np.radians(headings[0]); nose_f=pref; nose_prev=pref; I_y=0.0; thr_f=None
    seg=0; state="CRUISE"; href=CRUISE; log=[]; events=[]
    tgt=lambda: np.array(junct[seg]) if seg<len(junct) else np.array(exit)
    while t<30.0:
        s=fly.sense(); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
        b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
        nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*fly.dt/0.04; nrate=(nose_f-nose_prev)/fly.dt; nose_prev=nose_f
        T=tgt(); dist=np.hypot(T[0]-x,T[1]-y); head=np.radians(headings[seg]); thrust_extra=0.0
        if state=="CRUISE":
            Vcmd=Vc*np.clip(dist/0.08,0.0,1.0); pref=head; href=CRUISE
            if dist<0.020 and spd<0.02:
                if seg<len(junct): state="PIVOT"; I_y=0.0; events.append((round(t,1),f"corner{seg}",round(x*1e3),round(y*1e3)))
                else: state="LAND"; events.append((round(t,1),"arrive+land",round(x*1e3),round(y*1e3)))
        elif state=="PIVOT":
            Vcmd=0.0; nxt=np.radians(headings[seg+1])
            pref=pref+np.clip(((nxt-pref+np.pi)%(2*np.pi)-np.pi),-YAWRATE*fly.dt,YAWRATE*fly.dt); href=CRUISE
            if abs(((nose_f-nxt+np.pi)%(2*np.pi)-np.pi))<np.radians(2) and abs(nrate)<0.25:
                seg+=1; state="CRUISE"; I_s=0.0; I_y=0.0
        else:
            Vcmd=0.0; pref=head
            closeness=np.clip((-thr_f-0.012)/0.020,0.0,1.0) if thr_f is not None else 0.0
            href=max(href-DESC_RATE*(1.0-0.8*closeness)*fly.dt,0.003)
            if closeness>0.5 and abs(s['vz'])<0.02: thrust_extra=-0.012*closeness
        ctrl.h_ref=href
        e=Vcmd-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.6,0.6); pr=np.clip(0.30*e+1.1*I_s,-np.radians(8),np.radians(8))
        roll_ref=np.clip(-KLAT*v_lat,-np.radians(2),np.radians(2))
        eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*fly.dt,-1.0,1.0)
        uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
        u=ctrl.update(b,fly.dt,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
        thr_f=u[0] if thr_f is None else thr_f+(u[0]-thr_f)*fly.dt/0.15
        kin.set_control(thrust=u[0]+thrust_extra,roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy); fly.step(kin,t)
        log.append([t,x*1e3,y*1e3,z*1e3,np.degrees(psi),spd*1e3,seg,{"CRUISE":0,"PIVOT":1,"LAND":2}[state]]); t+=fly.dt
        if state=="LAND" and z<0.012 and abs(s['vz'])<0.01 and t>events[-1][0]+1.0: break
    return np.array(log), events, np.array(junct)*1e3, np.array(exit)*1e3


def make_figure(L, junct, exit, path="outputs/e32_sequencer.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    x,y,ph=L[:,1],L[:,2],L[:,7]
    fig,ax=plt.subplots(figsize=(8,6)); fig.patch.set_facecolor('white')
    cols={0:'#1d6fb8',1:'#c0392b',2:'#127a3d'}; labs={0:'cruise',1:'pivot',2:'land'}
    for p in [0,1,2]:
        seg=L[ph==p]
        if len(seg): ax.plot(seg[:,1],seg[:,2],'.',ms=2.5,color=cols[p],label=labs[p])
    ax.plot(x[0],y[0],'ko',ms=8,label='start')
    for j in junct: ax.plot(j[0],j[1],'s',color='#e08a1e',ms=10,mfc='none',mew=2)
    ax.plot(exit[0],exit[1],'*',color='#127a3d',ms=18,label='exit')
    ax.set_aspect('equal'); ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)'); ax.legend(fontsize=9)
    ax.set_title('Segment sequencer: known winding route, two 90 deg corners, land'); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path,dpi=110); print("saved ->",path)


if __name__ == "__main__":
    L,events,junct,exit=run()
    print("events:", events)
    print(f"final ({L[-1,1]:.0f},{L[-1,2]:.0f}) mm z {L[-1,3]:.1f} mm; segments completed {int(L[-1,6])}")
    np.savez("outputs/e32_sequencer.npz", L=L, junct=junct, exit=exit); make_figure(L, junct, exit)
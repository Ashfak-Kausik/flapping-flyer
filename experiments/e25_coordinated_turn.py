"""e25 — coordinated turn via body/heading-frame reformulation.
Key idea: feed the controller BODY-frame velocity (u_fwd, v_lat) instead of world
(vx,vy). Its lateral loop then regulates SIDESLIP->0 and banks exactly as needed;
a feedforward roll = atan(u*psi_dot/g) cancels the frame-rotation term it can't see.
Compare reformulated vs naive (world-frame feed, no FF) for a 90 deg turn."""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0,'.')
from src.flyer import Flyer
from src.controller import design
g=9.81; Vc=0.07; PSIDOT=0.3; TURN=np.radians(90); ROLL_FF=175.0/10399.0
fly=Flyer(Path("models/flyer.xml"))
ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,
                     Q=(150,150,20,2,2,250,250,6e4))
ctrl.K[:,0]=0.0   # decouple forward only (cruise pitch loop); keep lateral loop intact


def run(mode):   # mode: 'reform' or 'naive'
    ctrl.reset(); fly.reset(kin=kin,height=0.05)
    psi_f=psi_prev=pref=I_y=I_s=0.0; t=0.0; log=[]
    t_turn_end=2.0+TURN/PSIDOT
    while t < t_turn_end+2.5:
        s=fly.sense()
        psi_f += (s['yaw']-psi_f)*fly.dt/0.04; rate=(psi_f-psi_prev)/fly.dt; psi_prev=psi_f
        pdot = PSIDOT if (2.0<=t<t_turn_end) else 0.0
        pref += pdot*fly.dt
        I_y=np.clip(I_y+(psi_f-pref)*fly.dt,-1.5,1.5)
        uy=np.clip(0.08*(psi_f-pref)+0.02*rate+0.10*I_y,-0.3,0.3)
        u_fwd= s['vx']*np.cos(psi_f)+s['vy']*np.sin(psi_f)
        v_lat=-s['vx']*np.sin(psi_f)+s['vy']*np.cos(psi_f)
        e=Vc-u_fwd; I_s=np.clip(I_s+e*fly.dt,-0.5,0.5); pr=np.clip(0.25*e+0.9*I_s,-np.radians(8),np.radians(8))
        if mode=='reform':
            s2=dict(s); s2['vx']=u_fwd; s2['vy']=v_lat            # feed body-frame velocity
            roll_ff=np.arctan(u_fwd*pdot/g)                       # cancel frame-rotation term
            u=ctrl.update(s2,fly.dt,pitch_ref=pr,vy_ref=0.0,roll_ref=roll_ff)
        else:                                                     # naive: world-frame feed, no FF
            u=ctrl.update(s,fly.dt,pitch_ref=pr,vy_ref=0.0,roll_ref=0.0)
        kin.set_control(thrust=u[0],roll=u[1]-ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t)
        beta=np.degrees(np.arctan2(v_lat,max(u_fwd,1e-6)))
        log.append([t,fly.x_com[0]*1e3,fly.x_com[1]*1e3,fly.x_com[2]*1e3,
                    np.degrees(s['yaw']),np.degrees(s['roll']),beta,u_fwd,v_lat])
        t+=fly.dt
    return np.array(log)


RE=run('reform'); NA=run('naive')
def stats(L,name):
    m=(L[:,0]>=2.0)&(L[:,0]<=2.0+TURN/PSIDOT)
    print(f"{name:11s} peak|sideslip|={np.max(np.abs(L[m,6])):5.1f}  mean|sideslip|={np.mean(np.abs(L[m,6])):5.1f}  "
          f"final head={L[-1,4]:6.1f}  final|sideslip|={abs(L[-1,6]):4.1f}  roll[{L[:,5].min():6.1f},{L[:,5].max():5.1f}]  dz={L[-1,3]-L[0,3]:+5.1f}mm")
print(f"cmd: 90deg @ {PSIDOT}rad/s, cruise {Vc}m/s; theory FF bank={np.degrees(np.arctan(Vc*PSIDOT/g)):.2f}deg, radius={Vc/PSIDOT*1e3:.0f}mm")
stats(RE,"reformed"); stats(NA,"naive")
np.savez("outputs/_e25.npz", re=RE, na=NA, meta=[Vc,PSIDOT,np.degrees(TURN)])
print("saved")
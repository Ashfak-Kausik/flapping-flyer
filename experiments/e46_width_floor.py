"""
e46 — how NARROW a corridor can this airframe fly? (the honest width floor)

Wingtip reach is WINGREACH=13.3mm from CoM, so a corridor of width W has walls at +/-W/2 and
the CENTERED clearance is W/2 - WINGREACH -> zero at W = 2*WINGREACH = 26.6mm. Below that the
corridor is physically narrower than the flyer. The FLYABLE floor sits above that: how narrow
before imperfect centering lets a wingtip touch. Narrower walls also make the wing-wash
centering STRONGER (walls closer -> more kappa), which helps hold center -- so the floor is a
real, measured tradeoff, not just the geometric bound.

Sweeps corridor width, flies a straight corridor at each, and reports completion + min wing
clearance + peak lateral excursion. The floor = narrowest width that completes with clearance>0.
Run at level=0 (best case) and with noise (realistic, tighter).

Run:  python experiments/e46_width_floor.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e36_reactive_course as e
import experiments.e39_branching as b39
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna
from src.safety import Clearance, WINGREACH
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe

CRUISE=e.CRUISE; CONTROL_RATE=1000

def geometry(W, L):
    hw=W/2.0
    walls=[[np.array([0.0, hw]),np.array([L, hw])],
           [np.array([0.0,-hw]),np.array([L,-hw])]]
    return walls, np.array([L,0.0])

def build_model(W, L, path="models/_width.xml"):
    xml=open("models/flyer.xml").read(); walls,fin=geometry(W,L); bx=[]
    for w in walls: bx+=b39.blocks(w)
    g="".join(f'    <geom name="bw{i}" type="box" pos="{cx:.4f} {cy:.4f} {e.WALL_H/2:.4f}" '
              f'size="{hs:.4f} {hs:.4f} {e.WALL_H/2:.4f}" group="3" rgba="0.62 0.66 0.72 1" '
              f'contype="0" conaffinity="0"/>\n' for i,(cx,cy,hs) in enumerate(bx))
    g+=(f'    <geom name="fin0" type="cylinder" pos="{fin[0]:.4f} {fin[1]:.4f} 0.001" '
        f'size="{e.FIN_PAD:.4f} 0.001" rgba="0.2 0.85 0.35 0.85" group="2" contype="0" conaffinity="0"/>\n')
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path

def run(W, level=0.0, seed=0, y0=0.0, L=0.25, rate=CONTROL_RATE, tmax=None, adaptive_safe=True):
    hw=W/2.0; walls,fin=geometry(W,L); nm=NoiseModel(level,seed); fly=Flyer(build_model(W,L))
    SB = min(e.SAFE_BUF, 0.55*hw) if adaptive_safe else e.SAFE_BUF   # width-aware veer trigger
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,
                         Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/rate)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly,n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=CRUISE,y=y0); ctrl.h_ref=CRUISE
    N=max(1,round((1.0/rate)/fly.dt)); dt_c=N*fly.dt
    t=0.0; stepi=0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0
    minc=1e3; struck=False; reached=None; ymax=0.0; strike_bound=hw-WINGREACH
    tmax=(L/e.Vc*2.6+8 if tmax is None else tmax); floor=dict(axis=2,sign=1,pos=0.0); pl=[]
    while t<tmax:
        if stepi%N==0:
            s=nm.sense(fly.sense()); psi=s['yaw']; x,y,z=fly.x_com; b=bodyframe(s); v_lat=b['vy']
            nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*dt_c/0.04; nrate=(nose_f-nose_prev)/dt_c; nose_prev=nose_f
            f0,f50p,f50m,dL,dR=nm.feel(ant.feel([0,50,-50,90,-90])); fwd=f0
            rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*dt_c/0.10; pl=e.planes(psi,fly.x_com,dL,dR)
            ln=min(f50p,dL); rn=min(f50m,dR); safe=0.0
            if rn<SB: safe+=e.KVEER*(SB-rn)/SB
            if ln<SB: safe-=e.KVEER*(SB-ln)/SB
            slow=0.35 if min(ln,rn)<SB else 1.0
            Vcmd=e.Vc*np.clip((fwd-e.STOP)/0.04,0.0,1.0)*slow
            roll_ref=np.clip(-e.Kc*rd_f-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
            pref+=safe*dt_c
            er=Vcmd-b['vx']; I_s=np.clip(I_s+er*dt_c,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
            eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*dt_c,-1.0,1.0)
            uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
            u=ctrl.update(b,dt_c,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1]-e.ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[floor])); x,y,z=fly.x_com
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            ymax=max(ymax,abs(y))
            if abs(y)>strike_bound: struck=True          # wingtip reaches a side wall (geometric)
        if np.hypot(fin[0]-x,fin[1]-y)<e.FIN_ZONE: reached=True; break
        if abs(y)>hw+0.02 or z<0.0 or z>0.2: break
        t+=fly.dt; stepi+=1
    margin_mm=(strike_bound-ymax)*1e3                     # + = wingtip never reached the wall
    return dict(W_mm=W*1e3, level=level, seed=seed, reached=bool(reached), struck=struck,
                margin_mm=margin_mm, ymax_mm=ymax*1e3, strike_bound_mm=strike_bound*1e3)

def sweep(widths_mm=(64,56,48,42,38,34,32,30,28), levels=(0.0,1.0), seeds=(1,), L=0.25, adaptive_safe=True):
    tag="adaptive-veer" if adaptive_safe else "stock-veer"
    print(f"WIDTH FLOOR sweep [{tag}] (straight corridor L={L*1e3:.0f}mm, 1kHz); physical floor {2*WINGREACH*1e3:.1f}mm")
    print(" width | level | reach | strike | peak|y|(mm) | strike bound(mm) | margin(mm)")
    rows=[]
    for W in widths_mm:
        for lv in levels:
            rc=0; st=0; ym=[]; mg=[]
            for sd in seeds:
                r=run(W/1e3, level=lv, seed=sd, L=L, adaptive_safe=adaptive_safe)
                rc+=r['reached']; st+=r['struck']; ym.append(r['ymax_mm']); mg.append(r['margin_mm'])
                rows.append(r)
            print(f"  {W:4.0f}  |  {lv:.1f}  |  {rc}/{len(seeds)} |  {st}/{len(seeds)}  |  {np.max(ym):5.1f}     |  {W/2.0-WINGREACH*1e3:5.1f}          |  {np.min(mg):+5.1f}")
    return rows

def make_figure(rows, path="outputs/e46_width_floor.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    W=np.array([r['W_mm'] for r in rows]); ym=np.array([r['ymax_mm'] for r in rows])
    ok=np.array([r['reached'] and not r['struck'] for r in rows])
    fig,ax=plt.subplots(figsize=(7,4.5))
    ax.scatter(W[ok],ym[ok],c="#0d9488",label="completed, no strike",zorder=3)
    ax.scatter(W[~ok],ym[~ok],c="#dc2626",marker='x',label="strike/fail",zorder=3)
    Wl=np.linspace(W.min(),W.max(),50); ax.plot(Wl,Wl/2-WINGREACH*1e3,'--',color="k",lw=1,label="strike bound (hw−WINGREACH)")
    ax.axvline(2*WINGREACH*1e3,color='gray',ls=':',lw=1,label=f"physical floor {2*WINGREACH*1e3:.1f}mm")
    ax.set_xlabel("corridor width (mm)"); ax.set_ylabel("peak lateral deviation |y| (mm)")
    ax.set_title("width floor: lateral deviation vs corridor width"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)

def save_csv(r_stock, r_adapt, path="outputs/e46_width_floor.csv"):
    import csv
    with open(path,"w",newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["policy","W_mm","level","seed","reached","struck","margin_mm","ymax_mm","strike_bound_mm"])
        for tag,rows in [("stock",r_stock),("adaptive",r_adapt)]:
            for r in rows:
                w.writerow([tag,r['W_mm'],r['level'],r['seed'],int(r['reached']),int(r['struck']),
                            r['margin_mm'],r['ymax_mm'],r['strike_bound_mm']])
        w.writerow([])
        w.writerow(["policy","level","flyable_floor_mm"])
        for tag,rows in [("stock",r_stock),("adaptive",r_adapt)]:
            levels=sorted(set(r['level'] for r in rows))
            for lv in levels:
                ok=[r for r in rows if r['level']==lv and r['reached'] and not r['struck']]
                w.writerow([tag,lv,min(r['W_mm'] for r in ok) if ok else ""])
    print("saved ->",path)

if __name__=="__main__":
    print("========== STOCK safety heuristic (SAFE_BUF fixed) ==========")
    r_stock=sweep(levels=(0.0,), adaptive_safe=False)
    print("\n========== ADAPTIVE safety heuristic (width-aware veer) ==========")
    r_adapt=sweep(levels=(0.0,1.0), adaptive_safe=True)
    make_figure(r_adapt)
    save_csv(r_stock, r_adapt)
    for tag,rows in [("stock",r_stock),("adaptive",r_adapt)]:
        ok=[r for r in rows if r['reached'] and not r['struck']]
        if ok: print(f"FLYABLE FLOOR [{tag}]: {min(r['W_mm'] for r in ok):.0f}mm   (geometric bound {2*WINGREACH*1e3:.1f}mm)")
        for lv in sorted(set(r['level'] for r in rows)):
            ok_lv=[r for r in rows if r['level']==lv and r['reached'] and not r['struck']]
            if ok_lv: print(f"  [{tag}] level={lv}: floor {min(r['W_mm'] for r in ok_lv):.0f}mm")
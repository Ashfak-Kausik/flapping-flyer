"""
e39 — BRANCHING (phase 2): an asymmetric Y-fork. A straight approach opens into two
branches at UNEQUAL angles (upper +22 deg, lower -34 deg) separated by a central wedge
island; a finish pad sits at the end of each branch so the flyer lands wherever it commits.

The question on trial: when the corridor opens at the fork, the wing-wash centering
(roll_dist) goes *quiet* (both walls recede -> authority ~0), handing the transition to
the antenna feeler-steering, which already turns toward the more-open side. Does the
EXISTING reactive controller commit to a branch cleanly and re-center, and is the wedge-tip
clearance safe?  Operating point matches phase 1: 1 kHz discrete controller, wall+floor aero.

Run:  python experiments/e39_branching.py        (layout -> outputs/e39_layout.png,
                                                   trajectory -> outputs/e39_branching.png)
"""
import sys; sys.path.insert(0, '.'); import numpy as np
import experiments.e36_reactive_course as e
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna, BIG
from src.safety import Clearance, WINGREACH
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe

S=e.SCALE; hw=e.Wd/2.0                       # corridor half-width (fixed; only lengths scale)
XF=0.060*S; L=0.085*S                        # approach length; branch length
WALL_H=e.WALL_H; CRUISE=e.CRUISE; FIN_ZONE=e.FIN_ZONE
SAFE_BUF=e.SAFE_BUF; KVEER=e.KVEER; CONTROL_RATE=1000

def configure(tu_deg=22.0, tl_deg=34.0, bo=0.0):
    """Build fork geometry for upper/lower branch angles (deg). Symmetric Y = equal angles."""
    global TU,TL,uu,ul,F,Ue,Le,n_uo,n_ui,n_li,n_lo,TIP,BO,Au,Al,UPPER_OUTER,LOWER_OUTER,WEDGE,WALLS,FIN
    TU=np.radians(tu_deg); TL=np.radians(tl_deg); BO=bo
    uu=np.array([np.cos(TU), np.sin(TU)]); ul=np.array([np.cos(TL), -np.sin(TL)])
    F=np.array([XF,0.0]); Ue=F+L*uu; Le=F+L*ul
    n_uo=np.array([-np.sin(TU), np.cos(TU)]); n_ui=np.array([ np.sin(TU),-np.cos(TU)])
    n_li=np.array([ np.sin(TL), np.cos(TL)]); n_lo=np.array([-np.sin(TL),-np.cos(TL)])
    a=np.array([[uu[0],-ul[0]],[uu[1],-ul[1]]]); sui=F+hw*n_ui; sli=F+hw*n_li
    t,_=np.linalg.solve(a, sli-sui); TIP=sui+t*uu          # apex of the central island
    Au=TIP+BO*uu; Al=TIP+BO*ul                             # blunt-nose endpoints (BO=0 -> razor tip)
    UPPER_OUTER=[np.array([0.0,hw]),  np.array([XF,hw]),  Ue+hw*n_uo]
    LOWER_OUTER=[np.array([0.0,-hw]), np.array([XF,-hw]), Le+hw*n_lo]
    WEDGE      =[Ue+hw*n_ui, Au, Al, Le+hw*n_li]           # island: upper-inner -> nose -> lower-inner
    WALLS=[UPPER_OUTER, LOWER_OUTER, WEDGE]; FIN=[Ue, Le]  # two finish pads (upper, lower)

configure(22.0, 34.0)                                      # default: asymmetric Y (tuned operating point)

def blocks(poly, step=0.007, hs=0.005):
    out=[]
    for i in range(len(poly)-1):
        a,b=poly[i],poly[i+1]; v=b-a; ln=np.linalg.norm(v)
        if ln<1e-6: continue
        n=max(1,int(np.ceil(ln/step)))
        for k in range(n+1):
            p=a+(k/n)*v; out.append((float(p[0]),float(p[1]),hs))
    return out

def build_model(path="models/_branching_course.xml"):
    xml=open("models/flyer.xml").read(); bx=[]
    for w in WALLS: bx+=blocks(w)
    g="".join(f'    <geom name="bw{i}" type="box" pos="{cx:.4f} {cy:.4f} {WALL_H/2:.4f}" '
              f'size="{hs:.4f} {hs:.4f} {WALL_H/2:.4f}" group="3" rgba="0.62 0.66 0.72 1" '
              f'contype="0" conaffinity="0"/>\n' for i,(cx,cy,hs) in enumerate(bx))
    for j,(fx,fy) in enumerate(FIN):
        g+=(f'    <geom name="fin{j}" type="cylinder" pos="{fx:.4f} {fy:.4f} 0.001" '
            f'size="{e.FIN_PAD:.4f} 0.001" rgba="0.2 0.85 0.35 0.85" group="2" '
            f'contype="0" conaffinity="0"/>\n')
    key='<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
    open(path,"w").write(xml.replace(key,key+g)); return path

def fork_timeout(): return (XF+L)/e.Vc*2.5 + 14.0

def run(level=0.0, seed=0, rate=CONTROL_RATE, rec=True):
    nm=NoiseModel(level,seed); fly=Flyer(build_model())
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,
                         Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/rate)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly,n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=CRUISE); ctrl.h_ref=CRUISE
    N=max(1,round((1.0/rate)/fly.dt)); dt_c=N*fly.dt
    t=0.0; stepi=0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; state="CRUISE"; tdir=0; turn0=None
    minc=1e3; mloc=(0.,0.); crashed=False; reached=None; tmax=fork_timeout(); floor=dict(axis=2,sign=1,pos=0.0); pl=[]; traj=[]
    flips=0; last_bias=0.0
    while t<tmax:
        if stepi%N==0:
            s=nm.sense(fly.sense()); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
            b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
            nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*dt_c/0.04; nrate=(nose_f-nose_prev)/dt_c; nose_prev=nose_f
            f0,fLp,fRp,f50p,f50m,dL,dR=nm.feel(ant.feel([0,30,-30,50,-50,90,-90])); fwd,fL,fR=f0,fLp,fRp
            rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*dt_c/0.10; pl=e.planes(psi,fly.x_com,dL,dR)
            left_near=min(f50p,dL); right_near=min(f50m,dR); safe=0.0
            if right_near<SAFE_BUF: safe+=KVEER*(SAFE_BUF-right_near)/SAFE_BUF
            if left_near <SAFE_BUF: safe-=KVEER*(SAFE_BUF-left_near)/SAFE_BUF
            slow=0.35 if min(left_near,right_near)<SAFE_BUF else 1.0
            if state=="CRUISE":
                diagopen=max(fLp,fRp,f50p,f50m)                  # most-open off-axis (branch) feeler
                if fwd<0.14 and diagopen>e.CLEAR:                # FORK: blocked ahead but a branch open off-axis
                    bias=1.0 if max(fLp,f50p)>=max(fRp,f50m) else -1.0   # commit toward the more-open side
                    if last_bias!=0.0 and bias!=last_bias: flips+=1      # dithering diagnostic: commit-side reversals
                    last_bias=bias
                    pref+=(bias*e.YAWRATE+safe)*dt_c            # turn toward the open branch
                    Vcmd=e.Vc*np.clip((diagopen-e.STOP)/0.04,0.0,1.0)*0.45*slow   # ease through the junction
                    roll_ref=np.clip(bias*np.radians(4.5)-e.Kd*v_lat,-np.radians(5.0),np.radians(5.0))  # hug OUTER wall hard, clear the tip
                else:                                            # normal corridor: speed/steer off the forward feeler
                    Vcmd=e.Vc*np.clip((fwd-e.STOP)/0.04,0.0,1.0)*slow
                    pref+=(np.clip(e.Ksteer*(min(fL,e.FMAX)-min(fR,e.FMAX)),-0.5,0.5)+safe)*dt_c
                    roll_ref=np.clip(-e.Kc*rd_f-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                if fwd<e.STOP and diagopen<e.STOP and spd<0.03: tdir=+1 if fL>=fR else -1; state="TURN"; turn0=nose_f; I_y=0.0
            else:
                Vcmd=0.0; pref+=tdir*e.YAWRATE*dt_c; roll_ref=np.clip(-e.KLAT*v_lat,-np.radians(2),np.radians(2))
                turned=abs(((nose_f-turn0+np.pi)%(2*np.pi)-np.pi))
                if fwd>e.CLEAR and turned>np.radians(20): state="CRUISE"; I_s=0.0; rd_f=0.0
                elif turned>np.radians(175): tdir=-tdir; turn0=nose_f
            er=Vcmd-u_fwd; I_s=np.clip(I_s+er*dt_c,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
            eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*dt_c,-1.0,1.0)
            uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
            u=ctrl.update(b,dt_c,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1]-e.ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[floor])); x,y,z=fly.x_com
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            c=clr.min_clearance()
            if c<minc: minc=c; mloc=(float(x),float(y))
            if c<WINGREACH: crashed=True
            if rec: traj.append((x,y))
        du=np.hypot(Ue[0]-x,Ue[1]-y); dl=np.hypot(Le[0]-x,Le[1]-y)
        if min(du,dl)<FIN_ZONE: reached=('upper' if du<dl else 'lower'); break
        if z<0.0 or z>0.2: break
        t+=fly.dt; stepi+=1
    return dict(reached=reached, min_clear_mm=minc*1e3, mloc_mm=(mloc[0]*1e3,mloc[1]*1e3), crashed=crashed, t=t, end=(float(x),float(y)), traj=traj, flips=flips)

def layout_plot(traj=None, path="outputs/e39_layout.png", title="e39 asymmetric-Y fork"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(7.5,6)); allpts=[]
    for w in WALLS:
        for (cx,cy,hs) in blocks(w):
            ax.add_patch(plt.Rectangle((cx-hs,cy-hs),2*hs,2*hs,color="#9aa4b0",lw=0)); allpts.append((cx,cy))
    for (fx,fy) in FIN: ax.add_patch(plt.Circle((fx,fy),e.FIN_PAD,color="#2fbf52",alpha=0.65)); allpts.append((fx,fy))
    ax.plot([0],[0],'ko',ms=6,label="start")
    ax.plot([TIP[0]],[TIP[1]],'b^',ms=8,label="wedge tip")
    if traj: tj=np.array(traj); ax.plot(tj[:,0],tj[:,1],'r-',lw=1.8,label="flight")
    ap=np.array(allpts); ax.set_xlim(ap[:,0].min()-0.03,ap[:,0].max()+0.03); ax.set_ylim(ap[:,1].min()-0.03,ap[:,1].max()+0.03)
    ax.set_aspect('equal'); ax.set_title(title); ax.legend(fontsize=9,loc="upper left"); ax.grid(alpha=0.2)
    fig.savefig(path,dpi=120,bbox_inches='tight'); print("saved ->",path)

def sweep(levels=(0.0,0.5,1.0,1.5,2.0,3.0), seeds=(1,2,3)):
    """Robustness: does noise flip the branch choice, and does the wedge-tip margin survive?"""
    rows=[]
    print(f"branching robustness sweep (SCALE={S}, 1kHz, aero-on); levels {list(levels)} x {len(seeds)} seeds")
    for lv in levels:
        cl=[]; pick={'upper':0,'lower':0,'none':0}; cr=0
        for sd in seeds:
            r=run(level=lv, seed=sd, rec=False)
            key=r['reached'] if r['reached'] else 'none'; pick[key]+=1
            if r['crashed']: cr+=1
            if r['reached']: cl.append(r['min_clear_mm'])
        mc=float(np.mean(cl)) if cl else float('nan'); sc=float(np.std(cl)) if cl else 0.0
        comp=pick['upper']+pick['lower']
        rows.append((lv,mc,sc,comp,len(seeds),cr,pick['upper'],pick['lower']))
        print(f"  level {lv:>3}: clear {mc:5.1f}+-{sc:3.1f}mm (completed)  reach {comp}/{len(seeds)}  "
              f"crash {cr}/{len(seeds)}  branch[U/L]={pick['upper']}/{pick['lower']}")
    print(" level | clear(mm,completed) | reach | crash | branch U/L")
    for lv,mc,sc,comp,n,cr,pu,pl in rows:
        print(f"  {lv:>4} | {mc:5.1f} +- {sc:3.1f}        | {comp}/{n}   |  {cr}/{n}  |  {pu}/{pl}")
    return rows

def make_figure(rows, path="outputs/e39_branching_sweep.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    lv=[r[0] for r in rows]; mc=[r[1] for r in rows]; sc=[r[2] for r in rows]
    comp=[r[3]/r[4]*100 for r in rows]; up=[r[6] for r in rows]; lo=[r[7] for r in rows]
    fig,ax=plt.subplots(1,2,figsize=(11,4.2))
    ax[0].errorbar(lv,mc,yerr=sc,marker='o',capsize=3,color="#2563eb")
    ax[0].axhline(WINGREACH*1e3,ls='--',color='r',label=f"wing-strike {WINGREACH*1e3:.1f}mm")
    ax[0].set_xlabel("noise level (x realistic)"); ax[0].set_ylabel("wedge-tip clearance (mm, completed)")
    ax[0].set_title("fork clearance vs noise"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[1].plot(lv,comp,marker='s',color="#059669",label="completion %")
    ax[1].set_xlabel("noise level (x realistic)"); ax[1].set_ylabel("completion %"); ax[1].set_ylim(-5,105)
    ax[1].set_title("fork completion & branch choice"); ax[1].grid(alpha=0.3)
    for x,u,l in zip(lv,up,lo): ax[1].annotate(f"U{u}/L{l}",(x,8),fontsize=7,ha='center',color='#444')
    ax[1].legend(fontsize=8); fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)

def tiebreak_test(tu=28.0, tl=28.0, level=1.0, seeds=range(1,11)):
    """Symmetric-fork probe: with no geometric bias, does sensor noise split the branch choice
    (a clean stochastic 50/50) or make it dither at the wedge (Buridan)? flips = commit reversals."""
    configure(tu,tl); seeds=list(seeds)
    U=Lo=NONE=cr=0; cl=[]; fl=[]
    print(f"symmetric-Y tie-break ({tu:.0f}/{tl:.0f} deg), level {level}, {len(seeds)} seeds")
    for sd in seeds:
        r=run(level=level, seed=sd, rec=False); k=r['reached']
        if k=='upper': U+=1
        elif k=='lower': Lo+=1
        else: NONE+=1
        if r['crashed']: cr+=1
        if k: cl.append(r['min_clear_mm'])
        fl.append(r['flips'])
        print(f"  seed{sd:>2}: {str(k):5}  clear {r['min_clear_mm']:5.1f}mm  flips {r['flips']:>2}  {'CRASH' if r['crashed'] else ''}", flush=True)
    mc=float(np.mean(cl)) if cl else float('nan')
    print(f"  => upper {U} / lower {Lo} / none {NONE}   crash {cr}/{len(seeds)}   mean clear {mc:.1f}mm   mean flips {np.mean(fl):.1f} (max {max(fl)})")
    return dict(upper=U,lower=Lo,none=NONE,crash=cr,mean_clear=mc,flips=fl)

if __name__=="__main__":
    import sys
    if len(sys.argv)>1 and sys.argv[1].startswith("sym"):
        tiebreak_test(28.0,28.0, level=1.0, seeds=range(1,11))
    else:
        configure(22.0,34.0)
        print(f"branching fork (asymmetric Y) on SCALE={S}: approach {XF*1e3:.0f}mm, branches {L*1e3:.0f}mm")
        print(f"  upper finish {np.round(Ue*1e3).astype(int)}mm  lower finish {np.round(Le*1e3).astype(int)}mm  wedge tip {np.round(TIP*1e3).astype(int)}mm")
        r=run(level=0.0, seed=0)
        print(f"  nominal: branch {r['reached']}   min clearance {r['min_clear_mm']:.1f}mm @({r['mloc_mm'][0]:.0f},{r['mloc_mm'][1]:.0f})mm   "
              f"{'CRASH' if r['crashed'] else 'ok'}   t={r['t']:.1f}s")
        layout_plot(r['traj'], path="outputs/e39_branching.png",
                    title=f"e39 fork: took {r['reached']} branch, min clear {r['min_clear_mm']:.1f}mm")
        rows=sweep(); make_figure(rows)
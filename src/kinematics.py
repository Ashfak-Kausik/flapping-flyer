import numpy as np
class FlapKinematics:
    def __init__(self, f_hz, stroke_amp_deg=60.0, feather_amp_deg=45.0, rot_phase=0.0,
                 feather_sign=-1.0, k_splitcycle=1.0):
        self.W=2*np.pi*f_hz; self.PHI=np.deg2rad(stroke_amp_deg); self.PSI=np.deg2rad(feather_amp_deg)
        self.d=rot_phase; self.fs=feather_sign; self.K_SC=k_splitcycle
        self.u_thrust=0.0; self.u_roll=0.0; self.u_pitch=0.0; self.u_yaw=0.0
    def set_control(self,thrust=0.0,roll=0.0,pitch=0.0,yaw=0.0):
        self.u_thrust,self.u_roll,self.u_pitch,self.u_yaw=thrust,roll,pitch,yaw
    def _amp(self,wing): s=1.0 if wing=="R" else -1.0; return self.PHI*(1.0+self.u_thrust+s*self.u_roll)
    def _offset(self,wing): s=1.0 if wing=="R" else -1.0; return s*self.u_pitch
    def signals(self,t,wing):
        mirror=1.0 if wing=="R" else -1.0; A=self._amp(wing); off=self._offset(wing)
        wt=self.W*t
        sc=self.K_SC*self.u_yaw*mirror              # split-cycle, opposite per wing -> yaw couple
        xi=wt+sc*np.sin(wt); dxidt=self.W*(1.0+sc*np.cos(wt))
        stroke=mirror*A*np.cos(xi)+off
        dstroke=-mirror*A*np.sin(xi)*dxidt
        a=wt+self.d; pitch=self.fs*self.PSI*np.tanh(3*np.sin(a))
        dpitch=self.fs*self.PSI*3*self.W*np.cos(a)*(1-np.tanh(3*np.sin(a))**2)
        return stroke,dstroke,pitch,dpitch
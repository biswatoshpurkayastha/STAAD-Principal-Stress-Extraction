# =============================================================================
# STAAD.Pro Interactive Beam Principal-Stress Explorer
# Features retained: magnitude-scaled ticks, thin flow-net, no endpoint dots,
# unified navigation, section view, and hover numerical values.
#
# FIX: The "tau_max (max shear)" contour is now computed on a FILLED elevation
# plane through the depth, combining bending AND transverse shear at every
# interior point:  tau_max = sqrt((sigma_x/2)^2 + tau_xy^2).
# This gives the physically-correct max-shear field, so it no longer collapses
# to |sigma|/2 on the surface and no longer dies to zero at the supports.
# =============================================================================
import sys, ctypes, math
import numpy as np
import comtypes.client
from comtypes import automation
from PySide6 import QtWidgets, QtCore, QtGui
import pyvista as pv
from pyvistaqt import QtInteractor
try:
    import vtk
    HAVE_VTK = True
except Exception:
    vtk = None; HAVE_VTK = False

INCLUDE_TORSION=False; GRID_N=46; SWEEP_STATIONS=41; BOUNDARY_N=72
TRAJ_NX=160; TRAJ_NY=26; TICK_FRAC=.95; TICK_WIDTH=1; FORCE_STATIONS=121
SEED_MAJOR=18; SEED_MINOR=18; RK_STEP_FRAC=.004; RK_MAX_STEPS=6000
NET_WIDTH=1; NET_OPACITY=.88; HIDE_CONTOUR=True; FACE_ON_VIEW=True
BEAM_OPACITY=.12; SHEAR_SIGN=-1.0; HOVER_PICK_TOLERANCE=.008
MAXSHEAR_ELEV_NX=161; MAXSHEAR_ELEV_NT=61       # resolution of the filled elevation plane
TAUMAX_NAME='tau_max (max shear)'               # stress option that gets the filled fix
VONMISES_NAME='von Mises'                        # von Mises also gets the filled fix
FILLED_MODES=(TAUMAX_NAME,VONMISES_NAME)         # modes computed on the filled elevation plane

if HAVE_VTK:
    class MultiNavStyle(vtk.vtkInteractorStyleUser):
        def __init__(self, plotter=None):
            super().__init__(); self.plotter=plotter; self.box_style=None; self.mode=None; self.lx=self.ly=0
            self.AddObserver('LeftButtonPressEvent',self.left_down); self.AddObserver('LeftButtonReleaseEvent',self.button_up)
            self.AddObserver('MiddleButtonPressEvent',self.middle_down); self.AddObserver('MiddleButtonReleaseEvent',self.button_up)
            self.AddObserver('RightButtonPressEvent',self.right_down); self.AddObserver('RightButtonReleaseEvent',self.button_up)
            self.AddObserver('MouseMoveEvent',self.mouse_move); self.AddObserver('MouseWheelForwardEvent',self.wheel_in)
            self.AddObserver('MouseWheelBackwardEvent',self.wheel_out); self.AddObserver('CharEvent',self.char_event)
        def renderer_camera(self):
            i=self.GetInteractor(); x,y=i.GetEventPosition(); r=i.FindPokedRenderer(x,y)
            return (r,r.GetActiveCamera()) if r else (None,None)
        def left_down(self,o,e):
            i=self.GetInteractor(); self.mode='rotate' if i.GetControlKey() else 'zoom' if i.GetShiftKey() else 'pan'; self.lx,self.ly=i.GetEventPosition()
        def middle_down(self,o,e): self.mode='pan'; self.lx,self.ly=self.GetInteractor().GetEventPosition()
        def right_down(self,o,e): self.mode='zoom'; self.lx,self.ly=self.GetInteractor().GetEventPosition()
        def button_up(self,o,e): self.mode=None
        def mouse_move(self,o,e):
            if not self.mode:return
            i=self.GetInteractor(); x,y=i.GetEventPosition(); dx,dy=x-self.lx,y-self.ly; r,c=self.renderer_camera()
            if c:
                if self.mode=='pan': self.pan(r,c,dx,dy)
                elif self.mode=='rotate': c.Azimuth(-.5*dx); c.Elevation(.5*dy); c.OrthogonalizeViewUp(); r.ResetCameraClippingRange()
                else:self.zoom(r,c,dy)
                i.Render()
            self.lx,self.ly=x,y
        @staticmethod
        def pan(r,c,dx,dy):
            fp=np.asarray(c.GetFocalPoint(),float); cp=np.asarray(c.GetPosition(),float); r.SetWorldPoint(*fp,1.); r.WorldToDisplay(); d=r.GetDisplayPoint()
            r.SetDisplayPoint(d[0]-dx,d[1]-dy,d[2]); r.DisplayToWorld(); w=r.GetWorldPoint()
            if abs(w[3])<1e-15:return
            m=np.asarray(w[:3],float)/w[3]-fp; c.SetFocalPoint(*(fp+m)); c.SetPosition(*(cp+m)); r.ResetCameraClippingRange()
        @staticmethod
        def zoom(r,c,dy):
            f=max(.01,1+.005*dy); c.SetParallelScale(c.GetParallelScale()/f) if c.GetParallelProjection() else c.Dolly(f); r.ResetCameraClippingRange()
        def zoom_step(self,f):
            r,c=self.renderer_camera()
            if not c:return
            c.SetParallelScale(c.GetParallelScale()/f) if c.GetParallelProjection() else c.Dolly(f); r.ResetCameraClippingRange(); self.GetInteractor().Render()
        def wheel_in(self,o,e):self.zoom_step(1.15)
        def wheel_out(self,o,e):self.zoom_step(1/1.15)
        def char_event(self,o,e):
            if (self.GetInteractor().GetKeySym() or '').lower()=='b':
                i=self.GetInteractor(); rb=vtk.vtkInteractorStyleRubberBandZoom(); rb.AddObserver('LeftButtonReleaseEvent',lambda o,e:i.SetInteractorStyle(self)); self.box_style=rb; i.SetInteractorStyle(rb)

def sa_double(n):return automation._midlSAFEARRAY(ctypes.c_double).create([0.]*n)
def sa_long(n):return automation._midlSAFEARRAY(ctypes.c_long).create([0]*n)
def var_array_ref(a,vt):
    v=automation.VARIANT(); v._.c_void_p=ctypes.addressof(a); v.vt=automation.VT_ARRAY|vt|automation.VT_BYREF; return v
def double_ref():
    x=ctypes.c_double(0.); v=automation.VARIANT(); v._.c_void_p=ctypes.addressof(x); v.vt=automation.VT_R8|automation.VT_BYREF; return v,x

class Staad:
    def __init__(self):self.os=comtypes.client.GetActiveObject('StaadPro.OpenSTAAD')
    def selected_beams(self):
        try:
            g=self.os.Geometry; g._FlagAsMethod('GetNoOfSelectedBeams','GetSelectedBeams'); n=int(g.GetNoOfSelectedBeams())
            if n<=0:return []
            a=sa_long(n); g.GetSelectedBeams(var_array_ref(a,automation.VT_I4),0); return [int(a[0][i]) for i in range(n)]
        except Exception:return []
    def beam_length(self,b):g=self.os.Geometry; g._FlagAsMethod('GetBeamLength'); return float(g.GetBeamLength(int(b)))
    def section_props(self,b):
        p=self.os.Property; p._FlagAsMethod('GetBeamProperty'); vW,W=double_ref(); vD,D=double_ref(); vA,A=double_ref(); vAy,Ay=double_ref(); vAz,Az=double_ref(); vIx,Ix=double_ref(); vIy,Iy=double_ref(); vIz,Iz=double_ref()
        p.GetBeamProperty(int(b),vW,vD,vA,vAy,vAz,vIx,vIy,vIz); return {'W':W.value,'D':D.value,'Ax':A.value,'Ay':Ay.value,'Az':Az.value,'Ix':Ix.value,'Iy':Iy.value,'Iz':Iz.value}
    def forces_at(self,b,x,lc):
        o=self.os.Output; o._FlagAsMethod('GetIntermediateMemberForcesAtDistance'); a=sa_double(6); o.GetIntermediateMemberForcesAtDistance(int(b),float(x),int(lc),var_array_ref(a,automation.VT_R8)); return [float(a[0][i]) for i in range(6)]
    def end_forces(self,b,e,lc):
        o=self.os.Output; o._FlagAsMethod('GetMemberEndForces'); a=sa_double(6); o.GetMemberEndForces(int(b),int(e),int(lc),var_array_ref(a,automation.VT_R8),0); return [float(a[0][i]) for i in range(6)]
    def safe_forces_at(self,b,x,L,lc):
        try:return self.forces_at(b,x,lc)
        except Exception:return self.end_forces(b,0 if x<L/2 else 1,lc)
    def section_name(self,b):
        try:p=self.os.Property
        except Exception:return ''
        for m in ('GetBeamSectionName','GetBeamSectionDisplayName','GetSectionPropertyName','GetProfileName','GetBeamSectionProfile'):
            try:p._FlagAsMethod(m); v=getattr(p,m)(int(b)); return str(v).strip() if v else ''
            except Exception:pass
        return ''
    def detect_shape(self,b,p):
        n=self.section_name(b).upper()
        if any(k in n for k in ('PIPE','CIRC','ROUND','ROD','CHS','O/D','OD ')):return 'Circle / Pipe'
        if any(k in n for k in ('TUBE','BOX','RECT','SHS','RHS','HSS')):return 'Rectangle'
        W,D,A=p['W'],p['D'],p['Ax']
        if W>0 and D>0 and A>0:
            f=A/(W*D); sq=abs(W-D)/max(W,D)<.05
            if sq and f<.86:return 'Circle / Pipe'
            if f>=.90:return 'Rectangle'
            if not sq and f<.55:return 'I-Section'
        return None

def section_mask(s,W,D,Y,Z):
    if s=='Circle / Pipe':return Y*Y+Z*Z<=(D/2)**2
    if s=='I-Section':
        tf,tw=.12*D,.12*W; return ((np.abs(Y)>=D/2-tf)&(np.abs(Z)<=W/2))|((np.abs(Z)<=tw/2)&(np.abs(Y)<=D/2))
    return (np.abs(Y)<=D/2)&(np.abs(Z)<=W/2)
def section_grid_points(s,W,D,n=GRID_N):
    Z,Y=np.meshgrid(np.linspace(-W/2,W/2,n),np.linspace(-D/2,D/2,n)); m=section_mask(s,W,D,Y,Z); return Y[m].ravel(),Z[m].ravel()
def section_boundary(s,W,D,n=BOUNDARY_N):
    if s=='Circle / Pipe':a=np.linspace(0,2*np.pi,n); return D/2*np.sin(a),D/2*np.cos(a)
    if s=='I-Section':
        tf,tw,hy,hz=.12*D,.12*W,D/2,W/2; a=np.array([(-hy,-hz),(-hy,hz),(-hy+tf,hz),(-hy+tf,tw/2),(hy-tf,tw/2),(hy-tf,hz),(hy,hz),(hy,-hz),(hy-tf,-hz),(hy-tf,-tw/2),(-hy+tf,-tw/2),(-hy+tf,-hz),(-hy,-hz)],float); return a[:,0],a[:,1]
    hy,hz=D/2,W/2; return np.array([-hy,-hy,hy,hy,-hy]),np.array([-hz,hz,hz,-hz,-hz])

def shear_shape_factor(s):
    # peak transverse shear = k * V/A  (k=1.5 rect, 4/3 solid circle)
    if s=='Circle / Pipe':return 4.0/3.0
    return 1.5

def transverse_shear_profile(s,W,D,Y,Z,use_xy):
    # normalized through-depth shape of VQ/(I b), 1.0 at neutral axis -> 0 at fibers
    if use_xy:
        h0=max(D/2,1e-9); base=np.clip(1-(Y/h0)**2,0,1)
    else:
        h0=max(W/2,1e-9); base=np.clip(1-(Z/h0)**2,0,1)
    if s=='I-Section':
        tw=.12*W; tf=.12*D
        if use_xy:
            web=(np.abs(Z)<=tw/2); base=np.where(web,base,base*0.06)
        else:
            web=(np.abs(Y)<=(D/2-tf)); base=np.where(web,base,base*0.06)
    return base

def stress_field(f,p,Y,Z,m):
    Fx,Fy,Fz,Mx,My,Mz=f; W,D=p['W'],p['D']; A=max(p['Ax'],1e-12); Iy=max(p['Iy'],1e-12); Iz=max(p['Iz'],1e-12)
    sig=Fx/A+Mz*Y/Iz-My*Z/Iy; ty=1.5*Fy/A*np.clip(1-(Y/max(D/2,1e-9))**2,0,1); tz=1.5*Fz/A*np.clip(1-(Z/max(W/2,1e-9))**2,0,1); tau=np.hypot(ty,tz); c=sig/2; r=np.hypot(c,tau)
    if m=='S1 (max principal)':return c+r
    if m=='S2 (min principal)':return c-r
    if m=='tau_max (max shear)':return r
    return np.sqrt(sig*sig+3*tau*tau)
def principal_2d(sig,tau):c=.5*float(sig); r=math.hypot(c,float(tau)); return c+r,c-r,.5*math.atan2(2*float(tau),float(sig))

class ElevationField:
    def __init__(self,s,b,lc,L,p):
        self.props,self.L=p,float(L); q=[s.safe_forces_at(b,x,L,lc) for x in np.linspace(0,L,9)]; self.use_xy=sum(abs(f[1]) for f in q)>=sum(abs(f[2]) for f in q); self.h=max(float(p['D'] if self.use_xy else p['W']),1e-9); self.xs=np.linspace(0,L,FORCE_STATIONS); self.F=np.array([s.safe_forces_at(b,x,L,lc) for x in self.xs],float)
    @property
    def plane_name(self):return 'Local X-Y' if self.use_xy else 'Local X-Z'
    def forces(self,x):x=np.clip(x,0,self.L); return np.array([np.interp(x,self.xs,self.F[:,i]) for i in range(6)])
    def stress_state(self,x,t):
        p=self.props; A=max(p['Ax'],1e-12); Iy=max(p['Iy'],1e-12); Iz=max(p['Iz'],1e-12); Fx,Fy,Fz,Mx,My,Mz=self.forces(x)
        if self.use_xy:
            h=max(p['D']/2,1e-9); return Fx/A+Mz*t/Iz,SHEAR_SIGN*1.5*Fy/A*max(0,1-(t/h)**2)
        h=max(p['W']/2,1e-9); return Fx/A-My*t/Iy,SHEAR_SIGN*1.5*Fz/A*max(0,1-(t/h)**2)
    def principal_state(self,x,t):return principal_2d(*self.stress_state(x,t))
    def dir_major(self,x,t):a=self.principal_state(x,t)[2]; return math.cos(a),math.sin(a)
    def dir_minor(self,x,t):a=self.principal_state(x,t)[2]; return -math.sin(a),math.cos(a)
    def dir_shear1(self,x,t):a=self.principal_state(x,t)[2]+math.pi/4; return math.cos(a),math.sin(a)   # +45deg to principal
    def dir_shear2(self,x,t):a=self.principal_state(x,t)[2]-math.pi/4; return math.cos(a),math.sin(a)   # -45deg (conjugate plane)
    def tau_max(self,x,t):s1,s2,_=self.principal_state(x,t); return .5*(s1-s2)                          # Mohr radius = max shear

def integrate_trajectory(field,x0,t0,major,step,max_steps):
    L,h=field.L,field.h; lo,hi=-h/2,h/2; step=abs(step)
    def raw(x,t):
        d=field.dir_major(x,t) if major else field.dir_minor(x,t); n=math.hypot(*d); return None if n<1e-15 else (d[0]/n,d[1]/n)
    def oriented(x,t,s,ref):
        d=raw(x,t)
        if d is None:return None
        d=(s*d[0],s*d[1])
        if ref is not None and d[0]*ref[0]+d[1]*ref[1]<0:d=(-d[0],-d[1])
        return d
    def march(s):
        pts=[]; x,t,prev=float(x0),float(t0),None
        for _ in range(max_steps):
            if not(0<=x<=L and lo<=t<=hi):break
            pts.append((x,t)); k1=oriented(x,t,s,prev)
            if k1 is None:break
            k2=oriented(x+step*k1[0]/2,t+step*k1[1]/2,s,k1)
            if k2 is None:break
            k3=oriented(x+step*k2[0]/2,t+step*k2[1]/2,s,k2)
            if k3 is None:break
            k4=oriented(x+step*k3[0],t+step*k3[1],s,k3)
            if k4 is None:break
            dx=step*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6; dt=step*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6; q=math.hypot(dx,dt)
            if q<1e-15:break
            prev=(dx/q,dt/q); x,t=x+dx,t+dt
        return pts
    f,b=march(1),march(-1); b.reverse(); return b[:-1]+f if b and f else b or f

def no_verts(poly):
    if poly is not None:
        try:poly.verts=np.empty(0,dtype=np.int64)
        except Exception:pass
    return poly
def make_tick_segments(c,d,L):
    c,d,L=np.asarray(c,float),np.asarray(d,float),np.asarray(L,float); n=np.linalg.norm(d,axis=1); v=np.isfinite(c).all(1)&np.isfinite(d).all(1)&np.isfinite(L)&(n>1e-15)&(L>1e-15)
    if not np.any(v):return None
    c,d,L,n=c[v],d[v],L[v,None],n[v,None]; u=d/n; a,b=c-L*u/2,c+L*u/2; N=len(c); p=pv.PolyData(np.vstack((a,b))); p.lines=np.column_stack((np.full(N,2),np.arange(N),np.arange(N)+N)).ravel(); return no_verts(p)
def make_polyline(points):
    if len(points)<2:return None
    p=pv.PolyData(np.asarray(points,float)); p.lines=np.array([len(points),*range(len(points))],np.int64); return no_verts(p)
def make_loop(points):
    p=pv.PolyData(np.asarray(points,float)); p.lines=np.array([len(points)+1,*range(len(points)),0],np.int64); return no_verts(p)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('STAAD Beam Principal-Stress and Crack-Trajectory Explorer'); self.resize(1440,820)
        self.staad=self.beam=self.props=None; self.L=0.; self.lc=1; self.slice_actor=None; self.traj_actors=[]; self.first_view=True; self.qt_mode=self.qt_pos=self.nav_style=None
        self.hover_field=None; self.hover_picker=vtk.vtkCellPicker() if HAVE_VTK else None; self.hover_families={}; self.hover_timer=QtCore.QElapsedTimer(); self.hover_timer.start()
        if self.hover_picker:self.hover_picker.SetTolerance(HOVER_PICK_TOLERANCE)
        c=QtWidgets.QWidget(); self.setCentralWidget(c); root=QtWidgets.QVBoxLayout(c); views=QtWidgets.QHBoxLayout(); root.addLayout(views,1); self.plot_beam=QtInteractor(c); self.plot_sec=QtInteractor(c); views.addWidget(self.plot_beam,3); views.addWidget(self.plot_sec,2); self.plot_beam.set_background('white'); self.plot_sec.set_background('white'); self.plot_beam.installEventFilter(self); self.plot_beam.setMouseTracking(True)
        try:self.plot_beam.enable_anti_aliasing('msaa')
        except Exception:pass
        ctl=QtWidgets.QHBoxLayout(); root.addLayout(ctl); self.spin_beam=QtWidgets.QSpinBox(); self.spin_beam.setRange(1,10_000_000); self.spin_lc=QtWidgets.QSpinBox(); self.spin_lc.setRange(1,100000); self.spin_lc.setValue(1); self.btn_load=QtWidgets.QPushButton('Load Beam'); self.btn_load.clicked.connect(self.load); self.btn_sel=QtWidgets.QPushButton('Use GUI Selection'); self.btn_sel.clicked.connect(self.use_selection); self.combo_shape=QtWidgets.QComboBox(); self.combo_shape.addItems(['Rectangle','Circle / Pipe','I-Section']); self.combo_stress=QtWidgets.QComboBox(); self.combo_stress.addItems(['S1 (max principal)','S2 (min principal)','tau_max (max shear)','von Mises']); self.chk_traj=QtWidgets.QCheckBox('Show principal trajectories'); self.combo_mode=QtWidgets.QComboBox(); self.combo_mode.addItems(['Crosses (ticks)','Flow-net (curves)','Max-shear crosses (45deg)']); self.spin_cols=QtWidgets.QSpinBox(); self.spin_cols.setRange(5,1000); self.spin_cols.setValue(TRAJ_NX); self.spin_rows=QtWidgets.QSpinBox(); self.spin_rows.setRange(3,200); self.spin_rows.setValue(TRAJ_NY); self.spin_tick=QtWidgets.QSpinBox(); self.spin_tick.setRange(10,100); self.spin_tick.setValue(round(TICK_FRAC*100)); self.spin_size=QtWidgets.QDoubleSpinBox(); self.spin_size.setRange(.2,10); self.spin_size.setValue(10); self.spin_size.setSingleStep(.2); self.spin_width=QtWidgets.QSpinBox(); self.spin_width.setRange(1,12); self.spin_width.setValue(TICK_WIDTH); self.chk_scale=QtWidgets.QCheckBox('Length proportional to |stress|'); self.chk_scale.setChecked(True); self.chk_scale.setEnabled(False); self.combo_nav=QtWidgets.QComboBox(); self.combo_nav.addItems(['Unified (L-pan, Ctrl-rotate, Shift-zoom, b=box)','Box-zoom (drag)','Rotate (3D)']); self.btn_reset=QtWidgets.QPushButton('Reset View'); self.btn_reset.clicked.connect(self.reset_view)
        for label,w in (('Member:',self.spin_beam),('Load case:',self.spin_lc),(None,self.btn_load),(None,self.btn_sel),('Section:',self.combo_shape),('Stress:',self.combo_stress),(None,self.chk_traj),('Mode:',self.combo_mode),('Cols:',self.spin_cols),('Rows:',self.spin_rows),('Tick%:',self.spin_tick),('Size x:',self.spin_size),('Width:',self.spin_width),(None,self.chk_scale),('Nav:',self.combo_nav),(None,self.btn_reset)):
            if label:ctl.addWidget(QtWidgets.QLabel(label))
            ctl.addWidget(w)
        ctl.addStretch(1)
        for w in (self.combo_shape,self.combo_stress,self.combo_mode,self.spin_cols,self.spin_rows,self.spin_tick,self.spin_size,self.spin_width):w.currentIndexChanged.connect(self.refresh) if isinstance(w,QtWidgets.QComboBox) else w.valueChanged.connect(self.refresh)
        self.chk_traj.stateChanged.connect(self.toggle_trajectory); self.combo_nav.currentIndexChanged.connect(self.apply_nav); row=QtWidgets.QHBoxLayout(); root.addLayout(row); row.addWidget(QtWidgets.QLabel('Distance:')); self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(0,1000); self.slider.valueChanged.connect(self.slide); row.addWidget(self.slider,1); self.lbl_dist=QtWidgets.QLabel('0.000'); row.addWidget(self.lbl_dist); self.legend=QtWidgets.QLabel("<span style='color:#c00000'>RED = maximum principal direction, length proportional to |S1|</span> &nbsp;&nbsp; <span style='color:#0000c0'>BLUE = minimum principal direction, length proportional to |S2|</span>"); root.addWidget(self.legend); self.status=QtWidgets.QLabel('Connect STAAD, enter a member, then Load Beam.'); self.status.setStyleSheet('font-weight:bold'); root.addWidget(self.status); self.connect_staad()
    def actor_key(self,a):
        try:return a.GetAddressAsString('')
        except Exception:return str(id(a))
    def register_hover(self,a,f):self.hover_families[self.actor_key(a)]=f
    def show_hover(self,pos):
        if not(HAVE_VTK and self.hover_picker and self.hover_field and self.chk_traj.isChecked()) or self.qt_mode:return
        if self.hover_timer.elapsed()<25:return
        self.hover_timer.restart(); dpr=self.plot_beam.devicePixelRatioF(); x=int(pos.x()*dpr); y=int((self.plot_beam.height()-pos.y())*dpr); r=self.plot_beam.renderer
        if not self.hover_picker.Pick(x,y,0,r):QtWidgets.QToolTip.hideText(); return
        a=self.hover_picker.GetActor(); fam=self.hover_families.get(self.actor_key(a))
        if fam not in ('major','minor','shear1','shear2'):QtWidgets.QToolTip.hideText(); return
        q=np.asarray(self.hover_picker.GetPickPosition(),float); f=self.hover_field; xx=float(np.clip(q[0],0,f.L)); tt=float(np.clip(q[1] if f.use_xy else q[2],-f.h/2,f.h/2)); sig,tau=f.stress_state(xx,tt); s1,s2,ang=f.principal_state(xx,tt); taumax=.5*(s1-s2); off={'major':0.,'minor':90.,'shear1':45.,'shear2':-45.}.get(fam,0.); deg=math.degrees(ang)+off
        while deg>90:deg-=180
        while deg<=-90:deg+=180
        coord='Local y' if f.use_xy else 'Local z'
        if fam=='major':name='Maximum principal';val=s1
        elif fam=='minor':name='Minimum principal';val=s2
        elif fam=='shear1':name='Max-shear plane (+45 deg)';val=taumax
        else:name='Max-shear plane (-45 deg)';val=taumax
        text=f'Member: {self.beam}\nPlane: {f.plane_name}\nDirection: {name}\nx = {xx:.6g}\n{coord} = {tt:.6g}\nNormal stress = {sig:.6g}\nShear stress = {tau:.6g}\nS1 = {s1:.6g}\nS2 = {s2:.6g}\nMax shear (tau_max) = {taumax:.6g}\nSelected value = {val:.6g}\nDirection angle = {deg:.3f} deg'
        QtWidgets.QToolTip.showText(QtGui.QCursor.pos()+QtCore.QPoint(14,18),text,self.plot_beam)
    def eventFilter(self,w,e):
        if w is self.plot_beam:
            if e.type()==QtCore.QEvent.Type.MouseMove and not self.qt_mode:self.show_hover(e.position())
            if self.combo_nav.currentIndex()==0:
                if e.type()==QtCore.QEvent.Type.MouseButtonPress and e.button()==QtCore.Qt.MouseButton.LeftButton:
                    QtWidgets.QToolTip.hideText(); m=e.modifiers(); self.qt_mode='rotate' if m&QtCore.Qt.KeyboardModifier.ControlModifier else 'zoom' if m&QtCore.Qt.KeyboardModifier.ShiftModifier else 'pan'; self.qt_pos=e.position(); return True
                if e.type()==QtCore.QEvent.Type.MouseMove and self.qt_mode:
                    p=e.position(); dx,dy=p.x()-self.qt_pos.x(),p.y()-self.qt_pos.y(); r=self.plot_beam.renderer; c=r.GetActiveCamera()
                    if self.qt_mode=='pan':MultiNavStyle.pan(r,c,dx,-dy)
                    elif self.qt_mode=='rotate':c.Azimuth(-.5*dx); c.Elevation(.5*dy); c.OrthogonalizeViewUp()
                    else:
                        f=math.exp(-.01*dy); c.SetParallelScale(c.GetParallelScale()/f) if c.GetParallelProjection() else c.Dolly(f)
                    r.ResetCameraClippingRange(); self.qt_pos=p; self.plot_beam.render(); return True
                if e.type()==QtCore.QEvent.Type.MouseButtonRelease and e.button()==QtCore.Qt.MouseButton.LeftButton:self.qt_mode=self.qt_pos=None; return True
        return super().eventFilter(w,e)
    def connect_staad(self):
        try:self.staad=Staad(); s=self.staad.selected_beams(); self.spin_beam.setValue(s[0]) if s else None; self.status.setText('Connected to STAAD. Select or enter a member and click Load Beam.')
        except Exception as e:self.staad=None; self.status.setText(f'[!] Could not connect to STAAD: {e}')
    def use_selection(self):
        if not self.staad:self.connect_staad()
        s=self.staad.selected_beams() if self.staad else []
        if s:self.spin_beam.setValue(s[0]); self.load()
        else:self.status.setText('No beam is selected in STAAD.')
    def load(self):
        try:
            if not self.staad:self.connect_staad()
            self.beam,self.lc=self.spin_beam.value(),self.spin_lc.value(); self.L=self.staad.beam_length(self.beam); self.props=self.staad.section_props(self.beam); s=self.staad.detect_shape(self.beam,self.props)
            if s:self.combo_shape.blockSignals(True); self.combo_shape.setCurrentText(s); self.combo_shape.blockSignals(False)
            self.slider.setValue(0); self.first_view=True; self.refresh(); self.apply_nav()
        except Exception as e:self.status.setText(f'[!] Load failed: {e}')
    def toggle_trajectory(self):self.first_view=True; self.refresh()
    def slide(self):
        if self.L<=0:return
        x=self.slider.value()/1000*self.L; self.lbl_dist.setText(f'{x:.3f}'); self.update_section(x); self.slice(x)
    def refresh(self):
        if not self.props or self.L<=0:return
        QtWidgets.QToolTip.hideText(); self.build_beam(); self.update_section(self.slider.value()/1000*self.L)
    def build_beam(self):
        # FIX: correct max-shear AND von Mises -> filled elevation plane through the depth
        if self.combo_stress.currentText() in FILLED_MODES and not self.chk_traj.isChecked():
            self.build_filled_elevation(); return
        self.plot_beam.clear(); self.traj_actors=[]; self.slice_actor=None; self.hover_field=None; self.hover_families={}; s,m,p=self.combo_shape.currentText(),self.combo_stress.currentText(),self.props
        if not(self.chk_traj.isChecked() and HIDE_CONTOUR):
            y,z=section_boundary(s,p['W'],p['D']); xs=np.linspace(0,self.L,SWEEP_STATIONS); nb=len(y); X=np.zeros((nb,len(xs))); Y=X.copy(); Z=X.copy(); S=X.copy()
            for i,x in enumerate(xs):X[:,i]=x; Y[:,i]=y; Z[:,i]=z; S[:,i]=stress_field(self.staad.safe_forces_at(self.beam,x,self.L,self.lc),p,y,z,m)
            g=pv.StructuredGrid(X.reshape(nb,len(xs),1),Y.reshape(nb,len(xs),1),Z.reshape(nb,len(xs),1)); g[m]=S.ravel(order='F'); self.plot_beam.add_mesh(g,scalars=m,cmap='jet',opacity=BEAM_OPACITY if self.chk_traj.isChecked() else 1,show_scalar_bar=True)
        self.plot_beam.add_axes()
        if self.chk_traj.isChecked():
            _mode=self.combo_mode.currentText()
            if _mode.startswith('Flow'):self.add_flow()
            elif _mode.startswith('Max-shear'):self.add_shear_crosses()
            else:self.add_crosses()
        else:
            if self.first_view:self.plot_beam.view_isometric(); self.plot_beam.reset_camera(); self.first_view=False
            self.slice(0)
        self.plot_beam.render()
    def build_filled_elevation(self):
        # Correct max-shear AND von Mises: sweep a FILLED elevation plane through
        # the depth and combine bending AND transverse shear at every interior
        # point.  tau_max = sqrt((sigma_x/2)^2 + tau_xy^2);  von Mises =
        # sqrt(sigma_x^2 + 3*tau_xy^2).  Neither collapses on the surface, so
        # they do NOT die to zero at the supports (shear survives at neutral axis).
        mode=self.combo_stress.currentText()
        self.plot_beam.clear(); self.traj_actors=[]; self.slice_actor=None; self.hover_field=None; self.hover_families={}
        s,p=self.combo_shape.currentText(),self.props; A=max(p['Ax'],1e-12); Iy=max(p['Iy'],1e-12); Iz=max(p['Iz'],1e-12)
        q=[self.staad.safe_forces_at(self.beam,x,self.L,self.lc) for x in np.linspace(0,self.L,9)]
        use_xy=sum(abs(f[1]) for f in q)>=sum(abs(f[2]) for f in q)
        h=max(float(p['D'] if use_xy else p['W']),1e-9); h0=h/2
        nx,nt=MAXSHEAR_ELEV_NX,MAXSHEAR_ELEV_NT; xs=np.linspace(0,self.L,nx); ts=np.linspace(-h0,h0,nt)
        XX,TT=np.meshgrid(xs,ts)                                     # (nt, nx)
        F=np.array([self.staad.safe_forces_at(self.beam,x,self.L,self.lc) for x in xs],float)   # (nx,6)
        Fx=np.interp(XX.ravel(),xs,F[:,0]).reshape(XX.shape)
        if use_xy:
            V =np.interp(XX.ravel(),xs,F[:,1]).reshape(XX.shape)
            M =np.interp(XX.ravel(),xs,F[:,5]).reshape(XX.shape)
            sig=Fx/A + M*TT/Iz
        else:
            V =np.interp(XX.ravel(),xs,F[:,2]).reshape(XX.shape)
            M =np.interp(XX.ravel(),xs,F[:,4]).reshape(XX.shape)
            sig=Fx/A - M*TT/Iy
        k=shear_shape_factor(s)
        prof=transverse_shear_profile(s,p['W'],p['D'],TT,np.zeros_like(TT),True) if use_xy else transverse_shear_profile(s,p['W'],p['D'],np.zeros_like(TT),TT,False)
        tau=k*V/A*prof                                              # transverse shear through depth
        if mode==VONMISES_NAME:
            field=np.sqrt(sig*sig+3.0*tau*tau)                      # <-- correct von Mises
        else:
            field=np.sqrt((sig/2.0)**2+tau*tau)                     # <-- correct Mohr max shear
        if use_xy:X=XX.reshape(nt,nx,1); Y=TT.reshape(nt,nx,1); Z=np.zeros((nt,nx,1))
        else:X=XX.reshape(nt,nx,1); Y=np.zeros((nt,nx,1)); Z=TT.reshape(nt,nx,1)
        g=pv.StructuredGrid(X,Y,Z); g[mode]=field.ravel(order='F')
        self.plot_beam.add_mesh(g,scalars=mode,cmap='jet',show_scalar_bar=True)
        loop=[(0,-h0,0),(self.L,-h0,0),(self.L,h0,0),(0,h0,0)] if use_xy else [(0,0,-h0),(self.L,0,-h0),(self.L,0,h0),(0,0,h0)]
        self.plot_beam.add_mesh(make_loop(loop),color='black',line_width=2,pickable=False); self.plot_beam.add_axes()
        if self.first_view:self.plot_beam.view_xy() if use_xy else self.plot_beam.view_xz(); self.plot_beam.reset_camera(); self.first_view=False
        tag='von Mises' if mode==VONMISES_NAME else 'tau_max (Mohr)'
        self.plot_beam.render(); self.status.setText(f'Member {self.beam} | {tag} filled-depth plane={"Local X-Y" if use_xy else "Local X-Z"} | peak = {field.max():.6g}')
    def update_section(self,x):
        self.plot_sec.clear(); s,m,p=self.combo_shape.currentText(),self.combo_stress.currentText(),self.props; y,z=section_grid_points(s,p['W'],p['D']); v=stress_field(self.staad.safe_forces_at(self.beam,x,self.L,self.lc),p,y,z,m); c=pv.PolyData(np.c_[np.zeros_like(y),y,z]); c[m]=v
        try:self.plot_sec.add_mesh(c.delaunay_2d(),scalars=m,cmap='jet',show_scalar_bar=True)
        except Exception:self.plot_sec.add_mesh(c,scalars=m,cmap='jet',point_size=5)
        by,bz=section_boundary(s,p['W'],p['D']); self.plot_sec.add_mesh(make_loop(np.c_[np.zeros_like(by),by,bz]),color='black',line_width=2,pickable=False); self.plot_sec.view_yz(); self.plot_sec.render()
    def slice(self,x):
        if self.chk_traj.isChecked() or self.combo_stress.currentText() in FILLED_MODES:return
        if self.slice_actor:
            try:self.plot_beam.remove_actor(self.slice_actor)
            except Exception:pass
        p=self.props; b=pv.Cube(center=(x,0,0),x_length=max(.02*self.L,.15*p['D']),y_length=1.3*p['D'],z_length=1.3*p['W']); self.slice_actor=self.plot_beam.add_mesh(b,color='yellow',opacity=.3)
    @staticmethod
    def lift(x,t,xy):return (x,t,0) if xy else (x,0,t)
    def outline(self,f):
        h=f.h; loop=[(0,-h/2,0),(self.L,-h/2,0),(self.L,h/2,0),(0,h/2,0)] if f.use_xy else [(0,0,-h/2),(self.L,0,-h/2),(self.L,0,h/2),(0,0,h/2)]; self.traj_actors.append(self.plot_beam.add_mesh(make_loop(loop),color='black',line_width=2,pickable=False))
        if self.first_view:self.plot_beam.view_xy() if f.use_xy else self.plot_beam.view_xz(); self.plot_beam.reset_camera(); self.first_view=False
    def add_crosses(self):
        f=ElevationField(self.staad,self.beam,self.lc,self.L,self.props); self.hover_field=f; nx,ny=self.spin_cols.value(),self.spin_rows.value(); xs=np.linspace(0,self.L,nx); ts=np.linspace(-f.h/2,f.h/2,ny); ml=min(self.L/(nx-1),f.h/(ny-1))*self.spin_tick.value()/100*self.spin_size.value(); c=[]; d1=[]; d2=[]; s1=[]; s2=[]
        for x in xs:
            for t in ts:
                c.append(self.lift(x,t,f.use_xy)); a=f.dir_major(x,t); b=f.dir_minor(x,t); d1.append((a[0],a[1],0) if f.use_xy else (a[0],0,a[1])); d2.append((b[0],b[1],0) if f.use_xy else (b[0],0,b[1])); p1,p2,_=f.principal_state(x,t); s1.append(abs(p1)); s2.append(abs(p2))
        s1,s2=np.asarray(s1),np.asarray(s2); ref=max(np.max(s1),np.max(s2),1e-12)
        for poly,col,fam in ((make_tick_segments(c,d1,ml*s1/ref),'red','major'),(make_tick_segments(c,d2,ml*s2/ref),'blue','minor')):
            if poly is not None:a=self.plot_beam.add_mesh(poly,color=col,line_width=self.spin_width.value(),pickable=True); self.traj_actors.append(a); self.register_hover(a,fam)
        self.legend.setText("<span style='color:#c00000'>RED = maximum principal direction, length proportional to |S1|</span> &nbsp;&nbsp; <span style='color:#0000c0'>BLUE = minimum principal direction, length proportional to |S2|</span>")
        self.outline(f); self.status.setText(f'Member {self.beam} | Plane={f.plane_name} | Hover over a tick for numerical values')
    def add_shear_crosses(self):
        # Max-shear cross ticks: two conjugate planes oriented at +/-45deg to the
        # principal directions.  Both carry the SAME magnitude tau_max=(S1-S2)/2,
        # so each tick length is proportional to |tau_max|.
        f=ElevationField(self.staad,self.beam,self.lc,self.L,self.props); self.hover_field=f; nx,ny=self.spin_cols.value(),self.spin_rows.value(); xs=np.linspace(0,self.L,nx); ts=np.linspace(-f.h/2,f.h/2,ny); ml=min(self.L/(nx-1),f.h/(ny-1))*self.spin_tick.value()/100*self.spin_size.value(); c=[]; d1=[]; d2=[]; tm=[]
        for x in xs:
            for t in ts:
                c.append(self.lift(x,t,f.use_xy)); a=f.dir_shear1(x,t); b=f.dir_shear2(x,t); d1.append((a[0],a[1],0) if f.use_xy else (a[0],0,a[1])); d2.append((b[0],b[1],0) if f.use_xy else (b[0],0,b[1])); tm.append(abs(f.tau_max(x,t)))
        tm=np.asarray(tm); ref=max(np.max(tm),1e-12)
        for poly,col,fam in ((make_tick_segments(c,d1,ml*tm/ref),'green','shear1'),(make_tick_segments(c,d2,ml*tm/ref),'magenta','shear2')):
            if poly is not None:a=self.plot_beam.add_mesh(poly,color=col,line_width=self.spin_width.value(),pickable=True); self.traj_actors.append(a); self.register_hover(a,fam)
        self.legend.setText("<span style='color:#008000'>GREEN = max-shear plane (+45&deg; to principal)</span> &nbsp;&nbsp; <span style='color:#c000c0'>MAGENTA = conjugate plane (&minus;45&deg;)</span> &nbsp; length proportional to |tau_max|")
        self.outline(f); self.status.setText(f'Member {self.beam} | Plane={f.plane_name} | Max-shear ticks (+/-45deg to principal) | Hover for tau_max value')
    def add_flow(self):
        f=ElevationField(self.staad,self.beam,self.lc,self.L,self.props); self.hover_field=f; step=RK_STEP_FRAC*self.L
        for t0 in np.linspace(-.475*f.h,.475*f.h,SEED_MAJOR):
            for x0 in (0,.25*self.L,.5*self.L,.75*self.L):
                pts=integrate_trajectory(f,x0,t0,True,step,RK_MAX_STEPS); poly=make_polyline([self.lift(x,t,f.use_xy) for x,t in pts])
                if poly is not None:a=self.plot_beam.add_mesh(poly,color='red',line_width=NET_WIDTH,opacity=NET_OPACITY,pickable=True); self.traj_actors.append(a); self.register_hover(a,'major')
        for x0 in np.linspace(.01*self.L,.99*self.L,SEED_MINOR):
            for t0 in (-.25*f.h,0,.25*f.h):
                pts=integrate_trajectory(f,x0,t0,False,step,RK_MAX_STEPS); poly=make_polyline([self.lift(x,t,f.use_xy) for x,t in pts])
                if poly is not None:a=self.plot_beam.add_mesh(poly,color='blue',line_width=NET_WIDTH,opacity=NET_OPACITY,pickable=True); self.traj_actors.append(a); self.register_hover(a,'minor')
        self.outline(f); self.status.setText(f'Member {self.beam} | Flow-net plane={f.plane_name} | Hover over a trajectory for numerical values')
    def apply_nav(self):
        if HAVE_VTK and self.combo_nav.currentIndex()==0:self.nav_style=MultiNavStyle(self.plot_beam); self.plot_beam.iren.SetInteractorStyle(self.nav_style)
        elif self.combo_nav.currentIndex()==1:self.plot_beam.enable_rubber_band_2d_style()
        else:self.plot_beam.enable_trackball_style()
    def reset_view(self):
        if self.props and (self.chk_traj.isChecked() or self.combo_stress.currentText() in FILLED_MODES):
            q=[self.staad.safe_forces_at(self.beam,x,self.L,self.lc) for x in np.linspace(0,self.L,9)]; use_xy=sum(abs(f[1]) for f in q)>=sum(abs(f[2]) for f in q); self.plot_beam.view_xy() if use_xy else self.plot_beam.view_xz()
        else:self.plot_beam.view_isometric()
        self.plot_beam.reset_camera(); self.plot_sec.view_yz(); self.plot_sec.reset_camera()

if __name__=='__main__':
    app=QtWidgets.QApplication(sys.argv); window=MainWindow(); window.show(); sys.exit(app.exec())

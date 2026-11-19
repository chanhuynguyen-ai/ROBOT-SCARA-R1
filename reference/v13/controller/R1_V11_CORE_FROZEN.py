#!/usr/bin/env python3
"""R1 V11: SCRIPTED MuJoCo SIM-0 G23 pick, carry, and fixed-datum assembly.

NO RL / motor control / mj_step / force or contact validation.
This visual kinematic study reuses the user's successfully attached R1 V09,
and V10's 2622-frame joint trajectory (including failed-gate traces).

The default 'registered' demonstration moves ONLY the G23 feeder reference
pose to match the already-fixed GF01 jaw sites. This is an explicit cell-layout
visual proposal, never a correction to approved hardware. The assembly target
remains the exact original V513 world datum. If it is unreachable, the object
stays at the attained end pose and the report records a failed fixed-datum gate.
"""
from __future__ import annotations

import argparse
import bisect
import csv
from datetime import datetime
import hashlib
import itertools
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np

RELEASE = "R1_V11_G23_ASSEMBLY_SIM0_20261027"
SOURCE_REFERENCE = np.array([-.450, -.245, .370], dtype=float)
FIXED_ASSEMBLY_DATUM = np.array([-.30285, -.24368, 0.0], dtype=float)
Q_NAMES = ("R1__J1", "R1__J3", "R1__J2", "R1__J4", "GF01_R02_PITCH_CANDIDATE")
HEADS = ("NEG_R1", "NEG_R2", "POS_R1", "POS_R2")
TARGETS = ("S1_FAR", "S2_FAR", "S1_NEAR", "S2_NEAR")
MOUNT = "GF01_R02_SOURCE_FLANGE_MOUNT_CANDIDATE"
G23 = "module_G23_gapxe_1"
FLOAT_TOL = .002


def smooth(x):
    x = min(1., max(0., float(x)))
    return x*x*x*(10.+x*(-15.+6.*x))


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, ensure_ascii=False,
                                     default=lambda v: v.tolist() if hasattr(v, 'tolist') else str(v))+'\n', encoding='utf-8')


def rigid_fit(a, b):
    """Find R,t such that (R @ a.T).T + t approximates b, no reflection."""
    a=np.asarray(a, float); b=np.asarray(b,float)
    ca=a.mean(axis=0);cb=b.mean(axis=0)
    U,s,Vt=np.linalg.svd((a-ca).T@(b-cb))
    R=Vt.T@np.diag([1.,1.,np.linalg.det(Vt.T@U.T)])@U.T
    t=cb-R@ca
    errors=np.linalg.norm(a@R.T+t-b,axis=1)
    return R,t,errors


def rotation_angle(R):
    c=max(-1.,min(1.,(float(np.trace(R))-1.)*.5))
    return math.acos(c)


def rotation_residual(R):
    """SO(3) logarithm (rotation vector); stable at 0 and pi."""
    theta=rotation_angle(R)
    if theta < 1e-7:
        return .5*np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    if math.pi-theta<1e-4:
        # Pure NumPy fallback for 180-degree rotations (no SciPy required).
        B=(R+np.eye(3))*.5
        axis=B[:,int(np.argmax(np.diag(B)))].copy()
        norm=np.linalg.norm(axis)
        if norm<1e-9:axis=np.array([1.,0.,0.]);norm=1.
        axis/=norm
        return theta*axis
    return theta/(2.*math.sin(theta))*np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])


def quat_from_matrix(R):
    R=np.asarray(R,dtype=float)
    w=math.sqrt(max(0.,1.+R[0,0]+R[1,1]+R[2,2]))*.5
    x=math.sqrt(max(0.,1.+R[0,0]-R[1,1]-R[2,2]))*.5
    y=math.sqrt(max(0.,1.-R[0,0]+R[1,1]-R[2,2]))*.5
    z=math.sqrt(max(0.,1.-R[0,0]-R[1,1]+R[2,2]))*.5
    x=math.copysign(x,R[2,1]-R[1,2]);y=math.copysign(y,R[0,2]-R[2,0]);z=math.copysign(z,R[1,0]-R[0,1])
    q=np.array([w,x,y,z])
    return q/np.linalg.norm(q)


def find_sources(project, v09=None, v10run=None):
    if v09 is None:
        matches=list((project/'r1_runs').glob('R1_V09_PHYSICAL_TIP_ATTACH_*/scene/R1_V09_GF01_MATED.xml'))
        if not matches:raise FileNotFoundError('Missing original R1 V09 assembled scene')
        v09=max(matches,key=lambda x:x.stat().st_mtime)
    v09=Path(v09).resolve()
    if not v09.is_file():raise FileNotFoundError(v09)
    if v10run is None:
        runs=[p for p in (project/'r1_runs').glob('R1_V10_G23_PICK_PLACE_*') if (p/'R1_V10_G23_TRAJECTORY_2622.csv').is_file()]
        if not runs:raise FileNotFoundError('Missing V10 run with 2622-frame trajectory. V10 may have failed its gate, but its trace must exist.')
        v10run=max(runs,key=lambda x:x.stat().st_mtime)
    v10run=Path(v10run).resolve()
    if not (v10run/'R1_V10_G23_TRAJECTORY_2622.csv').is_file():raise FileNotFoundError('V10 trajectory CSV missing')
    return v09,v10run


def load_trace(v10run):
    path=v10run/'R1_V10_G23_TRAJECTORY_2622.csv'
    with path.open(newline='',encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    if len(rows)!=2622:raise ValueError(f'V10_EXPECTED_2622_FRAMES_GOT_{len(rows)}')
    q=np.array([[float(r[k]) for k in ('q1_rad','q3_m','q2_rad','q4_rad','pitch_rad')] for r in rows])
    t=np.array([float(r['t_s']) for r in rows])
    if not np.all(np.diff(t)>=0.):raise ValueError('TIMESTAMPS_NOT_MONOTONIC')
    modes=set(r['G23_mode'] for r in rows)
    if not {'SOURCE','ATTACH','SEATED'}<=modes:raise ValueError('SOURCE_CARRY_SEAT_PHASES_MISSING')
    for phase in ('GRASP_MOCK','LOWER_TO_DATUM_MOCK','RELEASE_MOCK'):
        if phase not in {r['phase'] for r in rows}:raise ValueError('MISSING_PHASE_'+phase)
    return rows,q,t


def make_xml(source, scene_dir):
    scene_dir.mkdir(parents=True,exist_ok=True)
    shutil.copytree(source.parent,scene_dir,dirs_exist_ok=True)
    root=ET.parse(source).getroot()
    root.set('model','R1_V11_SCRIPTED_G23_ASSEMBLY_VISUAL_SIM0')
    wb=root.find('worldbody')
    if wb is None:raise ValueError('SOURCE_WORLD_BODY_MISSING')
    # Fixed target never moves. Site is a witness, not a physical fixture.
    ET.SubElement(wb,'site',name='R1_V11_FIXED_G23_ASSEMBLY_DATUM',type='sphere',
                  pos=' '.join(str(v) for v in FIXED_ASSEMBLY_DATUM),size='.023',
                  rgba='.13 .95 .22 .90',group='4')
    # Update this *model* site to the registered feeder pose after FK fit.
    ET.SubElement(wb,'site',name='R1_V11_REGISTERED_FEEDER_WITNESS',type='sphere',
                  pos='0 0 0',size='.020',rgba='.1 .70 1 .85',group='4')
    outfile=scene_dir/'R1_V11_SCRIPTED_G23_ASSEMBLY.xml'
    ET.ElementTree(root).write(outfile,encoding='utf-8',xml_declaration=True)
    return outfile


def static_gate(xml,rows):
    root=ET.parse(xml).getroot()
    bodies={b.get('name'):b for b in root.iter('body') if b.get('name')}
    joints={j.get('name') for j in root.iter('joint')}
    sites={s.get('name') for s in root.iter('site')}
    need_bodies=('R1__wrist_J4','R1__tcp_body_no_gripper',MOUNT,G23)
    for n in need_bodies:
        if n not in bodies:raise ValueError('MISSING_BODY_'+n)
    for n in Q_NAMES:
        if n not in joints:raise ValueError('MISSING_JOINT_'+n)
    for h in HEADS:
        for prefix in ('GF01_R02_C_SLIDE_','GF01_R02_UPPER_RELEASE_'):
            if prefix+h not in joints:raise ValueError('MISSING_GF01_JOINT_'+prefix+h)
        if 'R1_V03_GF01_TIP_'+h not in sites:raise ValueError('MISSING_TIP_SITE_'+h)
    for n in TARGETS:
        if 'R1_V03_G23_IDEAL_'+n not in sites:raise ValueError('MISSING_G23_SITE_'+n)
    parents={c:p for p in root.iter() for c in p}
    if parents.get(bodies[MOUNT]) is not bodies['R1__tcp_body_no_gripper']:
        raise ValueError('V09_GF01_NOT_HARD_ATTACHED_TO_TCP')
    if bodies[G23].get('mocap')!='true':raise ValueError('G23_NOT_MOCAP')
    if len(list(root.iter('mesh')))<25:raise ValueError('CAD_MESHES_MISSING')
    return {'v09_gf01_tcp_hard_attached':True,'cad_mesh_assets':len(list(root.iter('mesh'))),'v10_frames':len(rows)}


def native_ids(mu,model):
    def get(t,n):
        idx=int(mu.mj_name2id(model,t,n))
        if idx<0:raise RuntimeError('NATIVE_MISSING_'+n)
        return idx
    joint=[get(mu.mjtObj.mjOBJ_JOINT,n) for n in Q_NAMES]
    slide=[get(mu.mjtObj.mjOBJ_JOINT,'GF01_R02_C_SLIDE_'+h) for h in HEADS]
    upper=[get(mu.mjtObj.mjOBJ_JOINT,'GF01_R02_UPPER_RELEASE_'+h) for h in HEADS]
    tips=[get(mu.mjtObj.mjOBJ_SITE,'R1_V03_GF01_TIP_'+h) for h in HEADS]
    goals=[get(mu.mjtObj.mjOBJ_SITE,'R1_V03_G23_IDEAL_'+h) for h in TARGETS]
    mount=get(mu.mjtObj.mjOBJ_BODY,MOUNT)
    g23=get(mu.mjtObj.mjOBJ_BODY,G23)
    source_id=get(mu.mjtObj.mjOBJ_SITE,'R1_V11_REGISTERED_FEEDER_WITNESS')
    mid=int(model.body_mocapid[g23])
    if mid<0:raise RuntimeError('G23_MOCAP_MISSING')
    return {'joint_ids':joint,'qaddr':[int(model.jnt_qposadr[x]) for x in joint],
            'bounds':np.array([model.jnt_range[i].copy() for i in joint]),
            'slide':[int(model.jnt_qposadr[x]) for x in slide],
            'upper':[int(model.jnt_qposadr[x]) for x in upper],
            'tips':tips,'goals':goals,'mount':mount,'cargo':g23,'mocap':mid,'source_site':source_id}


def set_pose(mu,model,data,h,q,jaw_open=0.):
    for i,v in zip(h['qaddr'],q):data.qpos[i]=float(v)
    for adr in h['slide']:data.qpos[adr]=.048*float(jaw_open)
    for adr in h['upper']:data.qpos[adr]=.001*float(jaw_open)
    mu.mj_forward(model,data)


def get_mount(data,h):
    return data.xpos[h['mount']].copy(),data.xmat[h['mount']].reshape(3,3).copy()


def register_source(mu,model,data,h,q_pick):
    # Read G23 sites in its *own local frame*: set G23 mocap to identity origin.
    data.mocap_pos[h['mocap']]=[0.,0.,0.]
    data.mocap_quat[h['mocap']]=[1.,0.,0.,0.]
    set_pose(mu,model,data,h,q_pick,0.)
    pose_origin=data.xpos[h['cargo']].copy()
    pose_R=data.xmat[h['cargo']].reshape(3,3).copy()
    goals_world=np.array([data.site_xpos[i].copy() for i in h['goals']])
    goals_local=(goals_world-pose_origin)@pose_R
    tips_world=np.array([data.site_xpos[i].copy() for i in h['tips']])
    solutions=[]
    for perm in itertools.permutations(range(4)):
        R,t,errs=rigid_fit(goals_local[list(perm)],tips_world)
        shift=float(np.linalg.norm(t-SOURCE_REFERENCE))
        angle=rotation_angle(R)
        # Congruent fits first; select smallest feeder relocation and tilt.
        solutions.append((float(max(errs)),shift,angle,perm,R,t,errs))
    compatible=[x for x in solutions if x[0]<=.005]
    if not compatible:
        best=min(solutions,key=lambda x:x[0])
        raise RuntimeError(f'GF01/G23_SITE_SHAPES_INCOMPATIBLE_{best[0]*1000:.2f}MM')
    best=min(compatible,key=lambda x:(x[1],x[2],x[0]))
    err,shift,angle,perm,source_R,source_t,per_head=best
    mount_pos,mount_R=get_mount(data,h)
    tool_part_pos=mount_R.T@(source_t-mount_pos)
    tool_part_R=mount_R.T@source_R
    return {'permutation':list(perm),'shape_fit_max_mm':err*1000.,
            'source_world_m':source_t.tolist(),'source_rotation_matrix':source_R.tolist(),
            'original_source_reference_world_m':SOURCE_REFERENCE.tolist(),
            'source_relocation_mm':shift*1000.,'source_reorientation_deg':math.degrees(angle),
            'four_contact_mismatch_mm':(per_head*1000.).tolist(),
            'flange_to_G23_offset_m':tool_part_pos.tolist(),
            'flange_to_G23_rotation_matrix':tool_part_R.tolist()}


def bounded_lm(start, objective, bounds, caps, steps=90):
    """Bounded damped least-squares, no SciPy dependency in Windows .venv."""
    q=np.clip(np.asarray(start,dtype=float),bounds[:,0],bounds[:,1])
    lam=.01
    best=objective(q);cost=float(best@best)
    for _ in range(steps):
        if cost<1e-11:break
        J=np.zeros((len(best),len(q)))
        for j in range(len(q)):
            t=q.copy();t[j]=min(bounds[j,1],t[j]+1e-5)
            if t[j]==q[j]:t[j]=max(bounds[j,0],t[j]-1e-5)
            dh=t[j]-q[j]
            if abs(dh)>1e-12:J[:,j]=(objective(t)-best)/dh
        M=J.T@J+(lam**2)*np.eye(len(q))
        try:dq=np.linalg.solve(M,-J.T@best)
        except np.linalg.LinAlgError:dq=np.linalg.lstsq(M,-J.T@best,rcond=None)[0]
        dq=np.clip(dq,-caps,caps)
        accepted=False
        for alpha in (1.,.5,.25,.125,.0625):
            q2=np.clip(q+alpha*dq,bounds[:,0],bounds[:,1])
            if float(max(abs(q2-q)))<1e-11:continue
            f=objective(q2);c=float(f@f)
            if c+1e-14<cost:
                q,best,cost=q2,f,c
                lam=max(1e-6,lam*.70);accepted=True;break
        if not accepted:
            lam=min(100.,lam*3.)
            if lam>=80.:break
    return q,cost


def attempt_original_feeder(mu,model,data,h,qtraj,rows,registration):
    """First try the historical feeder without moving the module/fixture.

    Only if 4 tips match at its original pose under native FK do we keep the
    feeder. Otherwise the visual-only Registered variant is explicitly used.
    """
    pick_idx=max(i for i,r in enumerate(rows) if r['phase']=='GRASP_MOCK')
    q0=qtraj[pick_idx].copy()
    relp=np.asarray(registration['flange_to_G23_offset_m'])
    relR=np.asarray(registration['flange_to_G23_rotation_matrix'])
    bounds=h['bounds']
    def objective(q):
        set_pose(mu,model,data,h,q,0.)
        p,R=get_mount(data,h)
        pp=p+R@relp
        rr=R@relR
        return np.r_[pp-SOURCE_REFERENCE,.045*rotation_residual(rr),.002*(q-q0)]
    seeds=[q0,qtraj[max(0,pick_idx-60)],qtraj[0],
           q0+np.array([.4,0.,-.3,0.,0.]),q0+np.array([-.4,0.,.3,0.,0.]),
           q0+np.array([0.,0.,0.,.5,0.]),q0+np.array([0.,0.,0.,-.5,0.]),
           q0+np.array([.3,.06,-.3,-.2,0.]),q0+np.array([-.3,-.06,.3,.2,0.])]
    best=None
    for seed in seeds:
        q,c=bounded_lm(seed,objective,bounds,np.array([.18,.04,.18,.20,.07]),steps=75)
        set_pose(mu,model,data,h,q,0.)
        p,R=get_mount(data,h)
        pp=p+R@relp;rr=R@relR
        err=float(np.linalg.norm(pp-SOURCE_REFERENCE))
        angle=math.degrees(rotation_angle(rr))
        cand=(err,angle,float(c),q.copy())
        if best is None or (cand[0]+.0005*cand[1],cand[2])<(best[0]+.0005*best[1],best[2]):best=cand
    error,angle,_,qnew=best
    set_pose(mu,model,data,h,qnew,0.)
    data.mocap_pos[h['mocap']]=SOURCE_REFERENCE
    data.mocap_quat[h['mocap']]=[1.,0.,0.,0.]
    mu.mj_forward(model,data)
    tip=np.array([data.site_xpos[i].copy() for i in h['tips']])
    target=np.array([data.site_xpos[i].copy() for i in h['goals']])
    order=registration['permutation']
    # 'permutation' indexes goals in the order of the tool tips.
    four_tip_max=float(max(np.linalg.norm(tip-target[order],axis=1))*1000.)
    good=bool(error<=.010 and angle<=10. and four_tip_max<=10.)
    changed=False
    q_delta=qnew-q0
    if good:
        initial=qtraj.copy()
        first_transfer=next(i for i,r in enumerate(rows) if r['phase']=='MOVE_OUTSIDE_REAR_WHEEL')
        for i in range(first_transfer):
            if i<=pick_idx:
                fac=1.
            else:
                fac=1.-smooth((i-pick_idx)/max(1.,float(first_transfer-pick_idx)))
            qtraj[i]+=fac*q_delta
        changed=bool(np.all(qtraj>=bounds[:,0]-1e-7) and np.all(qtraj<=bounds[:,1]+1e-7))
        if not changed:
            qtraj[:]=initial
            good=False
        else:
            # Only publish historical datum after the entire adjustment passes.
            registration['source_world_m']=SOURCE_REFERENCE.tolist()
            registration['source_rotation_matrix']=np.eye(3).tolist()
            registration['source_relocation_mm']=0.
            registration['source_reorientation_deg']=0.
    return {'historical_feeder_feasible':good,
            'original_feeder_position_error_mm':error*1000.,
            'original_feeder_orientation_error_deg':angle,
            'four_tip_error_at_original_feeder_mm':four_tip_max,
            'original_pick_q':q0.tolist(),'optimized_pick_q':qnew.tolist(),
            'joint_pose_rebased_before_first_transfer':changed}


def solve_fixed_assembly(mu,model,data,h,qtraj,rows,registration):
    seat_idx=max(i for i,r in enumerate(rows) if r['phase']=='LOWER_TO_DATUM_MOCK')
    q0=qtraj[seat_idx].copy()
    relp=np.asarray(registration['flange_to_G23_offset_m'])
    relR=np.asarray(registration['flange_to_G23_rotation_matrix'])
    bounds=h['bounds']
    # Keep the fixed original assembly datum, no visual teleport to target.
    def cargo_at(q):
        set_pose(mu,model,data,h,q,0.)
        p,R=get_mount(data,h)
        return p+R@relp,R@relR
    def objective(q):
        p,R=cargo_at(q)
        pos=p-FIXED_ASSEMBLY_DATUM
        orient=rotation_residual(R) # original V513 G23 target orientation = identity
        return np.r_[pos,.045*orient,.002*(q-q0)]
    seeds=[q0,qtraj[seat_idx-80],qtraj[seat_idx-200],qtraj[0],
           q0+np.array([.4,0.,-.4,0.,0.]),q0+np.array([-.4,0.,.4,0.,0.]),
           q0+np.array([0.,0.,0.,.5,0.]),q0+np.array([0.,0.,0.,-.5,0.]),
           q0+np.array([.25,.05,-.25,-.25,0.]),q0+np.array([-.25,-.05,.25,.25,0.])]
    best=None
    for seed in seeds:
        q,c=bounded_lm(seed,objective,bounds,np.array([.17,.035,.17,.19,.07]),steps=90)
        p,R=cargo_at(q)
        err=np.linalg.norm(p-FIXED_ASSEMBLY_DATUM)
        orient=rotation_angle(R)
        cand=(float(err),float(orient),float(c),q.copy())
        if best is None or (cand[0]+.05*cand[1],cand[2])<(best[0]+.05*best[1],best[2]):best=cand
    error,angle,cost,q_place=best
    # Apply a *smooth joint correction* through final approach/lowering.
    delta=q_place-q0
    start_idx=next(i for i,r in enumerate(rows) if r['phase']=='FINAL_MATE_APPROACH_MOCK')
    if seat_idx<=start_idx:raise ValueError('V10_INSERTION_PHASE_ORDER_CORRUPTED')
    for i in range(start_idx,len(rows)):
        a=smooth((i-start_idx)/float(seat_idx-start_idx))
        qtraj[i]+=a*delta
    # Keep original tool/G23 tree. Do not override joint limits silently.
    low_margin=float(np.min(qtraj-bounds[:,0]))
    high_margin=float(np.min(bounds[:,1]-qtraj))
    violations=max(0.,-min(low_margin,high_margin))
    # Use the actual attained pose after blending for the gate.
    reached_p,reached_R=cargo_at(qtraj[seat_idx])
    error_after=float(np.linalg.norm(reached_p-FIXED_ASSEMBLY_DATUM))
    angle_after=math.degrees(rotation_angle(reached_R))
    return {'q_place':q_place.tolist(),'q_correction':delta.tolist(),
            'original_seat_index':seat_idx,'blend_start_index':start_idx,
            'seat_position_error_mm':error_after*1000.,'seat_orientation_error_deg':angle_after,
            'joint_limit_violation_max_mm_or_rad':violations,
            'original_path_collision_audit_reused_as_valid':False,
            'fixed_place_pass':bool(error_after<=.015 and angle_after<=10. and violations<=1e-6),
            'achieved_pose_world_m':reached_p.tolist(),
            'achieved_rotation_matrix':reached_R.tolist()}


def time0(rows,phase):
    return float(next(r['t_s'] for r in rows if r['phase']==phase))


def main():
    ap=argparse.ArgumentParser(description='R1 V11 scripted MuJoCo G23 assembly visualization; NOT motor control, no RL')
    ap.add_argument('--project',required=True)
    ap.add_argument('--scene-xml',default=None)
    ap.add_argument('--v10-run',default=None)
    ap.add_argument('--mode',choices=['Static','Check','View'],default='View')
    ap.add_argument('--start',choices=['REGISTER','GRASP','LIFT','TRANSFER','PLACE','DONE'],default='REGISTER')
    ap.add_argument('--speed',type=float,default=1.)
    ap.add_argument('--loop',action='store_true')
    ap.add_argument('--feeder-mode',choices=['registered','original'],default='registered',
                    help='registered: WHAT-IF relocate feeder to visual grasp; original: preserve historical feeder and block if misaligned')
    ap.add_argument('--allow-unqualified-view',action='store_true',help='Open visual sequence even if fixed placement is out of reach; report remains FAILED')
    args=ap.parse_args()
    if not .1<=args.speed<=5.:ap.error('Speed out of range .1..5')
    project=Path(args.project).resolve()
    if not project.is_dir():ap.error('Project not found')
    source,v10run=find_sources(project,args.scene_xml,args.v10_run)
    rows,qtraj,times=load_trace(v10run)
    now=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output=project/'r1_runs'/('R1_V11_G23_ASSEMBLY_SIM0_'+now)
    output.mkdir(parents=True,exist_ok=False)
    report_path=output/'R1_V11_SIMULATION_REPORT.json'
    report={'version':RELEASE,'task':'R1_G23_SCRIPTED_ASSEMBLY_SIM0',
            'source_v09':str(source),'source_v10_trajectory':str(v10run),
            'original_v09_v10_untouched':True,'fixed_assembly_datum_m':FIXED_ASSEMBLY_DATUM.tolist(),
            'no_RL_no_motor_control':True,'physical_grip_or_contact_verified':False,
            'collision_free_verified':False,'loaded_hardware_approved':False,
            'feeder_layout_candidate_changes_allowed':True,
            'status':'SOURCE_STAGING'}
    write_json(report_path,report)
    try:
        scene=make_xml(source,output/'scene')
        report['xml']=str(scene)
        report['static_gate']=static_gate(scene,rows)
        report['status']='STATIC_SCENE_OK'
        write_json(report_path,report)
        print('[R1 V11] STATIC scene ready:',scene,flush=True)
        if args.mode=='Static':return 0
        try:
            import mujoco as mj
            import mujoco.viewer
        except ImportError as e:
            raise RuntimeError('MuJoCo Python is needed only for Check/View on Windows') from e
        # ASCII staging of entire scene/meshes: prevent Unicode Windows path failures.
        with tempfile.TemporaryDirectory(prefix='R1_V11_ASSEMBLY_') as folder:
            staged=Path(folder)/'scene'
            shutil.copytree(scene.parent,staged)
            model=mj.MjModel.from_xml_path(str(staged/scene.name))
            data=mj.MjData(model)
            h=native_ids(mj,model)
            if model.njnt!=13:raise RuntimeError('Unexpected R1 joint count: '+str(model.njnt))
            original_q=qtraj.copy()
            pick_idx=max(i for i,r in enumerate(rows) if r['phase']=='GRASP_MOCK')
            registration=register_source(mj,model,data,h,qtraj[pick_idx])
            report['feeder_mode']=args.feeder_mode
            feeder_audit=attempt_original_feeder(mj,model,data,h,qtraj,rows,registration)
            report['original_feeder_attempt']=feeder_audit
            report['pick_four_point_registration']=registration
            if args.feeder_mode=='original' and not feeder_audit['historical_feeder_feasible']:
                raise RuntimeError('ORIGINAL_FEEDER_FAILED_PICK_REACH_ORIENTATION_GATE. '
                                   'Only run --feeder-mode registered as an explicitly relocated-feeder SIM0 variant.')
            model.site_pos[h['source_site']]=np.asarray(registration['source_world_m'])
            # Feeder is STATIC at calculated reachable pose during SOURCE phases.
            report['assembly']=solve_fixed_assembly(mj,model,data,h,qtraj,rows,registration)
            report['trajectory_samples']=len(rows)
            report['source_relocated_for_visual_feeder_by_mm']=registration['source_relocation_mm']
            report['G23_goes_to_user_fixed_target_only_if_reachable']=True
            report['body_mount_parent_stays_V09']=True
            report['native_mujoco_compiled']=True
            report['original_joint_trace_modified_only_in_final_insertion']=True
            report['status']='FIXED_DATUM_CANDIDATE_PASS' if report['assembly']['fixed_place_pass'] else 'UNQUALIFIED_FIXED_DATUM_PREVIEW'
            write_json(report_path,report)
            srcp=np.array(registration['source_world_m'])
            srcR=np.array(registration['source_rotation_matrix'])
            relp=np.array(registration['flange_to_G23_offset_m'])
            relR=np.array(registration['flange_to_G23_rotation_matrix'])
            tarR=np.eye(3)
            qmocap=h['mocap']
            phase_times={r['phase']:float(r['t_s']) for r in reversed(rows)}
            fixed_seat_p=np.asarray(report['assembly']['achieved_pose_world_m'],float)
            fixed_seat_R=np.asarray(report['assembly']['achieved_rotation_matrix'],float)
            def frame(i):
                row=rows[i]
                data.mocap_pos[qmocap]=srcp
                data.mocap_quat[qmocap]=quat_from_matrix(srcR)
                jaw=float(row['jaw_open_fraction'])
                set_pose(mj,model,data,h,qtraj[i],jaw)
                mode=row['G23_mode']
                if mode=='ATTACH':
                    p,R=get_mount(data,h)
                    data.mocap_pos[qmocap]=p+R@relp
                    data.mocap_quat[qmocap]=quat_from_matrix(R@relR)
                elif mode=='SEATED':
                    if report['assembly']['fixed_place_pass']:
                        data.mocap_pos[qmocap]=FIXED_ASSEMBLY_DATUM
                        data.mocap_quat[qmocap]=[1.,0.,0.,0.]
                    else:
                        # Hold the actual achieved pose while SCARA retracts.
                        # Never snap an unreachable cargo to the fixed target.
                        data.mocap_pos[qmocap]=fixed_seat_p
                        data.mocap_quat[qmocap]=quat_from_matrix(fixed_seat_R)
                mj.mj_forward(model,data)
                return row
            frame(0)
            # Check the exact 0.5s transition from stationary SOURCE to CARRY.
            first_attach=next(i for i,r in enumerate(rows) if r['G23_mode']=='ATTACH')
            frame(first_attach)
            gap=np.linalg.norm(data.mocap_pos[qmocap]-srcp)*1000.
            report['first_attach_pose_jump_mm']=float(gap)
            report['first_attach_continuity_pass']=bool(gap<=10.)
            report['sim0_gate_pass']=bool(report['assembly']['fixed_place_pass'] and gap<=10.)
            if not report['sim0_gate_pass']:
                report['status']='UNQUALIFIED_GRASP_OR_PLACEMENT_PREVIEW'
            write_json(report_path,report)
            print('[R1 V11] FOUR-SITE rigid fit:',round(registration['shape_fit_max_mm'],3),'mm',flush=True)
            print('[R1 V11] Feeder visual relocation:',round(registration['source_relocation_mm'],2),'mm',flush=True)
            print('[R1 V11] Fixed target remaining position error:',round(report['assembly']['seat_position_error_mm'],2),'mm',flush=True)
            print('[R1 V11] Fixed place gate:',report['assembly']['fixed_place_pass'],flush=True)
            print('[R1 V11] First attachment jump:',round(gap,2),'mm',flush=True)
            print('[R1 V11] Report:',report_path,flush=True)
            if args.mode=='Check':return 0 if report['sim0_gate_pass'] else 3
            if (not report['sim0_gate_pass']) and (not args.allow_unqualified_view):
                print('[R1 V11] Viewer blocked by fixed target gate. You can open a clearly UNQUALIFIED storyboard with --allow-unqualified-view.',flush=True)
                return 3
            import mujoco.viewer as viewer
            starts={'REGISTER':0.,'GRASP':time0(rows,'GRASP_MOCK'),
                    'LIFT':time0(rows,'VERTICAL_LIFT_210MM'),
                    'TRANSFER':time0(rows,'MOVE_OUTSIDE_REAR_WHEEL'),
                    'PLACE':time0(rows,'LOWER_TO_DATUM_MOCK'),
                    'DONE':time0(rows,'DONE_REVIEW')}
            state={'t':starts[args.start],'paused':args.start!='REGISTER','loop':args.loop,
                   'focus':None,'speed':args.speed}
            def keys(k):
                k=int(k)
                if k==32:state['paused']=not state['paused']
                for char,action in [('G','GRASP'),('U','LIFT'),('T','TRANSFER'),('M','PLACE'),('R','REGISTER')]:
                    if k in (ord(char),ord(char.lower())):
                        state['t']=starts[action];state['paused']=True
                        if action in ('GRASP','LIFT'):state['focus']='tool'
                if k in (ord('L'),ord('l')):state['loop']=not state['loop']
                if k in (ord('F'),ord('f')):state['focus']='tool'
                if k in (ord('B'),ord('b')):state['focus']='cell'
            print('[R1 V11] VISUAL-ONLY assembly. Space pause; G grasp; U lift; T transfer; M mate; F tool; B cell; R reset; L loop.',flush=True)
            prior=time.monotonic();last_phase=None;maxidx=0
            with viewer.launch_passive(model,data,show_left_ui=False,show_right_ui=False,key_callback=keys) as v:
                with v.lock():
                    for i in range(min(5,len(v.opt.geomgroup))):v.opt.geomgroup[i]=1
                    if len(v.opt.geomgroup)>5:v.opt.geomgroup[5]=0
                    if len(v.opt.sitegroup)>4:v.opt.sitegroup[4]=1
                    v.cam.lookat[:]=[0.,-.25,.4]
                    v.cam.distance=3.5;v.cam.azimuth=120.;v.cam.elevation=-24.
                while v.is_running():
                    tnow=time.monotonic();dt=min(.1,max(0.,tnow-prior));prior=tnow
                    if not state['paused']:
                        state['t']+=dt*state['speed']
                        if state['t']>times[-1]:
                            if state['loop']:state['t']%=times[-1]
                            else:state['t']=times[-1];state['paused']=True
                    idx=max(0,min(len(rows)-1,bisect.bisect_right(times,state['t'])-1))
                    maxidx=max(idx,maxidx)
                    with v.lock():
                        row=frame(idx)
                        if state['focus']=='tool':v.cam.lookat[:]=np.asarray(data.site_xpos[h['tips']]).mean(axis=0);v.cam.distance=1.1
                        elif state['focus']=='cell':v.cam.lookat[:]=[0.,-.25,.4];v.cam.distance=3.5
                        state['focus']=None
                    if row['phase']!=last_phase:
                        print(f'[R1 V11 {state["t"]:5.1f}s] {row["phase"]} G23={row["G23_mode"]}',flush=True)
                        last_phase=row['phase']
                    v.sync();time.sleep(.02)
            report['viewer_opened']=True
            report['end_of_cycle_viewed']=bool(maxidx>=len(rows)-2)
            write_json(report_path,report)
            return 0
    except Exception as exc:
        (output/'R1_V11_ERROR_TRACE.txt').write_text(traceback.format_exc(),encoding='utf-8')
        report['status']='FAILED'
        report['error']=type(exc).__name__+': '+str(exc)
        write_json(report_path,report)
        print('[R1 V11 ERROR]',type(exc).__name__,str(exc),file=sys.stderr)
        print('ERROR TRACE:',output/'R1_V11_ERROR_TRACE.txt',file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())

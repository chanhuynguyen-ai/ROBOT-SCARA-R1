#!/usr/bin/env python3
"""SCARA R1-T03-P7: independently packaged R13 dual-slot CAD candidate.

READ-ONLY frozen V13 source. No physical collision/force/retention/safety approval.
Verify and ViewSafe require stdlib + NumPy; optional MuJoCo and VTK checks
have deliberately narrower claims than hardware collision qualification.
"""
from __future__ import annotations
import argparse, csv, datetime as dt, hashlib, importlib, json, math, shutil, subprocess, sys, tempfile, traceback
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
sys.dont_write_bytecode = True

KIT=Path(__file__).resolve().parents[1]
CFG=KIT/'config/P6_OFFLINE_ENGINEERING_REPORT.json'
CSV=KIT/'outputs/R1_T03_P7_TRAJECTORY_2622.csv'
P7_MESH=KIT/'assets/G23_NO_COVERS_DUAL_SLOT_R13_SCENE_METERS.stl'
P7_CONT=KIT/'assets/guide_section_contours_r13.json'
P6_CONT=KIT/'assets/guide_section_contours_p6_reference.json'
SOURCE_REL='scene/R1_V09_GF01_MATED.xml'
CTRL_REL='controller'
ORIGINAL_CSV_REL=CTRL_REL+'/R1_V13_TRAJECTORY_2622.csv'
QCOLS=('q1_rad','q3_m','q2_rad','q4_rad','pitch_rad')
RADIUS_SHAFT=10.0
MIN_RESEARCH_MARGIN=2.0
STAGE_NAME='R1_T03_P7_DUAL_SLOT_R13_STAGED.xml'


def require(cond,msg):
    if not cond: raise RuntimeError(msg)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for part in iter(lambda:f.read(1<<20),b''): h.update(part)
    return h.hexdigest()


def verify_package():
    m=json.loads((KIT/'PACKAGE_SHA256.json').read_text(encoding='utf-8'))['files']
    actual={p.relative_to(KIT).as_posix() for p in KIT.rglob('*') if p.is_file() and p.name!='PACKAGE_SHA256.json' and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    require(set(m)==actual, 'P7_PACKAGE_FILESET_CHANGED missing='+str(sorted(set(m)-actual))+' unexpected='+str(sorted(actual-set(m))))
    bad=[name for name,digest in m.items() if sha(KIT/name)!=digest]
    require(not bad,'P7_PACKAGE_SHA256_FAIL '+str(bad))
    print('P7_PACKAGE_SHA256_PASS',len(m),'immutable files',flush=True)
    return json.loads(CFG.read_text(encoding='utf-8'))


def find_frozen(project,explicit,cfg):
    if explicit:
        choice=Path(explicit).expanduser().resolve()
        require((choice/'CONTENT_MANIFEST.json').is_file(),'P7_BAD_EXPLICIT_FROZEN_PATH')
        return choice
    n='R1_V13_COVER_ALIGN_SPEEDFIX_FROZEN_20261028'
    roots=[project/'reference/v13', project/'r1_runs/R1_T03_READONLY_FROZEN_20261028_120617'/n,
           project/'R1_T03_READONLY_FROZEN_20261028_120617'/n,project/n]
    roots.extend(sorted((project/'r1_runs').glob('R1_T03_READONLY_FROZEN_*/'+n)))
    candidates=[r.resolve() for r in roots if (r/'CONTENT_MANIFEST.json').is_file()]
    matched=[]
    for r in dict.fromkeys(candidates):
        sources={'original_v09_scene':r/SOURCE_REL, 'original_v13_trajectory':r/ORIGINAL_CSV_REL,
                 'legacy_G23_stl':r/'scene/G23_gapxe_1.stl'}
        if all(p.is_file() and sha(p)==cfg['source_sha256'][key] for key,p in sources.items()): matched.append(r)
    require(matched,'P7_FROZEN_SOURCE_NOT_FOUND_OR_BAD_SHA; supply --FreezeDir')
    print('P7_FROZEN_SELECTED',matched[0],flush=True)
    return matched[0]


def check_frozen(frozen,cfg,full=True):
    names={'original_v09_scene':SOURCE_REL,'original_v13_trajectory':ORIGINAL_CSV_REL,
           'legacy_G23_stl':'scene/G23_gapxe_1.stl'}
    for k,rel in names.items():
        path=frozen/rel
        require(path.is_file() and sha(path)==cfg['source_sha256'][k], 'P7_FROZEN_SOURCE_BAD_SHA '+k)
    if full:
        tool=frozen/'verify_snapshot.py'
        require(tool.is_file(),'P7_FROZEN_VERIFIER_MISSING')
        proc=subprocess.run([sys.executable,str(tool),'--root',str(frozen)],text=True,
                            encoding='utf-8',errors='replace')
        require(proc.returncode==0,'P7_FROZEN_MANIFEST_VERIFY_FAILED')
    print('P7_V13_REFERENCE_VERIFIED_READ_ONLY',flush=True)


def load_plan(cfg):
    with CSV.open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
    require(len(rows)==2622,'P7_TRAJECTORY_EXPECTED_2622')
    require(rows[1681]['phase']=='FOLLOW_DUAL_AXLE_SLOTS_MOCK','P7_MISSING_SLOT_PHASE')
    require([r['G23_mode'] for r in rows[2171:2173]]==['ATTACH','SEATED'], 'P7_BAD_SEAT_TRANSITION')
    t=np.array([float(x['t_s']) for x in rows]);q=np.array([[float(x[k]) for k in QCOLS] for x in rows])
    p=np.array([[float(x[k]) for k in ('desired_G23_x_m','desired_G23_y_m','desired_G23_z_m')] for x in rows])
    require(np.isfinite(q).all() and np.isfinite(p).all(),'P7_NAN_POSE')
    require(float(np.max(np.abs(np.diff(t)-.025)))<1e-6,'P7_TIMEBASE_CHANGED')
    require(float(np.linalg.norm(p[2171]-np.array([-.30285,-.24368,0])))<1e-8,'P7_SEAT_DATUM_CHANGED')
    require(float(np.max(np.abs(q[:,0]+q[:,2]+q[:,3])))<1e-5,'P7_YAW_CHANGED')
    joints=np.array([[-2.61799,2.61799],[-.04,.24],[-1.74533,1.74533],[-2.0944,2.0944]])
    require(bool(np.all(q[:,:4]>=joints[:,0]-1e-8) and np.all(q[:,:4]<=joints[:,1]+1e-8)), 'P7_JOINT_LIMIT_FAIL')
    vmax=np.max(np.abs(np.diff(q[:,:4],axis=0)/.025),axis=0)
    require(vmax[1]<.02500001,'P7_J3_SPEED_CAP_EXCEEDED')
    # P7 uses the original P6 guided trajectory without editing a single byte.
    require(sha(CSV)==json.loads((KIT/'outputs/P7_OFFLINE_AUDIT_REPORT.json').read_text())['p6_guided_trajectory_sha256'],
            'P7_GUIDED_CSV_WAS_NOT_PRESERVED')
    print('P7_2622_POSES_PASS; J3 max %.3f mm/s; SEAT DATUM MATCH'%(vmax[1]*1000),flush=True)
    return rows,t,q,p


def profiles(path):
    obj=json.loads(path.read_text(encoding='utf-8'))
    require(set(obj)=={'3','11'},'P7_EXPECT_BOTH_STEEL_PLATES')
    for k,v in obj.items():
        require(len(v['outline_xy_mm'])>30,'P7_CONTOUR_TOO_SHORT_'+k)
    return obj


def nearest_boundary_distance(pt,poly):
    """Signed XY point-to-STEEL-boundary distance in CAD mm; negative inside steel.

    This 2D section does NOT certify a moving 3D mesh or axial retention.
    """
    v=np.asarray(poly,float)
    e=v[1:]-v[:-1]
    vpt=np.asarray(pt,float)
    d=vpt-v[:-1]
    t=np.clip(np.einsum('ij,ij->i',d,e)/np.maximum(np.einsum('ij,ij->i',e,e),1e-15),0.,1.)
    q=v[:-1]+t[:,None]*e
    dist=float(np.min(np.linalg.norm(q-vpt,axis=1)))
    x,y=vpt
    a,b,c,d=v[:-1,0],v[:-1,1],v[1:,0],v[1:,1]
    hit=(b>y)!=(d>y)
    xhit=a+(y-b)*(c-a)/np.where(abs(d-b)<1e-15,1e-15,d-b)
    inside=int(np.count_nonzero(hit & (xhit>x)))%2
    return -dist if inside else dist


def to_cad_xy(wheel_world,cargo_world,cfg):
    theta=math.radians(cfg['cad_to_frozen']['angle_about_step_z_deg'])
    r=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    origin=np.asarray(cfg['cad_to_frozen']['xy_translation_mm'])
    return (((np.asarray(wheel_world)-np.asarray(cargo_world))[:2]*1000.)-origin)@r


def slot_sweep(cfg,pos,contour,min_margin=MIN_RESEARCH_MARGIN):
    steel=profiles(contour)
    wheel=np.array(cfg['axle']['wheel_axle_center_world_m'],float)
    radius=float(cfg['axle']['rod_radius_mm_from_stl'])
    require(abs(radius-RADIUS_SHAFT)<1e-8,'P7_UNEXPECTED_AXLE_DIAMETER')
    min_center=[float('inf')]*2; min_frame=[None,None];failures=[];count=0;last=None
    traces=[]
    for i in range(1298,2172):
        xy=to_cad_xy(wheel,pos[i],cfg)
        traces.append([float(xy[0]),float(xy[1])])
        samples=[(i,xy)]
        if last is not None:
            samples.extend((i,(1.-a)*last+a*xy) for a in (.2,.4,.6,.8))
        for frame,point in samples:
            for j,k in enumerate(('3','11')):
                dist=nearest_boundary_distance(point,steel[k]['outline_xy_mm'])
                count+=1
                if dist<min_center[j]: min_center[j]=dist;min_frame[j]=frame
                if dist<radius+min_margin-1e-9:
                    if len(failures)<20: failures.append({'frame':frame,'plate':k,'center_to_steel_mm':dist})
        last=xy
    result={'sweep_xy_section_only':True,'frame_range_inclusive':[1298,2171],
            'two_plates':['3','11'],'total_frame_and_interpolation_samples':count,
            'interpolation_per_pair':4,'shaft_radius_mm':radius,'required_research_margin_mm':min_margin,
            'min_center_to_steel_mm':min(min_center),'min_axle_radial_margin_mm':min(min_center)-radius,
            'plate_center_distance_mm':min_center,'min_frame_by_plate':min_frame,
            'failed_samples_preview':failures,'passes_2d_section_gate':len(failures)==0}
    return result,traces


def vec(elem):
    v=np.fromstring(elem.get('pos',''),sep=' ')
    require(v.size==3,'P7_XML_BAD_POS '+str(elem.get('name')))
    return v


def fmt(arr):return ' '.join(format(float(x),'.15g') for x in arr)


def named(root,tag,name):
    hit=[x for x in root.iter(tag) if x.get('name')==name]
    require(len(hit)==1, 'P7_XML_BAD_UNIQUE '+tag+' '+name)
    return hit[0]


def import_v13(freeze):
    ctrl=freeze/CTRL_REL
    sys.path.insert(0,str(ctrl))
    import R1_V13_SIMULATION as v13
    import R1_V11_CORE_FROZEN as core
    return v13,core,json.loads((ctrl/'R1_V13_CALIBRATION.json').read_text())


def stage_scene(frozen,project,cfg,rows):
    v13,core,cal=import_v13(frozen)
    root_runs=project/'r1_runs';root_runs.mkdir(parents=True,exist_ok=True)
    run=root_runs/('R1_T03_P7_GUIDED_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir()
    staged=v13.build_stage(frozen/SOURCE_REL,run,cal)
    tree=ET.parse(staged);root=tree.getroot()
    old=staged.parent/'G23_gapxe_1.stl'
    require(old.is_file() and sha(old)==cfg['source_sha256']['legacy_G23_stl'], 'P7_STAGE_ORIGINAL_G23_MISMATCH')
    old.rename(staged.parent/'G23_ORIGINAL_V13_REFERENCE_ONLY.stl')
    target=staged.parent/P7_MESH.name
    shutil.copy2(P7_MESH,target)
    named(root,'mesh','cad_G23_gapxe_1').set('file',target.name)
    shift=np.array(cfg['branch_robot_base_extra_shift_world_m'],float)
    require(np.allclose(shift,[-.05,0,0]),'P7_BAD_BASE_SHIFT')
    for tag,key in [('frame','R1_D1_MOUNT_UNCALIBRATED'),('body','R1_V05_NAMED_CAD_PEDESTAL_PROXY'),('body','R1_V21_WORLD_FIXED_STAND')]:
        e=named(root,tag,key);e.set('pos',fmt(vec(e)+shift))
    wheel=named(root,'body','module_G08_banhxe_1')
    cargo=named(root,'body','module_G23_gapxe_1')
    seat=np.array(cfg['cad_to_frozen']['axle_seat_local_m'],float)
    for s,label in [(-1,'L'),(1,'R')]:
        ET.SubElement(wheel,'site',name='P7_AXLE_TIP_'+label,type='sphere',pos=fmt([0,0,s*.163]),size='.007',rgba='.98 .8 .12 .8',group='4')
        ET.SubElement(cargo,'site',name='P7_G23_SLOT_'+label,type='sphere',pos=fmt([seat[0],seat[1],s*.123]),size='.007',rgba='.0 .87 .98 .8',group='4')
    root.set('model','R1_P7_R13_DUAL_SLOT_GEOMETRY_CANDIDATE_NONPHYSICAL')
    dest=staged.parent/STAGE_NAME
    tree.write(dest,encoding='utf-8',xml_declaration=True)
    static=core.static_gate(dest,rows)
    require(static['v09_gf01_tcp_hard_attached'],'P7_GF01_MOUNT_BROKEN')
    require(abs(vec(named(root,'frame','R1_D1_MOUNT_UNCALIBRATED'))[0]-cfg['candidate_staged_base_mount_m'][0])<1e-7,'P7_BASE_SHIFT_WRONG')
    require(named(root,'geom','display_G23_gapxe_1').get('contype')=='0', 'P7_UNEXPECTED_G23_CONTACT_ENABLED')
    report={'status':'SCENECHECK_PASS_VISUAL_ONLY','stage_xml':str(dest),'frozen_source_unmodified':True,
            '13_solid_G23_candidate_loaded':True,'static_gate':static,
            'visual_collision_geoms_disabled':True,'mj_step_or_contact_solve_tested':False,
            'physical_mating_approved':False,'robot_command_authorized':False}
    (run/'P7_SCENECHECK_REPORT.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print('P7_SCENECHECK_PASS; staged only:',dest,flush=True)
    return dest,run,core,cal


def native_kinematic_check(stage,run,core,cal,rows,cfg):
    try: import mujoco as mu
    except ImportError as e: raise RuntimeError('P7_MUJOCO_NOT_INSTALLED; use Verify or SceneCheck') from e
    with tempfile.TemporaryDirectory(prefix='r1_p7_ascii_scene_') as td:
        scenedir=Path(td)/'scene';shutil.copytree(stage.parent,scenedir)
        model=mu.MjModel.from_xml_path(str(scenedir/stage.name))
        data=mu.MjData(model)
        ids=core.native_ids(mu,model)
        wheel_id=mu.mj_name2id(model,mu.mjtObj.mjOBJ_BODY,'module_G08_banhxe_1')
        steel=profiles(P7_CONT)
        rel_t=np.asarray(cal['flange_to_G23_translation_m']);rel_r=np.asarray(cal['flange_to_G23_rotation_matrix'])
        max_tracking=0.;max_tip=0.;max_wheel=0.;min_center=1e9;fail=[];count=0;last=None;old_mounted=None;jump=0.
        for i,r in enumerate(rows):
            q=np.array([float(r[k]) for k in QCOLS]);core.set_pose(mu,model,data,ids,q,float(r['jaw_open_fraction']))
            flange,rot=core.get_mount(data,ids)
            pred=flange+rot@rel_t
            wanted=np.array([float(r[k]) for k in ('desired_tool_x_m','desired_tool_y_m','desired_tool_z_m')])
            max_tracking=max(max_tracking,float(np.linalg.norm(pred-wanted)*1000))
            if i==295:old_mounted=pred.copy()
            if i==296:jump=float(np.linalg.norm(pred-old_mounted)*1000)
            if r['G23_mode']=='SOURCE':cargo=np.array([-.450,-.245,.370]);orient=np.eye(3)
            elif r['G23_mode']=='ATTACH':cargo=pred;orient=rot@rel_r
            else:cargo=np.array([-.30285,-.24368,0]);orient=np.eye(3)
            data.mocap_pos[ids['mocap']]=cargo
            require(float(np.linalg.norm(orient-np.eye(3)))<1e-4,'P7_NATIVE_NONZERO_G23_ORIENTATION_FRAME_'+str(i))
            data.mocap_quat[ids['mocap']]=[1,0,0,0]
            mu.mj_forward(model,data)
            if r['G23_mode']=='ATTACH':
                tips=np.array([data.site_xpos[s].copy() for s in ids['tips']]);goals=np.array([data.site_xpos[s].copy() for s in ids['goals']])
                max_tip=max(max_tip,float(np.max(np.linalg.norm(tips-goals,axis=1))*1000))
            if 1298<=i<=2171:
                wp=data.xpos[wheel_id].copy();cp=data.xpos[ids['cargo']].copy()
                max_wheel=max(max_wheel,float(np.linalg.norm(wp-np.asarray(cfg['axle']['wheel_axle_center_world_m']))*1000))
                xy=to_cad_xy(wp,cp,cfg)
                for a in ([xy] if last is None else [xy]+[(1.-w)*last+w*xy for w in (.2,.4,.6,.8)]):
                    for k in ('3','11'):
                        c=nearest_boundary_distance(a,steel[k]['outline_xy_mm'])
                        min_center=min(min_center,c);count+=1
                        if c<12.-1e-9 and len(fail)<20:fail.append([i,k,c])
                last=xy
        ok=(max_tracking<1. and max_tip<5. and jump<1. and max_wheel<.05 and not fail)
        obj={'status':'PASS_KINEMATIC_REPLAY_ONLY' if ok else 'BLOCKED','MuJoCo_native_loaded':True,
             'num_frames':len(rows),'max_tracking_error_mm':max_tracking,'max_four_tip_error_mm':max_tip,
             'grasp_transition_jump_mm':jump,'max_wheel_center_error_mm':max_wheel,
             'min_axle_center_to_steel_mm':min_center,'min_radial_nominal_margin_mm':min_center-10.,
             'checked_2D_samples':count,'fail_preview':fail,'contact_geoms_disabled':True,
             'mj_step_and_contact_forces_tested':False,'full_3d_collision_certified':False,
             'hardware_authorized':False,'successful_training_teacher_qualified':False}
        out=run/'P7_NATIVE_KINEMATIC_REPORT.json';out.write_text(json.dumps(obj,indent=2))
        print('P7_NATIVE_KINEMATIC',obj['status'],'min nominal radial clearance %.3f mm'%(min_center-10.),flush=True)
        return ok


def mesh_audit(frozen,position,run=None):
    """Sparse triangular intersection audit, not a continuous swept safety certificate."""
    try:
        import vtk
        from vtk.util.numpy_support import numpy_to_vtk,vtk_to_numpy
    except ImportError as e: raise RuntimeError('P7_VTK_NOT_INSTALLED; pip install vtk for optional MeshAudit') from e
    g08=frozen/'02_FROZEN_V09_SCENE/scene/G08_banhxe_1.stl'
    require(g08.is_file(),'P7_G08_MESH_NOT_FOUND')
    cfg=json.loads(CFG.read_text())
    wheel=np.asarray(cfg['axle']['wheel_axle_center_world_m'],dtype=float)
    c=vtk.vtkSTLReader();c.SetFileName(str(P7_MESH));c.Update()
    w=vtk.vtkSTLReader();w.SetFileName(str(g08));w.Update()
    t0=vtk.vtkTransform();t1=vtk.vtkTransform();t1.Translate(wheel)
    cd=vtk.vtkCollisionDetectionFilter()
    cd.SetInputConnection(0,c.GetOutputPort());cd.SetInputConnection(1,w.GetOutputPort());cd.SetTransform(0,t0);cd.SetTransform(1,t1)
    cd.SetBoxTolerance(0.);cd.SetCellTolerance(0.);cd.SetNumberOfCellsPerNode(4)
    cd.SetCollisionModeToFirstContact();cd.GenerateScalarsOff()
    frames=[0,296,1015,1298,1400,1550,1680,1681,1750,1800,1850,1900,1950,2000,2023,2050,2100,2171,2172]
    records=[]
    for i in frames:
        t0.Identity();t0.Translate(position[i]);cd.Update()
        records.append({'frame':i,'triangle_intersections_detected':int(cd.GetNumberOfContacts())})
    # Signed vertex-to-triangle distance is supplemental, not a full triangle separation lower bound.
    sdf=vtk.vtkImplicitPolyDataDistance();sdf.SetInput(w.GetOutput())
    verts=vtk.util.numpy_support.vtk_to_numpy(c.GetOutput().GetPoints().GetData()).astype(float)
    sframes=sorted(set([0,272,295,296,1015,1146,1298,1324,1400,1450,1500,1550,1600,1650,1680,1681,1700,1750,1800,1850,1900,1950,2000,2023,2050,2100,2150,2171,2172,2200,2621]+list(range(1324,2172,25))))
    signed=[]
    for i in sframes:
        pts=np.ascontiguousarray(verts+position[i]-wheel)
        vi=numpy_to_vtk(pts,deep=False);dist=vtk.vtkDoubleArray();sdf.EvaluateFunction(vi,dist)
        vals=vtk_to_numpy(dist)
        signed.append({'frame':i,'minimum_signed_vertex_distance_mm':float(np.min(vals)*1000),'inside_vertices':int(np.count_nonzero(vals<0))})
    ok=(not any(x['triangle_intersections_detected'] for x in records) and not any(x['inside_vertices'] for x in signed))
    res={'status':'PASS_SAMPLED_TRIANGLES_ONLY' if ok else 'SAMPLED_MESH_COLLISION_DETECTED',
         'triangle_frame_count':len(records),'signed_vertex_sample_frames':len(signed),
         'minimum_sampled_signed_vertex_distance_mm':min(x['minimum_signed_vertex_distance_mm'] for x in signed),
         'triangle_tests':records,'signed_vertex_tests':signed,
         'GF01_full_mesh_continuous_checked':False,'complete_3D_swept_collision_certified':False,
         'physics_contact_or_reaction_force_qualified':False,'hardware_authorized':False}
    if run:
        path=run/'P7_MESH_3D_AUDIT.json';path.write_text(json.dumps(res,indent=2),encoding='utf-8')
        print('P7_MESH_REPORT',path,flush=True)
    print('P7_MESH_AUDIT',res['status'],'3D signed minimum %.3f mm'%res['minimum_sampled_signed_vertex_distance_mm'],flush=True)
    return ok


def export_view_html(project,cfg,position,oldres,newres):
    import html
    wheel=np.asarray(cfg['axle']['wheel_axle_center_world_m'],float)
    path=np.array([to_cad_xy(wheel,p,cfg) for p in position[1298:2172]],float)
    original=profiles(P6_CONT)['3']['outline_xy_mm']
    modified=profiles(P7_CONT)['3']['outline_xy_mm']
    # Plot X/Y in millimeters as a SVG local canvas. A full polyline traces the moving axle.
    xlow,xhigh=-1095.,-600.;ylow,yhigh=-247.,47.
    def pxy(x,y):return f'{x-xlow:.2f},{yhigh-y:.2f}'
    def pathd(points): return 'M'+' L'.join(pxy(float(x),float(y)) for x,y in points)+' Z'
    oldpath=pathd(original);newpath=pathd(modified)
    track=' '.join(pxy(x,y) for x,y in path)
    track_data=json.dumps(np.round(path,4).tolist(),separators=(',',':'))
    width=xhigh-xlow;height=yhigh-ylow
    old_margin=oldres['min_axle_radial_margin_mm'];new_margin=newres['min_axle_radial_margin_mm']
    # HTML content remains self-contained. No OpenGL; geometry in XY CAD section only.
    template='''<!doctype html><html lang="vi"><head><meta charset="utf-8"><title>R1 T03 P7 - P6 vs P7 CAD 2D</title>
<style>body{background:#0c1320;color:#e5efff;font-family:system-ui,Arial,sans-serif;margin:0;padding:20px} h1{font-size:22px;max-width:1100px;margin:0 auto 10px} p{max-width:1100px;margin:10px auto;color:#a9bbd1} .controls{max-width:1100px;margin:18px auto;padding:14px;background:#1c293d;border-radius:12px;display:flex;gap:14px;align-items:center;flex-wrap:wrap} input[type=range]{flex:1;min-width:200px}.twobox{max-width:1100px;margin:auto;display:grid;grid-template-columns:repeat(auto-fit,minmax(470px,1fr));gap:14px}.panel{border:1px solid #30425e;border-radius:12px;overflow:hidden;background:#142136;padding:12px}.panel h2{font-size:16px;margin:3px 0 6px}.panel small{color:#a9bbd1}svg{background:#0e1b2d;max-width:100%;height:auto;overflow:visible;border-radius:6px}.steel{fill:#66788c;fill-opacity:.65;stroke:#abc2d8;stroke-width:.65;fill-rule:evenodd}.track{fill:none;stroke:#ffcc6b;stroke-width:1.0;stroke-dasharray:4 3}.shaft{fill:#09a5d7;fill-opacity:.24;stroke:#30cfff;stroke-width:1.2}.margin{fill:none;stroke:#f8b453;stroke-width:.75;stroke-dasharray:3 2}.axis{fill:#eaf8ff}.foot{font-size:12px;line-height:1.45;color:#a9bbd1}</style></head><body>
<h1>SCARA R1 - T03 P7 | Kiểm tra rãnh CAD XY</h1><p>So sánh P6 và P7 với quỹ đạo giữ nguyên. Vòng màu xanh: trục Ø20 mm; vòng nét đứt: giới hạn biên nghiên cứu 2 mm. Đây <b>không phải</b> phép chứng nhận va chạm 3D hoặc lắp ráp vật lý.</p>
<div class="controls"><button onclick="jump(1298)">Bắt đầu vào</button><button onclick="jump(1681)">Theo rãnh</button><button onclick="jump(2171)">Điểm đặt</button><label>Frame <strong id="frame">2171</strong></label><input type="range" id="slider" min="1298" max="2171" step="1" value="2171" oninput="draw(+this.value)"><label id="coords"></label></div>
<div class="twobox"><section class="panel"><h2>P6: Rãnh cũ</h2><small>Biên nhỏ nhất @OLD@ mm</small><svg viewBox="0 0 @W@ @H@"><path class="steel" d="@OLDPATH@"/><polyline class="track" points="@TRACK@"/><circle class="margin" id="p6-margin" r="12"/><circle class="shaft" id="p6-shaft" r="10"/><circle class="axis" id="p6-center" r="1.3"/></svg></section>
<section class="panel"><h2>P7: Rãnh tăng R10.8 → R13</h2><small>Biên nhỏ nhất @NEW@ mm</small><svg viewBox="0 0 @W@ @H@"><path class="steel" d="@NEWPATH@"/><polyline class="track" points="@TRACK@"/><circle class="margin" id="p7-margin" r="12"/><circle class="shaft" id="p7-shaft" r="10"/><circle class="axis" id="p7-center" r="1.3"/></svg></section></div>
<p class="foot">Hai tấm thép dày 25 mm nằm ở Z ≈ ±130 mm, có cùng tiết diện XY. Chỉ khảo sát tâm trục–thép theo quỹ đạo 1298–2171 với 4 mẫu phụ/đoạn. Cần kiểm toán 3D liên tục, dung sai CAD, lực kẹp và độ bền trước nghiệm thu.</p>
<script>const pts=@POINTS@;const xl=@XLOW@,yh=@YHIGH@;function draw(f){let i=Math.min(pts.length-1,Math.max(0,f-1298));let p=pts[i],x=p[0]-xl,y=yh-p[1];document.getElementById('frame').textContent=f;document.getElementById('coords').textContent='Trục CAD: ('+p[0].toFixed(1)+', '+p[1].toFixed(1)+') mm';for(let a of ['p6','p7'])for(let b of ['shaft','margin','center']){let e=document.getElementById(a+'-'+b);e.setAttribute('cx',x);e.setAttribute('cy',y)}}function jump(f){document.getElementById('slider').value=f;draw(f)}draw(2171)</script></body></html>'''
    replacements={'@OLD@':f'{old_margin:.3f}','@NEW@':f'{new_margin:.3f}',
                 '@W@':str(width),'@H@':str(height),'@OLDPATH@':oldpath,'@NEWPATH@':newpath,
                 '@TRACK@':track,'@POINTS@':track_data,'@XLOW@':str(xlow),'@YHIGH@':str(yhigh)}
    for k,v in replacements.items():template=template.replace(k,v)
    folder=project/'r1_runs'/'R1_T03_P7_VIEW_SAFE';folder.mkdir(parents=True,exist_ok=True)
    dest=folder/'P7_P6_VS_P7_SLOT_2D_INTERACTIVE.html';dest.write_text(template,encoding='utf-8')
    print('P7_VIEWSAFE_HTML',dest,flush=True)
    return dest


def run(argv=None):
    ap=argparse.ArgumentParser(description='SCARA P7 two slot widening, research-only collision candidate')
    ap.add_argument('--ProjectRoot',required=True)
    ap.add_argument('--FreezeDir',default='')
    ap.add_argument('--Mode',choices=['Verify','Audit','ViewSafe','SceneCheck','NativeCheck','MeshAudit','ViewMuJoCo'],default='Verify')
    ap.add_argument('--StartFrame',type=int,default=1681)
    ap.add_argument('--Speed',type=float,default=0.8)
    ap.add_argument('--NoLoop',action='store_true')
    opt=ap.parse_args(argv)
    require(0<=opt.StartFrame<2622,'P7_BAD_VIEWER_START_FRAME')
    require(0.1<=opt.Speed<=5.0,'P7_BAD_VIEWER_SPEED')
    project=Path(opt.ProjectRoot).expanduser().resolve()
    require(project.is_dir(),'P7_PROJECT_ROOT_MISSING')
    cfg=verify_package()
    rows,t,q,pos=load_plan(cfg)
    p6old,_=slot_sweep(cfg,pos,P6_CONT,0.)
    p7new,traces=slot_sweep(cfg,pos,P7_CONT,MIN_RESEARCH_MARGIN)
    print('P7_P6_COMPARISON min nominal gap P6 %.3f mm => P7 %.3f mm; %d samples'%
         (p6old['min_axle_radial_margin_mm'],p7new['min_axle_radial_margin_mm'],p7new['total_frame_and_interpolation_samples']),flush=True)
    require(p7new['passes_2d_section_gate'],'P7_2D_SLOT_GATE_FAIL '+str(p7new['failed_samples_preview']))
    # Full V13 source verify required for every mode, and never modified.
    frozen=find_frozen(project,opt.FreezeDir,cfg)
    check_frozen(frozen,cfg)
    if opt.Mode=='Audit':
        out=project/'r1_runs'/'R1_T03_P7_OFFLINE_AUDIT';out.mkdir(parents=True,exist_ok=True)
        result={'status':'PASS_2D_ONLY','P6_reference':p6old,'P7_candidate':p7new,
                'physical_collision_safe':False,'3D_continuous_certified':False,'hardware_authorized':False}
        dest=out/'P7_2D_SLOT_AUDIT.json';dest.write_text(json.dumps(result,indent=2,ensure_ascii=False))
        print('P7_2D_REPORT',dest,flush=True)
    if opt.Mode=='Verify':print('P7_VERIFY_PASS: 2D research gate only; no physical approval.',flush=True)
    if opt.Mode=='ViewSafe':export_view_html(project,cfg,pos,p6old,p7new)
    if opt.Mode=='MeshAudit':
        dest=project/'r1_runs'/'R1_T03_P7_MESH_AUDIT';dest.mkdir(parents=True,exist_ok=True)
        require(mesh_audit(frozen,pos,dest),'P7_MESH_AUDIT_SAMPLED_COLLISION')
    if opt.Mode in ('SceneCheck','NativeCheck','ViewMuJoCo'):
        stage,run_dir,core,cal=stage_scene(frozen,project,cfg,rows)
        check_frozen(frozen,cfg,full=False)
        if opt.Mode=='NativeCheck':require(native_kinematic_check(stage,run_dir,core,cal,rows,cfg),'P7_NATIVE_KINEMATIC_BLOCKED')
        if opt.Mode=='ViewMuJoCo':
            from p7_live_viewer import replay_viewer
            replay_viewer(stage,run_dir,core,cal,rows,cfg,start_frame=opt.StartFrame,speed=opt.Speed,loop=not opt.NoLoop)
    return 0


if __name__=='__main__':
    try:sys.exit(run())
    except Exception as exc:
        traceback.print_exc()
        print('P7_BLOCKED_NO_ROBOT_HARDWARE_RELEASE:',str(exc),flush=True)
        sys.exit(2)

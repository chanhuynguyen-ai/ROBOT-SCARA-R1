#!/usr/bin/env python3
"""R1 V13 G23 staged mount-calibration CANDIDATE, never a physical deployment.

Native MuJoCo check verifies a deterministic 2,622-frame trajectory against
its original V09 assembled GF-01 and frozen V5.1.3 object waypoints.
It NEVER modifies the original files, never runs mj_step or motors, and
never indicates a physical collision-safe/force-qualified assembly.
"""
from __future__ import annotations
import argparse
import bisect
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import R1_V11_CORE_FROZEN as core

ROOT=Path(__file__).resolve().parent
CAL_PATH=ROOT/'R1_V13_CALIBRATION.json'
TRAJ=ROOT/'R1_V13_TRAJECTORY_2622.csv'
FROZEN=ROOT/'R1_V513_ORIGINAL.zip'
VERSION='R1_V13_COVER_ALIGN_SPEEDFIX_20261028'
CARGO=core.G23
FEEDER=np.array([-.450,-.245,.370])
DATUM=np.array([-.30285,-.24368,0.])
HEADS=core.HEADS
JOINTS=core.Q_NAMES


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for piece in iter(lambda:f.read(1048576),b''):
            h.update(piece)
    return h.hexdigest()


def require(condition,description):
    if not condition:
        raise RuntimeError(description)


def load_plan():
    cal=json.loads(CAL_PATH.read_text(encoding='utf-8-sig'))
    require(sha256(TRAJ)==cal['proposed_csv_sha256'],'V13_TRAJECTORY_SHA_MISMATCH')
    require(sha256(FROZEN)==cal['original_V513_zip_sha256'],'V513_FROZEN_ARCHIVE_SHA_MISMATCH')
    with TRAJ.open(newline='',encoding='utf-8-sig') as f:
        rows=list(csv.DictReader(f))
    require(len(rows)==2622,'EXPECTED_2622_FRAMES')
    require(all(r['phase'] and r['G23_mode'] for r in rows),'BAD_PHASES')
    modes={x['G23_mode'] for x in rows}
    require(modes=={'SOURCE','ATTACH','SEATED'},'BAD_CARGO_STATE_SET')
    t=np.array([float(r['t_s']) for r in rows],dtype=float)
    require(bool(np.all(np.diff(t)>0.0)),'TIMESTAMPS_NOT_STRICTLY_INCREASING')
    qs=np.array([[float(r[k]) for k in ('q1_rad','q3_m','q2_rad','q4_rad','pitch_rad')] for r in rows])
    require(bool(np.isfinite(qs).all()),'NONFINITE_JOINT_PLAN')
    return cal,rows,qs,t


def find_sources(project,scene=None,reference=None):
    if not scene:
        hits=list((project/'r1_runs').glob('R1_V09_PHYSICAL_TIP_ATTACH_*/scene/R1_V09_GF01_MATED.xml'))
        require(bool(hits),'ORIGINAL_R1_V09_SCENE_NOT_FOUND')
        scene=max(hits,key=lambda p:p.stat().st_mtime)
    else:scene=Path(scene)
    if not reference:
        hits=list((project/'r1_runs').glob('R1_V10_G23_PICK_PLACE_*/R1_V10_G23_TRAJECTORY_2622.csv'))
        require(bool(hits),'REFERENCE_V10_TRACE_NOT_FOUND')
        reference=max(hits,key=lambda p:p.stat().st_mtime)
    else:
        reference=Path(reference)
        if reference.is_dir():reference=reference/'R1_V10_G23_TRAJECTORY_2622.csv'
    require(scene.is_file(),'V09_XML_NOT_FOUND: '+str(scene))
    require(reference.is_file(),'V10_CSV_NOT_FOUND: '+str(reference))
    return scene,reference


def build_stage(source,run,cal):
    """Build a fresh visual-only staged scene; never alter V09 or frozen motion.

    Fixes all 3 independent scene issues:
      1. the required V11 registration witness site,
      2. alignment of BOTH stand proxy and its fixed ground-tile body,
      3. four orange GF01 C-head bridge visual geoms.
    """
    staged=run/'scene'
    shutil.copytree(source.parent,staged)
    root=ET.parse(source).getroot()
    root.set('model','R1_V13_COVER_ALIGN_SPEEDFIX_SIM_ONLY')
    shift=np.asarray(cal['base_mount_shift_world_m'],dtype=float)
    require(shift.shape==(3,) and np.isfinite(shift).all(),'BAD_CALIBRATION_SHIFT')

    def vec(s):
        v=np.fromstring(s,sep=' ',dtype=float)
        require(v.size==3 and np.isfinite(v).all(),'INVALID_VEC3_XML')
        return v

    def fmt(v):
        return ' '.join(format(float(x),'.15g') for x in v)

    def unique(tag,name):
        hits=[e for e in root.iter(tag) if e.get('name')==name]
        require(len(hits)==1,'MISSING_OR_DUPLICATE_'+name)
        return hits[0]

    # Reuse the exact, already-validated V13 motion solution.
    mount=unique('frame',cal['base_mount_frame'])
    original=vec(mount.get('pos'))
    require(float(np.linalg.norm(original-np.asarray(cal['original_base_mount_world_m'],float)))<1e-8,
            'SOURCE_BASE_POSE_CHANGED')
    moved_mount=original+shift
    mount.set('pos',fmt(moved_mount))

    # Pedestal model and WORLD ground tile are two separate bodies in V09.
    # Both must use the same X/Z shift; world Y of foot/tile stays unchanged.
    pedestal=unique('body','R1_V05_NAMED_CAD_PEDESTAL_PROXY')
    foundation=unique('body','R1_V21_WORLD_FIXED_STAND')
    shift_horizontal=np.array([shift[0],0.0,shift[2]],dtype=float)
    for body in (pedestal,foundation):
        body.set('pos',fmt(vec(body.get('pos'))+shift_horizontal))

    named={g.get('name'):g for g in pedestal.findall('geom')}
    def pedestal_geom(name):
        require(name in named,'MISSING_PEDESTAL_GEOM_'+name)
        return named[name]
    foot=pedestal_geom('R1_V05_STAND_FOOT_PLATE')
    top=pedestal_geom('R1_V05_STAND_TOP_AT_CAD_BASE')
    foot_p,foot_sz=vec(foot.get('pos')),vec(foot.get('size'))
    top_p,top_sz=vec(top.get('pos')),vec(top.get('size'))
    # Lower/raise only the top to retain its unchanged relation with the robot mount.
    top_p[1]+=shift[1]
    top.set('pos',fmt(top_p))
    # Re-size four vertical uprights; keep foot on original ground plane.
    foot_top=foot_p[1]+foot_sz[1]
    top_bottom=top_p[1]-top_sz[1]
    # 2 mm overlap at each end avoids visual gaps between welded/bolted proxy parts.
    post_low=foot_top-0.002
    post_high=top_bottom+0.002
    require(post_high-post_low>0.05,'PEDESTAL_POST_HEIGHT_NOT_POSSIBLE')
    post_y=(post_high+post_low)/2.
    post_half=(post_high-post_low)/2.
    for index in ('0_0','0_1','1_0','1_1'):
        g=pedestal_geom('R1_V05_STAND_POST_'+index)
        gp,gs=vec(g.get('pos')),vec(g.get('size'))
        gp[1]=post_y;gs[1]=post_half
        g.set('pos',fmt(gp));g.set('size',fmt(gs))
    for suffix in ('-1','1'):
        g=pedestal_geom('R1_V05_STAND_BRACE_'+suffix)
        gp=vec(g.get('pos'));gp[1]=post_y
        g.set('pos',fmt(gp))

    # Only the VISUAL skin with the arm cut-out changes parent.  It was
    # BASE_FIXED in V09 although its opening is aligned to the arm at q1=q3=0.
    # Attach to elevator J3 (child of rotating J1) so the CUT-OUT tracks the
    # actual arm through both J1 yaw and J3 lift.  Joint tree, FK, GF01, cargo,
    # robot stand, and the entire frozen 2622-frame motion remain unchanged.
    cover_name='R1_V05_VIS_R1_V05_PART_31_BASE_FIXED_00'
    cover=unique('geom',cover_name)
    require(cover.get('type')=='mesh','COVER_NOT_MESH')
    require(cover.get('mass')=='0','COVER_MUST_BE_MASSLESS_VISUAL')
    require(cover.get('contype')=='0' and cover.get('conaffinity')=='0',
            'COVER_MUST_BE_NONCOLLIDING_VISUAL')
    original_cover_body=unique('body','R1__R1V22_IDENTITY_SOURCE_ROBOT_FRAME')
    follow_body=unique('body','R1__elevator_J3')
    require(cover in list(original_cover_body.findall('geom')),
            'COVER_PARENT_UNEXPECTED_IN_FROZEN_V09')
    require(any(j.get('name')=='R1__J3' for j in follow_body.findall('joint')),
            'J3_KINEMATIC_BODY_MISSING')
    require(unique('body','R1__rotating_tower_J1') is not follow_body,
            'COVER_MUST_TRACK_BOTH_J1_AND_J3')
    original_cover_body.remove(cover)
    follow_body.append(cover)

    # Remove, rather than recolor, the four visual-only amber GF01 squares.
    # They are zero-mass, non-colliding marker plates, not the structural CAD
    # fingers or the TCP.  This also avoids alpha/material overrides in Viewer.
    bridge_names=('R1_V05_CHEAD_POSITION_BRIDGE_NEG_R1',
                  'R1_V05_CHEAD_POSITION_BRIDGE_NEG_R2',
                  'R1_V05_CHEAD_POSITION_BRIDGE_POS_R1',
                  'R1_V05_CHEAD_POSITION_BRIDGE_POS_R2')
    for name in bridge_names:
        marker=unique('geom',name)
        require(marker.get('mass')=='0' and
                marker.get('contype')=='0' and marker.get('conaffinity')=='0',
                'GRIPPER_ORANGE_ELEMENT_NOT_VISUAL_'+name)
        parents=[p for p in root.iter() if marker in list(p)]
        require(len(parents)==1,'ORANGE_PARENT_NOT_UNIQUE_'+name)
        parents[0].remove(marker)

    # Recolor auxiliary dock-lock markers too: no bright amber rectangles
    # remain in the staged cell.  These belong to the dock, not the gripper.
    for suffix in ('0_-1','0_1','1_-1','1_1'):
        unique('geom','R1_C1_DOCK_LOCK_'+suffix).set('rgba','.42 .50 .60 .95')

    # Restore exactly the witness required by R1_V11_CORE_FROZEN.native_ids.
    world=root.find('worldbody')
    require(world is not None,'MISSING_WORLD_BODY')
    witnesses=[s for s in root.iter('site') if s.get('name')=='R1_V11_REGISTERED_FEEDER_WITNESS']
    require(not witnesses,'DUPLICATE_V11_REGISTERED_WITNESS_SOURCE')
    ET.SubElement(world,'site',name='R1_V11_REGISTERED_FEEDER_WITNESS',type='sphere',
                  pos='-.450 -.245 .370',size='.014',rgba='.1 .7 1 .8',group='4')
    ET.SubElement(world,'site',name='R1_V13_FIXED_ASSEMBLY_DATUM',type='sphere',
                  pos='-.30285 -.24368 0',size='.017',rgba='.15 1 .2 .9',group='4')
    ET.SubElement(world,'site',name='R1_V13_ORIGINAL_ROBOT_BASE',type='sphere',
                  pos=fmt(original),size='.02',rgba='1 .2 .15 .8',group='4')
    ET.SubElement(world,'site',name='R1_V13_CANDIDATE_ROBOT_BASE',type='sphere',
                  pos=fmt(moved_mount),size='.02',rgba='.1 .85 1 .8',group='4')

    path=staged/'R1_V13_G23_STAGED_CALIBRATION_CANDIDATE.xml'
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return path


def verify_visual_alignment(stage,cal):
    """Assert alignment of visually separate stand/foundation and required site."""
    root=ET.parse(stage).getroot()
    def named(tag,name):
        hits=[e for e in root.iter(tag) if e.get('name')==name]
        require(len(hits)==1,'VISFIX_MISSING_OR_DUPLICATE_'+name)
        return hits[0]
    def vec(element,key):
        return np.fromstring(element.get(key),sep=' ',dtype=float)
    mount=named('frame',cal['base_mount_frame'])
    pedestal=named('body','R1_V05_NAMED_CAD_PEDESTAL_PROXY')
    ground=named('body','R1_V21_WORLD_FIXED_STAND')
    mp=vec(mount,'pos');pp=vec(pedestal,'pos');gp=vec(ground,'pos')
    dist_ground_ped=float(np.linalg.norm((gp-pp)[[0,2]])*1000.)
    dist_mount_ped=float(np.linalg.norm((mp-pp)[[0,2]])*1000.)
    # R1_V21 pedestal ground tile top vs R1_V05 stand foot bottom.
    foot=named('geom','R1_V05_STAND_FOOT_PLATE')
    tile=named('geom','R1_V21_PED_GROUND_TILE')
    foot_bottom_m=pp[1]+vec(foot,'pos')[1]-vec(foot,'size')[1]
    ground_top_m=gp[1]+vec(tile,'pos')[1]+vec(tile,'size')[1]
    feet_gap_mm=1000.*(foot_bottom_m-ground_top_m)
    # The support top follows the mount height shift while the foot stays grounded.
    top=named('geom','R1_V05_STAND_TOP_AT_CAD_BASE')
    top_center_m=pp[1]+vec(top,'pos')[1]
    top_relative_mount_mm=1000.*(top_center_m-mp[1])
    bridges=('R1_V05_CHEAD_POSITION_BRIDGE_NEG_R1',
             'R1_V05_CHEAD_POSITION_BRIDGE_NEG_R2',
             'R1_V05_CHEAD_POSITION_BRIDGE_POS_R1',
             'R1_V05_CHEAD_POSITION_BRIDGE_POS_R2')
    # Physical and kinematic model unchanged: cover visual geometry only.
    geom_names={g.get('name') for g in root.iter('geom')}
    require(all(name not in geom_names for name in bridges),
            'AMBER_GRIPPER_MARKERS_STILL_PRESENT')
    require(all(g.get('rgba')!='0.98 0.70 0.17 .95' for g in root.iter('geom')),
            'AMBER_DOCK_MARKERS_STILL_PRESENT')
    cover=named('geom','R1_V05_VIS_R1_V05_PART_31_BASE_FIXED_00')
    cover_parent=[b for b in root.iter('body') if cover in list(b.findall('geom'))]
    require(len(cover_parent)==1 and cover_parent[0].get('name')=='R1__elevator_J3',
            'COVER_OPENING_NOT_TRACKING_J1_AND_J3')
    require(cover.get('mass')=='0' and cover.get('contype')=='0',
            'KINEMATIC_COVER_NOT_VISUAL_ONLY')
    # CAD axial Y spans (mm): sliding cover [110.345, 1033.497];
    # stationary lower sleeve [10, 355].  At highest J3, they still overlap.
    q3max=float(cal['joint_q_max'][1])
    sleeve_min_overlap_mm=355.0-(110.34538269+q3max*1000.0)
    require(sleeve_min_overlap_mm>40.,'VISUAL_SLEEVE_WOULD_DISCONNECT')
    named('site','R1_V11_REGISTERED_FEEDER_WITNESS')
    require(dist_ground_ped<1.0,'FOUNDATION_XZ_MISALIGNED')
    require(dist_mount_ped<1.0,'ROBOT_AND_PEDESTAL_XZ_MISALIGNED')
    require(abs(feet_gap_mm)<1.0,'FOOT_HOVERING_OVER_GROUND_TILE')
    require(abs(top_relative_mount_mm+108.0)<1.0,'TOP_PLATE_MOUNT_HEIGHT_MISMATCH')
    return {'site_present':True,'pedestal_ground_XZ_offset_mm':dist_ground_ped,
            'robot_pedestal_XZ_offset_mm':dist_mount_ped,
            'foot_to_ground_tile_vertical_gap_mm':feet_gap_mm,
            'top_plate_relative_to_mount_mm':top_relative_mount_mm,
            'removed_gripper_orange_bridges':len(bridges),
            'visual_cover_parent':'R1__elevator_J3',
            'visual_cover_tracks':['R1__J1','R1__J3'],
            'sleeve_min_overlap_at_max_lift_mm':sleeve_min_overlap_mm,
            'baseline_scene_and_kinematic_trajectory_unmodified':True,
            'structural_and_contact_approval':False,
            'visual_skin_motion_only_not_approved_hardware':True}

def load_reference(where):
    with zipfile.ZipFile(FROZEN) as z:
        require(z.testzip() is None,'V513_BAD_ZIP_CRC')
        out=Path(where).resolve()
        for item in z.infolist():
            target=(out/item.filename).resolve()
            require(target==out or out in target.parents,'V513_ARCHIVE_PATH_TRAVERSAL')
        z.extractall(out)
    sys.path.insert(0,str(out/'src'))
    for module in ('r1_v51_scene','r1_v51_autoplay','clearance_review'):
        sys.modules.pop(module,None)
    from r1_v51_scene import ensure_source
    from r1_v51_autoplay import target_sequence
    from clearance_review import geometry_snapshot,audit_wheel_clearance
    ensure_source(out)
    seq=target_sequence(geometry_snapshot(out/'source_v27'))
    audit=audit_wheel_clearance(seq,geometry_snapshot(out/'source_v27'))
    return seq,audit


def validate_against_reference(rows,reference_csv,seq):
    with Path(reference_csv).open(newline='',encoding='utf-8-sig') as f:
        v10=list(csv.DictReader(f))
    require(len(v10)==len(seq)==len(rows)==2622,'FRAME_COUNT_MISMATCH')
    max_ref=0.
    for i,(r,v,frozen) in enumerate(zip(rows,v10,seq)):
        require(r['phase']==v['phase']==frozen['phase'],'PHASE_MISMATCH_FRAME_'+str(i))
        require(r['G23_mode']==v['G23_mode']==frozen['object_state'],'STATE_MISMATCH_FRAME_'+str(i))
        require(abs(float(r['t_s'])-float(v['t_s']))<1e-5,'TIME_MISMATCH_FRAME_'+str(i))
        p=np.array([float(r[k]) for k in ('desired_G23_x_m','desired_G23_y_m','desired_G23_z_m')])
        max_ref=max(max_ref,float(np.linalg.norm(p-np.asarray(frozen['g23_world']))))
    require(max_ref<1e-8,'V513_G23_PATH_NOT_PRESERVED')
    return max_ref*1000.


def quat_from_rot(R):
    # G23 stays identity throughout this candidate; full helper for defensive use.
    A=np.asarray(R,float)
    trace=float(np.trace(A))
    if trace>0:
        s=math.sqrt(trace+1.)*2
        q=np.array([s*.25,(A[2,1]-A[1,2])/s,(A[0,2]-A[2,0])/s,(A[1,0]-A[0,1])/s])
    else:
        from math import sqrt
        k=int(np.argmax(np.diag(A)))
        if k==0:
            s=sqrt(max(1e-16,1.+A[0,0]-A[1,1]-A[2,2]))*2
            q=np.array([(A[2,1]-A[1,2])/s,s*.25,(A[0,1]+A[1,0])/s,(A[0,2]+A[2,0])/s])
        elif k==1:
            s=sqrt(max(1e-16,1.+A[1,1]-A[0,0]-A[2,2]))*2
            q=np.array([(A[0,2]-A[2,0])/s,(A[0,1]+A[1,0])/s,s*.25,(A[1,2]+A[2,1])/s])
        else:
            s=sqrt(max(1e-16,1.+A[2,2]-A[0,0]-A[1,1]))*2
            q=np.array([(A[1,0]-A[0,1])/s,(A[0,2]+A[2,0])/s,(A[1,2]+A[2,1])/s,s*.25])
    q/=np.linalg.norm(q)
    return q


def run_native(mu,model,data,h,rows,qs,t,cal,baseline_aabb):
    relp=np.array(cal['flange_to_G23_translation_m'])
    relR=np.array(cal['flange_to_G23_rotation_matrix'])
    bounds=h['bounds']
    require(np.all(qs>=bounds[:,0]-1e-8) and np.all(qs<=bounds[:,1]+1e-8), 'JOINT_LIMIT_BREACH')
    rates=np.max(np.abs(np.diff(qs,axis=0)/np.diff(t)[:,None]),axis=0)
    jumps=np.max(np.abs(np.diff(qs,axis=0)),axis=0)
    limit_margin=np.min(np.minimum(qs-bounds[:,0],bounds[:,1]-qs),axis=0)
    grasp=max(i for i,r in enumerate(rows) if r['phase']=='GRASP_MOCK')
    attach=next(i for i,r in enumerate(rows) if r['G23_mode']=='ATTACH')
    seat=max(i for i,r in enumerate(rows) if r['phase']=='LOWER_TO_DATUM_MOCK')
    require(attach==grasp+1,'GRASP_TO_ATTACH_GAP')
    max_tr=0.;max_ang=0.;max_tip=0.;first_jump=0.;grasp_tip=0.;seat_err=1e9;seat_ori=1e9
    last_grasp=None
    for i,(row,q) in enumerate(zip(rows,qs)):
        core.set_pose(mu,model,data,h,q,float(row['jaw_open_fraction']))
        mp,mR=core.get_mount(data,h)
        cp=mp+mR@relp
        cR=mR@relR
        desired=np.array([float(row[k]) for k in ('desired_G23_x_m','desired_G23_y_m','desired_G23_z_m')])
        err=float(np.linalg.norm(cp-desired)*1000.)
        ang=math.degrees(core.rotation_angle(cR))
        if row['G23_mode']=='ATTACH':
            max_tr=max(max_tr,err)
            max_ang=max(max_ang,ang)
        if i==grasp:last_grasp=cp.copy()
        if i==attach:first_jump=float(np.linalg.norm(cp-last_grasp)*1000.)
        if i==seat:seat_err=float(np.linalg.norm(cp-DATUM)*1000.);seat_ori=ang
        if row['G23_mode']=='SOURCE':
            payload=FEEDER;payload_R=np.eye(3)
        elif row['G23_mode']=='ATTACH':
            payload=cp;payload_R=cR
        else:
            payload=DATUM;payload_R=np.eye(3)
        data.mocap_pos[h['mocap']]=payload
        data.mocap_quat[h['mocap']]=quat_from_rot(payload_R)
        mu.mj_forward(model,data)
        if i==grasp or row['G23_mode']=='ATTACH':
            tips=np.array([data.site_xpos[x].copy() for x in h['tips']])
            goals=np.array([data.site_xpos[x].copy() for x in h['goals']])
            dist=np.linalg.norm(tips-goals,axis=1)
            errmm=float(np.max(dist)*1000.)
            max_tip=max(max_tip,errmm)
            if i==grasp:grasp_tip=errmm
    # If the reference failed strict transit, this project cannot claim a pass.
    clearance_mm=baseline_aabb['strict_transit_min_aabb_gap_m']*1000.
    native_pass=bool(max_tr<=1.0 and max_ang<=1.0 and max_tip<=5.0
        and first_jump<=1.0 and seat_err<=5.0 and seat_ori<=5.0
        and clearance_mm>=25. and rates[1]<=.025+1e-6
        and np.all(jumps[[0,2,3,4]]<=.075+1e-6)
        and jumps[1]<=.006+1e-6 and np.min(limit_margin)>.01)
    return {
        'native_mujoco_compiled':True,'native_kinematic_replay_gate_pass':native_pass,
        'physical_insertion_or_grasp_approved':False,
        'force_contact_dynamics_validated':False,
        'max_cargo_tracking_error_mm':max_tr,'max_cargo_orientation_error_deg':max_ang,
        'max_four_tip_error_carry_mm':max_tip,'four_tip_error_at_grasp_mm':grasp_tip,
        'first_attachment_jump_mm':first_jump,
        'fixed_datum_position_error_mm':seat_err,'fixed_datum_angle_error_deg':seat_ori,
        'min_joint_margin_radian_except_J3_m':limit_margin.tolist(),
        'max_joint_velocity_units_s':rates.tolist(),'max_frame_delta_units':jumps.tolist(),
        'strict_wheel_AABB_reference_gap_mm':clearance_mm,
        'reference_mating_unqualified':True,
        'mechanical_support_relocated_or_checked':False,
        'braking_gripper_collision_narrowphase_checked':False,
    }


def native_viewer(mu,viewer_lib,model,data,h,rows,qs,t,cal,accepted,args):
    relp=np.array(cal['flange_to_G23_translation_m'])
    relR=np.array(cal['flange_to_G23_rotation_matrix'])
    pose_at_seat=max(i for i,r in enumerate(rows) if r['phase']=='LOWER_TO_DATUM_MOCK')
    starts={name:float(t[next(i for i,r in enumerate(rows) if r['phase']==phase)])
            for name,phase in [('REGISTER','REGISTER'),('GRASP','GRASP_MOCK'),
                               ('LIFT','VERTICAL_LIFT_210MM'),('TRANSFER','MOVE_OUTSIDE_REAR_WHEEL'),
                               ('PLACE','LOWER_TO_DATUM_MOCK'),('DONE','DONE_REVIEW')]}
    state={'t':starts[args.start], 'paused':args.start!='REGISTER', 'loop':args.loop,
           'focus':None, 'speed':args.speed}
    def key(k):
        if k==32:state['paused']=not state['paused']
        for ch,name in [('G','GRASP'),('U','LIFT'),('T','TRANSFER'),('M','PLACE'),('R','REGISTER')]:
            if k in (ord(ch),ord(ch.lower())):state['t']=starts[name];state['paused']=True
        if k in (ord('L'),ord('l')):state['loop']=not state['loop']
        if k in (ord('+'),ord('=')):
            state['speed']=min(3.,round(state['speed']+.1,2))
            print('[R1 V13 COVERFIX] Viewer speed x%.2f'%state['speed'],flush=True)
        if k in (ord('-'),ord('_')):
            state['speed']=max(.3,round(state['speed']-.1,2))
            print('[R1 V13 COVERFIX] Viewer speed x%.2f'%state['speed'],flush=True)
        if k in (ord('F'),ord('f')):state['focus']='tool'
        if k in (ord('B'),ord('b')):state['focus']='cell'
    def draw(i):
        row=rows[i];mode=row['G23_mode']
        if not accepted and mode=='SEATED':
            i=pose_at_seat
            row=rows[i]
            mode='BLOCKED_HOLD'
        jaw=0. if mode=='BLOCKED_HOLD' else float(row['jaw_open_fraction'])
        core.set_pose(mu,model,data,h,qs[i],jaw)
        mp,mR=core.get_mount(data,h)
        if mode=='SOURCE': payload=FEEDER;R=np.eye(3)
        elif mode in ('ATTACH','BLOCKED_HOLD'):
            payload=mp+mR@relp;R=mR@relR
        else:payload=DATUM;R=np.eye(3)
        data.mocap_pos[h['mocap']]=payload
        data.mocap_quat[h['mocap']]=quat_from_rot(R)
        mu.mj_forward(model,data)
        return row,mode
    draw(0);prev='';then=time.monotonic()
    print('[R1 V13 COVERFIX] '+'KINEMATIC_REPLAY_PASS, PHYSICAL ASSEMBLY NOT QUALIFIED' if accepted
          else '[R1 V13 COVERFIX] BLOCKED: DO NOT RELEASE OR SEAT',flush=True)
    print('[R1 V13 COVERFIX] Space pause; G/U/T/M phases; F tool; B cell; R reset; L loop; +/- viewer speed',flush=True)
    print('[R1 V13 COVERFIX] Viewer start speed x%.2f'%args.speed,flush=True)
    with viewer_lib.launch_passive(model,data,show_left_ui=False,show_right_ui=False,key_callback=key) as v:
        with v.lock():
            v.cam.lookat[:]=[0.,-.25,.4];v.cam.distance=3.3;v.cam.azimuth=120;v.cam.elevation=-20
            for idx in (0,1,2,3,4):
                if idx<len(v.opt.geomgroup):v.opt.geomgroup[idx]=1
            if len(v.opt.sitegroup)>4:v.opt.sitegroup[4]=1
        try:
            while v.is_running():
                now=time.monotonic();dt=min(.1,max(0.,now-then));then=now
                if not state['paused']:
                    state['t']+=dt*state['speed']
                    if state['t']>t[-1]:
                        if state['loop']:state['t']%=t[-1]
                        else:state['t']=t[-1];state['paused']=True
                i=min(len(t)-1,max(0,bisect.bisect_right(t,state['t'])-1))
                row,mode=draw(i)
                if prev!=row['phase']:
                    print('[R1 V13 COVERFIX] %6.2fs %-35s G23=%s'%(t[i],row['phase'],mode),flush=True)
                    prev=row['phase']
                if state['focus']:
                    with v.lock():
                        if state['focus']=='tool':v.cam.lookat[:]=data.xpos[h['mount']];v.cam.distance=1.35
                        else:v.cam.lookat[:]=[0.,-.25,.4];v.cam.distance=3.3
                    state['focus']=None
                v.sync();time.sleep(.012)
        except KeyboardInterrupt:
            print('[R1 V13 COVERFIX] Viewer closed by user',flush=True)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--project',required=True)
    ap.add_argument('--mode',choices=['Static','Check','View'],default='Check')
    ap.add_argument('--scene-xml',default='')
    ap.add_argument('--v10-run',default='')
    ap.add_argument('--start',choices=['REGISTER','GRASP','LIFT','TRANSFER','PLACE','DONE'],default='REGISTER')
    ap.add_argument('--speed',type=float,default=1.3)
    ap.add_argument('--loop',action='store_true')
    ap.add_argument('--allow-unqualified-view',action='store_true')
    args=ap.parse_args()
    require(args.speed>0. and args.speed<=5.,'INVALID_VIEWER_SPEED')
    require(not args.allow_unqualified_view or args.mode=='View','UNQUALIFIED_PREVIEW_ONLY_IN_VIEW_MODE')
    cal,rows,qs,t=load_plan()
    project=Path(args.project)
    src,ref_csv=find_sources(project,args.scene_xml,args.v10_run)
    require(sha256(src)==cal['original_V09_scene_sha256'],'SOURCE_V09_SCENE_SHA_MISMATCH_ABORT')
    require(sha256(ref_csv)==cal['reference_V10_trace_sha256'],'SOURCE_V10_TRACE_SHA_MISMATCH_ABORT')
    require((qs[:,4].max()-qs[:,4].min())<1e-8,'UNEXPECTED_GRIPPER_PITCH_DRIFT')
    runs=project/'r1_runs';runs.mkdir(parents=True,exist_ok=True)
    run=runs/(VERSION+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True,exist_ok=False)
    report_path=run/'R1_V13_NATIVE_REPORT.json'
    report={'release':VERSION,'mode':args.mode,'state':'STATIC_STAGE_PREPARED',
            'simulation_only':True,'hardware_ready':False,
            'original_v09_sha256':cal['original_V09_scene_sha256'],
            'V513_reference_sha256':cal['original_V513_zip_sha256'],
            'original_v10_sha256':cal['reference_V10_trace_sha256'],
            'base_mount_shift_xyz_m':cal['base_mount_shift_world_m'],
            'feeder_m':FEEDER.tolist(),'datum_m':DATUM.tolist(),
            'support_structure_unchanged_unapproved':True,
            'moving_visual_cover_kinematic_only':True,
            'G23_teleport_to_datum':False,
            'frozen_baselines_unmodified':True}
    try:
        stage=build_stage(src,run,cal)
        report['visual_alignment_audit']=verify_visual_alignment(stage,cal)
        structure=core.static_gate(stage,rows)
        report['static_gate']=structure
        report['staged_scene']=str(stage)
        with tempfile.TemporaryDirectory(prefix='R1_V13_REF_') as frozen:
            seq,broadphase=load_reference(frozen)
            report['v513_path_position_max_disagreement_mm']=validate_against_reference(rows,ref_csv,seq)
            report['reference_strict_wheel_aabb_gap_mm']=broadphase['strict_transit_min_aabb_gap_m']*1000
            report['unqualified_mating_frames']=broadphase['unqualified_mating_aabb_near_or_overlap_frames']
        core.write_json(report_path,report)
        print('[R1 V13 COVERFIX] STAGED ONLY: robot-frame shift (X,Y,Z) m =',cal['base_mount_shift_world_m'],flush=True)
        print('[R1 V13 COVERFIX] SHELL CUTOUT follows J1+J3; 4 orange GF01 markers removed; Viewer x%.2f'%args.speed,flush=True)
        print('[R1 V13 COVERFIX] Source / Feeder / Fixed target unchanged. Scene:',stage,flush=True)
        if args.mode=='Static':
            print('[R1 V13 COVERFIX] STATIC PASS; no MuJoCo imported; no motion proven',flush=True)
            return 0
        try:
            import mujoco as mu
            import mujoco.viewer as viewer
        except ImportError as exc:
            raise RuntimeError('NATIVE_MUJOCO_MISSING_IN_PROJECT_VENV') from exc
        with tempfile.TemporaryDirectory(prefix='R1_V13_ASCII_') as temp:
            snap=Path(temp)/'scene';shutil.copytree(stage.parent,snap)
            model=mu.MjModel.from_xml_path(str(snap/stage.name))
            data=mu.MjData(model)
            h=core.native_ids(mu,model)
            result=run_native(mu,model,data,h,rows,qs,t,cal,broadphase)
            report.update(result)
            accepted=bool(result['native_kinematic_replay_gate_pass'])
            report['state']='NATIVE_KINEMATIC_REPLAY_PASS_UNQUALIFIED_MATING' if accepted else 'BLOCKED_NO_RELEASE_NO_SEATED'
            core.write_json(report_path,report)
            print('[R1 V13 COVERFIX] Native goal errors: pick tips',round(result['four_tip_error_at_grasp_mm'],5),
                  'mm; seat',round(result['fixed_datum_position_error_mm'],5),'mm',flush=True)
            print('[R1 V13 COVERFIX] Native full tracking max',round(result['max_cargo_tracking_error_mm'],5),
                  'mm; angular',round(result['max_cargo_orientation_error_deg'],5),'deg',flush=True)
            print('[R1 V13 COVERFIX] Native check:',report['state'],flush=True)
            print('[R1 V13 COVERFIX] Report:',report_path,flush=True)
            if args.mode=='Check':return 0 if accepted else 3
            if not accepted and not args.allow_unqualified_view:
                print('[R1 V13 COVERFIX] VIEW BLOCKED on failed geometry gate',flush=True)
                return 3
            native_viewer(mu,viewer,model,data,h,rows,qs,t,cal,accepted,args)
            return 0 if accepted else 3
    except Exception as ex:
        report['state']='ERROR_ABORTED_NO_RELEASE';report['error']=repr(ex)
        report['traceback']=traceback.format_exc()
        core.write_json(report_path,report)
        print('[R1 V13 COVERFIX] ERROR:',repr(ex),flush=True)
        print('[R1 V13 COVERFIX] Report:',report_path,flush=True)
        return 2

if __name__=='__main__':
    raise SystemExit(main())

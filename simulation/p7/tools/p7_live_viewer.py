#!/usr/bin/env python3
"""Open P6/P7 G23 motion in native MuJoCo viewer (visual-only kinematic replay).

No mj_step or contact forces: the original visual G23/wheel geometries have
contact disabled. A seemingly clear render is NOT a 3D collision certificate.
"""
from __future__ import annotations
import bisect
import math
import shutil
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from r1_p7_windows import KIT, P7_MESH, P7_CONT, P6_CONT, profiles, nearest_boundary_distance, to_cad_xy

P6_MESH = KIT / 'assets/G23_P6_NO_SIDE_COVERS_SCENE_METERS.stl'


def build_reference_scene(p7_stage: Path) -> Path:
    """Only change the visible G23 mesh inside a NEW scene XML; do not modify P7/V13."""
    root = ET.parse(p7_stage).getroot()
    entries = [e for e in root.iter('mesh') if e.get('name') == 'cad_G23_gapxe_1']
    if len(entries) != 1:
        raise RuntimeError('P7_COMPARE_MESH_NOT_UNIQUE')
    dest_stl = p7_stage.parent / P6_MESH.name
    shutil.copy2(P6_MESH, dest_stl)
    entries[0].set('file', dest_stl.name)
    root.set('model', 'P6_REFERENCE_G23_GEOMETRY_VISUAL_ONLY')
    scene = p7_stage.parent / 'P7_VIEW_P6_REFERENCE_ONLY.xml'
    ET.ElementTree(root).write(scene, encoding='utf-8', xml_declaration=True)
    return scene


def replay_viewer(p7_stage, run, core, cal, rows, cfg, start_frame=1681, speed=0.8, loop=True):
    try:
        import mujoco as mj
        import mujoco.viewer as viewer
    except ImportError as err:
        raise RuntimeError('P7_MUJOCO_MISSING: run .venv\\Scripts\\python.exe -m pip install mujoco') from err

    p7_stage = Path(p7_stage)
    p6_stage = build_reference_scene(p7_stage)
    times = np.asarray([float(r['t_s']) for r in rows], dtype=float)
    joint_plan = np.array([[float(r[k]) for k in ('q1_rad','q3_m','q2_rad','q4_rad','pitch_rad')] for r in rows])
    rel_t = np.asarray(cal['flange_to_G23_translation_m'],float)
    rel_r = np.asarray(cal['flange_to_G23_rotation_matrix'],float)
    frozen_axle = np.asarray(cfg['axle']['wheel_axle_center_world_m'],float)
    # The frozen trajectory's cargo orientation is identity throughout this branch.
    # Set/verify the orientation rather than silently rotating geometry in the viewer.
    def pose(i,model,data,ids):
        r=rows[i]
        core.set_pose(mj, model, data, ids, joint_plan[i], float(r['jaw_open_fraction']))
        flange,rot=core.get_mount(data, ids)
        if r['G23_mode']=='SOURCE':
            payload=np.array([-.450,-.245,.370])
        elif r['G23_mode']=='ATTACH':
            payload=flange+rot@rel_t
            if not np.allclose(rot@rel_r,np.eye(3),atol=1.e-4):
                raise RuntimeError('P7_VIEW_NONZERO_G23_ORIENTATION '+str(i))
        elif r['G23_mode']=='SEATED':
            payload=np.array([-.30285,-.24368,0.])
        else:
            raise RuntimeError('P7_VIEW_UNKNOWN_G23_MODE: '+str(r['G23_mode']))
        data.mocap_pos[ids['mocap']]=payload
        data.mocap_quat[ids['mocap']]=[1.,0.,0.,0.]
        mj.mj_forward(model,data)
        return payload

    state={'t':float(times[start_frame]), 'paused':False, 'loop':bool(loop),
           'speed':float(speed), 'variant':'P7', 'request_variant':None, 'focus':None,
           'last_announce':None}
    starts={'G':296,'A':1298,'T':1681,'D':2171,'R':0}
    def keys(code):
        if code==32:state['paused']=not state['paused']
        for key,frame in starts.items():
            if code in (ord(key),ord(key.lower())):
                state['t']=float(times[frame]);state['paused']=True
                print('[P7 VIEW] Jump to',frame,rows[frame]['phase'],flush=True)
        if code in (ord('+'),ord('=')):
            state['speed']=min(5.,round(state['speed']+.2,2))
        if code in (ord('-'),ord('_')):
            state['speed']=max(.1,round(state['speed']-.2,2))
        if code in (ord('L'),ord('l')):state['loop']=not state['loop']
        if code in (ord('F'),ord('f')):state['focus']='tool'
        if code in (ord('B'),ord('b')):state['focus']='cell'
        if code in (ord('W'),ord('w')):state['focus']='wheel'
        if code in (ord('C'),ord('c')):
            state['request_variant']='P6' if state['variant']=='P7' else 'P7'
        if code==262:  # GLFW right arrow
            state['paused']=True;state['t']=min(times[-1],state['t']+.25)
        if code==263:  # GLFW left arrow
            state['paused']=True;state['t']=max(times[0],state['t']-.25)

    print('\n=== SCARA R1 T03 P7 - MUJOCO 3D VIEWER (VISUAL / KINEMATIC ONLY) ===',flush=True)
    print('G: grasp | A: approach | T: enter slot | D: seat | R: restart',flush=True)
    print('Space: pause/play | C: compare P6/P7 (viewer reopens) | L: loop | +/-: speed',flush=True)
    print('F: tool camera | W: wheel camera | B: full cell | Left/Right: frame step 0.25s',flush=True)
    print('NOT CERTIFIED: 3D swept collision, physics contact, forces, and hardware safety.',flush=True)

    with tempfile.TemporaryDirectory(prefix='r1_p7_mujoco_ascii_') as td:
        scene_dir = Path(td) / 'scene'
        shutil.copytree(p7_stage.parent,scene_dir)
        stages={'P7':scene_dir / p7_stage.name,'P6':scene_dir / p6_stage.name}
        while True:
            variant=state['variant']
            model=mj.MjModel.from_xml_path(str(stages[variant]))
            data=mj.MjData(model)
            ids=core.native_ids(mj,model)
            wheel_id=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'module_G08_banhxe_1')
            if wheel_id<0:raise RuntimeError('P7_VIEW_G08_BODY_MISSING')
            visual_geom_ids=[]
            for gname in ('display_G23_gapxe_1',):
                geom_id=mj.mj_name2id(model,mj.mjtObj.mjOBJ_GEOM,gname)
                if geom_id>=0:visual_geom_ids.append(geom_id)
            if visual_geom_ids:
                if any(model.geom_contype[x]!=0 or model.geom_conaffinity[x]!=0 for x in visual_geom_ids):
                    raise RuntimeError('P7_VIEW_UNEXPECTED_PHYSICAL_CONTACT_GEOM')
            print('[P7 VIEW] OPENING',variant,'/ native MuJoCo, stage=',stages[variant].name,flush=True)
            last=time.monotonic()
            with viewer.launch_passive(model,data,show_left_ui=False,show_right_ui=False,key_callback=keys) as v:
                with v.lock():
                    v.cam.lookat[:]=[-.40,-.24,.03]
                    v.cam.distance=1.45
                    v.cam.azimuth=120
                    v.cam.elevation=-18
                    for j in range(min(5,len(v.opt.geomgroup))):v.opt.geomgroup[j]=1
                    if len(v.opt.sitegroup)>4:v.opt.sitegroup[4]=1
                announced_frame=-999
                while v.is_running():
                    now=time.monotonic();delta=min(.12,max(0.,now-last));last=now
                    if not state['paused']:
                        state['t']+=delta*state['speed']
                        if state['t']>times[-1]:
                            if state['loop']:
                                state['t']=times[start_frame]+(state['t']-times[-1])
                            else:
                                state['t']=float(times[-1]);state['paused']=True
                    idx=min(len(rows)-1,max(0,bisect.bisect_right(times,state['t'])-1))
                    position=pose(idx,model,data,ids)
                    if (idx-announced_frame>=60 or idx<announced_frame or rows[idx]['phase']!=state['last_announce']):
                        msg='frame %4d / 2621 | %6.2f s | %-32s | %s'%(idx,times[idx],rows[idx]['phase'],variant)
                        if 1298<=idx<=2171:
                            wp=data.xpos[wheel_id].copy()
                            xy=to_cad_xy(wp,position,cfg)
                            contour=profiles(P7_CONT if variant=='P7' else P6_CONT)
                            margin=min(nearest_boundary_distance(xy,contour[k]['outline_xy_mm']) for k in ('3','11'))-10.
                            msg+=' | 2D radial clearance ~%.2f mm'%(margin)
                        print('[P7 VIEW]',msg,flush=True)
                        announced_frame=idx
                        state['last_announce']=rows[idx]['phase']
                    if state['focus']:
                        with v.lock():
                            if state['focus']=='tool':
                                v.cam.lookat[:]=data.xpos[ids['mount']];v.cam.distance=.85
                            elif state['focus']=='wheel':
                                v.cam.lookat[:]=frozen_axle;v.cam.distance=.82
                            else:
                                v.cam.lookat[:]=[0.,-.25,.4];v.cam.distance=3.3
                        state['focus']=None
                    v.sync()
                    if state['request_variant']:
                        state['variant']=state['request_variant'];state['request_variant']=None
                        break
                    time.sleep(.015)
                if not v.is_running() and not state['request_variant'] and state['variant']==variant:
                    print('[P7 VIEW] Viewer closed.',flush=True)
                    break
            # If variant changed, the with-block closes the old window and reopens the new one.
            if state['variant']==variant:break
    print('[P7 VIEW] Exit: visual geometry only; NO contact/force safety certificate.',flush=True)

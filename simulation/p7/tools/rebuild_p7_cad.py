#!/usr/bin/env python3
"""Rebuild P7 R13 candidate from bundled P6 CAD. Optional: cadquery, shapely, trimesh.

Writes ONLY into caller-selected output directory. Does not modify P6/V13 or the kit.
"""
from pathlib import Path
import argparse,json,math
import numpy as np

KIT=Path(__file__).resolve().parents[1]

def rebuild(out):
    import cadquery as cq
    import trimesh
    from shapely.geometry import LineString,Polygon
    out.mkdir(parents=True,exist_ok=True)
    src=KIT/'assets/G23_P6_REFERENCE_13_SOLIDS.step'
    solids=cq.importers.importStep(str(src)).solids().vals()
    assert len(solids)==13, 'P6 base must contain exactly 13 solids'
    line=np.loadtxt(KIT/'assets/slot_centerline_step_mm.csv',delimiter=',',skiprows=1)
    buffered=LineString(line).buffer(13.0,resolution=12,cap_style=1,join_style=1)
    assert buffered.is_valid and buffered.geom_type=='Polygon'
    # Cut is extended along Z, but selectively SUBTRACTED from only the two 25mm plates.
    cut=cq.Workplane('XY',origin=(0,0,-200)).polyline(list(buffered.exterior.coords)[:-1]).close().extrude(400).val()
    out_solids=[];changes=[]
    for idx,solid in enumerate(solids):
        if idx in (3,10):
            newer=solid.cut(cut)
            assert newer.isValid() and len(newer.Solids())==1, 'Slot cut split or invalidated a plate'
            changes.append({'zero_based_solid':idx,'volume_before_mm3':solid.Volume(),
                            'volume_after_mm3':newer.Volume(),'removed_mm3':solid.Volume()-newer.Volume()})
            out_solids.append(newer)
        else:out_solids.append(solid)
    shape=cq.Compound.makeCompound(out_solids)
    assert len(shape.Solids())==13 and all(s.isValid() for s in shape.Solids())
    step=out/'G23_NO_COVERS_DUAL_SLOT_R13_CANDIDATE.step'
    raw=out/'G23_CAD_R13_RAW_MM.stl'
    aligned=out/'G23_NO_COVERS_DUAL_SLOT_R13_SCENE_METERS.stl'
    cq.exporters.export(shape,str(step))
    cq.exporters.export(shape,str(raw),tolerance=.12,angularTolerance=.1)
    mesh=trimesh.load(raw,force='mesh',process=False)
    cfg=json.loads((KIT/'config/P6_OFFLINE_ENGINEERING_REPORT.json').read_text())
    a=math.radians(cfg['cad_to_frozen']['angle_about_step_z_deg'])
    R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
    shift=np.array(cfg['cad_to_frozen']['xy_translation_mm']+[cfg['cad_to_frozen']['z_translation_mm']])
    mesh.vertices=(mesh.vertices@R.T+shift)/1000.
    mesh.export(aligned)
    oldcontours=json.loads((KIT/'assets/guide_section_contours_p6_reference.json').read_text())
    newcontours={}
    for key,c in oldcontours.items():
        newpoly=Polygon(c['outline_xy_mm']).difference(buffered)
        assert newpoly.is_valid and newpoly.geom_type=='Polygon'
        newcontours[key]={'outline_xy_mm':[[float(x),float(y)] for x,y in newpoly.exterior.coords],
                          'area_mm2':float(newpoly.area)}
    (out/'guide_section_contours_r13.json').write_text(json.dumps(newcontours,ensure_ascii=False,indent=2))
    result={'source_step':str(src),'source_solid_count':len(solids),'new_solid_count':len(shape.Solids()),
            'buffer_radius_mm':13.,'new_slot_width_mm_nominal':26.,'two_changed_solid_indices_zero_based':[3,10],
            'unmodified_solid_indices_zero_based':[i for i in range(13) if i not in (3,10)],
            'plate_volume_changes':changes,'scene_stl_triangles':len(mesh.faces),
            'scene_stl_bounds_m':mesh.bounds.tolist(),'WARNING':'CAD candidate; no FEM/dimensional tolerances/physical approval'}
    (out/'P7_REBUILD_LOG.json').write_text(json.dumps(result,indent=2))
    print('P7_CAD_REBUILT_STEP',step,'\nP7_CAD_REBUILT_STL',aligned,'\nP7 volume changes mm3:',changes)
    return result

if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('--OutputDir',required=True,help='An external, writable scratch directory. Never pass the original P6 folder.')
    a=ap.parse_args()
    dest=Path(a.OutputDir).expanduser().resolve()
    assert dest!=KIT and KIT not in dest.parents, 'Must not modify the signed P7 kit'
    rebuild(dest)

import unittest, importlib.util, json, hashlib
from pathlib import Path

KIT=Path(__file__).resolve().parents[1]
SCRIPT=KIT/'tools/r1_p7_windows.py'

class TestP7Static(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('p7_runner_test',SCRIPT)
        cls.m=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.m)
        cls.cfg=json.loads((KIT/'config/P6_OFFLINE_ENGINEERING_REPORT.json').read_text())
        cls.rows,cls.t,cls.q,cls.pos=cls.m.load_plan(cls.cfg)

    def test_sha_manifest(self):
        self.m.verify_package()

    def test_p7_2d_sweep_greater_than_2mm(self):
        obj,_=self.m.slot_sweep(self.cfg,self.pos,self.m.P7_CONT)
        self.assertTrue(obj['passes_2d_section_gate'])
        self.assertEqual(obj['total_frame_and_interpolation_samples'],8732)
        self.assertGreater(obj['min_axle_radial_margin_mm'],2.)

    def test_p7_improves_on_p6(self):
        before,_=self.m.slot_sweep(self.cfg,self.pos,self.m.P6_CONT,0.)
        after,_=self.m.slot_sweep(self.cfg,self.pos,self.m.P7_CONT,0.)
        self.assertGreater(after['min_axle_radial_margin_mm']-before['min_axle_radial_margin_mm'],2.)

    def test_mesh_has_stl_magic_and_13solid_step_exists(self):
        self.assertGreater(self.m.P7_MESH.stat().st_size,1000000)
        self.assertTrue((KIT/'assets/G23_NO_COVERS_DUAL_SLOT_R13_CANDIDATE.step').is_file())

    def test_same_final_datum_and_hash(self):
        self.assertTrue(abs(self.pos[2171,0]+.30285)<1e-8)
        self.assertTrue(abs(self.pos[2171,1]+.24368)<1e-8)
        audit=json.loads((KIT/'outputs/P7_OFFLINE_AUDIT_REPORT.json').read_text())
        self.assertEqual(self.m.sha(self.m.CSV),audit['p6_guided_trajectory_sha256'])

if __name__=='__main__':unittest.main()

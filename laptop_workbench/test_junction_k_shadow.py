import json, unittest
from types import SimpleNamespace
import numpy as np
from junction_k import KPath
from shadow_offline import specification
from visual_pid import DEFAULTS

class KShadowTests(unittest.TestCase):
    def test_specification_accepts_k_without_calibration(self):
        app=SimpleNamespace(store=SimpleNamespace(get=lambda key:dict(rev=1,data=dict(parameters=DEFAULTS))))
        spec=specification(app,dict(groups=[dict(algorithm='junction-k-v1',pid_enabled=False)]),5)
        self.assertEqual(spec['groups'][0]['algorithm'],'junction-k-v1')
        self.assertIsNone(spec['groups'][0]['calibration'])
        self.assertFalse(spec['hardware_output'])
    def test_blank_new_input_has_no_fabricated_path_and_resets_on_gap(self):
        k=KPath();image=np.zeros((720,1280,3),np.uint8);mask=np.zeros((720,1280),bool)
        for t in [0,.1,.2]:
            r=k.step(image,mask,t);self.assertFalse(r['valid']);json.dumps(r,allow_nan=False)
        k.step(image,mask,3)
        self.assertEqual(k.index,1)
        self.assertEqual(k.tracker.state,'APPROACH')
        self.assertFalse(r['hardware_output'])

if __name__=='__main__':unittest.main()

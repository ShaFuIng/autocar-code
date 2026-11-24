import ast,re,unittest
from pathlib import Path
class IntegrationSafety(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  tree=ast.parse(Path(__file__).with_name('integration_remote.py').read_text(encoding='utf-8-sig'));fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='permitted');scope={'re':re};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<allowlist>','exec'),scope);cls.allowed=staticmethod(scope['permitted'])
 def test_reject_unlisted_control(self):
  for p in ['/api/reboot','/api/uart','/api/flash','/api/model/deploy','/api/manual/file/a/../../token']:
   self.assertFalse(self.allowed('POST',p));self.assertFalse(self.allowed('GET',p))
 def test_get_cannot_start_motion(self):
  for p in ['/api/manual/start','/api/motor/start','/api/servo']:self.assertFalse(self.allowed('GET',p))
 def test_existing_protected_endpoints(self):
  for p in ['/api/manual/input','/api/manual/stop','/api/servo','/api/motor/heartbeat']:self.assertTrue(self.allowed('POST',p))
  self.assertTrue(self.allowed('GET','/api/manual/ws'))
 def test_shadow_isolated_from_controls(self):
  self.assertTrue(self.allowed('POST','/shadow/api/analyze'));self.assertTrue(self.allowed('POST','/shadow/api/review'));self.assertFalse(self.allowed('POST','/shadow/api/start'));self.assertFalse(self.allowed('POST','/shadow/api/manual/start'))
if __name__=='__main__':unittest.main()

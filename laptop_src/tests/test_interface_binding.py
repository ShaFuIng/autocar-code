import json,unittest
from unittest.mock import patch
from types import SimpleNamespace
from test_dashboard import dashboard

class InterfaceBindingTests(unittest.TestCase):
    def test_only_assigned_global_ipv4_and_no_wildcard(self):
        result=SimpleNamespace(returncode=0,stdout=json.dumps([{'addr_info':[{'family':'inet','scope':'global','local':'10.10.1.8'},{'family':'inet','scope':'host','local':'127.0.0.1'},{'family':'inet','scope':'global','local':'0.0.0.0'}]}]))
        with patch('subprocess.run',return_value=result):
            self.assertEqual(dashboard.interface_ipv4_addresses(['wlP1p1s0','wlP1p1s0']),['10.10.1.8'])
    def test_absent_interface_and_invalid_name(self):
        with patch('subprocess.run',return_value=SimpleNamespace(returncode=1,stdout='')):
            self.assertEqual(dashboard.interface_ipv4_addresses(['wlP1p1s0']),[])
        with self.assertRaises(ValueError):dashboard.interface_ipv4_addresses(['bad name;command'])

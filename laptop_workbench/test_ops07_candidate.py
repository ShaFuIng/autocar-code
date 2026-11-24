import copy
import io
import unittest
from ops07_candidate import Candidate, read_go


class Store:
    def __init__(self):self.rows={};self.fail=False
    def get(self,k):return copy.deepcopy(self.rows.get(k,dict(rev=0,data={})))
    def save(self,k,rev,data):
        if self.fail:raise OSError('disk failed')
        assert self.get(k)['rev']==rev
        self.rows[k]=dict(rev=rev+1,data=copy.deepcopy(data))


class CandidateTests(unittest.TestCase):
    def test_origin_survives_relocation_and_pause_center(self):
        self.c.command('prepare',dict(traffic_light=False,location='ramp'),self.package)
        self.c.command('start',{});self.step(4)
        self.c.command('pause',{});count=self.c.executor.center_requests
        self.assertEqual(self.c.executor.center_status,'confirmed_fake')
        self.c.command('resume',dict(location='crest',moved=True,map_lap='first'),self.package)
        self.assertEqual(self.c.session['origin'],'ramp');self.assertEqual(self.c.session['entry'],'crest')
        self.assertEqual(self.c.executor.center_requests,count+1)
    def test_front_wait_automatic_but_user_pause_not_automatic(self):
        self.c.command('prepare',dict(traffic_light=False,front_car=True,location='flag'),self.package)
        self.c.command('start',{});self.c.command('front_distance',dict(value=1));self.step()
        self.assertEqual(self.c.session['state'],'front_wait');self.assertFalse(self.c.executor.active)
        self.c.command('front_distance',dict(value=None));self.step();self.assertFalse(self.c.executor.active)
        self.c.command('front_distance',dict(value=3));self.step();self.assertTrue(self.c.executor.active)
        self.c.command('pause',{});self.step();self.assertFalse(self.c.executor.active)
    def test_i2c_fault_never_reports_center_confirmed(self):
        self.prepare(False);self.c.command('health',dict(key='i2c',value=False))
        self.assertEqual(self.c.executor.center_status,'unconfirmed')

    def setUp(self):
        self.now=0.;self.store=Store();self.c=Candidate(self.store,lambda:self.now)
        self.package={'identity':'fixed','parameters':{'max_gap':.4}}
    def step(self,dt=.051):self.now+=dt;self.c.tick()
    def prepare(self,traffic=True):
        self.c.command('prepare',dict(traffic_light=traffic,location='flag'),self.package)
        self.c.command('start',{})
    def green(self):
        self.c.command('flag',{'value':0});self.step()
        self.c.command('flag',{'value':1});self.step()
    def resume(self,**kw):
        self.c.command('resume',dict(location='flag',moved=False,map_lap='first',**kw),self.package)
    def test_initial_green_then_edge_one_shot(self):
        self.c.command('flag',{'value':1});self.prepare();self.step()
        self.assertFalse(self.c.executor.active)
        self.green();self.assertTrue(self.c.executor.active)
        count=self.c.poll_count;self.step(200)
        self.assertEqual(count,self.c.poll_count);self.assertEqual(self.c.executor.releases,1)
    def test_unknown_requires_new_zero(self):
        self.prepare();self.c.command('flag',{'value':0});self.step()
        self.c.command('flag',{'value':None});self.step()
        self.c.command('flag',{'value':1});self.step()
        self.assertFalse(self.c.executor.active)
    def test_cancel_and_fault_do_not_release_on_green(self):
        for stop in ('pause','stop'):
            self.setUp();self.prepare();self.c.command(stop,{})
            self.green();self.assertFalse(self.c.executor.active)
        self.setUp();self.prepare();self.c.command('health',{'key':'estop','value':True})
        self.green();self.assertEqual(self.c.session['state'],'fault')
    def test_unlimited_without_browser_and_flag_zero_not_stop(self):
        self.prepare(False);self.c.command('flag',{'value':0});self.step(86400)
        self.assertTrue(self.c.executor.active);self.assertIsNone(self.c.session['deadline'])
        self.assertEqual(self.c.session['elapsed_s'],86400)
        self.assertGreater(len(self.c.session['laps']),1000)
    def test_pause_resume_same_session_clock_and_snapshot(self):
        self.prepare(False);self.step(10);uid=self.c.session['id']
        self.c.command('pause',{});self.step(20)
        self.assertEqual(self.c.session['elapsed_s'],10)
        self.resume();self.step(4)
        self.assertEqual(self.c.session['id'],uid);self.assertEqual(self.c.session['elapsed_s'],14)
        self.assertEqual(len(self.c.session['segments']),2)
    def test_changed_package_and_invalid_entry_rejected(self):
        self.prepare(False);self.c.command('pause',{})
        with self.assertRaises(ValueError):self.c.command('resume',dict(location='flag',moved=False,map_lap='first'),{'identity':'changed'})
        with self.assertRaises(ValueError):self.c.command('resume',dict(location='curve_middle',moved=True,map_lap='first'),self.package)
        self.assertFalse(self.c.executor.active)
    def test_move_invalidates_lap_and_can_resume_away_from_start(self):
        self.prepare(False);self.step(5);self.c.command('pause',{})
        self.c.command('resume',dict(location='crest',moved=True,map_lap='after_down'),self.package)
        self.assertEqual(self.c.session['s_m'],60.4);self.step(25)
        self.assertFalse(self.c.session['laps'][0]['valid'])
        self.step(67);self.assertTrue(self.c.session['laps'][1]['valid'])
    def test_restart_persists_session_never_auto_releases(self):
        self.prepare(False);self.step(5);uid=self.c.session['id']
        restored=Candidate(self.store,lambda:self.now)
        self.assertEqual(restored.session['id'],uid);self.assertEqual(restored.session['state'],'paused')
        self.assertTrue(restored.session['needs_location']);self.assertFalse(restored.executor.active)
    def test_mcu_reset_fault_repair_does_not_resume(self):
        self.prepare(False);self.c.command('restart_mcu',{})
        self.assertFalse(self.c.executor.active);self.assertTrue(self.c.session['needs_location'])
        self.c.command('health',dict(key='i2c',value=False))
        with self.assertRaises(ValueError):self.resume()
        self.c.command('health',dict(key='i2c',value=True));self.assertFalse(self.c.executor.active)
        self.resume();self.assertTrue(self.c.session['incomplete'])
    def test_disk_failure_stops_fake_output(self):
        self.prepare(False);self.store.fail=True
        with self.assertRaises(OSError):self.step(2)
        self.assertFalse(self.c.executor.active);self.assertEqual(self.c.session['reason'],'RECORDING_FAILED')
    def test_reset_archives_and_refuses_running(self):
        self.prepare(False);uid=self.c.session['id']
        with self.assertRaises(ValueError):self.c.command('reset',{})
        self.c.command('pause',{});self.c.command('reset',{})
        self.assertIsNone(self.c.session['id']);self.assertTrue(self.store.get('ops07-lab-archive-'+uid)['data'])
    def test_reader_strict_content_and_timeout(self):
        for body,expected in [(b'0',0),(b'1\n',1),(b'error',None),(b'',None),(b'100',None)]:
            def opener(url,timeout):
                self.assertLessEqual(timeout,.2);f=io.BytesIO(body);f.status=200;return f
            self.assertEqual(read_go('http://test/go',opener),expected)
        def timeout(*a,**k):raise TimeoutError()
        self.assertIsNone(read_go('http://test/go',timeout))


if __name__=='__main__':unittest.main(verbosity=2)

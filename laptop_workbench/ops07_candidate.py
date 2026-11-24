"""Offline executable candidate. No vehicle imports, SSH, UART or motor APIs.

The fake executor is deliberately NOT a production actuator adapter. Resume
plans are semantic requests; rebuilding actual DashRoute/CurveMap is a separate
integration step. All time belongs to this process, never the browser.
"""
import copy
import math
import threading
import time
import uuid
from urllib.request import urlopen
from ops07_route import FrontCarGate

ENTRY_S = {'flag':17.65, '12_to_01':14, '02_to_03_before':27,
           '02_to_03_after':32, '04_to_05_before':43,
           '04_to_05_after':47, '05_to_06':49, 'ramp':58.4, 'crest':60.4}
SEGMENTS = [('downhill',0),('curve4',4.45),('straight_to_curve1',11.6),
            ('curve1',20.2),('straight_to_curve2',23.54),('curve2',35.3),
            ('straight_to_curve3',39.61),('curve3',49),('ramp',58.4),('upper',60.4)]


def read_go(url, opener=urlopen):
    """Production transport candidate; offline lab never calls it.

    Invalid content is unknown, never an implicit zero. URL provision and
    network verification remain deployment work; no default external host.
    """
    try:
        with opener(url, timeout=.2) as response:
            if response.status != 200:
                return None
            data = response.read(9).strip()
            return {b'0':0,b'1':1}.get(data)
    except (OSError, ValueError):
        return None


class FakeExecutor:
    """Output goes only into memory, cannot reach hardware."""
    def __init__(self):
        self.active = False
        self.releases = 0
        self.plan = None
        self.center_requests = 0
        self.center_status = 'unconfirmed'

    def prepare(self, plan):
        self.active = False
        self.plan = copy.deepcopy(plan)

    def release(self):
        self.active = True
        self.releases += 1

    def stop(self):
        self.active = False

    def center(self, available=True):
        self.active = False
        self.center_requests += 1
        self.center_status = 'confirmed_fake' if available else 'unconfirmed'


class Candidate:
    def __init__(self, store, clock=time.monotonic):
        self.store, self.clock = store, clock
        self.lock = threading.RLock()
        self.executor = FakeExecutor()
        self.last = clock()
        self.last_save = self.last
        self.last_poll = -float('inf')
        self.poll_count = 0
        self.flag = None
        self.front_distance=4.;self.front_gate=FrontCarGate()
        self.saw_zero = False
        self.health = dict(camera=True, path=True, uart=True, i2c=True, estop=False, generation=1)
        saved = store.get('ops07-lab-session')['data']
        self.session = saved or self.empty()
        self.session.setdefault('origin', self.session.get('entry', 'flag'))
        self.session.setdefault('wall_s', self.session.get('elapsed_s', 0.))
        # A process restart cannot release, re-arm the green latch or trust old
        # odometry; preserve the session but require an explicit resume entry.
        if saved and saved['state'] != 'idle':
            self.session.update(state='paused', relocated=True, incomplete=True,
                                reason='PROCESS_RESTART', needs_location=True)
            self.event('restart', reason='PROCESS_RESTART')
            self.save()

    @staticmethod
    def empty():
        return dict(id=None, state='idle', elapsed_s=0., lap_elapsed_s=0., laps=[],
                    segments=[], events=[], s_m=17.65, distance=0., incomplete=False,
                    relocated=False, needs_location=False, reason=None, package=None,
                    execution_mode='orin_continuous_candidate', deadline=None,origin='flag',wall_s=0.)

    def event(self, kind, **values):
        s=self.session
        s['events'].append(dict(kind=kind, elapsed_s=s['elapsed_s'], **values))

    def save(self):
        try:
            old=self.store.get('ops07-lab-session')
            self.store.save('ops07-lab-session',old['rev'],self.session)
            self.last_save=self.clock()
        except Exception:
            self.executor.stop()
            self.session.update(state='fault',reason='RECORDING_FAILED')
            self.saw_zero=False
            raise

    def safe(self):
        return all(self.health[k] for k in ('camera','path','uart','i2c')) and not self.health['estop']

    def release(self):
        if self.session['state'] not in ('ready','waiting','front_wait') or not self.safe():
            raise ValueError('尚未準備完成，或假車況仍有故障。')
        if self.session.get('front_car') and not self.front_clear():
            self.session['state']='front_wait';self.save();return
        self.executor.center(self.safe())
        self.event('center',stage='before_release',status=self.executor.center_status)
        self.saw_zero=False
        self.session.update(state='running',reason=None)
        self.session['segments'].append(dict(id=uuid.uuid4().hex,location=self.session['entry'],generation=self.health['generation']))
        self.event('release')
        # Commit intent before the output transition. A crash recovers paused.
        self.save()
        self.executor.release()
        self.last=self.clock()

    def halt(self, state, reason):
        self.executor.stop()
        # Fault/ESTOP never causes an attempted re-arm or automatic centering.
        if reason in ('USER_PAUSE','FRONT_WAIT'):
            self.executor.center(self.health['uart'] and self.health['i2c'] and not self.health['estop'])
            self.event('center',stage='pause',status=self.executor.center_status)
        else:self.executor.center_status='unconfirmed'
        self.saw_zero=False
        self.session.update(state=state,reason=reason)
        self.event('halt',reason=reason)
        self.save()

    def command(self, action, p, package=None):
        with self.lock:
            self.tick()
            s=self.session
            if action == 'flag':
                value=p.get('value')
                if value is not None and (type(value) is not int or value not in (0,1)):
                    raise ValueError('假旗標只能是 0、1 或離線。')
                self.flag=value
            elif action=='front_distance':
                value=p.get('value')
                if value is not None and (type(value) not in (int,float) or not math.isfinite(value) or value<0):raise ValueError('假距離格式不符。')
                self.front_distance=value
            elif action == 'health':
                key=p.get('key');value=p.get('value')
                if key not in ('camera','path','uart','i2c','estop') or type(value) is not bool:
                    raise ValueError('無效的假車況。')
                self.health[key]=value
                if not self.safe() and s['state'] in ('waiting','running','ready','front_wait'):
                    self.halt('fault',key.upper())
            elif action == 'prepare':
                if s['state'] != 'idle':raise ValueError('本場已存在，請暫停接續或重置假場次。')
                if not package:raise ValueError('請先讀取固定方案。')
                if type(p.get('traffic_light')) is not bool:raise ValueError('起跑選項格式不符。')
                if type(p.get('front_car',False)) is not bool:raise ValueError('前車選項格式不符。')
                location=p.get('location','flag')
                if location not in ENTRY_S:raise ValueError('不支援此接續點。')
                if not self.safe():raise ValueError('請先排除假車況故障。')
                s.update(id=uuid.uuid4().hex,package=copy.deepcopy(package),entry=location,
                         s_m=ENTRY_S[location],state='ready',incomplete=location!='flag',
                         generation=self.health['generation'],traffic_light=p['traffic_light'],front_car=p.get('front_car',False),map_lap=p.get('map_lap','first'),origin=location)
                if s['map_lap'] not in ('first','after_down'):raise ValueError('路線階段不符。')
                self.executor.prepare(dict(location=location, map_lap=s['map_lap'], reset_history=True))
                self.executor.center()
                self.event('prepare');self.save()
            elif action == 'start':
                if s['state']!='ready':raise ValueError('請先準備假車。')
                if not self.safe():raise ValueError('假車況有故障。')
                if s['traffic_light']:
                    s['state']='waiting';self.saw_zero=False;self.last_poll=-float('inf')
                    self.event('wait_green');self.save()
                else:self.release()
            elif action in ('pause','stop'):
                if s['state']=='idle':raise ValueError('尚無假場次。')
                self.halt('paused', 'USER_'+action.upper())
            elif action == 'resume':
                if s['state'] not in ('paused','fault'):raise ValueError('請先暫停。')
                if not self.safe():raise ValueError('假車況故障尚未排除。')
                if not package or package['identity']!=s['package']['identity']:
                    raise ValueError('接續必須使用本場原固定方案。')
                location=p.get('location');moved=p.get('moved')
                phase=p.get('map_lap')
                if location not in ENTRY_S or type(moved) is not bool or phase not in ('first','after_down'):
                    raise ValueError('請確認白名單位置、是否搬車與路線階段。')
                moved=moved or s['needs_location'] or self.health['generation']!=s['generation']
                if not moved and location!=s['entry']:raise ValueError('改變接續入口時請標記搬車。')
                if moved:s.update(s_m=ENTRY_S[location],incomplete=True,relocated=True,distance=0.)
                s.update(entry=location,generation=self.health['generation'],state='ready',needs_location=False,map_lap=phase)
                self.executor.prepare(dict(location=location,map_lap=phase,reset_history=True,
                                           same_place=not moved,checkpoint_s=s['s_m']))
                self.event('resume',moved=moved,plan=self.executor.plan)
                self.release()  # Explicit resume; never wait on another race's green.
            elif action == 'restart_mcu':
                self.health['generation']+=1
                if s['state']!='idle':
                    s.update(needs_location=True,incomplete=True)
                    self.halt('paused','MCU_RESTART')
            elif action == 'reset':
                if s['state'] in ('running','waiting','ready','front_wait'):raise ValueError('先停止假車，再重置本場。')
                if s['id']:
                    key='ops07-lab-archive-'+s['id'];old=self.store.get(key)
                    self.store.save(key,old['rev'],s)
                self.session=self.empty();self.saw_zero=False;self.save()
            else:raise ValueError('未知的離線測試操作。')
            return self.snapshot()

    def front_clear(self):
        reading=None if self.front_distance is None else dict(captured_at=self.clock(),distance_m=self.front_distance)
        return self.front_gate.can_continue(reading,self.clock(),.4)

    def tick(self):
        with self.lock:
            now=self.clock();dt=max(0.,now-self.last);self.last=now;s=self.session
            if s['id']:s['wall_s']+=dt
            if s['state'] in ('running','waiting','ready','front_wait') and not self.safe():
                self.halt('fault','LOCAL_PROTECTION');return
            if s.get('front_car'):
                if s['state']=='running' and not self.front_clear():self.halt('front_wait','FRONT_WAIT');return
                if s['state']=='front_wait' and self.front_clear():self.release();return
            if s['state']=='waiting' and now-self.last_poll>=.05:
                self.last_poll=now;self.poll_count+=1
                if self.flag is None:self.saw_zero=False
                elif self.flag==0:self.saw_zero=True
                elif self.flag==1 and self.saw_zero:self.release()
                return
            if s['state']=='running':
                # Fake odometry: 1 m/s. Only this lab synthesizes motion/flags.
                old=s['s_m'];travel=dt
                while travel>0:
                    to_flag=(17.65-old)%66.8 or 66.8
                    step=min(travel,to_flag);s['elapsed_s']+=step;s['lap_elapsed_s']+=step;s['distance']+=step
                    old=(old+step)%66.8;travel-=step
                    if step==to_flag:
                        old=17.65  # Exact anchor avoids a second floating-point crossing.
                        valid=not s['incomplete'] and s['distance']>=66.8-1e-6
                        s['laps'].append(dict(seconds=s['lap_elapsed_s'],valid=valid,source='fake_flag'))
                        self.event('fake_flag_cross',valid=valid)
                        s.update(lap_elapsed_s=0.,distance=0.,incomplete=False)
                s['s_m']=old
                if now-self.last_save>=1:self.save()

    def snapshot(self):
        with self.lock:
            result=copy.deepcopy(self.session)
            result.update(simulation=True,actuator='memory_only',health=dict(self.health),
                          flag=self.flag,saw_zero=self.saw_zero,poll_count=self.poll_count,
                          front_distance=self.front_distance,
                          outputs_active=self.executor.active,release_count=self.executor.releases)
            result.update(center_status=self.executor.center_status,center_requests=self.executor.center_requests)
            result['segment']=next(name for name,start in reversed(SEGMENTS) if result['s_m']>=start)
            return result

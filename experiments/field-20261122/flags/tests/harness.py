"""從 live_pid.py 文字中抽出 (1) 下坡旗標區塊（'dpr=None' 起到 '_ds_t0=None' 那行）與 (2) _dl_ok 方法（若有），
以假的 self 逐格重播紀錄的 dash_route 俯仰。測的是檔案中真正的程式文字，不是另寫的模型。"""
import re,textwrap,types,json,os,pickle,tempfile
DOWN_STEEP=-8.
def _block(src):
    L=src.splitlines(True)
    i=next(k for k,l in enumerate(L) if l.strip()=='dpr=None' and 'down_until' in ''.join(L[k:k+8]))
    j=next(k for k in range(i,len(L)) if "if not down_tail:self._bm_yaw0=None;self._bm_t0=None;self._ds_t0=None" in L[k])
    return textwrap.dedent(''.join(L[i:j+1]))
def _method(src,name):
    m=re.search(r'\n(    def %s\(self.*?)(?=\n    def |\nclass )'%name,src,re.S)
    return None if m is None else textwrap.dedent(m.group(1))
class _Dip:
    def __init__(s):s.info=None;s.fdg=None
    def step(s,now,p,route):
        if s.fdg is not None:s.info={'raw_pitch_rel_deg':s.fdg};return -2.0+.1
        s.info=None;return p
class Replayer:
    def __init__(self,src,cfg=None):
        self.code=compile(_block(src),'<flag-block>','exec')
        ms=_method(src,'_dl_ok');self.tmp=tempfile.mkdtemp(prefix='dlg_')
        if cfg is not None:json.dump(cfg,open(os.path.join(self.tmp,'down_level.json'),'w'))
        self.g=dict(json=json,Path=__import__('pathlib').Path,DOWN_STEEP=DOWN_STEEP,time=__import__('time'),__file__=os.path.join(self.tmp,'live_pid.py'))
        self.S=types.SimpleNamespace(dipg=_Dip(),bguide=None,heading=None,latest={},run={'id':'replay'},down_until=None)
        if ms is not None:
            ns={};exec(compile(ms,'<_dl_ok>','exec'),self.g,ns);self.S._dl_ok=types.MethodType(ns['_dl_ok'],self.S)
        self.prev_route=None
    def step(self,r):
        S=self.S
        if self.prev_route=='DONE' and r['route']=='WAIT_CLIMB':S.down_until=None   # dash-route lap step (live_pid 1970)
        self.prev_route=r['route'];S.dipg.fdg=r['fdg'];S.latest={}
        loc=dict(self=S,now=r['con'],result={'dash_route':{'pitch_rel_deg':r['dpr'],'route':r['route']}})
        exec(self.code,self.g,loc)
        return (loc['down_tail'],bool(getattr(S,'down_steep',False)),getattr(S,'down_level_t0',None),S.latest.get('down_level_guard'))
    def close(self):
        import shutil;shutil.rmtree(self.tmp,ignore_errors=True)
def rows_of(run,tmp):
    return [r for r in pickle.load(open(os.path.join(tmp,run+'.pkl'),'rb'))['rows'] if r['con'] is not None]

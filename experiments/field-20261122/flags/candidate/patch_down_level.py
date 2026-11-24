"""down-level-guard-r1：對 orin-web-snapshot-20261122c/live_pid.py 打補丁 -> candidate/live_pid.py。
每個錨點必須剛好出現 1 次，否則不寫檔。新行為由 web/down_level.json 的 min_steep_s 開；檔案／鍵缺 = 現況（原條件為假時不呼叫新函式；為真時新函式回 True）。
用法：python patch_down_level.py [來源 live_pid.py] [輸出 live_pid.py]"""
import sys,os,hashlib
HERE=os.path.dirname(os.path.abspath(__file__))
SRC=sys.argv[1] if len(sys.argv)>1 else os.path.join(HERE,'..','..','..','orin-web-snapshot-20261122c','live_pid.py')
DST=sys.argv[2] if len(sys.argv)>2 else os.path.join(HERE,'live_pid.py')
BASE_SHA='b3267293'
s=open(SRC,encoding='utf-8').read()
h=hashlib.sha256(s.encode('utf-8')).hexdigest()
if not h.startswith(BASE_SHA):sys.exit('來源雜湊 %s 不是預期底 %s，不寫檔'%(h[:8],BASE_SHA))
A1="                        elif dpr is not None and getattr(self,'down_steep',False) and getattr(self,'down_level_t0',None) is None:self.down_level_t0=now\n"
B1="                        elif dpr is not None and getattr(self,'down_steep',False) and getattr(self,'down_level_t0',None) is None and self._dl_ok(now):self.down_level_t0=now   # down-level-guard-r1: _dl_ok is True unless web/down_level.json min_steep_s is set\n"
A2="    def _bm_turned(self):\n"
B2=('''    def _dl_ok(self,now):   # down-level-guard-r1 (field-20261122/flags): the landing flag down_level_t0 needs the steep slope (first pitch < DOWN_STEEP of this
        # down_tail, _ds_t0) to have lasted min_steep_s (web/down_level.json, read once per run). 54cb lap 3: dash-route pitch -8.18 -> -7.95 -> -12.75 at the
        # slope mouth set the flag 0.03 s after the steep start and land4 latched LAND on the slope. 104 slope passes today: steep start -> landing 1.92-2.56 s.
        # File / key absent = True on every call (old rule). A rejected frame is logged in vision.down_level_guard.
        rid=(self.run or {}).get('id') if isinstance(self.run,dict) else None
        if getattr(self,'_dlg_rid','-')!=rid:
            self._dlg_rid=rid;self._dlg_ms=None;self._dlg_n=0
            try:
                c=json.loads((Path(__file__).parent/'down_level.json').read_text(encoding='utf8'))
                if isinstance(c,dict) and c.get('min_steep_s') is not None:self._dlg_ms=float(c['min_steep_s'])
            except Exception:self._dlg_ms=None
        ms=self._dlg_ms;t0=getattr(self,'_ds_t0',None)
        if ms is None or t0 is None or now-t0>=ms:return True
        self._dlg_n+=1
        if self.latest is not None:self.latest['down_level_guard']=dict(revision='down-level-guard-r1',steep_s=round(now-t0,3),min_steep_s=ms,n=self._dlg_n)
        return False

'''+A2)
for a,b in ((A1,B1),(A2,B2)):
    n=s.count(a)
    if n!=1:sys.exit('錨點出現 %d 次（需 1 次），不寫檔：%r'%(n,a[:80]))
    s=s.replace(a,b)
open(DST,'w',encoding='utf-8',newline='').write(s)
print('寫出',DST,hashlib.sha256(s.encode('utf-8')).hexdigest()[:8])

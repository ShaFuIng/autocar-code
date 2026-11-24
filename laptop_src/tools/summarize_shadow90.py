"""Summarize the authorized real-video run; never starts inference or hardware."""
import json
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'laptop_workbench'
ID='31a8163b457d4ecc8069ceaa68f1fbc2'
folder=ROOT/'data/analysis'/ID
def load(p):return json.loads(p.read_text(encoding='utf-8'))
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    assert load(folder/'status.json')['state']=='completed'
    result=load(folder/'result.json');notes=load(folder/'run_notes.json')
    assert len(result['frames'])==716 and result['cache']['inferred']==716
    rows=[]
    for g in result['spec']['groups']:
        data=result['models'][g['id']];s=result['summaries'][g['id']]
        assert len(data)==716 and g['pid_revision']==37
        changes=[abs(r['steer_change_us']) for r in data if r['steer_change_us'] is not None]
        rows.append(dict(group=g['id'],algorithm=g['algorithm'],path_valid=s['path_valid'],pid_valid=s['pid_valid'],
            frames=s['frames'],path_valid_pct=round(s['path_valid']/s['frames']*100,2),
            pid_valid_pct=round(s['pid_valid']/s['frames']*100,2),first_stop=s['first_stop'],simulated_started=s['started'],
            modes=dict(Counter(r['A'].get('mode','unspecified') for r in data)),
            pid_reasons=dict(Counter(r['pid']['reason'] for r in data)),
            limited=s['limited'],max_steer_change_us=max(changes,default=None),
            mean_path_ms=sum(r['A']['path_ms'] for r in data)/len(data),
            transitions=dict(Counter(r['A'].get('transition','unspecified') for r in data))))
    warning='本輪為10/16完整跑道舊相機姿態的前90秒；v7校正來自下壓後姿態，明確不相符。v7只作探索，不用本輪判定校正效果或算法優劣。'
    if warning not in result['limitations']:result['limitations'].insert(0,warning)
    result['run_notes']=notes;save(folder/'result.json',result)
    report=dict(**notes,recognition=result['recognition'],cache=result['cache'],groups=rows,
                interpretation=warning,accuracy_measured=False,closed_loop_validated=False)
    out=ROOT/'data/experiments/offline-shadow-90s';out.mkdir(exist_ok=True)
    save(out/'report.json',report)
    p=ROOT/'integration.js';text=p.read_text(encoding='utf-8')
    a=text.index('const WORKBENCH_RELEASE=');b=text.index('\n};',a)
    block=text[a:b]
    block=block.replace("version:'workbench-shadow-20261021.1'","version:'workbench-shadow-20261021.2'")
    lines=block.splitlines()
    for i,line in enumerate(lines):
        if line.startswith(' status:'):lines[i]=" status:'06已完成完整跑道前90秒、716張實錄的V4-640共用辨識與三算法PID v37推演；工作紀錄標示完整跑道前90秒。v7姿態不符，只作探索，不作校正驗收。未連Orin、未動車。',"
        if line.startswith(' acceptance:'):lines[i]=" acceptance:'2026-10-21：實際V4只辨識716張一次；legacy9、temporal-v2、v7各完成716張路徑與PID v37推演。路徑有效分別為%s；這是可用率而非準確率。原測試未重跑，沒有實車驗收。'," % '／'.join(str(r['path_valid']) for r in rows)
        if '本輪使用假辨識器驗證流程，未重新跑V4 GPU' in line:lines[i]="  '真實V4前90秒比較已完成；v7仍需相機姿態相符的影片才能評估校正效果。現場閉環與轉向響應另案驗證。',"
        if line==' history:[':lines[i]+="\n  '2026-10-21：真實V4共用716張；三算法＋PID v37完成。v7為舊姿態錯配探索，未重跑測試、未連Orin。',"
    text=text[:a]+'\n'.join(lines)+text[b:];p.write_text(text,encoding='utf-8')
    p=ROOT/'README.md';text=p.read_text(encoding='utf-8').replace('`workbench-shadow-20261021.1`','`workbench-shadow-20261021.2`',1)
    marker='## 離線影子測試 v1'
    addition='## 真實影片比較（2026-10-21）\n\n已完成10/16完整跑道前90秒（716張，最後時間89.958385秒），V4-640僅辨識一次，三組共用遮罩；PID凍結v37（中心1360µs），原算法、temporal-v2、v7各716張。06工作紀錄「完整跑道前90秒」可回看。v7姿態不符，僅探索，不作校正或算法優劣驗收；路徑有效率不是準確率。摘要 `data/experiments/offline-shadow-90s/report.json`，工作ID `'+ID+'`。未重跑舊測試、未連Orin、未動車。\n\n'
    if '## 真實影片比較' not in text:text=text.replace(marker,addition+marker,1)
    p.write_text(text,encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()

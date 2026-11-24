"""Build a local, read-only field summary from preserved measurement files."""
import json, re, statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'data'
turn_path = DATA / 'manual-turning-measurements.json'
turn = json.loads(turn_path.read_text(encoding='utf-8'))
for row in turn['measurements']:
    if row['steering_us'] in (940, 1210):
        row['pwm_pct'] = 20
        row['pwm_confirmation'] = '使用者後續明確確認為20% PWM'
turn_path.write_text(json.dumps(turn, ensure_ascii=False, indent=2), encoding='utf-8')
legacy = DATA / 'manual-left940-radius.json'
old = json.loads(legacy.read_text(encoding='utf-8'))
old.update(pwm_pct=20, notes='使用者自行手動測試；右側外輪軌跡直徑。20% PWM已確認；非車身中心半徑。')
legacy.write_text(json.dumps(old, ensure_ascii=False, indent=2), encoding='utf-8')

straight = []
for path in sorted(DATA.glob('ground-center1300-trial*.json')):
    d = json.loads(path.read_text(encoding='utf-8'))
    c = d['result']['config']; obs = d['user_observation']
    straight.append(dict(source=path.name, trial=int(re.search(r'trial(\d+)', path.name)[1]),
        center_us=1300, pwm_pct=c['right_pct'], seconds=c['seconds'], distance_cm=obs['distance_cm'],
        distance_per_command_time_mps=obs['distance_cm']/100/c['seconds'],
        observation=obs, telemetry_summary=d['result']['summary'],
        completion=d['result']['reason']))
selected = [s for s in straight if s['trial'] in (6,7,8)]
mean_cm = statistics.mean(s['distance_cm'] for s in selected)
report = dict(date='2026-10-18', source='現場手動量測＋既有遙測檔；本次整理沒有連線或控制車輛',
    straight=straight, turning=turn['measurements'],
    preliminary_trials=[dict(source='ground-first-10pct-1s.json',pwm_pct=10,seconds=1,distance_cm=3,
        note='使用者聊天回報約3cm；早於1300 µs中心校準，不併入同中心比較'),
        dict(source='ground-second-15pct-3s.json',pwm_pct=15,seconds=3,distance_cm=None,
        note='早於中心校準，使用者指出不直，未取得可比距離')],
    derived=dict(straight_15pct_5s_selected_trials=[6,7,8],mean_distance_cm=mean_cm,
        mean_distance_per_command_time_mps=mean_cm/500,range_cm=2,
        range_over_mean_pct=2/mean_cm*100,
        endpoint_right_radius_reduction_pct=(1-52.5/82.5)*100,
        endpoint_right_radius_reduction_bounds_pct=[(1-55/80)*100,(1-50/85)*100]),
    limitations=['距離／設定秒數包含起步與停止影響，不是穩態速度或精確行駛時間。',
        '102cm可能偏行，保留原值但不納入三次重複性摘要。',
        '外輪軌跡半徑不是後軸中心路徑半徑；未由此換算實際輪角。',
        '手動轉圈的精確時長、接近方向、供電、輪胎滑移及重複性未完整記錄。',
        '1450 µs無法在場地量完，不填零、無限半徑或虛構下界。',
        '機構間隙、靜摩擦、負載與幾何非線性尚無法分離；不能定量宣稱50 µs死區。',
        '最大速度、加速度、精確停止距離、10m直行誤差及自動循跡尚未驗證。'])
(DATA/'field-performance-20261018.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

straight_rows=[]
for s in straight:
    su=s['telemetry_summary']
    straight_rows.append([f"校準後 #{s['trial']}",str(s['pwm_pct'])+'%',str(s['seconds']),str(s['distance_cm']),
        f"{s['distance_per_command_time_mps']:.3f}",f"{su['mean_right_cps']:.0f} / {su['mean_left_cps']:.0f}",
        '可能偏行，保留但不納入重複性摘要' if s['trial']==5 else '基本直行／未見明顯滑行（現場觀察）'])
straight_rows.extend([['校準前','10%','1','約3','約0.030','—','中心不同，獨立保留'],
    ['校準前','15%','3','未量得','—','—','使用者回報不直，不與校準後合併']])
turn_rows=[]
for s in sorted(turn['measurements'],key=lambda x:(x['direction'],x['steering_us'])):
    known=s['diameter_cm'] is not None
    turn_rows.append(['左' if s['direction']=='left' else '右',str(s['steering_us']),f"{s['steering_us']-1300:+d}",'20%',
        f"{s['diameter_cm']} ±{s['diameter_uncertainty_cm']}" if known else '場地無法量測',
        f"{s['radius_cm']:g} ±{s['radius_uncertainty_cm']:g}" if known else '未知',
        '右側外輪' if s['direction']=='left' else '左側外輪'])
tables={'straight':{'headers':['紀錄','PWM','秒','距離 cm','距離／秒 m/s','右／左 counts/s','備註'],'rows':straight_rows},
        'turn':{'headers':['方向','指令 µs','相對中心 µs','PWM','直徑 cm','半徑 cm','量測基準'],'rows':turn_rows}}
html='''<section id="fieldPerformance" style="margin:20px 0;padding:20px;border:1px solid #bdcebc;border-radius:12px">
<h2>今日落地性能整理 · 2026-10-18</h2>
<p>保存中心 <b>1300 µs</b>。8筆校準後直行、2筆校準前紀錄、7筆轉彎直徑與1筆無法量測觀察。轉彎全部確認為 <b>20% PWM</b>。這是歷史實測摘要，非即時車況；未改控制參數或白模的實量設定。</p>
<h3>目前能推算的性能</h3><ul>
<li>10%／3秒：約20cm，距離／時間約 <b>0.067 m/s</b>；12%／3秒：36cm，約 <b>0.120 m/s</b>。</li>
<li>15%／3秒：63、62cm，平均62.5cm，約 <b>0.208 m/s</b>。15%／5秒後三次：106、108、106cm，平均106.7cm，約 <b>0.213 m/s（0.768 km/h）</b>。全距2cm約為平均的1.9%，只有三次，非通用精度保證。</li>
<li>15%／5秒另有102cm，使用者後續註明可能偏行；保留而不混入上述三次摘要。所有速度均為量得距離除以設定秒數，含起步／停止影響，不能當作穩態速度。10%是本次低速起動邊緣的觀察，非精確起轉門檻。</li>
<li>15%直行資料右側編碼器計數率高於左側約1.4–2.9%；這是計數率差，非精確地面速度差，仍受每圈尺度、輪徑與滑移影響。一次左側新增1個無效跳變，其餘校準後直行紀錄未新增。</li></ul>
<h3>直行原始量測</h3>__STRAIGHT__
<h3>轉彎原始量測</h3><p>誤差為使用者估計的量測範圍，非統計信賴區間；半徑＝直徑／2。量測是外側輪胎軌跡，不是車身／後軸中心路徑。</p>__TURN__
<h3>左右轉向差異</h3><ul>
<li>暫定左端940 µs：外輪半徑82.5 ±2.5cm；右端1900 µs：52.5 ±2.5cm。右側最小已量半徑約小 <b>36%</b>（按誤差端點估約31–41%）。只是兩個已試設定，不是已驗證機械極限。</li>
<li>兩側指令幅度不對稱：左端距中心−360 µs，右端＋600 µs，不能視為相同轉角的公平比較。相同106.5cm外輪半徑出現在左1030（−270）與右1750（＋450），顯示需要分左右建表，不能共用一條線性µs比例。</li>
<li>右1450（＋150）只有小幅偏右且圈超出場地；右1600已量半徑195cm。中心到1600之間仍是未定量區域，不能直接連成準確曲線或當成完全不轉。</li></ul>
<h3>機構間隙：觀察與推論分開</h3><p>你曾觀察約50 µs以上才看見前輪移動，而且從左／右接近會出現不同角度。這支持存在方向相關的鬆動或遲滯，但目前沒有「同一µs、相反接近方向」成對重複量測，<b>無法算出精確間隙或補償值</b>。靜摩擦、輪胎受力、連桿幾何及中心偏置也可能影響結果；不能將左右半徑不同全部歸因於間隙。</p>
<p>第一版可建立左右各自的實測對照表，用於已量範圍的粗略白模展示；先不外推中心附近、精確輪角或其他PWM，不自動加入回差補償。尚未驗證：重複轉圈、反向接近回差、精確停止距離、最大速度／加速度及自動循跡。本輪暫不選擇或部署路徑追蹤演算法。</p>
<details><summary>停止與資料來源</summary><p>直行觀察「無明顯滑行」不是測得零停止距離。獨立左940 µs／20%／10秒自動試驗已限時結束；PWM歸零後前兩筆仍有編碼器增量，後六筆為零，無精確滑行距離。此試驗不與手動半徑量測拼成同一次試驗，也不以轉圈資料推算20%直行速度。</p><p>來源：data/ground-center1300-trial*.json、ground-first-10pct-1s.json、ground-second-15pct-3s.json、manual-turning-measurements.json、ground-left940-trial1-20pct-10s.json。彙整：data/field-performance-20261018.json。原始檔與102cm修正註記保留。</p></details>
</section>'''
def table_markup(key):
    return f'''<div data-performance-table="{key}"><label>每頁筆數 <select aria-label="{key} 每頁筆數" onchange="fieldPerformancePage('{key}',0,this.value)"><option>10</option><option>20</option><option>50</option></select></label><div style="max-height:340px;overflow:auto" tabindex="0" aria-label="可捲動量測表"><table style="min-width:680px;width:100%"><thead><tr>{''.join('<th>'+h+'</th>' for h in tables[key]['headers'])}</tr></thead><tbody id="fp-{key}-rows">{''.join('<tr>'+''.join('<td>'+c+'</td>' for c in row)+'</tr>' for row in tables[key]['rows'][:10])}</tbody></table></div><div class="actions"><button type="button" id="fp-{key}-prev" disabled onclick="fieldPerformancePage('{key}',-1)">上一頁</button><span id="fp-{key}-status">1 / 1 · 共 {len(tables[key]['rows'])} 筆</span><button type="button" id="fp-{key}-next" disabled onclick="fieldPerformancePage('{key}',1)">下一頁</button></div></div>'''
html=html.replace('__STRAIGHT__',table_markup('straight')).replace('__TURN__',table_markup('turn'))
js_path=ROOT/'integration.js';js=js_path.read_text(encoding='utf-8')
marker='\n// Read-only field report, generated by build_field_performance.py\n'
end_marker='\n// END GENERATED FIELD PERFORMANCE\n'
suffix=js.split(end_marker,1)[1] if end_marker in js else ''
js=js.split(marker)[0]
# Replace superseded running summaries; retain prior per-run cards within a bounded disclosure.
js=re.sub(r'<div class="notice" id="manualTurningMeasurements">.*?</div>', '',js,count=1)
if '${fieldPerformancePanel()}' not in js:
    start=js.index('<div class="notice" id="left940Trial1">')
    end=js.index('<details><summary>查看實測結果與待驗證項目',start)
    history=js[start:end]
    js=js[:start]+'${fieldPerformancePanel()}<details><summary>逐輪測試與先前版本驗證紀錄</summary><div style="max-height:360px;overflow:auto">'+history+'</div></details>'+js[end:]
js+='\n// Read-only field report, generated by build_field_performance.py\n'
js+='function fieldPerformancePanel(){return '+json.dumps(html,ensure_ascii=False)+';}\n'
js+='const fieldPerformanceTables='+json.dumps(tables,ensure_ascii=False)+';\n'
js+='''const fieldPerformancePaging={};
function fieldPerformancePage(key,delta,size){
 const data=fieldPerformanceTables[key];if(!data)return;
 const state=fieldPerformancePaging[key]||(fieldPerformancePaging[key]={page:0,size:10});
 if(size){state.size=[10,20,50].includes(Number(size))?Number(size):10;state.page=0;}
 const pages=Math.max(1,Math.ceil(data.rows.length/state.size));
 state.page=Math.max(0,Math.min(pages-1,state.page+delta));
 const body=document.getElementById('fp-'+key+'-rows');body.replaceChildren();
 for(const row of data.rows.slice(state.page*state.size,(state.page+1)*state.size)){
  const tr=document.createElement('tr');for(const value of row){const td=document.createElement('td');td.textContent=value;tr.append(td);}body.append(tr);
 }
 document.getElementById('fp-'+key+'-status').textContent=(state.page+1)+' / '+pages+' · 共 '+data.rows.length+' 筆';
 document.getElementById('fp-'+key+'-prev').disabled=state.page===0;
 document.getElementById('fp-'+key+'-next').disabled=state.page===pages-1;
}
'''
js_path.write_text(js+end_marker+suffix,encoding='utf-8')
print(json.dumps(report['derived'],ensure_ascii=False))

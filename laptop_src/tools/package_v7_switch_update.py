"""Packaging only; do not rerun historical suites or connect to Orin."""
import hashlib
import json
import zipfile
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'laptop_workbench/data/experiments/v7-pid-integration'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def read(name):
    return json.loads((OUT/name).read_text(encoding='utf-8'))

def main():
    report, manifest, tests = read('report.json'), read('manifest.json'), read('switch-tests.json')
    assert tests['passed'] and tests['tests'] == 8
    assert all(sha(ROOT/n) == h for n,h in tests['source_sha256'].items())
    entries = read('entry-gates-tests.json')
    assert entries['source_sha256'] == sha(ROOT/'laptop_workbench/pid-live.js')
    assert entries['test_sha256'] == sha(ROOT/'laptop_workbench/test_pid_entry_gates.js')
    changed = {n for n,h in manifest['files'].items() if sha(ROOT/n) != h}
    allowed = {'web/calibrated_path.py', 'web/live_model_worker.py'}
    assert changed <= allowed, changed
    manifest.setdefault('pre_switch_files', dict(manifest['files']))
    manifest['files'] = {n:sha(ROOT/n) for n in manifest['files']}
    manifest['laptop_only_files'] = {n:sha(ROOT/n) for n in manifest['laptop_only_files']}
    manifest.update(format=3, updated_at=datetime.now(timezone.utc).isoformat(),
        switch_version=1, verification={
            'historical_core_tests':95, 'historical_entry_tests':11,
            'historical_suites_rerun':False, 'core_files_unchanged':False,
            'changed_remote_files':sorted(allowed), 'switch_delta_tests':tests['tests'],
            'switch_results_sha256':sha(OUT/'switch-tests.json'),
            'note':'95項屬舊核心版本紀錄；新切換行為以8項差異測試驗證，其他遠端檔案雜湊核對未變。'})
    target=OUT/'v7-pid-switch-v1.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as archive:
        for n in [*manifest['files'],*manifest['laptop_only_files']]:
            archive.write(ROOT/n,n)
        for n in ['switch-tests.json','entry-gates-tests.json']:
            archive.write(OUT/n,'verification/'+n)
        for n in ['tests/test_v7_switch.py','tests/test_calibrated_pid.py']:
            archive.write(ROOT/n,'verification/'+n)
        archive.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    with zipfile.ZipFile(target) as archive:
        assert archive.testzip() is None
        for n,h in {**manifest['files'],**manifest['laptop_only_files']}.items():
            assert hashlib.sha256(archive.read(n)).hexdigest() == h
    report.setdefault('pre_switch_bundle', report['bundle'])
    report['bundle'] = dict(path=str(target.relative_to(ROOT)), sha256=sha(target))
    for n,v in [('manifest.json',manifest),('report.json',report)]:
        (OUT/n).write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(bundle=report['bundle'],switch_tests=8,remote_changed=sorted(changed)),
                     ensure_ascii=False))

if __name__ == '__main__':
    main()

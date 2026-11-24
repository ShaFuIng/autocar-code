"""Package only: reuse unchanged core evidence and the new entry-gate results."""
import hashlib
import json
import zipfile
from datetime import datetime, timezone
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'laptop_workbench/data/experiments/v7-pid-integration'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    report=json.loads((OUT/'report.json').read_text(encoding='utf-8'))
    manifest=json.loads((OUT/'manifest.json').read_text(encoding='utf-8'))
    tests=json.loads((OUT/'entry-gates-tests.json').read_text(encoding='utf-8'))
    assert report['passed'] and report['tests']==95
    assert tests['passed'] and tests['tests']==11
    assert tests['source_sha256']==sha(ROOT/'laptop_workbench/pid-live.js')
    assert tests['test_sha256']==sha(ROOT/'laptop_workbench/test_pid_entry_gates.js')
    # Fail instead of reusing old verification for a changed vehicle controller.
    assert all(sha(ROOT/name)==digest for name,digest in manifest['files'].items())
    laptop=['laptop_workbench/pid-live.js','laptop_workbench/app.js']
    manifest.update(format=2,updated_at=datetime.now(timezone.utc).isoformat(),
        laptop_only_files={name:sha(ROOT/name) for name in laptop},
        verification={'core_tests':95,'core_rerun':False,'core_files_unchanged':True,
                      'entry_tests':11,'entry_results_sha256':sha(OUT/'entry-gates-tests.json')},
        installation_note='files只部署到Orin；laptop_only_files只用於筆電工作台。不要整包盲目覆寫到Orin。未包含自動部署或重啟程式。')
    target=OUT/'v7-pid-entry-update.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as archive:
        for name in [*manifest['files'],*laptop]:archive.write(ROOT/name,name)
        archive.write(OUT/'entry-gates-tests.json','verification/entry-gates-tests.json')
        archive.write(ROOT/'laptop_workbench/test_pid_entry_gates.js','verification/test_pid_entry_gates.js')
        archive.writestr('manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    with zipfile.ZipFile(target) as archive:
        assert all(hashlib.sha256(archive.read(name)).hexdigest()==digest
                   for name,digest in {**manifest['files'],**manifest['laptop_only_files']}.items())
    report.setdefault('previous_bundle',report['bundle'])
    report['bundle']={'path':str(target.relative_to(ROOT)),'sha256':sha(target)}
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    (OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'bundle':report['bundle'],'core_unchanged':True,'new_tests':11},ensure_ascii=False))

if __name__=='__main__':main()

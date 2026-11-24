"""Read-only field readiness snapshot. Never starts tasks or opens devices."""
import datetime
import json
import shutil
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BASE = 'http://127.0.0.1:8093'

def main():
    report = {'checked_at': datetime.datetime.now().astimezone().isoformat(),
              'scope': '唯讀；不啟用馬達、舵機、相機錄影或分析工作',
              'disk_free_gib': round(shutil.disk_usage(ROOT).free / 2**30, 1), 'checks': {}}
    for name, path in [('workbench', '/api/status'), ('orin', '/api/orin/status'),
                       ('records', '/api/orin/records')]:
        try:
            with urllib.request.urlopen(BASE + path, timeout=55) as response:
                value = json.load(response)
            if name == 'orin':
                value = {k: value.get(k) for k in ('connected', 'host', 'error', 'vehicle', 'telemetry', 'disk_free_bytes', 'services')}
            elif name == 'records':
                value = {'sync': value.get('sync'), 'local_record_count': len(value.get('local', []))}
            report['checks'][name] = value
        except Exception as error:
            report['checks'][name] = {'error': str(error)}
    target = ROOT / 'data' / ('field-check-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    target.parent.mkdir(exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=True, indent=2))
    print('Saved:', target)

if __name__ == '__main__':
    main()

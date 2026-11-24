"""Run on Orin: python3 ~/orin_interface_v1/tools/record_only.py start|stop|status."""
import argparse
import base64
import json
import os
from pathlib import Path
import sys
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser(description='Camera-only recording; no vehicle movement')
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    args = parser.parse_args()
    base = os.environ.get('ORIN_RECORD_URL', 'http://192.168.50.1:8000').rstrip('/')
    token = (Path.home() / '.config/orin_lab/token').read_text().strip()
    headers = {'Authorization': 'Basic ' + base64.b64encode(('jetson:' + token).encode()).decode(),
               'Origin': base, 'X-Orin-Control': '1', 'Content-Type': 'application/json'}
    req = urllib.request.Request(base + '/api/record/' + args.action, headers=headers,
                                 data=None if args.action == 'status' else b'{}')
    try:
        with urllib.request.urlopen(req, timeout=10) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        print(exc.read().decode(), file=sys.stderr)
        return 1
    except urllib.error.URLError as exc:
        print('Request not confirmed; check status before retrying: ' + str(exc.reason), file=sys.stderr)
        return 1
    run = result.get('current')
    if run:
        print('Status:', run['status'], '| Frames:', run['frames'], '| Reason:', run['reason'])
        print('Files:', run['path'])
    else:
        print('No recording in this service session.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

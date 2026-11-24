"""Stage all three packages without changing the running web directory or service."""
import base64
import hashlib
import io
import json
import shlex
import subprocess
import zipfile
from pathlib import Path
from inspect_ops07_remote import ROOT, OUT, SSH


def remote(script, payload=b''):
    host=json.loads((OUT/'before.json').read_text(encoding='utf8'))['ssh_host']
    p=subprocess.run(SSH+['jetson@'+host,'python3 -c '+shlex.quote(script)],input=payload,capture_output=True,timeout=60)
    if p.returncode:raise RuntimeError(p.stderr.decode(errors='replace')[-2000:])
    return json.loads(p.stdout)


def stage():
    before=json.loads((OUT/'before.json').read_text(encoding='utf8'))
    # Only known Python/JSON source; never copy the authentication token.
    script=r'''
import base64,hashlib,json,sys
from pathlib import Path
root=Path.home()/'orin_interface_v1/web';expected=json.load(sys.stdin)
out={}
for name,digest in expected.items():
 if Path(name).name!=name or Path(name).suffix not in ('.py','.json'):raise ValueError('Invalid source name')
 data=(root/name).read_bytes()
 if hashlib.sha256(data).hexdigest()!=digest:raise ValueError('Remote changed: '+name)
 out[name]=base64.b64encode(data).decode()
print(json.dumps(out))
'''
    data=remote(script,json.dumps(before['manifest']).encode())
    baseline=OUT/'remote-before-r5';baseline.mkdir(exist_ok=True)
    for name,encoded in data.items():(baseline/name).write_bytes(base64.b64decode(encoded))
    candidate=ROOT/'data/experiments/ops07-adapter-20261109'
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as z:
        for p in (candidate/'candidates').rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts:z.write(p,p.relative_to(candidate).as_posix())
        z.write(candidate/'integration_remote.candidate.py','integration_remote.candidate.py')
        z.write(candidate/'acceptance.json','offline-acceptance.json')
    script=r'''
import hashlib,io,json,sys,zipfile
from pathlib import Path
root=Path.home()/'orin_interface_v1/ops07/releases/20261109-r5'
if root.exists():raise ValueError('Release already exists; do not overwrite')
data=sys.stdin.buffer.read();z=zipfile.ZipFile(io.BytesIO(data))
for item in z.infolist():
 target=root/item.filename
 if not target.resolve().is_relative_to(root.resolve()) or item.is_dir():raise ValueError('Invalid archive entry')
root.mkdir(parents=True)
for item in z.infolist():
 target=root/item.filename;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(item))
result=dict(path=str(root),archive_sha256=hashlib.sha256(data).hexdigest(),files=len(z.infolist()),active_web_changed=False,restarted=False)
print(json.dumps(result))
'''
    result=remote(script,buffer.getvalue())
    (OUT/'staged.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result))


if __name__=='__main__':stage()

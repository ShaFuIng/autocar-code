"""Run once with sudo on Orin; retain the USB management hotspot."""
import configparser,json,os,subprocess,time,uuid,shutil
from pathlib import Path

def main():
    if os.geteuid()!=0:raise SystemExit('Run: sudo python3 /home/jetson/wifi_adapter_setup/setup_ttu_wifi.py')
    userhome=Path('/home/jetson')
    secret=userhome/'.config/orin_car/ttu_wifi_credentials.json'
    credentials=json.loads(secret.read_text())
    target=Path('/etc/NetworkManager/system-connections/Orin-ttuwifi5g.nmconnection')
    backup=userhome/'wifi_adapter_setup'/('backup_ttu_'+time.strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(mode=0o700,parents=True,exist_ok=False)
    if target.exists():shutil.copy2(target,backup/target.name)
    profile=configparser.ConfigParser(interpolation=None)
    ident=str(uuid.uuid4())
    profile['connection']={'id':'Orin-ttuwifi5g','uuid':ident,'type':'wifi','interface-name':'wlP1p1s0','autoconnect':'true','autoconnect-priority':'10'}
    profile['wifi']={'mode':'infrastructure','ssid':'ttuwifi5g','band':'a'}
    profile['wifi-security']={'key-mgmt':'wpa-eap','proto':'rsn;'}
    profile['802-1x']={'eap':'ttls;','identity':credentials['identity'],'anonymous-identity':'anonymous','phase2-auth':'pap','password':credentials['password'],'password-flags':'0','ca-cert':'/etc/ssl/certs/ca-certificates.crt','domain-suffix-match':'ttu.edu.tw'}
    profile['ipv4']={'method':'auto','route-metric':'100'}
    profile['ipv6']={'method':'auto','addr-gen-mode':'stable-privacy'}
    temp=target.with_suffix('.pending')
    fd=os.open(temp,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
    with os.fdopen(fd,'w') as f:profile.write(f)
    os.chmod(temp,0o600);os.replace(temp,target)
    def run(args,timeout=60):
        r=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        output=(r.stdout+r.stderr).replace(credentials['password'],'[redacted]')
        print(output.strip(),flush=True)
        return r.returncode
    if run(['nmcli','connection','load',str(target)]):raise SystemExit('Profile load failed; USB hotspot was not changed')
    if run(['nmcli','--wait','45','connection','up','uuid',ident],55):
        run(['nmcli','connection','modify','uuid',ident,'connection.autoconnect','no'])
        raise SystemExit('Campus connection failed; USB hotspot retained. Do not repeat blindly; inspect authentication logs.')
    # Prefer the user's phone at future auto-connect; keep campus Wi-Fi as fallback.
    run(['nmcli','connection','modify','ShaFuIng','connection.autoconnect','yes','connection.autoconnect-priority','999'])
    run(['nmcli','-t','-f','DEVICE,STATE,CONNECTION','device'])
    run(['ip','-4','-brief','address','show','dev','wlP1p1s0'])
    print('Campus profile is persistent. USB hotspot remains at http://192.168.50.1:8000/.',flush=True)
    print('Use the wlP1p1s0 IPv4 address above with :8000/manual for campus access.',flush=True)

if __name__=='__main__':main()

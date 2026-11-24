"""Run interactively on Orin with sudo; no vehicle service or device access."""
import os,subprocess,time
SSID='R18-AIRACE';PROFILE='Orin-R18-AIRACE';DEV='wlP1p1s0'
def run(args,**kwargs):return subprocess.run(args,check=True,text=True,**kwargs)
def main():
 if os.geteuid()!=0:raise SystemExit('Run with sudo in an interactive terminal')
 def find_target():
  listing=run(['nmcli','-t','--escape','no','-f','SSID,FREQ,SECURITY','device','wifi','list','ifname',DEV,'--rescan','no'],capture_output=True).stdout
  found=[]
  for line in listing.splitlines():
   parts=line.split(':',2)
   if len(parts)!=3 or parts[0]!=SSID:continue
   try:frequency=int(parts[1].split()[0])
   except (ValueError,IndexError):continue
   if 4900<=frequency<5900:found.append(parts)
  return found
 targets=find_target()
 if not targets:
  print('Requesting Wi-Fi scan; waiting up to 20 seconds for results...',flush=True)
  run(['nmcli','device','wifi','rescan','ifname',DEV])
  for _ in range(10):
   time.sleep(2);targets=find_target()
   if targets:break
 if not targets:raise SystemExit('R18-AIRACE 5GHz not found after waiting; existing connection unchanged')
 print('Found '+SSID+' at '+targets[0][1],flush=True)
 security=targets[0][2]
 if '802.1X' in security:raise SystemExit('Enterprise authentication detected; credentials/certificate settings must be confirmed first')
 old=run(['nmcli','-g','connection.autoconnect-priority','connection','show','Orin-ttuwifi5g'],capture_output=True).stdout.strip()
 priority=max(30,int(old)+1)
 if priority>999:raise SystemExit('Campus priority already maximum; confirm preference before changing it')
 present=subprocess.run(['nmcli','-g','802-11-wireless.ssid','connection','show',PROFILE],capture_output=True,text=True)
 if present.returncode==0:
  if present.stdout.strip()!=SSID:raise SystemExit('Profile name collision; no change made')
 else:
  args=['nmcli','connection','add','type','wifi','ifname',DEV,'con-name',PROFILE,'ssid',SSID,'802-11-wireless.band','a','connection.autoconnect','no']
  if 'WPA' in security:args+=['wifi-sec.key-mgmt','wpa-psk' if 'WPA2' in security else 'sae']
  elif security.strip() not in ('','--'):raise SystemExit('Unsupported security; existing connection unchanged')
  run(args)
 run(['nmcli','connection','modify',PROFILE,'802-11-wireless.band','a','connection.autoconnect-priority',str(priority)])
 print('Enter Wi-Fi credentials locally when prompted. Existing campus profile is retained.',flush=True)
 run(['nmcli','--ask','--wait','45','connection','up',PROFILE])
 run(['nmcli','connection','modify',PROFILE,'connection.autoconnect','yes'])
 run(['nmcli','-g','connection.id,802-11-wireless.ssid,802-11-wireless.band,connection.autoconnect,connection.autoconnect-priority','connection','show',PROFILE])
 run(['ip','-br','-4','address','show',DEV])
 print('Done. Vehicle services were not restarted. No actuator commands were sent.')
if __name__=='__main__':main()

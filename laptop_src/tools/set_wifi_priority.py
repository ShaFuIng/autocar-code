"""Set saved Wi-Fi preference without switching the active connection."""
import os,subprocess

def main():
    if os.geteuid()!=0:raise SystemExit('Run with sudo')
    for name in ('ShaFuIng','Orin-ttuwifi5g'):
        subprocess.run(['nmcli','-g','connection.id','connection','show',name],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['nmcli','connection','modify','ShaFuIng','connection.autoconnect','yes','connection.autoconnect-priority','999'],check=True)
    subprocess.run(['nmcli','connection','modify','Orin-ttuwifi5g','connection.autoconnect','yes','connection.autoconnect-priority','10'],check=True)
    for name in ('ShaFuIng','Orin-ttuwifi5g'):
        print(name,flush=True)
        subprocess.run(['nmcli','-g','connection.autoconnect,connection.autoconnect-priority','connection','show',name],check=True)
    print('Saved. The active connection and USB management hotspot were not switched.')

if __name__=='__main__':main()

#!/usr/bin/python3
"""Run with sudo on Orin; configure only the AIC USB adapter as a hotspot."""
import os
import pathlib
import pwd
import secrets
import subprocess

IFACE = 'wlx688fc970b5a9'
PROFILE = 'Orin-Car-USB'
SSID = 'Orin-Car'
ADDRESS = '192.168.50.1/24'


def run(*args):
    result = subprocess.run(args, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or 'Command failed: ' + args[0])
    return result.stdout.strip()


def main():
    if os.geteuid() != 0:
        raise SystemExit('Run with sudo')
    if not pathlib.Path('/sys/class/net', IFACE).exists():
        raise SystemExit('USB Wi-Fi interface missing; do not change built-in Wi-Fi')
    device = pathlib.Path('/sys/class/net', IFACE, 'device').resolve()
    usb_device = next((p for p in [device, *device.parents]
                       if (p / 'idVendor').exists()), None)
    if usb_device is None or (usb_device / 'idVendor').read_text().strip() != 'a69c' \
            or (usb_device / 'idProduct').read_text().strip() != '8d81':
        raise SystemExit('Unexpected USB device; refusing to configure this interface')
    routes = run('ip', 'route').splitlines()
    if any(line.startswith('192.168.50.0/24 ') and 'dev ' + IFACE not in line for line in routes):
        raise SystemExit('Hotspot subnet overlaps an existing network')
    account = pwd.getpwnam('jetson')
    secret_dir = pathlib.Path(account.pw_dir, '.config/orin_car')
    secret_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chown(secret_dir, account.pw_uid, account.pw_gid)
    secret_file = secret_dir / 'hotspot_password'
    if secret_file.exists():
        password = secret_file.read_text().strip()
    else:
        password = secrets.token_urlsafe(18)
        fd = os.open(secret_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(password + '\n')
        os.chown(secret_file, account.pw_uid, account.pw_gid)
    names = run('nmcli', '-g', 'NAME', 'connection', 'show').splitlines()
    if PROFILE not in names:
        run('nmcli', 'connection', 'add', 'type', 'wifi', 'ifname', IFACE,
            'con-name', PROFILE, 'ssid', SSID)
    run('nmcli', 'connection', 'modify', PROFILE,
        'connection.interface-name', IFACE,
        'connection.autoconnect', 'yes', 'connection.autoconnect-priority', '50',
        '802-11-wireless.mode', 'ap', '802-11-wireless.band', 'bg',
        '802-11-wireless.channel', '6',
        '802-11-wireless.cloned-mac-address', 'permanent',
        '802-11-wireless-security.key-mgmt', 'wpa-psk',
        '802-11-wireless-security.proto', 'rsn',
        '802-11-wireless-security.psk', password,
        'ipv4.method', 'shared', 'ipv4.addresses', ADDRESS,
        'ipv4.never-default', 'yes', 'ipv6.method', 'disabled')
    # Keep UFW active. Only open dashboard/SSH/Notebook to the hotspot subnet.
    if subprocess.run(['systemctl', 'is-active', '--quiet', 'ufw']).returncode == 0:
        for port in ('22', '8000', '8888'):
            run('ufw', 'allow', 'in', 'on', IFACE, 'from', '192.168.50.0/24',
                'to', 'any', 'port', port, 'proto', 'tcp')
        for port, proto in [('67', 'udp'), ('53', 'udp'), ('53', 'tcp')]:
            run('ufw', 'allow', 'in', 'on', IFACE, 'to', 'any', 'port', port,
                'proto', proto)
        uplink = next((line.split(' dev ')[1].split()[0] for line in routes
                       if line.startswith('default ') and ' dev ' in line), None)
        if uplink and uplink != IFACE:
            run('ufw', 'route', 'allow', 'in', 'on', IFACE, 'out', 'on', uplink,
                'from', '192.168.50.0/24')
    try:
        run('nmcli', '--wait', '30', 'connection', 'up', PROFILE)
    except Exception:
        subprocess.run(['nmcli', 'connection', 'down', PROFILE], capture_output=True)
        run('nmcli', 'connection', 'modify', PROFILE, 'connection.autoconnect', 'no')
        raise
    print('Hotspot ready:', SSID)
    print('Dashboard: http://192.168.50.1:8000/')
    print('Password file:', secret_file)
    print(run('nmcli', '-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device'))
    print(run('ip', 'route'))


if __name__ == '__main__':
    main()

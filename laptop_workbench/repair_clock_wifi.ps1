$ErrorActionPreference = 'Stop'
$sshKey = Join-Path $env:USERPROFILE '.ssh/codex_orin_ed25519'
if (-not (Test-Path -LiteralPath $sshKey)) { throw 'Existing Orin SSH key not found.' }
$utcSeconds = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
Write-Host 'Uses this laptop UTC clock. Enter the Orin sudo password only in this terminal.'
Write-Host 'Sets time and activates the existing school Wi-Fi profile. Does not touch vehicle services.'
$remoteCommand = "sudo date -u -s '@$utcSeconds' && sudo nmcli --wait 30 connection up Orin-ttuwifi5g && date -u && timedatectl show -p NTP -p NTPSynchronized && hostname -I"
& ssh -t -i $sshKey -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=yes -o HostKeyAlias=orinnano-super.local jetson@orinnano-super.local $remoteCommand
if ($LASTEXITCODE -ne 0) { throw 'Clock/Wi-Fi repair did not fully finish. Keep USB connected and report the error; do not restart vehicle services.' }
Write-Host 'Clock and Wi-Fi command finished. Keep USB connected until wireless, camera and recording checks complete.'

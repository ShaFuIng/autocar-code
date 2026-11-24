$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
try {
    $workbenchResponse = Invoke-WebRequest -Uri 'http://127.0.0.1:8093/api/status' -TimeoutSec 2 -UseBasicParsing
    if ($workbenchResponse.StatusCode -eq 200) {
        Write-Host 'Workbench already running: http://127.0.0.1:8093/'
        return
    }
} catch { }
python server.py --port 8093

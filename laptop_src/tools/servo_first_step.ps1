param([string]$PortName='COM8', [ValidateRange(1,10)][int]$StepCount=1, [ValidateRange(1,10)][int]$HoldSeconds=5)
$ErrorActionPreference='Stop'
$port = [System.IO.Ports.SerialPort]::new($PortName,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$port.DtrEnable=$true
$port.RtsEnable=$false
$port.ReadTimeout=250
$port.WriteTimeout=500
$logPath=Join-Path $PSScriptRoot '../logs/servo_first_step_20261010.jsonl'
$timer=[Diagnostics.Stopwatch]::StartNew()
$baseline=$null
$started=$false
$stepped=$false
$lastState=''
$initialAt=0
$stepsSent=0
$confirmedAt=$null
$lastStepAt=-1
try {
  $port.Open()
  Write-Output 'COM8_CONNECTED: waiting for initial status; do not press START yet'
  while($timer.Elapsed.TotalSeconds -lt 120) {
    try { $line=$port.ReadLine() } catch [System.TimeoutException] { continue }
    try { $m=$line | ConvertFrom-Json } catch { continue }
    Add-Content -LiteralPath $logPath -Encoding utf8 -Value $line
    if($m.type -ne 'status') { if($m.type -eq 'event'){Write-Output ($m | ConvertTo-Json -Compress)}; continue }
    if($m.fw -notin @('orin-v1.1.0-cal','orin-v1.1.1-cal','orin-v1.1.2-cal') -or !$m.servo_calibration) { throw 'Unexpected firmware; no motion sent' }
    if($m.motor_a.pwm_command -ne 0 -or $m.motor_b.pwm_command -ne 0) { throw 'Motor PWM not zero' }
    if($null -eq $baseline) {
      $baseline=$m.generation
      Write-Output "PRESS_START_NOW: release ESTOP, then press and release START once. Baseline generation=$baseline state=$($m.state)"
    }
    if($m.state -ne $lastState) { Write-Output "STATE=$($m.state) reason=$($m.reason) generation=$($m.generation)"; $lastState=$m.state }
    if(!$started -and $m.state -eq 'READY' -and $m.generation -ne $baseline -and $m.pins.D11 -eq 1) {
      $port.Write('i')
      $started=$true
      $initialAt=$timer.Elapsed.TotalSeconds
      Write-Output 'SENT i: explicit 1340 us reference'
    }
    if($started -and $timer.Elapsed.TotalSeconds-$initialAt -ge 3) {
      if($m.state -ne 'READY' -or !$m.steer_output_enabled) { throw 'Vehicle stopped / servo output disabled; no further step sent' }
      $expected=1340+10*$stepsSent
      if($m.steer_us_command -eq $expected -and $stepsSent -lt $StepCount -and $timer.Elapsed.TotalSeconds-$lastStepAt -ge 0.5) {
        $port.Write('+')
        $lastStepAt=$timer.Elapsed.TotalSeconds
        $stepsSent++
        Write-Output "SENT +: step $stepsSent/$StepCount, requested $($expected+10) us"
      } elseif($stepsSent -eq $StepCount -and $m.steer_us_command -eq $expected) {
        $stepped=$true
        if($null -eq $confirmedAt) {
          $confirmedAt=$timer.Elapsed.TotalSeconds
          Write-Output "CONFIRMED: pulse $expected us, motor PWM 0/0; holding $HoldSeconds seconds for observation"
        }
        if($timer.Elapsed.TotalSeconds-$confirmedAt -ge $HoldSeconds) { break }
      }
    }
  }
  if(!$stepped) { Write-Output 'No step completed before deadline' }
} finally {
  if($port.IsOpen) { $port.Write('o'); Write-Output 'SENT o: steering pulses off and locked'; $port.Close() }
  $port.Dispose()
}

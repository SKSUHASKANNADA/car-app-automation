$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Runtime.WindowsRuntime
$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
  $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
  $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
function Await($op, $resultType) {
  $m = $asTaskGeneric.MakeGenericMethod($resultType)
  $t = $m.Invoke($null, @($op))
  $t.Wait(-1) | Out-Null
  return $t.Result
}
try { Start-Service icssvc -ErrorAction SilentlyContinue } catch { }
$profile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
if ($null -eq $profile) { Write-Output "NO_PROFILE"; exit 2 }
Write-Output ("PROFILE: " + $profile.ProfileName)
$mgr = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($profile)
Write-Output ("STATE: " + $mgr.TetheringOperationalState.ToString())
$res = Await ($mgr.StartTetheringAsync()) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
Write-Output ("START: " + $res.Status.ToString() + ((" | " + $res.AdditionalErrorMessage) | Out-String).Trim())
Start-Sleep -Seconds 3
$mgr2 = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile(
  [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile())
Write-Output ("STATE_AFTER: " + $mgr2.TetheringOperationalState.ToString())
Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Where-Object { $_.InterfaceAlias -like "*Local Area Connection*" -or $_.IPAddress -like "192.168.137.*" -or $_.IPAddress -like "192.168.173.*" } |
  Select-Object InterfaceAlias, IPAddress | Format-Table -AutoSize | Out-String | Write-Output

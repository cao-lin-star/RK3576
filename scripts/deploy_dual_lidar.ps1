param(
    [ValidateSet('footbath-rk-lan','footbath-rk-remote')]
    [string]$HostAlias = 'footbath-rk-lan',
    [string]$Version = 'v2.0.1',
    [switch]$MotorPowerOff,
    [switch]$PackageOnly
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
function Check-Exit([string]$Step) {
    if ($LASTEXITCODE -ne 0) { throw "$Step failed (exit $LASTEXITCODE)." }
}
if (!$PackageOnly -and !$MotorPowerOff) {
    throw 'Stop robot tasks and switch OFF 24V motor power; then pass -MotorPowerOff.'
}
$commit = (& git -C $repo rev-parse --verify "$Version^{commit}").Trim()
Check-Exit 'Resolve version'
if ($commit -notmatch '^[0-9a-f]{40}$') { throw 'Invalid commit' }
# Fail before upload if the selected version does not contain the LF fix.
$attrs = & git -C $repo show "${commit}:.gitattributes"
Check-Exit 'Read attributes'
if ($attrs -notcontains '* text=auto eol=lf') { throw 'Select v2.0.1 or a version with the Linux LF fix.' }
$id = (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,8)
$packageDir = Join-Path $env:TEMP "footbath-deploy-$id"
New-Item -ItemType Directory -Path $packageDir | Out-Null
$archive = Join-Path $packageDir 'source.tar.gz'
& git -C $repo archive --format=tar.gz -o $archive $commit
Check-Exit 'Archive'
# Normalize the deployment helper independently of the source version.
$helper = Join-Path $packageDir 'deploy.sh'
$body = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'deploy_dual_lidar_remote.sh')).Replace("`r`n", "`n")
[IO.File]::WriteAllText($helper, $body, [Text.UTF8Encoding]::new($false))
$hash = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
$helperHash = (Get-FileHash -LiteralPath $helper -Algorithm SHA256).Hash.ToLowerInvariant()
Write-Host "Version: $Version ($commit)"
Write-Host "Package: $packageDir"
if ($PackageOnly) { return }
$remoteDir = "/home/sky/footbath-upload-$id"
& ssh -o ConnectTimeout=10 $HostAlias "test `"`$(id -un)`" = sky && test `"`$(uname -m)`" = aarch64 && mkdir -m 700 '$remoteDir'"
Check-Exit 'Target preflight'
& scp $archive $helper "${HostAlias}:$remoteDir/"
Check-Exit 'Upload'
# Allocate a terminal for sudo. Password is entered on the target, never stored.
& ssh -tt -o ConnectTimeout=10 $HostAlias "sudo /bin/bash '$remoteDir/deploy.sh' '$remoteDir' '$commit' '$hash' '$helperHash'"
Check-Exit 'Remote deployment (inspect printed rollback status on failure)'
Write-Host 'Deployment startup checks passed. Verify sensors and mapping with motor power OFF first.'

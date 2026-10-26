# Packages a Release build. Run from PowerShell on a licensed MSVC build machine.
# Does not build, publish, delete existing files or create an archive.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$QtDir,
    [Parameter(Mandatory = $true)][string]$BuildDir,
    [Parameter(Mandatory = $true)][string]$OutputDir
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if ($env:OS -ne 'Windows_NT') { throw 'This package helper targets Windows x64.' }
$qtVersion = '6.8.3'
$sourceSha256 = '56001b905601bb9023d399f3ba780d7fa940f3e4861e496a7c490331f49e0b80'
$sourceUrl = "https://download.qt.io/archive/qt/6.8/$qtVersion/submodules/qtbase-everywhere-src-$qtVersion.tar.xz"
$projectDir = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$QtDir = (Resolve-Path -LiteralPath $QtDir).Path
$BuildDir = (Resolve-Path -LiteralPath $BuildDir).Path
$OutputDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputDir)

if (Test-Path -LiteralPath $OutputDir) {
    if (-not (Test-Path -LiteralPath $OutputDir -PathType Container)) {
        throw 'OutputDir must be a new or empty directory.'
    }
    if (@(Get-ChildItem -LiteralPath $OutputDir -Force).Count -gt 0) {
        throw 'OutputDir must be empty to avoid shipping stale files. Choose a new folder.'
    }
}
foreach ($protected in @($projectDir, $QtDir, $BuildDir)) {
    if ([string]::Equals($OutputDir, $protected, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'OutputDir cannot be the source, Qt SDK or build directory itself.'
    }
}
if ($OutputDir.StartsWith($QtDir + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'OutputDir cannot be inside the Qt SDK.'
}

$deployQt = Join-Path $QtDir 'bin/windeployqt.exe'
$qmake = Join-Path $QtDir 'bin/qmake.exe'
if (-not (Test-Path -LiteralPath $deployQt) -or -not (Test-Path -LiteralPath $qmake)) {
    throw 'QtDir must point to a Qt 6.8.3 MSVC x64 installation.'
}
$installedVersion = (& $qmake -query QT_VERSION | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $installedVersion -ne $qtVersion) {
    throw "This license bundle is pinned to Qt $qtVersion; SDK reports $installedVersion."
}

$executable = $null
foreach ($candidate in @((Join-Path $BuildDir 'Release/OpenTrackStudio.exe'),
                          (Join-Path $BuildDir 'OpenTrackStudio.exe'))) {
    if (Test-Path -LiteralPath $candidate -PathType Leaf) { $executable = $candidate; break }
}
if ($null -eq $executable) { throw 'No Release OpenTrackStudio.exe was found in BuildDir.' }

# Optional reuse of an already extracted, unmodified official QtBase source tree.
# The default cache stays in the build tree and is never copied into the package.
if ($env:OPENTRACK_QTBASE_SOURCE) {
    $qtSource = (Resolve-Path -LiteralPath $env:OPENTRACK_QTBASE_SOURCE).Path
} else {
    $cacheDir = Join-Path $BuildDir '_packaging'
    New-Item -ItemType Directory -Force -Path $cacheDir | Out-Null
    $archive = Join-Path $cacheDir "qtbase-everywhere-src-$qtVersion.tar.xz"
    if (-not (Test-Path -LiteralPath $archive)) {
        Write-Host "Downloading official QtBase $qtVersion license source archive..."
        Invoke-WebRequest -Uri $sourceUrl -OutFile $archive
    }
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $sourceSha256) {
        throw 'QtBase source checksum mismatch. The archive has not been extracted.'
    }
    $qtSource = Join-Path $cacheDir "qtbase-everywhere-src-$qtVersion"
    if (-not (Test-Path -LiteralPath (Join-Path $qtSource '.cmake.conf'))) {
        & tar -xf $archive -C $cacheDir
        if ($LASTEXITCODE -ne 0) { throw 'Could not extract the verified QtBase source archive.' }
    }
}
$versionFile = Join-Path $qtSource '.cmake.conf'
if (-not (Test-Path -LiteralPath $versionFile) -or
    (Get-Content -LiteralPath $versionFile -Raw) -notmatch 'QT_REPO_MODULE_VERSION\s+"6\.8\.3"') {
    throw 'The QtBase source tree must match Qt 6.8.3.'
}
if (-not (Test-Path -LiteralPath (Join-Path $qtSource 'LICENSES/LGPL-3.0-only.txt'))) {
    throw 'The QtBase source tree is missing its license texts.'
}

# Use redistributable CRT files from Visual Studio's Redist directory only.
$redistRoots = @()
if ($env:VCToolsRedistDir) { $redistRoots += $env:VCToolsRedistDir }
$vsWhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
$vsInstall = $null
if (Test-Path -LiteralPath $vsWhere) {
    $vsInstall = (& $vsWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Out-String).Trim()
    if ($vsInstall) {
        $versions = Get-ChildItem -LiteralPath (Join-Path $vsInstall 'VC/Redist/MSVC') -Directory |
            Where-Object { $_.Name -match '^\d+\.\d+\.\d+$' } |
            Sort-Object { [version]$_.Name } -Descending
        $redistRoots += @($versions | ForEach-Object { $_.FullName })
    }
}
$crtDir = $null
$redistRoot = $null
foreach ($root in $redistRoots) {
    $candidate = Join-Path $root 'x64/Microsoft.VC143.CRT'
    if (Test-Path -LiteralPath (Join-Path $candidate 'vcruntime140.dll')) {
        $crtDir = $candidate; $redistRoot = $root; break
    }
}
if ($null -eq $crtDir) {
    throw 'Cannot locate the licensed MSVC x64 Redist/Microsoft.VC143.CRT directory. Run from a VS 2022 developer environment.'
}

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
Copy-Item -LiteralPath $executable -Destination (Join-Path $OutputDir 'OpenTrackStudio.exe')
$oldPath = $env:PATH
$oldVcInstall = $env:VCINSTALLDIR
$oldRedist = $env:VCToolsRedistDir
try {
    $env:PATH = (Join-Path $QtDir 'bin') + ';' + $env:PATH
    $env:VCToolsRedistDir = $redistRoot
    if ($vsInstall) { $env:VCINSTALLDIR = (Join-Path $vsInstall 'VC') + '\' }
    # Explicit plugins below keep packaging confined to QtBase, even when the SDK
    # also has Qt SVG, TIFF, WebP or multimedia modules installed. QtNetwork
    # and the Windows-native Schannel TLS plugin support the opt-in AI API.
    & $deployQt --release --no-translations --no-plugins --no-opengl-sw `
        --no-system-d3d-compiler --no-system-dxc-compiler --compiler-runtime -network `
        --skip-plugin-types generic,networkinformation,tls,styles,imageformats,sqldrivers,iconengines,qmltooling `
        --dir $OutputDir (Join-Path $OutputDir 'OpenTrackStudio.exe')
    if ($LASTEXITCODE -ne 0) { throw 'windeployqt failed.' }
} finally {
    $env:PATH = $oldPath
    $env:VCINSTALLDIR = $oldVcInstall
    $env:VCToolsRedistDir = $oldRedist
}
foreach ($group in @('platforms', 'imageformats', 'styles', 'tls')) {
    New-Item -ItemType Directory -Force -Path (Join-Path $OutputDir $group) | Out-Null
}
$plugins = @('platforms/qwindows.dll', 'platforms/qoffscreen.dll',
             'imageformats/qjpeg.dll', 'imageformats/qgif.dll', 'imageformats/qico.dll',
             'styles/qmodernwindowsstyle.dll', 'tls/qschannelbackend.dll')
foreach ($plugin in $plugins) {
    $source = Join-Path (Join-Path $QtDir 'plugins') $plugin
    if (-not (Test-Path -LiteralPath $source)) { throw "Required QtBase plugin missing: $plugin" }
    Copy-Item -LiteralPath $source -Destination (Join-Path $OutputDir $plugin)
}
Get-ChildItem -LiteralPath $crtDir -Filter '*.dll' -File |
    Copy-Item -Destination $OutputDir
@'
[Paths]
Prefix=.
Plugins=.
'@ | Set-Content -LiteralPath (Join-Path $OutputDir 'qt.conf') -Encoding ascii

foreach ($name in @('README.md', 'LICENSE', 'THIRD_PARTY_NOTICES.md')) {
    Copy-Item -LiteralPath (Join-Path $projectDir $name) -Destination $OutputDir
}
Copy-Item -LiteralPath (Join-Path $projectDir 'docs') -Destination (Join-Path $OutputDir 'docs') -Recurse
$assetDir = Join-Path $OutputDir 'assets'
New-Item -ItemType Directory -Force -Path $assetDir | Out-Null
foreach ($name in @('app-icon.svg', 'app-icon.png', 'app-icon.ico')) {
    Copy-Item -LiteralPath (Join-Path (Join-Path $projectDir 'assets') $name) -Destination $assetDir
}
$sampleDir = Join-Path $OutputDir 'examples'
New-Item -ItemType Directory -Force -Path $sampleDir | Out-Null
foreach ($name in @('example-track.png', 'example.otproj', 'example.trk')) {
    $sample = Join-Path (Join-Path $BuildDir 'smoke') $name
    if (Test-Path -LiteralPath $sample) { Copy-Item -LiteralPath $sample -Destination $sampleDir }
}

$noticeRoot = Join-Path $OutputDir "licenses/qt-$qtVersion"
New-Item -ItemType Directory -Force -Path $noticeRoot | Out-Null
Copy-Item -LiteralPath (Join-Path $qtSource 'LICENSES') -Destination (Join-Path $noticeRoot 'LICENSES') -Recurse
if (Test-Path -LiteralPath (Join-Path $QtDir 'sbom')) {
    Copy-Item -LiteralPath (Join-Path $QtDir 'sbom') -Destination (Join-Path $noticeRoot 'sbom') -Recurse
}

function Copy-QtNotice([string]$SourceFile) {
    $resolved = (Resolve-Path -LiteralPath $SourceFile).Path
    if (-not $resolved.StartsWith($qtSource + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'A Qt attribution references a file outside the QtBase source tree.'
    }
    $relative = $resolved.Substring($qtSource.Length).TrimStart([char[]]@('\', '/'))
    $destination = Join-Path (Join-Path $noticeRoot 'attributions') $relative
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
    Copy-Item -LiteralPath $resolved -Destination $destination
}

$attributionCount = 0
foreach ($attribution in Get-ChildItem -LiteralPath (Join-Path $qtSource 'src') -Recurse -File -Filter qt_attribution.json) {
    Copy-QtNotice $attribution.FullName
    $records = @(Get-Content -LiteralPath $attribution.FullName -Raw | ConvertFrom-Json)
    foreach ($record in $records) {
        foreach ($field in @('LicenseFile', 'LicenseFiles', 'CopyrightFile', 'CopyrightFiles')) {
            $property = $record.PSObject.Properties[$field]
            if ($null -ne $property) {
                foreach ($reference in @($property.Value)) {
                    if ($reference) { Copy-QtNotice (Join-Path $attribution.DirectoryName $reference) }
                }
            }
        }
        ++$attributionCount
    }
}
@"
Qt $qtVersion licensing and attribution source

Official unmodified source archive: $sourceUrl
SHA256: $sourceSha256
Qt source: https://code.qt.io/cgit/qt/qtbase.git/
LGPLv3: https://www.gnu.org/licenses/lgpl-3.0.html

LICENSES contains the upstream license texts. attributions preserves the
QtBase source's qt_attribution.json metadata and its referenced license and
copyright files, including notices for QtCore, QtGui and the image plugins.
The catalog intentionally includes some QtBase components not deployed by
this application. It does not assert that every cataloged component is used.
sbom contains the SDK's upstream inventories when supplied by that SDK.

Qt is dynamically linked and has not been modified by this project.
Compatible modified Qt DLLs can replace the supplied Qt DLLs. Application
source and build instructions are supplied under MIT; no additional
restriction is imposed on debugging modifications to LGPL-covered Qt.
"@ | Set-Content -LiteralPath (Join-Path $noticeRoot 'SOURCES.txt') -Encoding utf8

@'
Microsoft Visual C++ x64 runtime

The runtime DLLs accompanying this application are unmodified redistributable
files from the licensed build machine's Visual Studio 2022 Redist/MSVC tree.
They are separate Microsoft software, not licensed under the application MIT
license. Their redistribution remains subject to Microsoft's software terms.

Runtime license terms: https://aka.ms/VCRedistLicense
Redistribution guidance:
https://learn.microsoft.com/cpp/windows/redistributing-visual-cpp-files
Local deployment guidance:
https://learn.microsoft.com/cpp/windows/choosing-a-deployment-method
'@ | Set-Content -LiteralPath (Join-Path $OutputDir 'licenses/Microsoft-VCRuntime.txt') -Encoding utf8

Write-Host "Portable folder ready. Qt attribution entries copied: $attributionCount"
Write-Host $OutputDir

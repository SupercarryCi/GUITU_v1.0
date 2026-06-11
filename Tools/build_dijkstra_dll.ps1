$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"

if (-not (Test-Path -LiteralPath $vswhere)) {
    throw "Visual Studio vswhere.exe not found."
}

$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) {
    throw "MSVC C++ build tools not found."
}

$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path -LiteralPath $vcvars)) {
    throw "vcvars64.bat not found: $vcvars"
}

Push-Location $root
try {
    # Build a self-contained 64-bit DLL for the current 64-bit Python runtime.
    $command = 'call "{0}" >nul && cl /nologo /LD /O2 /MT /W4 /Fe:dijkstra.dll dijkstra_test_wrapper.c' -f $vcvars
    cmd.exe /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "MSVC build failed with exit code $LASTEXITCODE"
    }

    foreach ($name in @("dijkstra_test_wrapper.obj", "dijkstra.lib", "dijkstra.exp")) {
        $path = Join-Path $root $name
        if (Test-Path -LiteralPath $path) {
            Remove-Item -LiteralPath $path -Force
        }
    }
    Write-Output "Built: $(Join-Path $root 'dijkstra.dll')"
}
finally {
    Pop-Location
}

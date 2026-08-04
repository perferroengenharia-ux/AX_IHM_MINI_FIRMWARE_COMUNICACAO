param(
    [string]$Compiler = ""
)

$ErrorActionPreference = "Stop"
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$outputDirectory = Join-Path $projectRoot ".pio\test-host"
$testExecutable = Join-Path $outputDirectory "test_protocol.exe"
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null

$sources = @(
    (Join-Path $projectRoot "test\host\test_protocol.c"),
    (Join-Path $projectRoot "src\modbus\modbus_crc.c"),
    (Join-Path $projectRoot "src\modbus\modbus_master.c"),
    (Join-Path $projectRoot "src\diagnostics\comm_diagnostics.c"),
    (Join-Path $projectRoot "src\services\ihm_parameters.c"),
    (Join-Path $projectRoot "src\services\ihm_command_service.c"),
    (Join-Path $projectRoot "test\host\mocks\parameter_storage.c")
)
$includeDirectories = @(
    (Join-Path $projectRoot "test\host\mocks"),
    (Join-Path $projectRoot "src"),
    (Join-Path $projectRoot "src\comm"),
    (Join-Path $projectRoot "src\modbus"),
    (Join-Path $projectRoot "src\diagnostics"),
    (Join-Path $projectRoot "src\services")
)

function Enable-MsvcEnvironment {
    $vsDevCandidates = @(
        "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat",
        "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
    )
    $vsDev = $vsDevCandidates |
        Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
    if ([string]::IsNullOrWhiteSpace($vsDev)) {
        throw "TinyCC/GCC/MSVC não encontrado. Informe -Compiler."
    }

    $commandLine = 'call "' + $vsDev +
        '" -arch=x64 -host_arch=x64 >nul && set'
    $environmentLines = & cmd.exe /d /c $commandLine
    foreach ($line in $environmentLines) {
        $separator = $line.IndexOf("=")
        if ($separator -gt 0) {
            $name = $line.Substring(0, $separator)
            if ($name -ceq "Path") {
                continue
            }
            [Environment]::SetEnvironmentVariable(
                $name, $line.Substring($separator + 1), "Process")
        }
    }
    return (Get-Command "cl.exe" -ErrorAction Stop).Source
}

$selectedCompiler = $Compiler
if ([string]::IsNullOrWhiteSpace($selectedCompiler)) {
    $nativeCompiler = Get-Command "tcc.exe" -ErrorAction SilentlyContinue
    if ($null -eq $nativeCompiler) {
        $nativeCompiler = Get-Command "gcc.exe" -ErrorAction SilentlyContinue
    }
    if ($null -ne $nativeCompiler) {
        $selectedCompiler = $nativeCompiler.Source
    }
    else {
        $selectedCompiler = Enable-MsvcEnvironment
    }
}
elseif ((Split-Path -Leaf $selectedCompiler) -ieq "cl.exe") {
    $selectedCompiler = Enable-MsvcEnvironment
}
else {
    $selectedCompiler = (Resolve-Path -LiteralPath $selectedCompiler).Path
}

if ((Split-Path -Leaf $selectedCompiler) -ieq "cl.exe") {
    $includeArguments = $includeDirectories |
        ForEach-Object { "/I$_" }
    & $selectedCompiler `
        /nologo /std:c11 /W4 /WX /wd4127 /D_CRT_SECURE_NO_WARNINGS `
        @includeArguments @sources `
        "/Fo:$outputDirectory\" `
        "/Fe:$testExecutable"
}
else {
    $includeArguments = $includeDirectories |
        ForEach-Object { "-I$_" }
    & $selectedCompiler `
        -std=c11 -Wall -Werror `
        @includeArguments @sources `
        -o $testExecutable
}
if ($LASTEXITCODE -ne 0) {
    throw "A compilação dos testes falhou com código $LASTEXITCODE."
}

& $testExecutable
if ($LASTEXITCODE -ne 0) {
    throw "Os testes falharam com código $LASTEXITCODE."
}

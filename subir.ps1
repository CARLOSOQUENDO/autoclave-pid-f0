# subir.ps1 - Compila, sube y abre el monitor serie del ESP32
#   Uso:  .\subir.ps1 03_ui_encoder_oled
#         .\subir.ps1 03_ui_encoder_oled -SoloMonitor
#         .\subir.ps1 03_ui_encoder_oled -Puerto COM12
#
# Si no se indica -Puerto, detecta automaticamente el ESP32 (CH340, CP210x o
# adaptadores CDC). Util porque al cambiar de placa cambia el puerto.
#
# La subida va a 115200 baudios, no a los 921600 por defecto: con el montaje
# definitivo la velocidad alta corta la escritura a mitad ("The chip stopped
# responding"). Tarda 21 s en vez de 4, pero no falla (ver CONTEXTO.md sec.10.8).
#
# Si sale "Wrong boot mode detected", poner el ESP32 en modo descarga a mano:
# mantener BOOT, pulsar y soltar EN, soltar BOOT.
param(
    [Parameter(Mandatory=$true)][string]$Sketch,
    [string]$Puerto = "",
    [string]$Fqbn   = "esp32:esp32:esp32:UploadSpeed=115200",
    [switch]$SoloMonitor
)

$cli = "C:\Users\camao\arduino-cli\arduino-cli.exe"
$ruta = Join-Path $PSScriptRoot $Sketch

if (-not (Test-Path $ruta)) { Write-Error "No existe el sketch: $ruta"; exit 1 }

if (-not $Puerto) {
    $candidatos = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match 'COM\d+' -and
                       $_.Name -match 'CH340|CP210|CH910|Silicon Labs|USB Serial|UART' }

    if (-not $candidatos) { Write-Error "No se detecto ningun ESP32 conectado."; exit 1 }
    if ($candidatos.Count -gt 1) {
        Write-Host "Varios puertos candidatos:" -ForegroundColor Yellow
        $candidatos | ForEach-Object { Write-Host "  $($_.Name)" }
        Write-Error "Indica cual con -Puerto COMx"; exit 1
    }

    $Puerto = ([regex]::Match($candidatos.Name, 'COM\d+')).Value
    Write-Host "==> Detectado: $($candidatos.Name)" -ForegroundColor DarkGray
}

if (-not $SoloMonitor) {
    Write-Host "==> Compilando $Sketch ..." -ForegroundColor Cyan
    & $cli compile --fqbn $Fqbn $ruta
    if ($LASTEXITCODE -ne 0) { Write-Error "Fallo la compilacion"; exit 1 }

    Write-Host "==> Subiendo a $Puerto ..." -ForegroundColor Cyan
    & $cli upload -p $Puerto --fqbn $Fqbn $ruta
    if ($LASTEXITCODE -ne 0) { Write-Error "Fallo la subida"; exit 1 }
}

Write-Host "==> Monitor serie ($Puerto @ 115200). Ctrl+C para salir." -ForegroundColor Green
& $cli monitor -p $Puerto --config baudrate=115200

# Запуск консоли оператора одной командой на Windows.
#
#   .\run.ps1                    # то же, что .\run.ps1 sim — консоль + симуляция
#   .\run.ps1 robot 10.0.0.5     # консоль против РЕАЛЬНОГО робота (edge-бокс 10.0.0.5)
#   .\run.ps1 build              # собрать образы
#   .\run.ps1 logs               # смотреть журнал
#   .\run.ps1 stop               # остановить и удалить контейнеры
#   .\run.ps1 doctor             # диагностика без запуска
#
# Полный функциональный аналог run.sh. Совместим с Windows PowerShell 5.1: без операторов
# && и ||, без тернарного и null-coalescing — только if/else.
#
# Скрипт НИ ПРИ КАКИХ УСЛОВИЯХ не читает и не печатает содержимое vlm.env: он только
# проверяет, что файл существует и что это именно файл, а не каталог.

param(
    [Parameter(Position = 0)][string]$Command = 'sim',
    [Parameter(Position = 1)][string]$Arg = ''
)

$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$ComposeDir = Join-Path $RepoRoot 'docker'
$VlmEnv = Join-Path $RepoRoot '..\object_tracking\planner_orchestrator\vlm.env'

$script:Overlays = @()
$script:Profiles = @()

function Write-Fail([string]$Message) {
    Write-Host ''
    Write-Host "ОШИБКА: $Message" -ForegroundColor Red
    exit 1
}

function Test-DockerAvailable {
    $docker = Get-Command docker -ErrorAction SilentlyContinue
    if ($null -eq $docker) {
        Write-Fail "docker не найден. Установите Docker Desktop: https://docs.docker.com/desktop/install/windows-install/"
    }
    # ErrorActionPreference временно снижается: в Windows PowerShell 5.1 перенаправление
    # stderr нативного exe оборачивает каждую строку в ErrorRecord, и при 'Stop' проверка
    # наличия docker сама превращалась бы в исключение вместо честного диагноза.
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & docker compose version > $null 2> $null
    $code = $LASTEXITCODE
    $ErrorActionPreference = $prev
    if ($code -ne 0) {
        Write-Fail "docker есть, а плагина 'docker compose' нет. Нужен Compose v2: команда 'docker compose version' должна отрабатывать."
    }
}

function Initialize-Env {
    $envFile = Join-Path $ComposeDir '.env'
    if (-not (Test-Path $envFile)) {
        Copy-Item (Join-Path $ComposeDir '.env.example') $envFile
        Write-Host 'Создан docker\.env из шаблона .env.example — при желании поправьте порты и EDGE_HOST.'
    }

    # ПОЧЕМУ ЭТОТ ШАГ ОБЯЗАТЕЛЕН. Каталог planner_orchestrator монтируется в /config, и
    # консоль пишет туда токен через os.replace. Если vlm.env не существует к моменту up,
    # Docker создаёт на месте отсутствующего bind-mount КАТАЛОГ: после этого записать
    # токен физически невозможно, а env_file молча не подхватывается — оркестратор уходит
    # в mock без единой ошибки. Это самая частая поломка первого запуска.
    if (Test-Path $VlmEnv -PathType Container) {
        Write-Fail "На месте vlm.env оказался КАТАЛОГ: $VlmEnv`nЕго создал Docker при запуске без предварительно созданного файла.`nУдалите каталог и запустите снова:`n    Remove-Item -Recurse -Force '$VlmEnv'"
    }
    if (-not (Test-Path $VlmEnv)) {
        $template = "$VlmEnv.example"
        if (-not (Test-Path $template)) {
            Write-Fail "Нет ни vlm.env, ни шаблона рядом с ним: $template"
        }
        # Copy-Item копирует байты, поэтому перекодировка шаблона исключена: Set-Content
        # без -Encoding записал бы ANSI, и в контейнере строки прочитались бы неверно.
        Copy-Item $template $VlmEnv
        Write-Host 'Создан vlm.env из шаблона. Токен VLM введите в консоли (шаг «VLM») — в файл он попадёт с правами только для владельца.'
    }
}

# Оверлей GPU подмешивается ТОЛЬКО при рабочей NVIDIA: блок с driver: nvidia проверяется
# compose до старта и на машине без nvidia-container-runtime отказывает наотрез.
function Set-Overlays {
    $smi = Get-Command nvidia-smi -ErrorAction SilentlyContinue
    $ok = $false
    if ($null -ne $smi) {
        $prev = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        & nvidia-smi > $null 2> $null
        $code = $LASTEXITCODE
        $ErrorActionPreference = $prev
        if ($code -eq 0) { $ok = $true }
    }
    if ($ok) {
        $script:Overlays += @('-f', 'docker-compose.gpu.yml')
        Write-Host 'NVIDIA найдена — подключён docker-compose.gpu.yml.'
    }
    else {
        $env:TORCH_INDEX_URL = 'https://download.pytorch.org/whl/cpu'
        Write-Host 'NVIDIA не найдена: образы будут собраны под CPU (TORCH_INDEX_URL=.../whl/cpu).'
    }
}

# ЗАПРЕТ, ради которого функция и существует: консоль порождает свои detect_target_server
# и orchestrator_node подпроцессами (edge_layer), а профили edge/all поднимают ВТОРУЮ пару
# теми же именами. Два action-сервера на detect_target ломают миссию МОЛЧА — клиент уходит
# к тому, кто ответил первым.
function Test-ProfilesCompatible {
    $hasConsole = $false
    $hasEdge = $false
    foreach ($p in $script:Profiles) {
        if ($p -eq 'console') { $hasConsole = $true }
        if ($p -eq 'edge' -or $p -eq 'all') { $hasEdge = $true }
    }
    if ($hasConsole -and $hasEdge) {
        Write-Fail "Профиль 'console' нельзя запускать вместе с 'edge' или 'all'.`nКонсоль поднимает детектор и оркестратор внутри своего контейнера, а профили edge/all`nподнимают их ещё раз отдельными контейнерами. Два action-сервера на detect_target и два`nподписчика /vlm_mission ломают миссию без единой ошибки в логе.`nВыберите одно:`n    .\run.ps1 sim    — всё в контейнере консоли`n    .\run.ps1 edge   — только детектор + оркестратор, без консоли"
    }
}

# Второй контур той же защиты. Первый (Test-ProfilesCompatible) ловит конфликт внутри
# ОДНОГО вызова, но реальный сценарий другой: человек делает .\run.ps1 edge, потом
# .\run.ps1 sim — и получает те же два детектора, только разнесённые по времени. Поэтому
# перед подъёмом смотрим, что уже крутится. `ps` контейнеров не создаёт.
function Test-NoConflictingRunning([string]$Want) {
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    Push-Location $ComposeDir
    try {
        $running = & docker compose -f docker-compose.yml --profile console --profile all ps --services --filter status=running 2> $null
    }
    finally {
        Pop-Location
        $ErrorActionPreference = $prev
    }
    if ($null -eq $running) { return }

    if ($Want -eq 'console') {
        if (($running -contains 'detector') -or ($running -contains 'orchestrator')) {
            Write-Fail "Уже запущены detector/orchestrator из профиля edge.`nКонсоль поднимет свои такие же внутри контейнера — в графе окажутся два action-сервера`nна detect_target, и миссия сломается молча. Сначала остановите их: .\run.ps1 stop"
        }
    }
    else {
        if ($running -contains 'console') {
            Write-Fail "Уже запущена консоль (профиль console), а она сама держит detector и orchestrator.`nПрофили edge/all добавили бы вторую пару. Сначала остановите консоль: .\run.ps1 stop"
        }
    }
}

function Invoke-Compose([string[]]$ComposeArgs) {
    $all = @('compose', '-f', 'docker-compose.yml')
    foreach ($o in $script:Overlays) { $all += $o }
    foreach ($p in $script:Profiles) { $all += '--profile'; $all += $p }
    foreach ($a in $ComposeArgs) { $all += $a }
    Push-Location $ComposeDir
    # docker compose пишет прогресс сборки в stderr — под 'Stop' это оборвало бы обычный
    # успешный `up` посреди работы. Поток отдаётся консоли как есть, код возврата
    # проверяется явно.
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & docker @all
        if ($LASTEXITCODE -ne 0) {
            Write-Host "docker compose завершился с кодом $LASTEXITCODE." -ForegroundColor Yellow
        }
    }
    finally {
        $ErrorActionPreference = $prev
        Pop-Location
    }
}

# Читает значение переменной из docker\.env. Секретов в этом файле нет по определению:
# токен VLM живёт в отдельном vlm.env, который скрипт не открывает никогда.
function Get-EnvValue([string]$Key, [string]$Default) {
    $envFile = Join-Path $ComposeDir '.env'
    if (-not (Test-Path $envFile)) { return $Default }
    $line = Select-String -Path $envFile -Pattern "^$Key=" -ErrorAction SilentlyContinue | Select-Object -Last 1
    if ($null -eq $line) { return $Default }
    $value = $line.Line.Substring($line.Line.IndexOf('=') + 1)
    if ([string]::IsNullOrWhiteSpace($value)) { return $Default }
    return $value
}

function Show-Address {
    $port = Get-EnvValue 'CONSOLE_PORT' '8090'
    Write-Host ''
    Write-Host '==================================================================='
    Write-Host "  Консоль оператора: http://127.0.0.1:$port" -ForegroundColor Green
    Write-Host '==================================================================='
    Write-Host '  Журнал:      .\run.ps1 logs'
    Write-Host '  Остановить:  .\run.ps1 stop'
    Write-Host '  Диагностика: .\run.ps1 doctor'
    Write-Host ''
    Write-Host 'Первый запуск: пройдите мастер сверху вниз (режим -> мир -> VLM -> проверка).'
    Write-Host 'Пока стек не поднялся, консоль честно пишет «Ожидаю подключения робота…» —'
    Write-Host 'это нормально, она специально стартует раньше ROS.'
}

function Show-Doctor {
    Write-Host '--- Диагностика ---'
    # Диагностика обязана договорить до конца даже там, где что-то не установлено.
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'

    $docker = Get-Command docker -ErrorAction SilentlyContinue
    if ($null -eq $docker) {
        Write-Host 'docker: НЕ НАЙДЕН'
    }
    else {
        Write-Host ("docker: " + (& docker --version))
        Write-Host ("compose: " + (& docker compose version --short))
    }

    if (Test-Path (Join-Path $ComposeDir '.env')) {
        Write-Host 'docker\.env: есть'
    }
    else {
        Write-Host 'docker\.env: НЕТ (создастся при запуске)'
    }

    # Только факт существования. Get-Content по vlm.env не вызывается ни в одной ветке.
    if (Test-Path $VlmEnv -PathType Container) {
        Write-Host 'vlm.env: это КАТАЛОГ — его надо удалить, иначе токен не записать'
    }
    elseif (Test-Path $VlmEnv) {
        Write-Host 'vlm.env: есть'
    }
    else {
        Write-Host 'vlm.env: НЕТ (создастся из шаблона при запуске)'
    }

    $drive = Get-PSDrive -Name (Split-Path -Qualifier $RepoRoot).TrimEnd(':') -ErrorAction SilentlyContinue
    if ($null -ne $drive) {
        $freeGb = [math]::Round($drive.Free / 1GB, 1)
        Write-Host "Свободно на диске: $freeGb ГБ"
        if ($freeGb -lt 30) {
            Write-Host 'Меньше 30 ГБ: образ full со сборкой torch может не поместиться.' -ForegroundColor Yellow
        }
    }

    if ($null -ne $docker) {
        Write-Host 'Образы:'
        & docker images --format '  {{.Repository}}:{{.Tag}}  {{.Size}}' 'mrn/*'
    }

    foreach ($key in @('CONSOLE_PORT', 'DASHBOARD_PORT')) {
        if ($key -eq 'CONSOLE_PORT') { $p = Get-EnvValue $key '8090' } else { $p = Get-EnvValue $key '8088' }
        $busy = Get-NetTCPConnection -LocalPort ([int]$p) -State Listen -ErrorAction SilentlyContinue
        if ($null -ne $busy) { Write-Host "Порт $p : ЗАНЯТ" } else { Write-Host "Порт $p : свободен" }
    }

    $host_ = $env:EDGE_HOST
    if ([string]::IsNullOrWhiteSpace($host_)) { $host_ = Get-EnvValue 'EDGE_HOST' '' }
    if (-not [string]::IsNullOrWhiteSpace($host_)) {
        # Роутер zenoh — единственный порт, который нужен от edge-бокса.
        $probe = Test-NetConnection -ComputerName $host_ -Port 7447 -WarningAction SilentlyContinue
        if ($probe.TcpTestSucceeded) {
            Write-Host "Роутер zenoh на ${host_}:7447 доступен."
        }
        else {
            Write-Host "Роутер zenoh на ${host_}:7447 НЕДОСТУПЕН — на edge не выполнен install_transport.sh или закрыт порт."
        }
    }

    $ErrorActionPreference = $prev
}

function Show-Usage {
    Write-Host 'Запуск стека одной командой.'
    Write-Host ''
    Write-Host '  .\run.ps1 [sim]           консоль оператора + симуляция (по умолчанию)'
    Write-Host '  .\run.ps1 robot [АДРЕС]   консоль против реального робота; АДРЕС — edge-бокс'
    Write-Host '  .\run.ps1 edge            только детектор + VLM-оркестратор (без консоли)'
    Write-Host '  .\run.ps1 all             симуляция + отдельные детектор и оркестратор'
    Write-Host '  .\run.ps1 build           собрать образы'
    Write-Host '  .\run.ps1 logs            журнал контейнеров'
    Write-Host '  .\run.ps1 stop            остановить и удалить контейнеры (синоним: down)'
    Write-Host '  .\run.ps1 shell           bash внутри контейнера консоли'
    Write-Host '  .\run.ps1 doctor          диагностика без запуска'
    Write-Host '  .\run.ps1 native          подсказка по запуску без Docker'
    Write-Host ''
    Write-Host 'Профиль console нельзя совмещать с edge/all — см. шапку docker\docker-compose.yml.'
}

switch ($Command) {
    { $_ -eq 'sim' -or $_ -eq 'up' } {
        Test-DockerAvailable
        Initialize-Env
        Set-Overlays
        $script:Profiles = @('console')
        Test-ProfilesCompatible
        Test-NoConflictingRunning 'console'
        Invoke-Compose @('up', '-d')
        Show-Address
        break
    }

    'robot' {
        Test-DockerAvailable
        Initialize-Env
        Set-Overlays
        if (-not [string]::IsNullOrWhiteSpace($Arg)) { $env:EDGE_HOST = $Arg }
        if ([string]::IsNullOrWhiteSpace($env:EDGE_HOST)) { $env:EDGE_HOST = Get-EnvValue 'EDGE_HOST' '' }
        if ([string]::IsNullOrWhiteSpace($env:EDGE_HOST)) {
            Write-Fail "Укажите адрес edge-бокса: .\run.ps1 robot 192.168.1.10 (или задайте EDGE_HOST в docker\.env)"
        }
        $script:Overlays += @('-f', 'docker-compose.robot.yml')
        $script:Profiles = @('console')
        Test-ProfilesCompatible
        Test-NoConflictingRunning 'console'
        Write-Host ("Подключение к роботу через роутер zenoh на tcp/" + $env:EDGE_HOST + ":7447.")
        Write-Host 'На edge и Pi должен быть выполнен deploy/transport/install_transport.sh.'
        Invoke-Compose @('up', '-d')
        Show-Address
        Write-Host 'Робот и edge поднимаются НА СВОИХ хостах — консоль в этом режиме стек не запускает,'
        Write-Host 'она ждёт, пока ключевые топики станут свежими, и только тогда пишет «ПОДКЛЮЧЕНО».'
        break
    }

    'edge' {
        Test-DockerAvailable
        Initialize-Env
        Set-Overlays
        $script:Profiles = @('edge')
        Test-ProfilesCompatible
        Test-NoConflictingRunning 'edge'
        Invoke-Compose @('up')
        break
    }

    'all' {
        Test-DockerAvailable
        Initialize-Env
        Set-Overlays
        $script:Profiles = @('all')
        Test-ProfilesCompatible
        Test-NoConflictingRunning 'edge'
        Invoke-Compose @('up')
        break
    }

    'build' {
        Test-DockerAvailable
        Initialize-Env
        Set-Overlays
        # Профили собираются по очереди: одновременный up им запрещён, а сборка образов
        # конфликта не создаёт.
        $script:Profiles = @('console')
        Invoke-Compose @('build')
        $script:Profiles = @('all')
        Invoke-Compose @('build')
        break
    }

    'logs' {
        Test-DockerAvailable
        # Оба профиля перечислены намеренно: logs ничего не создаёт, а какой профиль был
        # поднят — скрипт не знает; без --profile compose не покажет журнал сервиса.
        $script:Profiles = @('console', 'all')
        Invoke-Compose @('logs', '-f')
        break
    }

    { $_ -eq 'stop' -or $_ -eq 'down' } {
        Test-DockerAvailable
        # down тоже ничего не создаёт, поэтому запрет на совместный up здесь не действует.
        $script:Profiles = @('console', 'all')
        Invoke-Compose @('down')
        Write-Host 'Контейнеры остановлены и удалены. Настройки консоли сохранены в томе console-state.'
        break
    }

    'shell' {
        Test-DockerAvailable
        $script:Profiles = @('console')
        Invoke-Compose @('exec', 'console', 'bash')
        break
    }

    'doctor' {
        Show-Doctor
        break
    }

    'native' {
        Write-Host 'Запуск без Docker требует установленного ROS 2 Jazzy — на Windows это WSL Ubuntu-24.04.'
        Write-Host 'В терминале WSL:'
        Write-Host '    source ~/ros2_ws/install/setup.bash'
        Write-Host '    ros2 run operator_console console_node'
        Write-Host 'Веб-интерфейс поднимется на http://127.0.0.1:8090.'
        break
    }

    { $_ -eq 'help' -or $_ -eq '-h' -or $_ -eq '--help' } {
        Show-Usage
        break
    }

    default {
        Write-Host "Неизвестная подкоманда: $Command"
        Show-Usage
        exit 2
    }
}

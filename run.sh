#!/usr/bin/env bash
# Запуск консоли оператора одной командой на Linux и macOS.
#
#   ./run.sh              # то же, что ./run.sh sim — консоль + симуляция
#   ./run.sh robot 10.0.0.5   # консоль против РЕАЛЬНОГО робота (edge-бокс 10.0.0.5)
#   ./run.sh build        # собрать образы
#   ./run.sh logs         # смотреть журнал
#   ./run.sh stop         # остановить и удалить контейнеры
#   ./run.sh doctor       # диагностика без запуска
#
# Скрипт НИ ПРИ КАКИХ УСЛОВИЯХ не читает и не печатает содержимое vlm.env, не передаёт
# токен аргументом командной строки и не кладёт его в своё окружение. Он только
# гарантирует, что файл существует и что это именно файл.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_DIR="$REPO_ROOT/docker"
VLM_ENV="$REPO_ROOT/../object_tracking/planner_orchestrator/vlm.env"

# Оверлеи и профили заполняет prepare()/выбор подкоманды.
OVERLAYS=()
PROFILES=()

# --- вспомогательный вывод -------------------------------------------------
say()  { printf '%s\n' "$*"; }
warn() { printf 'ВНИМАНИЕ: %s\n' "$*" >&2; }
die()  { printf 'ОШИБКА: %s\n' "$*" >&2; exit 1; }

# --- проверки окружения ----------------------------------------------------
check_docker() {
  command -v docker >/dev/null 2>&1 \
    || die "docker не найден. Установите Docker Desktop (macOS) или docker-ce (Linux): https://docs.docker.com/get-docker/"
  docker compose version >/dev/null 2>&1 \
    || die "docker есть, а плагина 'docker compose' нет. Нужен Compose v2: 'docker compose version' должен отрабатывать."
}

# Создаёт docker/.env и vlm.env из шаблонов. Оба шага обязаны выполниться ДО первого
# `docker compose up`.
prepare_files() {
  if [ ! -f "$COMPOSE_DIR/.env" ]; then
    cp "$COMPOSE_DIR/.env.example" "$COMPOSE_DIR/.env"
    say "Создан docker/.env из шаблона .env.example — при желании поправьте порты и EDGE_HOST."
  fi

  # ПОЧЕМУ ЭТОТ ШАГ ОБЯЗАТЕЛЕН. Каталог planner_orchestrator монтируется в /config, а
  # консоль пишет туда токен через os.replace. Если vlm.env не существует к моменту
  # `up`, а кто-то смонтирует его напрямую (или запустит старый compose), Docker создаст
  # на месте несуществующего bind-mount КАТАЛОГ: после этого записать токен физически
  # невозможно, а env_file молча не подхватится — оркестратор уйдёт в mock без единой
  # ошибки. Это самая частая поломка первого запуска, поэтому проверка стоит здесь.
  if [ -d "$VLM_ENV" ]; then
    die "На месте vlm.env оказался КАТАЛОГ: $VLM_ENV
Его создал Docker при запуске без предварительно созданного файла.
Удалите каталог и запустите снова:
    rmdir '$VLM_ENV'   (если он пуст)
    rm -rf '$VLM_ENV'  (если внутри что-то есть — там нет ваших данных)"
  fi
  if [ ! -f "$VLM_ENV" ]; then
    [ -f "$VLM_ENV.example" ] || die "Нет ни vlm.env, ни шаблона vlm.env.example рядом с ним: $VLM_ENV.example"
    cp "$VLM_ENV.example" "$VLM_ENV"
    chmod 600 "$VLM_ENV"
    say "Создан vlm.env из шаблона. Токен VLM введите в консоли (шаг «VLM») — в файл он попадёт с правами 0600."
  fi
}

# Оверлей GPU подмешивается ТОЛЬКО при наличии рабочей NVIDIA: блок с driver: nvidia
# проверяется compose до старта и на машине без nvidia-runtime отказывает наотрез.
detect_gpu() {
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    OVERLAYS+=("-f" "docker-compose.gpu.yml")
    say "NVIDIA найдена — подключён docker-compose.gpu.yml."
  else
    export TORCH_INDEX_URL="https://download.pytorch.org/whl/cpu"
    say "NVIDIA не найдена: образы будут собраны под CPU (TORCH_INDEX_URL=.../whl/cpu)."
    if [ "$(uname -s)" = "Darwin" ]; then
      warn "macOS: оверлей GPU не применяется (nvidia-runtime там не существует). YOLOE на CPU для 640x480 непригоден — доступны планировщики flat и mock."
    fi
  fi
}

# ЗАПРЕТ, ради которого функция и существует: консоль порождает свои
# detect_target_server и orchestrator_node подпроцессами (edge_layer), а профили
# edge/all поднимают ВТОРУЮ пару теми же именами. Два action-сервера на detect_target
# ломают миссию МОЛЧА — клиент уходит к тому, кто ответил первым.
assert_profiles_compatible() {
  local has_console=0 has_edge=0 p
  if [ ${#PROFILES[@]} -gt 0 ]; then
    for p in "${PROFILES[@]}"; do
      if [ "$p" = "console" ]; then has_console=1; fi
      if [ "$p" = "edge" ] || [ "$p" = "all" ]; then has_edge=1; fi
    done
  fi
  if [ "$has_console" = "1" ] && [ "$has_edge" = "1" ]; then
    die "Профиль 'console' нельзя запускать вместе с 'edge' или 'all'.
Консоль поднимает детектор и оркестратор внутри своего контейнера, а профили edge/all
поднимают их ещё раз отдельными контейнерами. Два action-сервера на detect_target и два
подписчика /vlm_mission ломают миссию без единой ошибки в логе.
Выберите одно:
    ./run.sh sim        — всё в контейнере консоли
    ./run.sh edge       — только детектор + оркестратор, без консоли"
  fi
}

# Второй контур той же защиты. Первый (assert_profiles_compatible) ловит конфликт внутри
# ОДНОГО вызова, но реальный сценарий другой: человек делает ./run.sh edge, потом ./run.sh
# sim — и получает те же два детектора, только разнесённые по времени. Поэтому перед
# подъёмом смотрим, что уже крутится. `ps` контейнеров не создаёт, профили в нём безопасны.
assert_no_conflicting_running() {
  local want="$1" running
  running="$( cd "$COMPOSE_DIR" && docker compose -f docker-compose.yml --profile console --profile all ps --services --filter status=running 2>/dev/null || true )"
  if [ "$want" = "console" ]; then
    if printf '%s\n' "$running" | grep -qE '^(detector|orchestrator)$'; then
      die "Уже запущены detector/orchestrator из профиля edge.
Консоль поднимет свои такие же внутри контейнера — в графе окажутся два action-сервера на
detect_target, и миссия сломается молча. Сначала остановите их: ./run.sh stop"
    fi
  else
    if printf '%s\n' "$running" | grep -qx 'console'; then
      die "Уже запущена консоль (профиль console), а она сама держит detector и orchestrator.
Профили edge/all добавили бы вторую пару. Сначала остановите консоль: ./run.sh stop"
    fi
  fi
}

# Массивы собираются через проверку длины, а не через "${arr[@]:-}": на macOS штатный
# bash 3.2, где под set -u пустой "${arr[@]:-}" разворачивается в ОДИН пустой аргумент
# и docker compose получает лишний '' вместо ничего.
compose() {
  local args=("-f" "docker-compose.yml") p
  if [ ${#OVERLAYS[@]} -gt 0 ]; then args+=("${OVERLAYS[@]}"); fi
  if [ ${#PROFILES[@]} -gt 0 ]; then
    for p in "${PROFILES[@]}"; do args+=("--profile" "$p"); done
  fi
  ( cd "$COMPOSE_DIR" && docker compose "${args[@]}" "$@" )
}

# Читает значение переменной из docker/.env, не раскрывая ничего лишнего.
# Используется только для печати адреса — секретов в .env нет по определению.
env_value() {
  local key="$1" default="$2" line
  line="$(grep -E "^${key}=" "$COMPOSE_DIR/.env" 2>/dev/null | tail -n 1 || true)"
  if [ -n "$line" ]; then printf '%s' "${line#*=}"; else printf '%s' "$default"; fi
}

print_address() {
  local port
  port="$(env_value CONSOLE_PORT 8090)"
  say ""
  say "==================================================================="
  say "  Консоль оператора: http://127.0.0.1:${port}"
  say "==================================================================="
  say "  Журнал:     ./run.sh logs"
  say "  Остановить: ./run.sh stop"
  say "  Диагностика: ./run.sh doctor"
  say ""
  say "Первый запуск: пройдите мастер сверху вниз (режим -> мир -> VLM -> проверка)."
  say "Пока стек не поднялся, консоль честно пишет «Ожидаю подключения робота…» —"
  say "это нормально, она специально стартует раньше ROS."
}

doctor() {
  say "--- Диагностика ---"
  if command -v docker >/dev/null 2>&1; then
    say "docker: $(docker --version 2>/dev/null || echo 'не отвечает')"
    say "compose: $(docker compose version --short 2>/dev/null || echo 'плагин не найден')"
  else
    say "docker: НЕ НАЙДЕН"
  fi

  if [ -f "$COMPOSE_DIR/.env" ]; then say "docker/.env: есть"; else say "docker/.env: НЕТ (создастся при запуске)"; fi

  # Печатаем ТОЛЬКО факт существования и права. Содержимое vlm.env не читается никогда.
  if [ -d "$VLM_ENV" ]; then
    say "vlm.env: это КАТАЛОГ — его надо удалить, иначе токен не записать"
  elif [ -f "$VLM_ENV" ]; then
    say "vlm.env: есть, права $(ls -l "$VLM_ENV" | cut -d' ' -f1)"
  else
    say "vlm.env: НЕТ (создастся из шаблона при запуске)"
  fi

  say "Свободно на диске: $(df -h "$REPO_ROOT" 2>/dev/null | tail -n 1 | awk '{print $4}')"

  if command -v docker >/dev/null 2>&1; then
    say "Образы:"
    docker images --format '  {{.Repository}}:{{.Tag}}  {{.Size}}' 'mrn/*' 2>/dev/null || true
  fi

  local cport dport
  cport="$(env_value CONSOLE_PORT 8090)"
  dport="$(env_value DASHBOARD_PORT 8088)"
  if command -v ss >/dev/null 2>&1; then
    say "Порт $cport: $(ss -ltn "sport = :$cport" 2>/dev/null | tail -n +2 | grep -q . && echo 'ЗАНЯТ' || echo 'свободен')"
    say "Порт $dport: $(ss -ltn "sport = :$dport" 2>/dev/null | tail -n +2 | grep -q . && echo 'ЗАНЯТ' || echo 'свободен')"
  elif command -v lsof >/dev/null 2>&1; then
    say "Порт $cport: $(lsof -nP -iTCP:"$cport" -sTCP:LISTEN >/dev/null 2>&1 && echo 'ЗАНЯТ' || echo 'свободен')"
    say "Порт $dport: $(lsof -nP -iTCP:"$dport" -sTCP:LISTEN >/dev/null 2>&1 && echo 'ЗАНЯТ' || echo 'свободен')"
  else
    say "Проверка портов пропущена: нет ни ss, ни lsof."
  fi

  local host
  host="${EDGE_HOST:-$(env_value EDGE_HOST '')}"
  if [ -n "$host" ]; then
    # Роутер zenoh — единственный порт, который нужен от edge-бокса.
    # timeout есть не везде (на macOS его нет из коробки), поэтому проверка условная.
    if command -v timeout >/dev/null 2>&1; then
      if timeout 3 bash -c "</dev/tcp/$host/7447" >/dev/null 2>&1; then
        say "Роутер zenoh на $host:7447 доступен."
      else
        say "Роутер zenoh на $host:7447 НЕДОСТУПЕН — на edge не выполнен install_transport.sh или закрыт порт."
      fi
    else
      say "Проверка порта $host:7447 пропущена: нет утилиты timeout (macOS). Проверьте вручную: nc -vz $host 7447"
    fi
  fi
}

usage() {
  cat <<'EOF'
Запуск стека одной командой.

  ./run.sh [sim]            консоль оператора + симуляция (по умолчанию)
  ./run.sh robot [АДРЕС]    консоль против реального робота; АДРЕС — edge-бокс
  ./run.sh edge             только детектор + VLM-оркестратор (без консоли)
  ./run.sh all              симуляция + отдельные детектор и оркестратор
  ./run.sh build            собрать образы
  ./run.sh logs             журнал контейнеров
  ./run.sh stop             остановить и удалить контейнеры (синоним: down)
  ./run.sh shell            bash внутри контейнера консоли
  ./run.sh doctor           диагностика без запуска
  ./run.sh native           подсказка по запуску без Docker

Профиль console нельзя совмещать с edge/all — см. шапку docker/docker-compose.yml.
EOF
}

CMD="${1:-sim}"
ARG="${2:-}"

case "$CMD" in
  sim|up)
    check_docker; prepare_files; detect_gpu
    PROFILES=("console"); assert_profiles_compatible; assert_no_conflicting_running console
    compose up -d
    print_address
    ;;

  robot)
    check_docker; prepare_files; detect_gpu
    if [ -n "$ARG" ]; then export EDGE_HOST="$ARG"; fi
    if [ -z "${EDGE_HOST:-}" ]; then EDGE_HOST="$(env_value EDGE_HOST '')"; export EDGE_HOST; fi
    [ -n "${EDGE_HOST:-}" ] || die "Укажите адрес edge-бокса: ./run.sh robot 192.168.1.10 (или задайте EDGE_HOST в docker/.env)"
    OVERLAYS+=("-f" "docker-compose.robot.yml")
    PROFILES=("console"); assert_profiles_compatible; assert_no_conflicting_running console
    say "Подключение к роботу через роутер zenoh на tcp/${EDGE_HOST}:7447."
    say "На edge и Pi должен быть выполнен deploy/transport/install_transport.sh."
    compose up -d
    print_address
    say "Робот и edge поднимаются НА СВОИХ хостах — консоль в этом режиме стек не запускает,"
    say "она ждёт, пока ключевые топики станут свежими, и только тогда пишет «ПОДКЛЮЧЕНО»."
    ;;

  edge)
    check_docker; prepare_files; detect_gpu
    PROFILES=("edge"); assert_profiles_compatible; assert_no_conflicting_running edge
    compose up
    ;;

  all)
    check_docker; prepare_files; detect_gpu
    PROFILES=("all"); assert_profiles_compatible; assert_no_conflicting_running edge
    compose up
    ;;

  build)
    check_docker; prepare_files; detect_gpu
    # Собираем обе группы по отдельности: одновременный up им запрещён, а сборка
    # образов конфликта не создаёт.
    PROFILES=("console"); compose build
    PROFILES=("all"); compose build
    ;;

  logs)
    check_docker
    # Оба профиля перечислены намеренно: logs ничего не создаёт, а какой профиль был
    # поднят — скрипт не знает; без --profile compose не покажет журнал сервиса.
    PROFILES=("console" "all")
    compose logs -f
    ;;

  stop|down)
    check_docker
    # `down` тоже ничего не создаёт, поэтому запрет на совместный up здесь не действует.
    PROFILES=("console" "all")
    compose down
    say "Контейнеры остановлены и удалены. Настройки консоли сохранены в томе console-state."
    ;;

  shell)
    check_docker
    PROFILES=("console")
    compose exec console bash
    ;;

  doctor)
    doctor
    ;;

  native)
    say "Запуск без Docker требует уже собранного workspace с ROS 2 Jazzy (обычно WSL Ubuntu-24.04):"
    say "    source ~/ros2_ws/install/setup.bash"
    say "    ros2 run operator_console console_node"
    say "Веб-интерфейс поднимется на http://127.0.0.1:8090."
    ;;

  -h|--help|help)
    usage
    ;;

  *)
    say "Неизвестная подкоманда: $CMD"
    usage
    exit 2
    ;;
esac

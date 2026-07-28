#!/usr/bin/env bash
# Быстрое подключение хоста к транспорту флота: zenoh + сокет-буферы + chrony.
# ROADMAP Phase 1.1 (транспорт) и 1.2 (время). Запускать на КАЖДОМ хосте один раз.
#
# ЗАЧЕМ ЭТОТ СКРИПТ СУЩЕСТВУЕТ.
# README.md:34-35 предлагает «edit zenoh_session_config.json5 (EDGE_HOST) OR set
# ZENOH_CONFIG_OVERRIDE» — то есть править литерал руками на каждом хосте.
# Проблема в том, что незаменённый литерал НЕ является ошибкой конфигурации:
# строка "tcp/EDGE_HOST:7447#..." (zenoh_session_config.json5:36) — синтаксически
# корректный endpoint, zenoh принимает конфиг и просто вечно не может разрешить
# имя EDGE_HOST. Снаружи это выглядит как «узлы висят на старте» — их держит
# ZENOH_ROUTER_CHECK_ATTEMPTS=10 (transport_env.sh:14), — и диагностируется хуже
# всего, потому что ни один файл не жалуется. Здесь подстановка делается один раз
# и результат проверяется grep-ом.
#
# Скрипт НЕ читает и не печатает никаких учётных данных и не трогает vlm.env.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIME_SYNC_DIR="$(cd "$HERE/../time_sync" && pwd)"

ROLE=""
EDGE_HOST=""
DRY_RUN=0
CAN_BITRATE=1000000
CAN_IFACE=can0
ZENOH_PORT=7447

# ---------------------------------------------------------------- вывод

say()  { printf '%s\n' "$*"; }
step() { printf '\n[%s] %s\n' "$1" "$2"; }
ok()   { printf '    OK: %s\n' "$*"; }
warn() { printf '    ВНИМАНИЕ: %s\n' "$*"; }
die()  { printf '\nОШИБКА: %s\n' "$*" >&2; exit 1; }

usage() {
    cat <<'EOF'
Установка транспорта флота (zenoh + chrony + сокет-буферы).

ИСПОЛЬЗОВАНИЕ:
    ./install_transport.sh <edge|pi> [АДРЕС_EDGE] [опции]

РОЛИ:
    edge   GPU-машина: единственный роутер zenoh на весь флот + мастер времени.
    pi     Робот (и любой другой хост с ROS-узлами): клиентская сессия + клиент chrony.

АДРЕС EDGE:
    Позиционным вторым аргументом или через --edge-host АДРЕС.
    На роли edge адрес можно не указывать — он будет определён автоматически
    по интерфейсу, через который хост выходит в сеть.
    На роли pi адрес ОБЯЗАТЕЛЕН: именно он подставляется в connect/endpoints
    сессионного конфига и в строку server файла chrony.

ОПЦИИ:
    --edge-host АДРЕС   IP или имя edge-хоста.
    --can-bitrate N     Битрейт шины CAN для подсказки (по умолчанию 1000000,
                        как в README.md:81).
    --can-iface ИМЯ     Имя CAN-интерфейса (по умолчанию can0).
    --dry-run           Только напечатать, что будет сделано. Ничего не менять.
    -h, --help          Эта справка.

ПРИМЕРЫ:
    ./install_transport.sh edge                    # адрес определится сам
    ./install_transport.sh edge 192.168.1.10
    ./install_transport.sh pi 192.168.1.10
    ./install_transport.sh pi 192.168.1.10 --dry-run
EOF
}

# --dry-run обязан покрывать и конвейеры (sed | tee), поэтому шаги выполняются
# через eval строкой, а не массивом. Единственное, что подставляется в эти
# строки извне, — EDGE_HOST/CAN_*, и они прогнаны через регулярки ниже.
run() {
    if [ "$DRY_RUN" = 1 ]; then
        printf '    [сухой прогон] %s\n' "$1"
    else
        eval "$1"
    fi
}

# ---------------------------------------------------------------- аргументы

if [ "$#" -eq 0 ]; then
    usage
    exit 2
fi

while [ "$#" -gt 0 ]; do
    case "$1" in
        edge|pi)
            [ -z "$ROLE" ] || die "роль указана дважды: '$ROLE' и '$1'"
            ROLE="$1"
            ;;
        --edge-host)
            shift || true
            [ "$#" -gt 0 ] || die "--edge-host требует значение"
            EDGE_HOST="$1"
            ;;
        --edge-host=*) EDGE_HOST="${1#*=}" ;;
        --can-bitrate)
            shift || true
            [ "$#" -gt 0 ] || die "--can-bitrate требует значение"
            CAN_BITRATE="$1"
            ;;
        --can-bitrate=*) CAN_BITRATE="${1#*=}" ;;
        --can-iface)
            shift || true
            [ "$#" -gt 0 ] || die "--can-iface требует значение"
            CAN_IFACE="$1"
            ;;
        --can-iface=*) CAN_IFACE="${1#*=}" ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        -*) die "неизвестная опция: $1 (см. --help)" ;;
        *)
            if [ -z "$ROLE" ]; then
                die "первым аргументом должна идти роль edge или pi, получено: '$1'"
            elif [ -z "$EDGE_HOST" ]; then
                EDGE_HOST="$1"
            else
                die "лишний аргумент: '$1'"
            fi
            ;;
    esac
    shift
done

[ -n "$ROLE" ] || { usage; die "не указана роль (edge или pi)"; }

case "$CAN_BITRATE" in
    ''|*[!0-9]*) die "битрейт CAN должен быть числом, получено: '$CAN_BITRATE'" ;;
esac
case "$CAN_IFACE" in
    ''|*[!A-Za-z0-9._-]*) die "недопустимое имя CAN-интерфейса: '$CAN_IFACE'" ;;
esac

# Автоопределение адреса edge. Берём source-адрес маршрута наружу, а НЕ первый
# адрес hostname -I: на edge-ноутбуке почти всегда есть docker0/br-*, и
# hostname -I вернёт 172.17.0.1, до которого Pi никогда не достучится.
autodetect_edge_host() {
    local addr=''
    if command -v ip >/dev/null 2>&1; then
        addr="$(ip -4 route get 1.1.1.1 2>/dev/null \
                 | awk '{for (i = 1; i < NF; i++) if ($i == "src") { print $(i + 1); exit }}' || true)"
    fi
    if [ -z "$addr" ] && command -v hostname >/dev/null 2>&1; then
        addr="$(hostname -I 2>/dev/null | awk '{print $1}' || true)"
    fi
    printf '%s' "$addr"
}

if [ -z "$EDGE_HOST" ]; then
    if [ "$ROLE" = "edge" ]; then
        EDGE_HOST="$(autodetect_edge_host)"
        [ -n "$EDGE_HOST" ] || die "не удалось определить адрес edge автоматически; укажите --edge-host АДРЕС"
        say "Адрес edge определён автоматически: $EDGE_HOST"
        say "Если Pi живёт в другой подсети — прервите и передайте адрес явно."
    else
        die "на роли pi адрес edge обязателен: ./install_transport.sh pi <АДРЕС_EDGE>"
    fi
fi

# Проверяем форму адреса, а не достижимость: скрипт часто запускают до того, как
# edge поднят, и падать здесь было бы неправильно. Проверка связи — шаг 6.
# Литерал-заглушку ловим ПЕРЕД регуляркой: подчёркивание её и так не пройдёт, но
# сообщение «не похож на IP» ничего не объяснит человеку, который скопировал
# строку из шапки конфига.
if [ "$EDGE_HOST" = "EDGE_HOST" ]; then
    die "'EDGE_HOST' — это литерал-заглушка из zenoh_session_config.json5:36, а не адрес. Подставьте настоящий IP edge."
fi
if ! printf '%s' "$EDGE_HOST" | grep -Eq '^([0-9]{1,3}\.){3}[0-9]{1,3}$|^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$'; then
    die "'$EDGE_HOST' не похож на IP-адрес или имя хоста"
fi
if [ "$ROLE" = "pi" ]; then
    case "$EDGE_HOST" in
        127.0.0.1|localhost)
            die "на роли pi адрес edge не может быть localhost: сессия робота должна ходить на ДРУГОЙ хост"
            ;;
    esac
fi

SUDO="sudo"
if [ "$(id -u)" = "0" ]; then
    SUDO=""
fi

say "==============================================================="
say " Транспорт флота: роль=$ROLE, edge=$EDGE_HOST"
[ "$DRY_RUN" = 1 ] && say " РЕЖИМ СУХОГО ПРОГОНА: ни один файл не будет изменён"
say " Исходники: $HERE"
say "==============================================================="

# ------------------------------------------------- [1/6] сокет-буферы

step 1/6 "Сокетные буферы ядра (99-ros2-socket-buffers.conf)"
run "$SUDO install -m 0644 '$HERE/99-ros2-socket-buffers.conf' /etc/sysctl.d/99-ros2-socket-buffers.conf"
run "$SUDO sysctl --system >/dev/null"
if [ "$DRY_RUN" = 0 ]; then
    RMEM="$(sysctl -n net.core.rmem_max 2>/dev/null || echo '')"
    if [ "$RMEM" = "12582912" ]; then
        ok "net.core.rmem_max = $RMEM"
    else
        # Хинты #so_rcvbuf в endpoint-ах zenoh могут только УМЕНЬШИТЬСЯ до
        # net.core.rmem_max (см. zenoh_router_config.json5:32-35), поэтому
        # низкий rmem_max тихо превращает 12 МБ в дефолтные ~200 КБ и даёт
        # head-of-line stall на всплесках изображений.
        warn "net.core.rmem_max = '${RMEM:-неизвестно}', ожидалось 12582912. Буферы 12 МБ в endpoint-ах zenoh будут срезаны ядром."
    fi
fi

# ------------------------------------------------- [2/6] каталог конфигов

step 2/6 "Каталог /etc/zenoh"
run "$SUDO install -d -m 0755 /etc/zenoh"

# ------------------------------------------------- [3/6] сессионный конфиг

step 3/6 "Сессионный конфиг zenoh с подставленным адресом edge"
# Меняем только endpoint, а не все вхождения слова EDGE_HOST: в шапке файла оно
# упомянуто как объяснение (zenoh_session_config.json5:9), и затирать текст
# документации подставленным IP смысла нет.
run "sed 's|tcp/EDGE_HOST:|tcp/${EDGE_HOST}:|g' '$HERE/zenoh_session_config.json5' | $SUDO tee /etc/zenoh/zenoh_session_config.json5 >/dev/null"
if [ "$DRY_RUN" = 0 ]; then
    if grep -q "tcp/EDGE_HOST:" /etc/zenoh/zenoh_session_config.json5; then
        die "литерал tcp/EDGE_HOST: остался в /etc/zenoh/zenoh_session_config.json5 — подстановка не сработала"
    fi
    ok "connect/endpoints -> tcp/${EDGE_HOST}:${ZENOH_PORT}"
fi

# ------------------------------------------------- [4/6] роутер (только edge)

step 4/6 "Роутер zenoh (только на роли edge)"
if [ "$ROLE" = "edge" ]; then
    run "$SUDO install -m 0644 '$HERE/zenoh_router_config.json5' /etc/zenoh/zenoh_router_config.json5"
    run "$SUDO install -m 0644 '$HERE/rmw-zenoh-router.service' /etc/systemd/system/rmw-zenoh-router.service"
    run "$SUDO systemctl daemon-reload"
    run "$SUDO systemctl enable --now rmw-zenoh-router"
    if [ "$DRY_RUN" = 0 ]; then
        sleep 2
        STATE="$(systemctl is-active rmw-zenoh-router 2>/dev/null || true)"
        if [ "$STATE" = "active" ]; then
            ok "служба rmw-zenoh-router активна"
            if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":${ZENOH_PORT}"; then
                ok "порт ${ZENOH_PORT} слушается"
            else
                warn "порт ${ZENOH_PORT} не слушается. Юнит стартует ros2 из /opt/ros/jazzy (rmw-zenoh-router.service:21) — проверьте, что дистрибутив там и есть."
            fi
        else
            warn "служба rmw-zenoh-router в состоянии '${STATE:-неизвестно}'. Смотрите: journalctl -u rmw-zenoh-router -f"
        fi
    fi
else
    say "    пропущено: роутер во флоте ровно один и живёт на edge (README.md:13-14)"
fi

# ------------------------------------------------- [5/6] chrony

step 5/6 "Синхронизация времени (chrony)"
# Почему это часть транспорта, а не отдельная процедура: расхождение часов
# ломает те же самые потоки, что и плохой транспорт, но НЕЗАМЕТНО — окна
# сопоставления 0.2 s (TF) / 0.35 s (depth<->color) / 1.5 s (возраст пикселей)
# перечислены в deploy/time_sync/README.md:8-14.
if ! command -v chronyd >/dev/null 2>&1 && ! command -v chronyc >/dev/null 2>&1; then
    warn "chrony не установлен. Установите и запустите скрипт повторно: sudo apt install -y chrony"
else
    if [ "$ROLE" = "edge" ]; then
        run "$SUDO install -m 0644 '$TIME_SYNC_DIR/chrony-edge.conf' /etc/chrony/chrony.conf"
        say "    edge стал мастером времени (local stratum 10, chrony-edge.conf:18)"
        warn "в chrony-edge.conf:14 стоит широкое 'allow 192.168.0.0/16' — сузьте до вашей подсети"
    else
        run "sed 's|^server EDGE_HOST |server ${EDGE_HOST} |' '$TIME_SYNC_DIR/chrony-pi.conf' | $SUDO tee /etc/chrony/chrony.conf >/dev/null"
        if [ "$DRY_RUN" = 0 ] && grep -q '^server EDGE_HOST' /etc/chrony/chrony.conf; then
            die "литерал EDGE_HOST остался в /etc/chrony/chrony.conf — подстановка не сработала"
        fi
        [ "$DRY_RUN" = 0 ] && ok "источник времени -> ${EDGE_HOST}"
    fi

    # Юнит зовётся chrony в Debian/Ubuntu и chronyd в RHEL-подобных.
    CHRONY_UNIT=chrony
    if [ "$DRY_RUN" = 0 ] && ! systemctl list-unit-files 2>/dev/null | grep -q '^chrony\.service'; then
        CHRONY_UNIT=chronyd
    fi
    run "$SUDO systemctl restart ${CHRONY_UNIT}"

    if [ "$ROLE" = "pi" ]; then
        if [ "$DRY_RUN" = 1 ]; then
            printf '    [сухой прогон] bash %s/check_offset.sh\n' "$TIME_SYNC_DIR"
        else
            say "    --- вывод check_offset.sh (порог 0.02 с) ---"
            # Не гасим ненулевой код: сразу после restart chrony ещё не выбрал
            # источник, и FAIL здесь ожидаем. Скрипт печатает вердикт как есть,
            # а в чек-листе ниже сказано перезапустить проверку через минуту.
            bash "$TIME_SYNC_DIR/check_offset.sh" || warn "смещение ещё не сошлось — это нормально сразу после рестарта, повторите проверку через минуту"
            say "    --- конец вывода check_offset.sh ---"
        fi
    fi
fi

# ------------------------------------------------- [6/6] проверки на pi

step 6/6 "Проверки со стороны робота (только на роли pi)"
if [ "$ROLE" = "pi" ]; then
    if [ "$DRY_RUN" = 1 ]; then
        printf '    [сухой прогон] проверка %s и tcp-доступности %s:%s\n' "$CAN_IFACE" "$EDGE_HOST" "$ZENOH_PORT"
    else
        # --- CAN ---
        if ! command -v ip >/dev/null 2>&1; then
            warn "утилита ip недоступна, состояние $CAN_IFACE не проверено"
        elif ! ip -details link show "$CAN_IFACE" >/dev/null 2>&1; then
            warn "интерфейс $CAN_IFACE не существует. Проверьте overlay SPI-CAN в /boot/firmware/config.txt и подайте питание на плату."
        else
            CAN_STATE="$(ip -details link show "$CAN_IFACE" 2>/dev/null | tr '\n' ' ')"
            if printf '%s' "$CAN_STATE" | grep -q 'state UP'; then
                ok "$CAN_IFACE поднят"
                printf '%s' "$CAN_STATE" | grep -o 'bitrate [0-9]*' | head -1 | sed 's/^/    /' || true
            else
                # Сознательно НЕ выполняем ip link set сами: это изменение
                # состояния железа, а колёса на стенде должны быть подняты до
                # подачи питания на приводы (HIL_BRINGUP_CHECKLIST.md:12).
                warn "$CAN_IFACE существует, но опущен. Приводы EPOS4 не отзовутся."
                say "    Поднимите вручную (скрипт этого не делает — это изменение состояния железа):"
                say ""
                say "        sudo ip link set ${CAN_IFACE} up type can bitrate ${CAN_BITRATE}"
                say ""
                say "    Битрейт ${CAN_BITRATE} — из README.md:81; он обязан совпадать с настройкой самих EPOS4."
            fi
        fi

        # --- доступность роутера ---
        # /dev/tcp — встроенный в bash способ проверить TCP без nc/telnet,
        # которых на свежей Pi OS может не быть.
        if timeout 3 bash -c "exec 3<>/dev/tcp/${EDGE_HOST}/${ZENOH_PORT}" 2>/dev/null; then
            ok "Роутер zenoh на edge доступен (${EDGE_HOST}:${ZENOH_PORT})"
        else
            warn "Роутер недоступен: проверьте, что на edge выполнен ./install_transport.sh edge и открыт порт ${ZENOH_PORT}."
            say "    На edge: systemctl is-active rmw-zenoh-router && ss -ltn | grep ${ZENOH_PORT}"
            say "    Брандмауэр edge: sudo ufw allow ${ZENOH_PORT}/tcp"
        fi
    fi
else
    say "    пропущено: проверки CAN и доступности роутера относятся к роли pi"
fi

# ---------------------------------------------------------------- итог

cat <<EOF

===============================================================
 ГОТОВО (роль: $ROLE, edge: $EDGE_HOST)
===============================================================

ЧЕК-ЛИСТ ПРОВЕРКИ — выполните прямо сейчас:

EOF

if [ "$ROLE" = "edge" ]; then
    cat <<EOF
  1. Роутер жив:            systemctl is-active rmw-zenoh-router
  2. Порт слушается:        ss -ltn | grep ${ZENOH_PORT}
  3. Логи роутера:          journalctl -u rmw-zenoh-router -f
  4. Буферы:                sysctl net.core.rmem_max          -> 12582912
  5. Время раздаётся:       chronyc clients
  6. Один хост, без Pi:     bash ${HERE}/smoke_test_zenoh.sh  -> PASS
EOF
else
    cat <<EOF
  1. Буферы:                sysctl net.core.rmem_max          -> 12582912
  2. Конфиг подставлен:     grep -n 'tcp/' /etc/zenoh/zenoh_session_config.json5
  3. Роутер виден:          timeout 3 bash -c '</dev/tcp/${EDGE_HOST}/${ZENOH_PORT}' && echo OK
  4. Время сошлось:         bash ${TIME_SYNC_DIR}/check_offset.sh   -> PASS (offset и RMS <= 0.02 с)
  5. Шина CAN:              ip -details link show ${CAN_IFACE}      -> state UP, bitrate ${CAN_BITRATE}
  6. Сквозной pub/sub:      на edge 'ros2 topic echo /ping std_msgs/msg/String',
                            на Pi   'ros2 topic pub -r 1 /ping std_msgs/msg/String "{data: hi}"'
EOF
fi

cat <<EOF

ПРЕАМБУЛА ТЕРМИНАЛА — вставляйте в КАЖДЫЙ новый терминал на этом хосте
(она же повторяется девять раз в docs/RUNBOOK.md; без unset шести переменных
остатки настроек Fast DDS перебивают zenoh и узлы не находят друг друга):

    cd ~/ros2_ws
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
    export ROS_DISABLE_ROS2CLI_DAEMON=1
    source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

EOF

if [ "$ROLE" = "edge" ]; then
    cat <<EOF
СЛЕДУЮЩИЙ ШАГ (на роботе):

    ./install_transport.sh pi ${EDGE_HOST}

Затем поднимайте слои: сначала Pi, потом edge, потом консоль оператора.
Порядок и команды — docs/OPERATOR_CONSOLE.md, раздел «Быстрый старт с реальным роботом».
EOF
else
    cat <<EOF
СЛЕДУЮЩИЕ ШАГИ:

  1. На Pi, в терминале с преамбулой выше:
         ros2 launch ar_project hardware_bringup.launch.py
         ros2 launch ar_project realsense_rgbd_pi.launch.py \\
             rgb_camera.color_profile:=640x480x6 depth_module.depth_profile:=424x240x6

  2. На edge, в терминале с преамбулой:
         ros2 launch ar_project edge_bringup.launch.py

  3. ТОЛЬКО ПОСЛЕ шага 2 — тюнинг сжатия камеры на Pi:
         bash ~/ros2_ws/src/ar_project/deploy/tune_camera_compression.sh
     Раньше нельзя: параметры compressedDepth.* объявляются лениво, когда на
     топик подпишется потребитель, то есть edge-реле (docs/RUNBOOK.md:152-153).

  4. На ПК оператора — консоль:
         EDGE_HOST=${EDGE_HOST} ./run.sh robot          (Windows: .\\run.ps1 robot ${EDGE_HOST})
     Откройте http://127.0.0.1:8090

ВАЖНО: межхостовой канал Pi<->edge в репозитории пока проверен только на одном
хосте (deploy/transport/README.md:63-65). Джиттер на реальном Wi-Fi измеряется
здесь впервые — сверяйте с бюджетами 0.2 / 0.35 / 1.5 с.
EOF
fi

say ""

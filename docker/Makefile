# Обёртка над docker compose. Запускать из ar_project/docker/.
#   make build       # собрать образы sim + edge
#   make build-full  # собрать образ консоли (full): всё сразу, ~6 ГБ
#   make console     # поднять консоль оператора -> http://127.0.0.1:8090
#   make robot       # то же, но против РЕАЛЬНОГО робота (нужен EDGE_HOST)
#   make sim         # поднять FLAT в симуляции
#   make edge        # поднять детектор + оркестратор (GPU)
#   make down        # остановить и удалить контейнеры
#
# ВНИМАНИЕ: цели console/robot и цели edge/all НЕЛЬЗЯ поднимать одновременно —
# в графе окажутся два detect_target_server и два оркестратора (подробности в шапке
# docker-compose.yml). Кроссплатформенная обёртка с этой проверкой — ../run.sh и ../run.ps1.
COMPOSE = docker compose -f docker-compose.yml
COMPOSE_GPU = docker compose -f docker-compose.yml -f docker-compose.gpu.yml
COMPOSE_ROBOT = docker compose -f docker-compose.yml -f docker-compose.robot.yml

.PHONY: build build-full console robot sim edge all down logs ps

build:
	$(COMPOSE) --profile all build

# Отдельная цель: образ full тянет torch + веса зависимостей и собирается дольше всех,
# незачем ждать его при обычном `make build`.
build-full:
	$(COMPOSE) --profile console build

console:
	$(COMPOSE) --profile console up

# EDGE_HOST обязателен: из него собирается connect/endpoints сессии zenoh.
robot:
	@test -n "$(EDGE_HOST)" || { echo "Укажите адрес edge-бокса: make robot EDGE_HOST=192.168.1.10"; exit 2; }
	EDGE_HOST=$(EDGE_HOST) $(COMPOSE_ROBOT) --profile console up

sim:
	$(COMPOSE) --profile sim up

edge:
	$(COMPOSE) --profile edge up

all:
	$(COMPOSE) --profile all up

# Профили console и all перечислены вместе намеренно: `down` ничего не создаёт, он
# только удаляет уже существующие контейнеры, поэтому запрет на их совместный `up`
# здесь не действует.
down:
	$(COMPOSE) --profile all --profile console down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) --profile all --profile console ps

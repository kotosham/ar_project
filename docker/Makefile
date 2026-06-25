# Обёртка над docker compose. Запускать из ar_project/docker/.
#   make build   # собрать все образы
#   make sim     # поднять FLAT в симуляции
#   make edge    # поднять детектор + оркестратор (GPU)
#   make down    # остановить и удалить контейнеры
COMPOSE = docker compose -f docker-compose.yml

.PHONY: build sim edge all down logs ps

build:
	$(COMPOSE) --profile all build

sim:
	$(COMPOSE) --profile sim up

edge:
	$(COMPOSE) --profile edge up

all:
	$(COMPOSE) --profile all up

down:
	$(COMPOSE) --profile all down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps

# Thin wrapper around docker compose. Run from ar_project/docker/.
#   make build   # build all images
#   make sim     # start FLAT simulation
#   make edge    # start detector + orchestrator (GPU)
#   make down    # stop and remove containers
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

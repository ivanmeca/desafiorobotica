# HELP
# This will output the help for each task
# thanks to https://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
.PHONY: help

help: ## This help.
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

.DEFAULT_GOAL := help

default: help

init: ## Launch Web Bots application
	xhost +local:root > /dev/null 2>&1
	docker-compose run -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbots webots

stop: ## Safely stop the environment
	xhost -local:root > /dev/null 2>&1.
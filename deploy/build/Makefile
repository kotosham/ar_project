# Convenience wrapper around deploy.sh. Run from deploy/build/.
#   make setup   # one-time: create deploy.env from the example
#   make edge    # build detector+orchestrator on this (edge) box
#   make pi      # rsync source to the Pi + remote build (executive+HW)
#   make all     # edge then pi
#   make shell   # ssh into the Pi with the workspace sourced
#   make doctor  # check ssh + ROS on both ends
.PHONY: setup edge pi all shell doctor help
help: ; @sed -n '2,7p' $(MAKEFILE_LIST)
setup: ; @test -f deploy.env || (cp deploy.env.example deploy.env && echo "created deploy.env — edit PI_HOST/PI_USER")
edge:  ; @./deploy.sh edge
pi:    ; @./deploy.sh pi
all:   ; @./deploy.sh all
shell: ; @./deploy.sh shell
doctor:; @./deploy.sh doctor

PACKAGE_DIR := robot_arm_library
PACKAGE_ZIP := $(PACKAGE_DIR).zip
BLENDER ?= blender
PYTHON ?= $(if $(wildcard .venv/bin/python),$(abspath .venv/bin/python),python3)
SENSOR_LIBRARY ?= ../mujoco_sensor_library

.PHONY: all zip install clean

all: zip

zip:
	rm -f "$(PACKAGE_ZIP)"
	zip -r "$(PACKAGE_ZIP)" "$(PACKAGE_DIR)" -x '*/__pycache__/*' '*.pyc'

install: zip
	"$(BLENDER)" --background --python-exit-code 1 --python "$(abspath install_blender_addon.py)" -- --python "$(PYTHON)" --sensor-library "$(abspath $(SENSOR_LIBRARY))" --addon-zip "$(abspath $(PACKAGE_ZIP))"

clean:
	rm -f "$(PACKAGE_ZIP)"
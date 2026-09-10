PACKAGE_DIR := robot_arm_library
PACKAGE_ZIP := $(PACKAGE_DIR).zip

.PHONY: all zip clean

all: zip

zip:
	rm -f "$(PACKAGE_ZIP)"
	zip -r "$(PACKAGE_ZIP)" "$(PACKAGE_DIR)" -x '*/__pycache__/*' '*.pyc'

clean:
	rm -f "$(PACKAGE_ZIP)"
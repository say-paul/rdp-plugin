PACKAGE_DIR := robot_arm_library
PACKAGE_ZIP := $(PACKAGE_DIR).zip
BLENDER ?= blender

.PHONY: all zip install clean

all: zip

zip:
	rm -f "$(PACKAGE_ZIP)"
	zip -r "$(PACKAGE_ZIP)" "$(PACKAGE_DIR)" -x '*/__pycache__/*' '*.pyc'

install: zip
	$(BLENDER) --background --python-expr "import bpy, os; addon = 'robot_arm_library'; existing = bpy.context.preferences.addons.get(addon); existing and bpy.ops.preferences.addon_disable(module=addon); bpy.ops.preferences.addon_install(overwrite=True, filepath=os.path.abspath('$(PACKAGE_ZIP)')); bpy.ops.preferences.addon_enable(module=addon); bpy.ops.wm.save_userpref(); print('Installed and enabled', addon)"

clean:
	rm -f "$(PACKAGE_ZIP)"
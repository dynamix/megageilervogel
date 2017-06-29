# 1.8.3
ARDUINO_VERSION=10803
ARDUINO_HOME=/Applications/Arduino.app/Contents/Java
BOARD=teensy:avr:teensy36
MODEL=TEENSY36

# 

# needs greadlink on OSX
# work 

PROJECT_NAME=$(shell basename $(CURDIR)).ino
BUILD_PATH_BASE=build/
BUILDER_HW=-fqbn=$(BOARD) -hardware "$(ARDUINO_HOME)/hardware" 
BUILDER_TOOLS=-tools "$(ARDUINO_HOME)/tools-builder" -tools "$(ARDUINO_HOME)/hardware" 
BUILDER_LIBS=-built-in-libraries "$(ARDUINO_HOME)/libraries"
BUILDER_EXTRA+= -prefs=build.warn_data_percentage=75 
BUILDER_OPTS=$(BUILDER_EXTRA) -ide-version=$(ARDUINO_VERSION) $(BUILDER_HW) $(BUILDER_TOOLS) $(BUILDER_LIBS)
BUILD_PATH ?=$(CURDIR)/$(BUILD_PATH_BASE)


all: dirs build

dirs:
	mkdir -p $(BUILD_PATH)

$(BUILD_PATH)/$(PROJECT_NAME)._hex: $(EXTRA_BUILD_RULES)
	$(ARDUINO_HOME)/arduino-builder -compile -warnings=all -verbose=true -logger=human $(BUILDER_OPTS) -build-path "$(BUILD_PATH)" "$(PROJECT_NAME)"

build: $(BUILD_PATH)/$(PROJECT_NAME)._hex

clean:
	rm -rf $(BUILD_PATH)

deploy:
	scp $(BUILD_PATH)$(PROJECT_NAME).hex pi@raspberrypi.local:/home/pi/code.hex
	ssh -t pi@raspberrypi.local "sleep 3 && python reset.py" &
	ssh -t pi@raspberrypi.local "./teensy_loader_cli/teensy_loader_cli --mcu=$(MODEL) -w -v code.hex"

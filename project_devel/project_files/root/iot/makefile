# Configuration

# ROOT_NODE_DIR = ./esp32/esp32_root/esp32_root.ino
CAM_NODE_DIR = ./esp32/esp32_cam/esp32_cam.ino
ROBOT_NODE_DIR = ./esp32/esp32_robot/esp32_robot.ino
# NODE1_NODE_DIR = ./iot_testNode/esp8266/node1/node1.ino
# NODE2_NODE_DIR = ./iot_testNode/esp8266/node2/node2.ino
ROBOT1_NODE_DIR = ./esp8266/robot/robot.ino

ESP822_NODE_DIR= ./iot_testNode.ino

PORT1 = /dev/ttyUSB1
PORT0 = /dev/ttyUSB0
PORT2 = /dev/ttyUSB2
PORT3 = /dev/ttyUSB3

FQBN82 = esp8266:esp8266:nodemcuv2
FQBN32-root = esp32:esp32:d1_uno32
FQBN32-cam = esp32:esp32:esp32cam
FQBN32-robot = esp32:esp32:uPesy_wroom

FQBN32-default = esp32:esp32:esp32
# Targets
.PHONY: all root cam upload-root upload-cam clean

# Build and upload all nodes
all: root cam robot

# Build root node
root:
	@echo "Building root node..."
	arduino-cli compile --fqbn $(FQBN32-default) $(ROOT_NODE_DIR)  --build-property build.extra_flags=-DCORE_DEBUG_LEVEL=5

# Build cam node
cam:
	@echo "Building cam node..."
	arduino-cli compile --fqbn $(FQBN32-cam) $(CAM_NODE_DIR)

82-robot:
	@echo "building esp8266-robot"
	arduino-cli compile --fqbn ${FQBN82} ${ROBOT1_NODE_DIR}

robot: 
	@echo "Building robot node..."
	arduino-cli compile --fqbn $(FQBN32-robot) $(ROBOT_NODE_DIR)

node2: 
	@echo "compiling node2 from $(ESP822_NODE_DIR)..."
	arduino-cli compile --fqbn ${FQBN82} $(ESP822_NODE_DIR)

# Upload root node
upload-root: root
	@echo "Uploading root node to $(ROOT_NODE_PORT)..."
	arduino-cli upload -p $(PORT0) $(ROOT_NODE_DIR)
	arduino-cli monitor -p ${PORT0} -c 115200 --fqbn ${FQBN32-root}

# Upload cam node
upload-cam: cam
	@echo "Uploading cam node to $(CAM_NODE_PORT)..."
	arduino-cli upload -p $(PORT1) $(CAM_NODE_DIR) --fqbn ${FQBN32-cam}
	arduino-cli monitor -p ${PORT1} -c 115200 --fqbn ${FQBN32-cam}

upload-robot: robot
	@echo "Uploading robot node to $(ROBOT_NODE_DIR)..."
	arduino-cli upload -p $(PORT1) $(ROBOT_NODE_DIR)
	arduino-cli monitor -p ${PORT1} -c 115200 --fqbn ${FQBN32-robot}

upload-node2: node2
	@echo "Uploading node2 from $(ESP822_NODE_DIR)..."
	arduino-cli upload -p $(PORT1) $(ESP822_NODE_DIR) --fqbn ${FQBN82}
	arduino-cli monitor -p ${PORT1} -c 9600 --fqbn ${FQBN82}

upload-82-robot: 82-robot 
	@echo "uploading esp8266-robot"
	arduino-cli upload -p ${PORT1}  $(ROBOT1_NODE_DIR) --fqbn ${FQBN82}
	arduino-cli monitor -p ${PORT1} -c 9600 --fqbn ${FQBN82}
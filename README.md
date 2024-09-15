# PX4-AMT Engine Control Unit (ECU) Interface Driver

## Overview
This repository contains a custom driver for interfacing the PX4 firmware with the Engine Control Unit (ECU) by AMT Netherlands. The driver reads data from the ECU via UART ports on the Flight Controller, publishes it to a uORB topic, and streams the data using MAVLink, allowing it to be displayed on QGroundControl.

## Features
- Interfaces with AMT Netherlands' ECU via UART ports on PX4.
- Publishes ECU data to uORB topics for PX4 usage.
- Streams ECU data via MAVLink for display in QGroundControl.

## Setup Instructions

### 1. Clone the PX4-Autopilot Repository
To begin, clone the PX4-Autopilot repository:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### 2. Copy Necessary Files
```bash
cp -r path_to_this_repo/ecu1 PX4-Autopilot/src/drivers/
cp -r path_to_this_repo/ecu2 PX4-Autopilot/src/drivers/
cp path_to_this_repo/ecu1.msg PX4-Autopilot/msg/
cp path_to_this_repo/ecu2.msg PX4-Autopilot/msg/
```

### 3. Update MAVLink Message Definitions
Add the ecu_status.xml content to the PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml file. Insert the XML code appropriately within the <messages> section of common.xml.

### 4. Add MAVLink Stream Header
```bash
cp path_to_this_repo/ECU_STATUS.hpp PX4-Autopilot/src/modules/mavlink/streams/
```

### 5. Override MAVLink Source Files
```bash
cp path_to_this_repo/mavlink_main.cpp PX4-Autopilot/src/modules/mavlink/
cp path_to_this_repo/mavlink_messages.cpp PX4-Autopilot/src/modules/mavlink/
```

Compile the firmware and use the provided QGroundControl.exe application from this repository to upload the compiled firmware to your flight controller. Then use the same QGC app to connect to the Autopilot.

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

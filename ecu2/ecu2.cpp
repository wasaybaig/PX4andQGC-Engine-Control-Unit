/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ecu2.hpp"
#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>

#define SOH 0xFF

ECU2::ECU2(const char *port)
    : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
      _fd(-1)
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';
    memset(&_ecu, 0, sizeof(_ecu));
}

ECU2::~ECU2()
{
    stop();
}

bool ECU2::init()
{
    int ret = 0;

    do{
	_fd = ::open(_port, O_RDONLY | O_NOCTTY | O_NDELAY);
	if (_fd < 0) {
		PX4_ERR("Failed to open port: %s", _port);
		ret = PX4_ERROR;
	}

	termios tty{};
	if (tcgetattr(_fd, &tty) != 0) {
		PX4_ERR("Error from tcgetattr: %d", errno);
		ret = PX4_ERROR;
	}

	cfsetospeed(&tty, B230400);
	cfsetispeed(&tty, B230400);

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;

	// Set raw input mode
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_oflag &= ~OPOST;

	// No hardware flow control (disable RTS/CTS)
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
		PX4_ERR("Error from tcsetattr: %d", errno);
		ret = PX4_ERROR;
		return PX4_ERROR;
	}


	if (_fd < 0) {
		PX4_ERR("FAIL: laser fd");
		ret = -1;
		break;
	}
    } while(0);

    // close the fd
    ::close(_fd);
    _fd = -1;

    PX4_INFO("RET: %d",ret);
    if (ret == PX4_OK){
	PX4_INFO("Scheduling...");
	start();
    }
    return ret;
}

void ECU2::start()
{
    ScheduleOnInterval(100_ms);
}

void ECU2::stop()
{
    ScheduleClear();
}

void ECU2::Run()
{
	if (_fd < 0) {
		// open fd
		_fd = ::open(_port, O_RDONLY | O_NOCTTY | O_NDELAY);
	}
    	collect();
}

void ECU2::intToBinaryArray(int8_t n){
	for (int i = 7; i >= 0; i--){
		binaryArray[i] = n % 2;
		n /= 2;
	}
}


int ECU2::collect()
{   char _buffer;
    int n = ::read(_fd, &_buffer, 1);
    int count = 0;
    if (n > 0) {
	while(count < 6){
        	count = parseData(_buffer, count);
		::read(_fd, &_buffer, 1);
	}
    }
    else{
	PX4_ERR("Unable to read!");
    }
    return PX4_OK;
}

int ECU2::parseData(char data, int count)
{
    static char buff[6] = {0};

    if (((unsigned char)data == SOH)){
	buff[0] = data;
	count = 1;
    }

    else if ((count > 0) && (count < 6)){
	buff[count++] = (uint8_t)data;

	if (count == 5){

		uint8_t data1_1 = buff[1];


		//ECU # 1

		if (data1_1 != 0 && (unsigned char)data1_1 != 0x20){
                    uint8_t op_switch1 = data1_1 & 0x07u;
		    if (op_switch1 == 1){
			strncpy(_ecu.ops_switch, "Emergency Mode", sizeof(_ecu.ops_switch));
			_ecu.ops_switch[sizeof(_ecu.ops_switch) - 1] = '\0';
		    }
		    else if (op_switch1 == 2){
			strncpy(_ecu.ops_switch, "Auto Mode", sizeof(_ecu.ops_switch));
			_ecu.ops_switch[sizeof(_ecu.ops_switch) - 1] = '\0';
		    }
		    else if (op_switch1 == 4){
			strncpy(_ecu.ops_switch, "Runnng Mode", sizeof(_ecu.ops_switch));
			_ecu.ops_switch[sizeof(_ecu.ops_switch) - 1] = '\0';
		    }

		    uint8_t engine_status1 = data1_1 & 0xF8u;

		    if (engine_status1 == 8){
			strncpy(_ecu.eng_status, "Start Clearance", sizeof(_ecu.eng_status));
			_ecu.eng_status[sizeof(_ecu.eng_status) - 1] = '\0';
		    }
		    else if (engine_status1 == 16){
			strncpy(_ecu.eng_status, "Starting", sizeof(_ecu.eng_status));
			_ecu.eng_status[sizeof(_ecu.eng_status) - 1] = '\0';
		    }
		    else if (engine_status1 == 32){
			strncpy(_ecu.eng_status, "StartUp", sizeof(_ecu.eng_status));
			_ecu.eng_status[sizeof(_ecu.eng_status) - 1] = '\0';
		    }
		    else if (engine_status1 == 64){
			strncpy(_ecu.eng_status, "Idle Calib", sizeof(_ecu.eng_status));
			_ecu.eng_status[sizeof(_ecu.eng_status) - 1] = '\0';
		    }
		    else if (engine_status1 == 96){
			strncpy(_ecu.eng_status, "Full Operation", sizeof(_ecu.eng_status));
			_ecu.eng_status[sizeof(_ecu.eng_status) - 1] = '\0';
		    }

		    else if (engine_status1 == 224){
			strncpy(_ecu.eng_status, "Max RPM", sizeof(_ecu.eng_status));
			_ecu.eng_status[sizeof(_ecu.eng_status) - 1] = '\0';
		    }

		    float rpm = buff[2];
		    float egt = buff[3];
		    float thr = buff[4];
	      	    float vout = buff[5];

		    rpm = rpm * 500;
		    egt = egt * 4.6f - 50.0f;
		    thr = thr / 2;
		    vout = vout * 8.3f / 255.0f;

                    _ecu.rpm = rpm;
		    _ecu.egt = egt;
		    _ecu.thr = thr;
		    _ecu.vout = vout;
		    strncpy(_ecu.error, "", sizeof(_ecu.error));
		    _ecu.error[sizeof(_ecu.error) - 1] = '\0';
		}

		else if (data1_1 == 0){
		    uint8_t error_code1 = buff[2];
		    intToBinaryArray(error_code1);

		    if (binaryArray[7] == 1){
			strncpy(_ecu.error, "RPM Error", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }
		    else if (binaryArray[6] == 1){
			strncpy(_ecu.error, "Switch Ch Error", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }
		    else if (binaryArray[5] == 1){
			strncpy(_ecu.error, "Throttle Ch Error", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }
		    else if (binaryArray[4] == 1){
			strncpy(_ecu.error, "EGT Error", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }
		    else if (binaryArray[3] == 1){
			strncpy(_ecu.error, "High RPM Error", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }
		    else if (binaryArray[2] == 1){
			strncpy(_ecu.error, "Low Supply Error", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }
		    else if (binaryArray[1] == 1){
			strncpy(_ecu.error, "Low Supply for Auto", sizeof(_ecu.error));
			_ecu.error[sizeof(_ecu.error) - 1] = '\0';
		    }

		    float egt = buff[3];
		    float thr = buff[4];
	      	    float vout = buff[5];

                    egt = egt * 4.6f - 50.0f;
		    thr = thr / 2;
		    vout = vout * 8.3f / 255.0f;
		    _ecu.rpm = -1;
		    _ecu.egt = egt;
		    _ecu.thr = thr;
		    _ecu.vout = vout;

		}

		else if ((unsigned char)data1_1 == 0x20){
		    float idle_v = buff[2];
		    float bat_v = buff[3];
		    float maxRPM_v = buff[5];

		    idle_v = idle_v * 8.3f / 255.0f;
		    bat_v = bat_v * 8.3f / 255.0f;
		    maxRPM_v = 7 + (maxRPM_v * 9.3f / 255.0f);

		    uint8_t ecu_info = buff[4] & 0x01u;

		    if (ecu_info == 0){
		        _ecu.bldc = 0;
		    }
		    else if (ecu_info == 1){
			_ecu.bldc = 1;
		    }

		    _ecu.idle_v = idle_v;
		    _ecu.bat_v = bat_v;
		    _ecu.maxrpm_v = maxRPM_v;

		}

		_ecu.timestamp = hrt_absolute_time();
		_ecu_pub.publish(_ecu);

	}

	}

    return count;
}

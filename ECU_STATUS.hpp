/****************************************************************************
*
* Copyright (c) 2020 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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

#ifndef ECU_STATUS_HPP
#define ECU_STATUS_HPP

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/ecu1.h>
#include <uORB/topics/ecu2.h>

class MavlinkStreamECUStatus : public MavlinkStream
{
	public:

		static MavlinkStream *new_instance(Mavlink *mavlink)
		{
			return new MavlinkStreamECUStatus(mavlink);
		}

		const char *get_name() const override
		{
			return get_name_static();
		}

		static constexpr const char *get_name_static()
		{
			return "ECU_STATUS";
		}
		static constexpr uint16_t get_id_static()
		{
			return MAVLINK_MSG_ID_ECU_STATUS;
		}

		uint16_t get_id() override
		{
			return get_id_static();
		}
		unsigned get_size() override
		{
			return MAVLINK_MSG_ID_ECU_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

	private:
		//Subscription to array of uORB battery status instances
		uORB::Subscription _ecu1_sub{ORB_ID(ecu1)};
		uORB::Subscription _ecu2_sub{ORB_ID(ecu2)};
		// uORB::Subscription is used to subscribe to a single-instance topic

		explicit MavlinkStreamECUStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

		bool send() override
		{
			bool updated = false;
			// battery_status_s is a struct that can hold the battery object topic
			ecu1_s ecu1_status;
			ecu2_s ecu2_status;
			_ecu1_sub.copy(&ecu1_status);
			_ecu2_sub.copy(&ecu2_status);
			// mavlink_battery_status_demo_t is the MAVLink message object
			mavlink_ecu_status_t ecu_msg{};

			ecu_msg.rpm1 = (float)ecu1_status.rpm;

			ecu_msg.thr1 = (float)ecu1_status.thr;
			ecu_msg.egt1 = (float)ecu1_status.egt;
			ecu_msg.vout1 = (float)ecu1_status.vout;
			ecu_msg.bldc1 = (float)ecu1_status.bldc;
			memcpy(ecu_msg.error1, ecu1_status.error, sizeof(ecu_msg.error1));
			ecu_msg.error1[sizeof(ecu_msg.error1) - 1] = '\0';
			memcpy(ecu_msg.eng_status1, ecu1_status.eng_status, sizeof(ecu_msg.eng_status1));
			ecu_msg.eng_status1[sizeof(ecu_msg.eng_status1) - 1] = '\0';
			memcpy(ecu_msg.ops_switch1, ecu1_status.ops_switch, sizeof(ecu_msg.ops_switch1));
			ecu_msg.ops_switch1[sizeof(ecu_msg.ops_switch1) - 1] = '\0';
			ecu_msg.bat_v1 = (float)ecu1_status.bat_v;
			ecu_msg.idle_v1 = (float)ecu1_status.idle_v;
			ecu_msg.maxrpm_v1 = (float)ecu1_status.maxrpm_v;
			ecu_msg.rpm2 = (float)ecu2_status.rpm;
			ecu_msg.thr2 = (float)ecu2_status.thr;
			ecu_msg.egt2 = (float)ecu2_status.egt;
			ecu_msg.vout2 = (float)ecu2_status.vout;
			ecu_msg.bldc2 = (float)ecu2_status.bldc;
			memcpy(ecu_msg.error2, ecu2_status.error, sizeof(ecu_msg.error2));
			ecu_msg.error2[sizeof(ecu_msg.error2) - 1] = '\0';
			memcpy(ecu_msg.eng_status2, ecu2_status.eng_status, sizeof(ecu_msg.eng_status2));
			ecu_msg.eng_status2[sizeof(ecu_msg.eng_status2) - 1] = '\0';
			memcpy(ecu_msg.ops_switch2, ecu2_status.ops_switch, sizeof(ecu_msg.ops_switch2));
			ecu_msg.ops_switch2[sizeof(ecu_msg.ops_switch2) - 1] = '\0';
			ecu_msg.bat_v2 = (float)ecu2_status.bat_v;
			ecu_msg.idle_v2 = (float)ecu2_status.idle_v;
			ecu_msg.maxrpm_v2 = (float)ecu2_status.maxrpm_v;

			//Send the message
			mavlink_msg_ecu_status_send_struct(_mavlink->get_channel(), &ecu_msg);

			updated = true;

			return updated;
		}

};
#endif // ECU_STATUS_HPP


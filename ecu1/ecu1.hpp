/**
 * @file ecu1.hpp
 *
 * Defines for the ECU Driver
 */

#pragma once
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/ecu1.h>
#include <uORB/topics/air_data_boom.h>
#include <px4_time.h>


#define ECU_DEFAULT_PORT	"/dev/ttyS1"

using namespace time_literals;

class ECU1 : public px4::ScheduledWorkItem
{
public:
    ECU1(const char *port);
    virtual ~ECU1();

    bool init();


private:
    char _port[20] {};
    int _fd{-1};

    uORB::Publication<ecu1_s>     _ecu_pub{ORB_ID(ecu1)};
    uORB::Publication<air_data_boom_s>     _air_data_pub{ORB_ID(air_data_boom)};
    ecu1_s _ecu;
    air_data_boom_s air_data;
    int8_t binaryArray[8] = {};

    void Run() override;
    int collect();
    int parseData(char data, int count);
    void intToBinaryArray(int8_t n);
    void start();
    void stop();
};

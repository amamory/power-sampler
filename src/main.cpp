#ifdef __INTELLISENSE__
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#endif

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" 
{
 
#include "periodic.h"
#include "sensor_file.h"
#include "sensor_hwmon.h"
#include "sensor_iio.h"
#include "sensor_ina226.h"
#include "sensor_ina231.h"

#ifndef UDEV_NOTFOUND
#include "sensor_smartpower.h"
#endif
}
// ROS2 related

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>

#include <std_msgs/msg/u_int32.hpp>

#define DEG2RAD M_PI / 180.0


sig_atomic_t keep_sampling = 1;
sig_atomic_t mark_section = 0;

void mark(int signum __attribute((unused)),
          siginfo_t *info __attribute((unused)),
          void *ptr __attribute((unused))) {
    mark_section = 1;
}

void stop(int signum __attribute((unused)),
          siginfo_t *info __attribute((unused)),
          void *ptr __attribute((unused))) {
    keep_sampling = 0;
}

void init_signal_action() {
    struct sigaction action_int;
    struct sigaction action_usr1;

    memset(&action_int, 0, sizeof(action_int));
    memset(&action_usr1, 0, sizeof(action_usr1));

    action_int.sa_sigaction = stop;
    action_int.sa_flags = SA_SIGINFO;

    action_usr1.sa_sigaction = mark;
    action_usr1.sa_flags = SA_SIGINFO;

    sigaction(SIGINT, &action_int, NULL);
    sigaction(SIGTERM, &action_int, NULL);
    sigaction(SIGQUIT, &action_int, NULL);

    sigaction(SIGUSR1, &action_usr1, NULL);
}

int main(int argc, char *argv[]) {

    //#############################################
    // ROS2 related init
    //#############################################
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("energy_sensors");

    auto current_pub = node->create_publisher<std_msgs::msg::UInt32>("current", 10);

    rclcpp::WallRate loop_rate(30);

    std_msgs::msg::UInt32 msg;
    // start w fixed value just to setup the initial compilation
    msg.data = 1;

    RCLCPP_INFO(node->get_logger(), "starting node 'energy_sensors'");

    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    //#############################################
    // original initialization
    //#############################################
    LIST_HEAD(sensors_list);

    // Register signal handlers
    init_signal_action();

    list_splice_free(sensors_file_init(), &sensors_list);
    list_splice_free(sensors_hwmon_init(), &sensors_list);
    list_splice_free(sensors_iio_init(), &sensors_list);
    list_splice_free(sensors_ina226_init(), &sensors_list);
    list_splice_free(sensors_ina231_init(), &sensors_list);

#ifndef UDEV_NOTFOUND
    list_splice_free(sensors_smartpower_init(), &sensors_list);
#endif

    // Select the minimum update period among them all
    struct sensor *pos;
    long period_us = 50000L; // Maximum 20 times a second, in useconds

    list_for_each_entry(pos, &sensors_list, list) {
        if (pos->period_us > 0 && pos->period_us < period_us)
            period_us = pos->period_us;
    }

    printf("UPDATE_PERIOD_us %ld\n\n", period_us);

    // Turn on full buffering for stdout, avoiding a flush each printline
    fflush(stdout);
    setvbuf(stdout, NULL, _IOFBF, 0);

    struct timespec at;
    rt_start_period(&at);

    // Until the user sends a SIGINT
    while (keep_sampling && rclcpp::ok()) {
        if (mark_section) {
            printf("--------------------------------------------\n\n");
            mark_section = 0;
        }

        // Read data from device and print it
        list_for_each_entry(pos, &sensors_list, list) {
            pos->read(pos);
        }

        list_for_each_entry(pos, &sensors_list, list) {
            pos->print_last(pos);
        }

        printf("\n");
        fflush(stdout);

        // Wait next activation time
        rt_next_period(&at, period_us);

        current_pub->publish(msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Re-enable full buffering for stdout
    setvbuf(stdout, NULL, _IOLBF, 0);

    list_for_each_entry(pos, &sensors_list, list) {
        pos->close(pos);
    }

    rclcpp::shutdown();

    return 0;
}

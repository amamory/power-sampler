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
// ROS has its own method to provide a periodic task
//#include "periodic.h"
#include "sensor_file.h"
#include "sensor_hwmon.h"
#include "sensor_ina231.h"

#ifndef UDEV_NOTFOUND
#include "sensor_smartpower.h"
#endif
}

#include "sensor_iio.h"
#include "sensor_ina226.h"

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
// for the temperature topic
//#include <std_msgs/msg/float64.hpp>


sig_atomic_t keep_sampling = 1;
//sig_atomic_t mark_section = 0;

// void mark(int signum __attribute((unused)),
//           siginfo_t *info __attribute((unused)),
//           void *ptr __attribute((unused))) {
//     mark_section = 1;
// }

void stop(int signum __attribute((unused)),
          siginfo_t *info __attribute((unused)),
          void *ptr __attribute((unused))) {
    keep_sampling = 0;
}

void init_signal_action() {
    struct sigaction action_int;
    //struct sigaction action_usr1;

    memset(&action_int, 0, sizeof(action_int));
    //memset(&action_usr1, 0, sizeof(action_usr1));

    action_int.sa_sigaction = stop;
    action_int.sa_flags = SA_SIGINFO;

    // action_usr1.sa_sigaction = mark;
    // action_usr1.sa_flags = SA_SIGINFO;

    sigaction(SIGINT, &action_int, NULL);
    sigaction(SIGTERM, &action_int, NULL);
    sigaction(SIGQUIT, &action_int, NULL);

    //sigaction(SIGUSR1, &action_usr1, NULL);
}

rclcpp::Node::SharedPtr node = NULL;

int main(int argc, char *argv[]) {

    //#############################################
    // ROS2 related init
    //#############################################
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("energy_sensors");

    auto current_pub = node->create_publisher<std_msgs::msg::UInt32>("current", 10);
    //auto temp_pub = node->create_publisher<std_msgs::msg::Float64>("temperature", 10);

    // TODO ROS: set the period from configuration files
    //rclcpp::WallRate loop_rate(30);
    rclcpp::WallRate loop_rate(1);

    // std_msgs::msg::Float64 temp_msg;
    // temp_msg.data = 12.345;

    std_msgs::msg::UInt32 msg;
    // start w fixed value just to setup the initial compilation
    msg.data = 1;

    RCLCPP_INFO(node->get_logger(), "starting node 123 'energy_sensors'");

    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    RCLCPP_INFO(node->get_logger(), "starting node 1 'energy_sensors'");

    //#############################################
    // original initialization  
    //#############################################
    LIST_HEAD(sensors_list);
    RCLCPP_INFO(node->get_logger(), "starting node 2 'energy_sensors'");

    // Register signal handlers
    init_signal_action();
    RCLCPP_INFO(node->get_logger(), "starting node 3 'energy_sensors'");

    struct sensor *pos;

    // pass the ROS node pointer to the sensors
    // list_for_each_entry(pos, &sensors_list, list) {
    //     pos->node = node->get_node_base_interface() ;
    // }

    // the ROS topics are also setup in theirs respective init functions
    // both the publisher and the messages are created here
    // list_splice_free(sensors_file_init(), &sensors_list);
    // RCLCPP_INFO(node->get_logger(), "starting node 3.1 'energy_sensors'");
    // list_splice_free(sensors_hwmon_init(), &sensors_list);
    RCLCPP_INFO(node->get_logger(), "starting node 3.2 'energy_sensors'");
    list_splice_free(sensors_iio_init(), &sensors_list);
    RCLCPP_INFO(node->get_logger(), "starting node 3.3 'energy_sensors'");
    RCLCPP_INFO(node->get_logger(), "starting node 3.3.1 'energy_sensors'");
    list_splice_free(sensors_ina226_init(), &sensors_list);
    // RCLCPP_INFO(node->get_logger(), "starting node 3.4 -+*! 'energy_sensors'");
    // list_splice_free(sensors_ina231_init(), &sensors_list);

    RCLCPP_INFO(node->get_logger(), "starting node 4 'energy_sensors'");
#ifndef UDEV_NOTFOUND
    list_splice_free(sensors_smartpower_init(), &sensors_list);
#endif

    // Select the minimum update period among them all
    // TODO ROS: set the period from configuration files
    //long period_us = 50000L; // Maximum 20 times a second, in useconds
    //long period_us = 2000000L; // every 2 seconds, in useconds

    // list_for_each_entry(pos, &sensors_list, list) {
    //     if (pos->period_us > 0 && pos->period_us < period_us)
    //         period_us = pos->period_us;
    // }

    #ifndef DONT_PRINT
        //printf("UPDATE_PERIOD_us %ld\n\n", period_us);

        // Turn on full buffering for stdout, avoiding a flush each printline
        fflush(stdout);
        setvbuf(stdout, NULL, _IOFBF, 0);
    #endif

    // struct timespec at;
    // rt_start_period(&at);

    // Until the user sends a SIGINT
    RCLCPP_INFO(node->get_logger(), "outside the loop ...");
    while (keep_sampling && rclcpp::ok()) {
    //while (rclcpp::ok()) {
        //RCLCPP_INFO(node->get_logger(), "inside the loop ...");
    //for (int i =0; i < 200 ; i++){
        // #ifndef DONT_PRINT 
        //     if (mark_section) {
        //         printf("--------------------------------------------\n\n");
        //         mark_section = 0;
        //     }
        // #endif 

        RCLCPP_INFO(node->get_logger(), "reading sensors ...");
        // Read data from device and print it
        list_for_each_entry(pos, &sensors_list, list) {
            pos->read(pos);
        }
        //RCLCPP_INFO(node->get_logger(), "sensors read");

        #ifndef DONT_PRINT
            list_for_each_entry(pos, &sensors_list, list) {
                pos->print_last(pos);
            }
            printf("\n");
            fflush(stdout);
        #endif 


        // RCLCPP_INFO(node->get_logger(), "about to publish");
        // current_pub->publish(msg);
        // RCLCPP_INFO(node->get_logger(), "current published");
        // temp_pub->publish(temp_msg);
        // RCLCPP_INFO(node->get_logger(), "temp published");

        // // publish into their respective topics
        RCLCPP_INFO(node->get_logger(), "publishing all sensors again ...");
        list_for_each_entry(pos, &sensors_list, list) {
            pos->publish(pos);
        }
        RCLCPP_INFO(node->get_logger(), "sensors published");

        // Wait next activation time
        //rt_next_period(&at, period_us);

        // current_pub->publish(msg);
        // RCLCPP_INFO(node->get_logger(), "current published");
        // temp_pub->publish(temp_msg);
        // RCLCPP_INFO(node->get_logger(), "temp published");
        rclcpp::spin_some(node);

        //RCLCPP_INFO(node->get_logger(), "loop end - before sleep");
        loop_rate.sleep();
        //RCLCPP_INFO(node->get_logger(), "loop end - before sleep");
    }

#ifndef DONT_PRINT
    // Re-enable full buffering for stdout
    setvbuf(stdout, NULL, _IOLBF, 0);
#endif

    list_for_each_entry(pos, &sensors_list, list) {
        pos->close(pos);
    }

    RCLCPP_INFO(node->get_logger(), "closing node 'energy_sensors'");
    rclcpp::shutdown();

    return 0;
}

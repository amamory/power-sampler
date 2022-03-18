#ifndef SENSOR_INA226_H
#define SENSOR_INA226_H

#include "sensor.h"

#include <rclcpp/rclcpp.hpp>
// ROS message type used by this sensor
#include <std_msgs/msg/float64.hpp>

struct ina226_measure {
    char in_path[64];
    char out_path[64];
    long in_value;
    long out_value;
    long diff_value;
};

struct ina226_data {
    char linename[12];
    int rail; // enum INA226_US_PLUS_ZCU_102_RAIL
    struct ina226_measure current;
    struct ina226_measure voltage;
    struct ina226_measure power;
    struct list_head list;
};

struct sensor_ina226 {
    struct sensor base;
    struct list_head data_list;
    // ROS related attributes
    std_msgs::msg::Float64 msg;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
};

// ---------------------- METHODS ----------------------- //

// Close a connection with the INA226 driver
extern void sensor_ina226_close(struct sensor *sself);

// Read data from the INA226 driver
extern int sensor_ina226_read(struct sensor *sself);

// Print last data sample collected
extern void sensor_ina226_print_last(struct sensor *sself);

// Publish last data sample collected into its ROS topic
extern void sensor_ina226_publish(struct sensor *sself);

// ============ DETECTION AND INITIALIZATION ============ //

extern struct list_head *sensors_ina226_init();

#endif // SENSOR_INA226_H

#ifndef SENSOR_IIO_H
#define SENSOR_IIO_H

#include "sensor.h"

#include <rclcpp/rclcpp.hpp>
// ROS message type used by this sensor
#include <std_msgs/msg/float64.hpp>

struct sensor_iio {
    // original attributes
    struct sensor base;
    char fpath_offset[128]; // TODO: increase if necessary
    char fpath_raw[128];    // TODO: increase if necessary
    char fpath_scale[128];  // TODO: increase if necessary
    double offset;
    double scale;
    double raw;
    double value;
    // ROS related attributes
    std_msgs::msg::Float64::SharedPtr msg;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
    rclcpp::Node::SharedPtr node;
};

// ---------------------- METHODS ----------------------- //

// Close a connection with the iio driver
extern void sensor_iio_close(struct sensor *sself);

// Read data from the iio driver
extern int sensor_iio_read(struct sensor *sself);

// Print last data sample collected
extern void sensor_iio_print_last(struct sensor *sself);

// Publish last data sample collected into its ROS topic
extern void sensor_iio_publish(struct sensor *sself);

// ============ DETECTION AND INITIALIZATION ============ //

extern struct list_head *sensors_iio_init();
#endif // SENSOR_IIO_H

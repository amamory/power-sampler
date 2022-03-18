#include <dirent.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "private/ina226_private.h"
#include "readfile.h"
#include "sensor_ina226.h"

extern rclcpp::Node::SharedPtr node;

#define __SENSOR_INA226_BASE_INITIALIZER                                       \
    {                                                                          \
        {}, -1, "", sensor_ina226_read, sensor_ina226_close,                   \
            sensor_ina226_print_last, sensor_ina226_publish,                   \
    }

#define __SENSOR_INA226_INITIALIZER                                            \
    {                                                                          \
        __SENSOR_INA226_BASE_INITIALIZER,  {}, {}, NULL ,                      \
    }

const struct sensor_ina226 SENSOR_INA226_INITIALIZER =
    __SENSOR_INA226_INITIALIZER;

// ------------------ Private Methods ------------------- //

static inline void set_file_if_exists(char *dest, const char *fname_base,
                                      const int fname_base_len,
                                      const char *fname_suffix) {
    strcpy(dest, fname_base);
    strcpy(dest + fname_base_len, fname_suffix);

    fprintf(stderr, "DEBUG: EXISTS? %s\n", dest);

    // If file does not exist, do not set the file name
    if (access(dest, F_OK) != 0) {
        dest[0] = '\0';
    } else {
        fprintf(stderr, "DEBUG: EXISTS? TRUE\n");
    }
}

static inline void sensor_ina226_getlist(struct list_head *list) {
    DIR *dir;
    struct dirent *dirent;

    char buffer[100];
    char fname_buff[100] = HWMON_DIR;

    size_t name_len;

    dir = opendir(HWMON_DIR);

    fprintf(stderr, "DEBUG: !\n");

    while ((dirent = readdir(dir)) != NULL) {
        if (strcmp(".", dirent->d_name) == 0 ||
            strcmp("..", dirent->d_name) == 0) {
            continue;
        }

        name_len = strlen(dirent->d_name);

        strcpy(fname_buff + HWMON_BASE_IDX, dirent->d_name);
        strcpy(fname_buff + HWMON_BASE_IDX + name_len, HWMON_NAME);

        fprintf(stderr, "DEBUG: FNAME: %s\n", fname_buff);

        int res = readfile(fname_buff, buffer, sizeof(buffer) - 1);
        if (res < 1)
            continue;

        buffer[res] = '\0';
        if (buffer[res - 1] == '\n')
            buffer[res - 1] = '\0';

        for (int i = PS_MIN; i < PL_MAX; ++i) {
            // Skip VCC for ARM debug subsystem
            if (i == VCCOPS || i == VCCOPS3)
                continue;

            fprintf(stderr, "DEBUG: CMP: %s %s\n", buffer,
                    ina226_lines[i].linename);
            if (strcmp(buffer, ina226_lines[i].linename) == 0) {
                // The current directory represents a valid directory for the PS
                fprintf(stderr, "DEBUG: CMP TRUE!\n");

                // Get back original directory name
                fname_buff[HWMON_BASE_IDX + name_len] = '\0';

                struct ina226_data *data = (struct ina226_data *)malloc(sizeof(struct ina226_data));

                // Line and rail information
                strcpy(data->linename, ina226_lines[i].linename);
                data->rail = i;

                // Current
                set_file_if_exists(data->current.in_path, fname_buff,
                                   HWMON_BASE_IDX + name_len, HWMON_CURR_IN);
                set_file_if_exists(data->current.out_path, fname_buff,
                                   HWMON_BASE_IDX + name_len, HWMON_CURR_OUT);

                // Voltage
                set_file_if_exists(data->voltage.in_path, fname_buff,
                                   HWMON_BASE_IDX + name_len, HWMON_VOLT_IN);
                set_file_if_exists(data->voltage.out_path, fname_buff,
                                   HWMON_BASE_IDX + name_len, HWMON_VOLT_OUT);

                // Power
                set_file_if_exists(data->power.in_path, fname_buff,
                                   HWMON_BASE_IDX + name_len, HWMON_POWER_IN);
                set_file_if_exists(data->power.out_path, fname_buff,
                                   HWMON_BASE_IDX + name_len, HWMON_POWER_OUT);

                // Add to list
                list_add(&data->list, list);
            }
        }
    }

    closedir(dir);
}

#define READLONG_OR_BUST(path) (((path)[0] == '\0') ? 0 : readlong(path))

static inline void ina226_readmeasure(struct ina226_measure *measure) {
    measure->in_value = READLONG_OR_BUST(measure->in_path);
    measure->out_value = READLONG_OR_BUST(measure->out_path);
    measure->diff_value = labs(measure->in_value - measure->out_value);
}

// setup the sensor topic
void sensor_ina226_ros_init(struct sensor_ina226 *self){
    if (node == NULL){
        printf("ERROR: have to set the attribute `node` before calling init() for %s\n", self->base.name);
        exit(-1);
    }
    //printf("creating topic 'power_rail_list'\n");
    RCLCPP_INFO(node->get_logger(), "creating topic 'power_rail_list'");
    self->pub = node->create_publisher<std_msgs::msg::Float64>("power_rail_list", 10);
    RCLCPP_INFO(node->get_logger(), "topic 'power_rail_list' created");
    //printf("topic 'power_rail_list' created\n");
}

// ------------------- Public Methods ------------------- //

struct sensor_ina226 *sensor_ina226_new() {
    struct sensor_ina226 *ptr = (struct sensor_ina226 *) malloc(sizeof(struct sensor_ina226));
    RCLCPP_INFO(node->get_logger(), "new init %d ", sizeof(struct sensor_ina226));
    //if (ptr != NULL)
    //    *ptr = SENSOR_INA226_INITIALIZER;
    RCLCPP_INFO(node->get_logger(), "list head");
    INIT_LIST_HEAD(&ptr->data_list);
    return ptr;
}

// Success if return >= 0
long sensor_ina226_update_period(struct sensor_ina226 *self
                                 __attribute((unused)),
                                 const char *node __attribute((unused))) {
    // TODO:
    return 0;
}

void sensor_ina226_close(struct sensor *sself __attribute((unused))) {}

int sensor_ina226_read(struct sensor *sself) {
    struct sensor_ina226 *self = (struct sensor_ina226 *)sself;

    struct ina226_data *data;
    list_for_each_entry(data, &self->data_list, list) {
        ina226_readmeasure(&data->current);
        ina226_readmeasure(&data->voltage);
        ina226_readmeasure(&data->power);
    }

    return 0;
}

void sensor_ina226_print_last(struct sensor *sself) {
    struct sensor_ina226 *self = (struct sensor_ina226 *)sself;

    struct ina226_data *data;
    long current_sum __attribute((unused)) = 0;
    long voltage_sum __attribute((unused)) = 0;
    long power_sum = 0;
    long power_calc_sum = 0;

    list_for_each_entry(data, &self->data_list, list) {
        // current_sum += data->current.diff_value;
        // voltage_sum += data->voltage.diff_value;
        power_sum += data->power.diff_value;
        power_calc_sum += data->current.diff_value * data->voltage.diff_value;
    }

    // printf("%s_uA %ld\n", self->base.name, current_sum * 1000L);
    // printf("%s_uV %ld\n", self->base.name, voltage_sum * 1000L);
    printf("%s_uW %ld\n", self->base.name, power_sum);
    printf("%s_CALC_uW %ld\n", self->base.name, power_calc_sum);
}

void sensor_ina226_publish(struct sensor *sself __attribute((unused))) {
    struct sensor_ina226 *self = (struct sensor_ina226 *)sself;

    struct ina226_data *data;
    long current_sum __attribute((unused)) = 0;
    long voltage_sum __attribute((unused)) = 0;
    long power_sum = 0;
    long power_calc_sum = 0;

    list_for_each_entry(data, &self->data_list, list) {
        // current_sum += data->current.diff_value;
        // voltage_sum += data->voltage.diff_value;
        power_sum += data->power.diff_value;
        power_calc_sum += data->current.diff_value * data->voltage.diff_value;
    }

    // printf("%s_uA %ld\n", self->base.name, current_sum * 1000L);
    // printf("%s_uV %ld\n", self->base.name, voltage_sum * 1000L);
    printf("%s_uW %ld\n", self->base.name, power_sum);
    printf("%s_CALC_uW %ld\n", self->base.name, power_calc_sum);  
    
    self->msg.data = static_cast<double> (power_calc_sum);
    self->pub->publish(self->msg);
    RCLCPP_INFO(node->get_logger(), "power (uW):\t%f", self->msg.data);    
}

// =========================================================
// SENSORS DETECTION AND INITIALIZATION
// =========================================================

struct list_head *sensors_ina226_init() {
    RCLCPP_INFO(node->get_logger(), "entrei init");
    
    struct list_head *list = list_new();
    if (list == NULL)
        exit(EXIT_FAILURE);
    RCLCPP_INFO(node->get_logger(), "entrei2 init");

    struct sensor_ina226 *s = sensor_ina226_new();
    RCLCPP_INFO(node->get_logger(), "entrei3 init");
    sensor_ina226_getlist(&s->data_list);
    RCLCPP_INFO(node->get_logger(), "entrei4 init");
    strcpy(s->base.name, "sensor_ina226");

    RCLCPP_INFO(node->get_logger(), "antes ros init");
    sensor_ina226_ros_init(s);
    RCLCPP_INFO(node->get_logger(), "deopis ros init");

    if (list_empty(&s->data_list)) {
        free(s);
    } else {
        list_add(&s->base.list, list);
    }

    return list;
}

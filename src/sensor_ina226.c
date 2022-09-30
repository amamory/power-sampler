#include <dirent.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "private/ina226_private.h"
#include "readfile.h"
#include "sensor_ina226.h"

#define __SENSOR_INA226_BASE_INITIALIZER                                       \
    {                                                                          \
        {}, -1, "", sensor_ina226_read, sensor_ina226_close,                   \
            sensor_ina226_print_last,                                          \
    }

#define __SENSOR_INA226_INITIALIZER                                            \
    {                                                                          \
        __SENSOR_INA226_BASE_INITIALIZER, {}                                   \
    }

const struct sensor_ina226 SENSOR_INA226_INITIALIZER =
    __SENSOR_INA226_INITIALIZER;

// ------------------ Private Methods ------------------- //

static inline void set_file_if_exists(char *dest, const char *fname_base,
                                      const int fname_base_len,
                                      const char *fname_suffix) {
    strcpy(dest, fname_base);
    strcpy(dest + fname_base_len, fname_suffix);

    //fprintf(stderr, "DEBUG: EXISTS? %s\n", dest);

    // If file does not exist, do not set the file name
    if (access(dest, F_OK) != 0) {
        dest[0] = '\0';
    } 
    // else {
    //     fprintf(stderr, "DEBUG: EXISTS? TRUE\n");
    // }
}

static inline void sensor_ina226_getlist(struct list_head *list) {
    DIR *dir;
    struct dirent *dirent;

    char buffer[100];
    char fname_buff[100] = HWMON_DIR;

    size_t name_len;

    dir = opendir(HWMON_DIR);

    //fprintf(stderr, "DEBUG: !\n");

    while ((dirent = readdir(dir)) != NULL) {
        if (strcmp(".", dirent->d_name) == 0 ||
            strcmp("..", dirent->d_name) == 0) {
            continue;
        }

        name_len = strlen(dirent->d_name);

        strcpy(fname_buff + HWMON_BASE_IDX, dirent->d_name);
        strcpy(fname_buff + HWMON_BASE_IDX + name_len, HWMON_NAME);

        //fprintf(stderr, "DEBUG: FNAME: %s\n", fname_buff);

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

            //fprintf(stderr, "DEBUG: CMP: %s %s\n", buffer,
            //        ina226_lines[i].linename);
            if (strcmp(buffer, ina226_lines[i].linename) == 0) {
                // The current directory represents a valid directory for the PS
                //fprintf(stderr, "DEBUG: CMP TRUE!\n");

                // Get back original directory name
                fname_buff[HWMON_BASE_IDX + name_len] = '\0';

                struct ina226_data *data = malloc(sizeof(struct ina226_data));

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

// ------------------- Public Methods ------------------- //

struct sensor_ina226 *sensor_ina226_new() {
    struct sensor_ina226 *ptr = malloc(sizeof(struct sensor_ina226));
    if (ptr != NULL)
        *ptr = SENSOR_INA226_INITIALIZER;
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
/*
this function has been modified to print data in a CSV format
by Amory
*/
void sensor_ina226_print_last(struct sensor *sself) {
    struct sensor_ina226 *self = (struct sensor_ina226 *)sself;

    struct ina226_data *data;
    long current_sum __attribute((unused)) = 0;
    long voltage_sum __attribute((unused)) = 0;
    long ps_power_sum = 0;
    long pl_power_sum = 0;

    // this is used to get the board total power, dividing it by PS and PL power
    // the loops are duplicated such that the rail columns are nicely grouped, i.e, 1st all PS rails and then all the PL rails
    list_for_each_entry(data, &self->data_list, list) {
        for(int i=PS_MIN; i<PS_MAX; ++i) {
            if (strcmp(data->linename, ina226_lines[i].linename) == 0){
                printf("%ld,", data->power.diff_value);
                ps_power_sum += data->power.diff_value;
            }
        }
    }
    list_for_each_entry(data, &self->data_list, list) {
        for(int i=PL_MIN; i<PL_MAX; ++i) {
            if (strcmp(data->linename, ina226_lines[i].linename) == 0){
                printf("%ld,", data->power.diff_value);
                pl_power_sum += data->power.diff_value;
            }
        }
    }

    /* print only the power lines of interest, which are:
        VCCINT - the main power line for the PL part. includes LUTs, DSPs, clocks, 
        VCCBRAM - PL BRAM power line
        VCCPSPLL - the main power line for the PS part
        VCCPSDDR and VCCPSDDRPLL - the main power line for the DRAM    
    */
   // uncomment this code to print only selected columns of the CSV
   /*
    list_for_each_entry(data, &self->data_list, list) {
        if (data->rail == VCCINT || data->rail == VCCBRAM || data->rail == VCCPSPLL || data->rail == VCCPSDDR || data->rail == VCCPSDDRPLL){
            printf("%ld,", data->current.diff_value * data->voltage.diff_value);
        }
        // power_calc_sum += data->current.diff_value * data->voltage.diff_value;
    }
    */

    // printf("%s_uA %ld\n", self->base.name, current_sum * 1000L);
    // printf("%s_uV %ld\n", self->base.name, voltage_sum * 1000L);
    // printf("%s_uW %ld\n", self->base.name, power_sum);
    // printf("%s_CALC_uW %ld\n", self->base.name, power_calc_sum);
    printf("%ld,%ld,%ld,", ps_power_sum, pl_power_sum, ps_power_sum + pl_power_sum);
}

// =========================================================
// SENSORS DETECTION AND INITIALIZATION
// =========================================================

struct list_head *sensors_ina226_init() {
    struct list_head *list = list_new();
    if (list == NULL)
        exit(EXIT_FAILURE);

    struct sensor_ina226 *s = sensor_ina226_new();
    sensor_ina226_getlist(&s->data_list);
    strcpy(s->base.name, "sensor_ina226");

    if (list_empty(&s->data_list)) {
        free(s);
    } else {
        list_add(&s->base.list, list);
    }
    // // used just to write the CSV header line
    struct ina226_data *data;
    // uncomment this code if you want to print only few columns in the CSV
    /*
    list_for_each_entry(data, &s->data_list, list) {
        if (data->rail == VCCINT || data->rail == VCCBRAM || data->rail == VCCPSPLL || data->rail == VCCPSDDR || data->rail == VCCPSDDRPLL){
            // convert the linename into railname
            // struct string_pair *data_names;
            for(int i=0; i<PL_MAX; ++i) {
                if (strcmp(data->linename, ina226_lines[i].linename) == 0){
                    printf("%s,", ina226_lines[i].railname);
                }
            }
            // printf("%s,", data->linename);
        }        
    }
    */
    // make sure it prints PS and PL rails nicely grouped
    list_for_each_entry(data, &s->data_list, list) {
        for(int i=PS_MIN; i<PS_MAX; ++i) {
            if (strcmp(data->linename, ina226_lines[i].linename) == 0){
                printf("%s,", ina226_lines[i].railname);
            }
        }
    }
    list_for_each_entry(data, &s->data_list, list) {
        for(int i=PL_MIN; i<PL_MAX; ++i) {
            if (strcmp(data->linename, ina226_lines[i].linename) == 0){
                printf("%s,", ina226_lines[i].railname);
            }
        }
    }    
    //PS power: from VCCPSINTFP to VCCPSDDRPLL, == PS_MIN to PS_MAX
    //PL power: from     VCCINT to MGTAVTT, == PL_MIN to PL_MAX
    printf("total_ps_power, total_pl_power,total_power\n");

    return list;
}

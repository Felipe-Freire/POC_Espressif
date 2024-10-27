#ifndef COMMS_DATA_H_
#define COMMS_DATA_H_

#include <stdint.h>

struct comms_data_motion {
    float front_left;
    float front_right;
    float rear_left;
    float rear_right;
    float kick;
    uint32_t id;
};

struct comms_data_electronic {
    float battery;
    float voltage;
    float temperature;
};

#endif /* COMMS_DATA_H_ */

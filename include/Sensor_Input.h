//
// Created by Dat on 5/13/17.
//

#ifndef UNSCENTEDKF_SENSOR_INPUT_H
#define UNSCENTEDKF_SENSOR_INPUT_H
#include "Dense"

enum class SensorType{
    LASER,
    RADAR
};

class SensorInput{
    public:
        long long       timestamp_;
        Eigen::VectorXd data_;
        SensorType      sensor_type_;
};
#endif //UNSCENTEDKF_SENSOR_INPUT_H

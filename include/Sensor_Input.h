//
// Created by Dat on 5/13/17.
//

#ifndef UNSCENTEDKF_SENSOR_INPUT_H
#define UNSCENTEDKF_SENSOR_INPUT_H
#include "Dense"

class SensorInput{
    public:
        long long       timestamp_;
        Eigen::VectorXd data_;

        enum SensorType{
            LASER,
            RADAR
        } sensor_type_;
};
#endif //UNSCENTEDKF_SENSOR_INPUT_H

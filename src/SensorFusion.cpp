//
// Created by dat on 5/17/17.
//
#include "SensorFusion.h"

#include <iostream>
/**
 * Constructor
 * @param new_input
 */
SensorFusion::SensorFusion(KalmanFilter* filter){
    filter_ = filter;
}
void SensorFusion::Process(const Sensor& new_input) {

    /**
      TODO:
      */
    if (!initialized_) {
        filter_->initialize(new_input);
        prev_time_us_ = new_input.timestamp_;
        initialized_ = true;
    }
    else{
        float delta_t = (float)((new_input.timestamp_ - prev_time_us_) / 1000000.0);
        if (delta_t < 0.0001)   // zero-check
            return;
        prev_time_us_ = new_input.timestamp_;


        // std::cout <<" Time step: " << delta_t << std::endl;
        /*****************************************************************************
        *  Prediction
        ****************************************************************************/
        while(delta_t > 0.01){
            filter_->Predict(delta_t);
            delta_t -= 0.01;
        }
        filter_->Predict(delta_t);

        /*****************************************************************************
        *  Update
        ****************************************************************************/
        if (new_input.sensor_type_== SensorType::RADAR)
            filter_->Update(new_input);


        /*****************************************************************************
        *  Update Normalized Innovation Squared error
        ****************************************************************************/\

    }
}
/**
* Calculate Normalized Innovation Squared of a new measurement
*
* Formula: Error = (Z_mea - Z_pred)^T * S^-1 * (Z_mea - Z_pred)
*/
double SensorFusion::calculate_NIS(const Sensor &measured) {
    double  NIS = 0.0;
    Eigen::VectorXd z_diff = measured.data_ - z_pred_;
    NIS = z_diff.transpose()*S_*z_diff;
    return NIS;
}


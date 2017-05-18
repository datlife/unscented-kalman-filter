//
// Created by dat on 5/17/17.
//
#include "SensorFusion.h"

/**
 * Constructor
 * @param new_input
 */
SensorFusion::SensorFusion(KalmanFilter* filter){
    filter_ = filter;
}
void SensorFusion::Process(const Sensor& new_input) {

    if (initialized_ != true) {

    }
    else{
            float delta_time = (float) ((new_input.timestamp_ - prev_time_step_) / 1000000.0);
            if (delta_time < 0.0001)
                return;
            prev_time_step_ = new_input.timestamp_;

            filter_->Predict(delta_time);
            filter_->Update(new_input);
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


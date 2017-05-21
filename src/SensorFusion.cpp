//
// Created by dat on 5/17/17.
//
#include "SensorFusion.h"

#include <iostream>
/**
 * Constructor
 * @param new_input
 */
SensorFusion::SensorFusion(KalmanFilterBase* filter): filter_(filter){

}

/***
 * Driver for Sensor Fusion.
 * @param new_input Input Signal from Sensor
 *
 * Return: Updated State
 */
void SensorFusion::Process(const Sensor& new_input) {

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

        /*****************************************************************************
        *  Prediction
        ****************************************************************************/
        while(delta_t > 0.05){
           filter_->Predict(delta_t);
            delta_t -= 0.05;
        }
        filter_->Predict(delta_t);

        /*****************************************************************************
        *  Update
        ****************************************************************************/
        //if (new_input.sensor_type_== SensorType::RADAR)
            filter_->Update(new_input);
    }
}
/**
* Calculate Normalized Innovation Squared of a new measurement
*
* Formula: Error = (Z_mea - Z_pred)^T * S^-1 * (Z_mea - Z_pred)
*/
double SensorFusion::calculate_NIS() const {
    // Has been calculated during update step
    return filter_->getNIS();
}

/**
* Calculate Root Mean Squared Error
*
* Formula: Error = sqrt((X_estimated - X_gt)^2)
*/
Eigen::VectorXd SensorFusion::calculate_RMSE(const std::vector<Eigen::VectorXd> &estimations,
                                             const std::vector<Eigen::VectorXd> &ground_truth){

    int size = 4;
    // Measure error of postion and velocity
    Eigen::VectorXd rmse = Eigen::VectorXd::Zero(size);

    // Check the validity of the following inputs:
    if (estimations.size() == 0 ){
        std::cout << "No estimation is found.\n";
        return rmse;
    }

    // The estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()){
        std::cout  << "Size mismatched between estimation and ground truth\n";
        return rmse;
    }

    // Accumulate squared residuals
    for(int i=0; i <estimations.size(); ++i){
        // Calculate x - x_true
        Eigen::VectorXd residual = estimations[i].head(size) - ground_truth[i].head(size);

        //coefficient-wise multiplication : (x - x_true)^2
        residual = residual.array()*residual.array();

        rmse += residual;
    }
    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();
    //return the result
    return rmse;
}

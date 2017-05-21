//
// Created by dat on 5/20/17.
//
#include <iostream>
#include "EKF.h"

EKF::EKF(){
    nx_       = 4;

    x_        = Eigen::VectorXd::Zero(nx_);
    P_        = Eigen::MatrixXd::Zero(nx_, nx_);
    F_        = Eigen::MatrixXd::Zero(nx_, nx_);
    Q_        = Eigen::MatrixXd::Zero(nx_, nx_);

    noise_ax_ = 9;
    noise_ax_ = 9;

    R_laser   = Eigen::MatrixXd::Zero(2, 2);
    R_radar   = Eigen::MatrixXd::Zero(3, 3);
    H_laser   = Eigen::MatrixXd::Zero(2, nx_);
    H_radar   = Eigen::MatrixXd::Zero(3, nx_);
}

void EKF::initialize(const Sensor& new_input){

    // (0,2) and (1, 3) need to be updated every time step = Delta_T
    F_ << \
       1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;

    P_.diagonal()      << 1, 1, 1, 1;
    R_laser.diagonal() << 0.0225, 0.0225;
    R_radar.diagonal() << 0.09, 0.0009, 0.09;
    H_laser << 1, 0, 0, 0,
               0, 1, 0 ,0;

    switch(new_input.sensor_type_){
        case SensorType::RADAR:{
            double rho   = new_input.data_(0);
            double theta = new_input.data_(1);
            x_ << rho*sin(theta), rho*cos(theta)*-1, 0, 0;
            break;
        }
        case SensorType::LASER:{
            double px = new_input.data_(0);
            double py = new_input.data_(1);
            x_ << px, py, 0 ,0;
            break;
        }
    }

}
void EKF::Predict(double delta_time){

    updateTimePeriod(delta_time);
    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}
void EKF::Update(const Sensor& new_input){

    int size = new_input.data_.rows();
    SensorType type = new_input.sensor_type_;

    /**
     * *******************************************************************************
     * Transform X_ from State to Measurement Space
     * *******************************************************************************
     */
    // Convert space --> measurement space
     Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(size);
    switch(type){
        case SensorType::RADAR: {
            // Update Jacobian Matrix
            calculateJacobian();

            double rho     = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
            double theta   = atan2(x_(1), x_(0));
            double rho_dot = 0.0;

            if (fabs(rho) > 0.0001) rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho; // rhod = (px*vx + py*vy)/rho
            // Normalize angle
            while (z_pred(1) < -M_PI) z_pred(1) += 2 * M_PI;
            while (z_pred(1) > M_PI)  z_pred(1) -= 2 * M_PI;
            z_pred << rho, theta, rho_dot;
            break;
        }
        case SensorType::LASER:{
            z_pred = H_laser*x_;
            break;
        }
    }

    /**
    * *******************************************************************************
    * Calculate Kalman Gain
    * *******************************************************************************
    */
    Eigen::VectorXd z_diff = new_input.data_ - z_pred;
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(size, size);
    Eigen::MatrixXd H = (type==SensorType::RADAR) ? H_radar : H_laser;

    if (type == SensorType::RADAR){
        while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
        while (z_diff(1) > M_PI)  z_diff(1) -= 2 * M_PI;
    }

    S = H*P_*H.transpose() + ((type==SensorType::RADAR)?R_radar:R_laser);

    // KALMAN GAIN
    Eigen::MatrixXd K = (P_*H.transpose())*S.inverse();

    /**
    * *******************************************************************************
    * Update State
    * *******************************************************************************
    */
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nx_, nx_);
    x_ = x_ + K*z_diff;
    P_ = (I - K*H)*P_;

}

void EKF::updateTimePeriod(double &new_delta_t){
    float dt   = new_delta_t;
    float dt_2 = dt*dt;
    float dt_3 = dt*dt_2;
    float dt_4 = dt*dt_3;

    F_(0, 2) = F_(1, 3) = dt;

    Q_ << \
    noise_ax_*(dt_4/4) ,0 ,                 noise_ax_*(dt_3/2), 0,
    0,                  noise_ay_*(dt_4/4), 0,                  noise_ay_*(dt_3/2),
    noise_ax_*(dt_3/2), 0,                  noise_ax_*(dt_2),   0,
    0,                  noise_ay_*(dt_3/2), 0,                  noise_ay_*(dt_2);

}
/**
 * For Radar
 */
void EKF::calculateJacobian(){

    float px = x_(0);
    float py = x_(1);
    float distance = sqrt(px*px + py*py);
    if (distance < 0.000001){     //check division by zero
        std::cout << "CalculateJacobian() error - division is zero  at...." << std::endl;
        H_radar.setZero();
        return;
    }

    //compute the Jacobian matrix
    float vx = x_(2);
    float vy = x_(3);

    // Watch : Linearization non-linear models to understand Taylor series first
    // Range Rho
    float d_rho_posx = px / distance;
    float d_rho_posy = py / distance;

    // Angle thea
    float d_theta_posx = - py / (distance*distance);
    float d_theta_posy =   px / (distance*distance);

    // Range Rate:
    float d_rho_rate_posx = py*(vx*py - vy*px)/(distance*distance*distance);
    float d_rho_rate_posy = px*(vy*px - vx*py)/(distance*distance*distance);
    float d_rho_rate_velx = d_rho_posx;
    float d_rho_rate_vely = d_rho_posy;

    // Compute Jacobian matrix
    H_radar <<   \
            d_rho_posx,      d_rho_posy,      0,     0,
            d_theta_posx,    d_theta_posy,    0,     0,
            d_rho_rate_posx, d_rho_rate_posy, d_rho_rate_velx, d_rho_rate_vely;
}


double EKF::getNIS() const{
    return 0.0;
}


EKF::~EKF() {

}
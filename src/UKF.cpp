#include "UKF.h"
#include "Tools.h"
#include "Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

/**
TODO:

Complete the initialization. See ukf.h for other member properties.

Hint: one or more values initialized above might be wildly off...
*/
    is_initialized_ = false;
    use_laser_      = true;         // if this is false, laser measurements will be ignored (except during init)
    use_radar_      = true;         // if this is false, radar measurements will be ignored (except during init)

    n_x_            = 5;            // UKF state dimension          (px, py, v, yaw, yaw_rate)
    n_aug_          = 7;            // UKF Augmented State Dimension(px, py, v, yaw, yaw_rate, noise_a, noise_y)

    lambda_         = 3 - n_x_;     // Scaling factor for sigma points
    n_sig_pts       = 2*n_aug_ + 1; // Formula to calculate number of sigma points
    prev_time_us_        = 0;

    std_a_          = 30;           // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_      = 30;           // Process noise standard deviation yaw acceleration in rad/s^2

    std_laspx_      = 0.15;         // Laser measurement noise standard deviation position1 in m
    std_laspy_      = 0.15;         // Laser measurement noise standard deviation position2 in m

    std_radr_       = 0.3;          // Radar measurement noise standard deviation radius in m
    std_radphi_     = 0.03;         // Radar measurement noise standard deviation angle in rad
    std_radrd_      = 0.3;          // Radar measurement noise standard deviation radius change in m/s


    // State
    x_                = VectorXd(n_x_);                // initial state vector
    P_                = MatrixXd(n_x_,   n_x_);        // initial covariance matrix
    P_aug_            = MatrixXd(n_aug_, n_aug_);      // initial augmented covariance matrix
    Xsig_pred_        = MatrixXd(n_x_,   n_sig_pts);   // initial Sigma Points Matrix
    Z_radar_sig_pred_ = MatrixXd(3,      n_sig_pts);   // Sigma Pts in Radar Measurement Space
    Z_laser_sig_pred_ = MatrixXd(3,      n_sig_pts);   // Sigma Pts in Radar Measurement Space

    // Noises
    Q_       = MatrixXd(2, 2);     // Simplified because of using Augmented State
    R_LiDar_ = MatrixXd(2, 2);
    R_RaDar_ = MatrixXd(3, 3);
}

/**
* ProcessMeasurement
*
* Wrapper of Prediction and Update steps.
* This function will decide to call UpdateLiDar or UpdateRaDar
*
* @param  The latest measurement data of either radar or laser
*/
void UKF::ProcessMeasurement(SensorInput new_input) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    if (!is_initialized_){
        Initialize_State(new_input);
    }
    else{
        double delta_t = (new_input.timestamp_ - prev_time_us_) / 10000;
        if (delta_t < 0.0001)   // zero-check
            return;
        /*****************************************************************************
        *  Prediction
        ****************************************************************************/
        Predict(delta_t, new_input.sensor_type_);

        /*****************************************************************************
        *  Update
        ****************************************************************************/
        if(new_input.sensor_type_ == SensorType::LASER)
            UpdateLidar(new_input);
        else
            UpdateRadar(new_input);

    }
}
void UKF::Initialize_State(const SensorInput &new_input) {
    std::cout<<"Initializing UKF... \n";

    VectorXd measurement = new_input.data_;
    // @TODO: find a correct initialization
    // R/L --> state space
    if (new_input.sensor_type_ == SensorType::RADAR){
        double rho = measurement(0);
        double phi = measurement(1);
        double rho_dot = measurement(2);
        x_ << rho*cos(phi), rho*sin(phi), 0, phi, 0;
    }
    else{
        x_ << measurement(0), measurement(1), 0, 0, 0;
    }
    P_ <<   1000, 0, 0, 0, 0,
            0, 1000, 0, 0, 0,
            0, 0, 1000, 0, 0,
            0, 0, 0, 1000, 0,
            0, 0, 0, 0, 1000;

    Q_  << std_a_*std_a_,         0,
           0, std_yawdd_*std_yawdd_;

    R_LiDar_ << std_laspx_*std_laspx_,   0,
                0  , std_laspy_*std_laspy_;

    R_RaDar_ << std_radr_*std_radr_, 0,     0,
                0, std_radphi_*std_radphi_, 0,
                0,   0, std_radrd_*std_radrd_;

    prev_time_us_ = new_input.timestamp_;
    is_initialized_ = true;
}
/**
 * Predicts sigma points, the state, and the state covariance matrix.
 *
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Predict(double delta_t, SensorType sensor_type) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
    MatrixXd SigmaPts(n_aug_, n_sig_pts);
    VectorXd x_aug(n_aug_);

    // Init Augmented Covariance matrix
    P_aug_.topLeftCorner(n_x_, n_x_) = P_;
    P_aug_.bottomRightCorner(2, 2)   = Q_;

    // UKF Augmentation: Convert from state --> augmented state
    x_aug.head(n_x_) = x_;
    x_aug.tail(2) << 0.0, 0.0;  // noise of mean state is zero

    // Set the first element of Sigma Point Matrix to current state
    SigmaPts.col(0) = x_aug;

    // Square root of P
    MatrixXd A = P_aug_.llt().matrixL();
    /*****************************************************************************
    *  1. Generate Augmented Sigma Points
    ****************************************************************************/
    for (int i = 0; i < n_aug_; i++){
        // Calculate sigma points
        SigmaPts.col(i + 1)          = x_aug + sqrt(lambda_ + n_x_)*A.col(i);
        SigmaPts.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_x_)*A.col(i);
    }

    /*****************************************************************************
    *  2. Predict Sigma Points State
    ****************************************************************************/
    for (int i = 0; i < n_sig_pts; i++){
        VectorXd sigma_point = SigmaPts.col(i);

        // Predict next state of sigma point
        PredictSigmaPoint(sigma_point, delta_t, i);

        // Saved in Z_[sensortype]_sig_pred
        ConvertStateToMeasurement(sigma_point, sensor_type);
    }

    /*****************************************************************************
    *  3. Calculate measurement mean and covariance
    ****************************************************************************/
    //

}

/**
 * ----------------------------------------------------------------------------------------------------
 * PredictSigmaPoint()
 * ---------------------------------------------------------------------------------------------------
 * Idea: Linearize Non-linear model.
 * Instead of converting a whole covariance shape. We only predict based on the current sigma points.
 * new_sigma = current_sigma + rate_of_change + noise
 *
 * new_sigma = posx          + d_posx         + 0.5*noise_accer*cos(yaw)*(dt)^2
 *             posy          + d_posy         + 0.5*noise_accer*sin(yaw)*(dt)^2
 *             vel           + 0              + noise_accer*dt
 *             yaw           + d_yaw*dt       + 0.5*noise_dd_yaw*(dt)^2
 *             yaw_rate      + 0              + noise_dd_yaw*dt
 * */
void UKF::PredictSigmaPoint(VectorXd &sigma_pt, const double &delta_t, int i){

    VectorXd rate_of_change(5);
    VectorXd noise(5);

    double dt2 = delta_t*delta_t;
    // Generate curr state values for read-ability
    double p_x       = sigma_pt[0]; // position in x-plane
    double p_y       = sigma_pt[1]; // position in y-plane
    double velocity  = sigma_pt[2];
    double yaw       = sigma_pt[3]; // orientation (yaw)
    double yawd      = sigma_pt[4]; // orientation rate
    double nu_a      = sigma_pt[5]; // process noise of acceleration
    double nu_yawdd  = sigma_pt[6]; // process noise of yaw
    double px_rate, py_rate;

    if(fabs(yawd) > 0.001) {    //avoid division by zero
        px_rate = velocity/yawd*(sin(yaw + yawd*delta_t) - sin(yaw));
        py_rate = velocity/yawd*(-cos(yaw + yawd*delta_t) + cos(yaw));
    }
    else{
        px_rate = velocity*cos(yaw)*delta_t;
        py_rate = velocity*sin(yaw)*delta_t;
    }

    rate_of_change << px_rate,
                      py_rate,
                      0,
                      yawd*delta_t,
                      0;

    noise <<0.5  * nu_a*cos(yaw) * dt2,
            0.5  * nu_a*sin(yaw) * dt2,
            nu_a * delta_t,
            0.5  * nu_yawdd * dt2,
            nu_yawdd * delta_t;

    //write predicted sigma points into right column
    Xsig_pred_.col(i) = sigma_pt.head(n_x_) + rate_of_change + noise;
}
/**
 * Helper of Predict()
 * @param {MeasurementPackage} meas_package
 */
void UKF::ConvertStateToMeasurement(VectorXd &sigma_pt, SensorType type){

    switch(type){
        case SensorType::LASER:
            //Save Z_laser_sig_pred
            break;
        case SensorType::RADAR:
            // save Z_radar_sig_pred
            break;
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(SensorInput meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {SensorInput} meas_package
 */

void UKF::UpdateRadar(SensorInput meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}


UKF::~UKF() {}

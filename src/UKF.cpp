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
    is_initialized_ = false;
    use_laser_      = true;         // if this is false, laser measurements will be ignored (except during init)
    use_radar_      = true;         // if this is false, radar measurements will be ignored (except during init)

    n_x_            = 5;            // UKF state dimension          (px, py, v, yaw, yaw_rate)
    n_aug_          = 7;            // UKF Augmented State Dimension(px, py, v, yaw, yaw_rate, noise_a, noise_y)

    lambda_         = 3 - n_x_;     // Scaling factor for sigma points
    n_sig_pts       = 2*n_aug_ + 1; // Formula to calculate number of sigma points
    time_us_        = 0;

    std_a_          = 30;           // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_      = 30;           // Process noise standard deviation yaw acceleration in rad/s^2

    std_laspx_      = 0.15;         // Laser measurement noise standard deviation position1 in m
    std_laspy_      = 0.15;         // Laser measurement noise standard deviation position2 in m

    std_radr_       = 0.3;          // Radar measurement noise standard deviation radius in m
    std_radphi_     = 0.03;         // Radar measurement noise standard deviation angle in rad
    std_radrd_      = 0.3;          // Radar measurement noise standard deviation radius change in m/s


    // State
    x_                = VectorXd(n_x_);                // initial state vector
    P_                = MatrixXd(n_x_, n_x_);          // initial covariance matrix
    P_aug_            = MatrixXd(n_aug_, n_aug_);      // initial augmented covariance matrix
    Xsig_pred_        = MatrixXd(n_sig_pts, n_aug_);   // initial Sigma Points Matrix
    Z_radar_sig_pred_ = MatrixXd(n_sig_pts, 3);        // Sigma Pts in Radar Measurement Space
    Z_laser_sig_pred_ = MatrixXd(n_sig_pts, 3);        // Sigma Pts in Radar Measurement Space

    // Noises
    Q_       = MatrixXd(2, 2);     // Simplified because of using Augmented State
    R_LiDar_ = MatrixXd(2, 2);
    R_RaDar_ = MatrixXd(3, 3);
    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
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
        std::cout<<"Initializing UKF... \n";
        // Assert dimension
        VectorXd measurement = new_input.data_;

        // @TODO: find a correct initialization
        // R/L --> state space
        if (new_input.sensor_type_ == RADAR)
            x_ << measurement(0), measurement(1), 0, 0 ,0;
        else{
            x_ << measurement(0), measurement(1), 0, 0, 0;
        }
        P_ << 1000, 0, 0, 0, 0,
              0, 1000, 0, 0, 0,
              0, 0, 1000, 0, 0,
              0, 0, 0, 1000, 0,
              0, 0, 0, 0, 1000;

        // Init Noise -- should it be at initialize UKF?

        Q_       << std_a_*std_a_,         0,
                    0, std_yawdd_*std_yawdd_;

        R_LiDar_ << std_laspx_*std_laspx_,   0,
                    0  , std_laspy_*std_laspy_;

        R_RaDar_ << std_radr_*std_radr_, 0,     0,
                    0, std_radphi_*std_radphi_, 0,
                    0,   0, std_radrd_*std_radrd_;

        time_us_ = new_input.timestamp_;
        is_initialized_ = true;
    }
    else{
        std::cout<<"Initialized. \n";
        double delta_t = (new_input.timestamp_ - time_us_) / 10000;
        if (delta_t < 0.0001)   // zero-check
            return;

        Predict(delta_t);

        // Update
        if(new_input.sensor_type_ == LASER)
            UpdateLidar(new_input);
        else
            UpdateRadar(new_input);

    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 *
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Predict(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

    // Init Augmented Covariance matrix
    P_aug_.topLeftCorner(n_x_, n_x_) = P_;
    P_aug_.bottomRightCorner(2, 2)   = Q_;

    // Set the first element of Sigma Point Matrix to current state
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(5) = x_;
    x_aug.tail(2) << 0.0, 0.0;

    Xsig_pred_.col(0) = x_aug;

    for (int i = 1; i < n_sig_pts; i++){
        // Calculate sigma points
        VectorXd sigma_pt = VectorXd(n_aug_);

        // Predict new sigma point
        PredictSigmaPoint(sigma_pt);

        // Transform to Measurement Space -- Saved in ZSig_Pred
        ConvertStateToMeasurement(sigma_pt, RADAR);
    }
    // Calculate measurement mean and covariance

}


void UKF::PredictSigmaPoint(VectorXd &sigma_pt){

}
void UKF::ConvertStateToMeasurement(VectorXd &sigma_pt, SensorType type){

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
 * @param {MeasurementPackage} meas_package
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

#include "UKF.h"
#include "Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Helper function to Normalize Angle from [-2 * PI, 2 * PI]
 */
void UKF::normalize_angle(double &angle){
   atan2(sin(angle), cos(angle));
}

/**
 * Unscented Kalman Filter Constructor
 */
UKF::UKF() {
    n_x_        = 5;                                     // UKF state dimension          (px, py, v, yaw, yaw_rate)
    n_aug_      = 7;                                     // UKF Augmented State Dimension(px, py, v, yaw, yaw_rate, noise_a, noise_y)
    lambda_     = 3 - n_aug_;                            // Scaling factor for sigma points

    // State
    x_          = VectorXd::Zero(n_x_);                  // initial state vector
    P_          = MatrixXd::Zero(n_x_,   n_x_);          // initial covariance matrix
    Q_          = MatrixXd::Zero(2, 2);                  // Simplified because of using Augmented State

    std_a_      = 9;                                     // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_  = 2*M_PI;                                // Process noise standard deviation yaw acceleration in rad/s^2

    std_lasx    = 0.15;
    std_lasy    = 0.15;

    std_radr    = 0.3;                                   // Radar measurement noise standard deviation radius in m
    std_radphi  = 0.03;                                // radar measurement noise standard deviation angle in rad
    std_radrd   = 0.01;                                  // radar measurement noise standard deviation radius change in m/s

    R_lidar     = MatrixXd::Zero(2, 2);
    R_radar     = MatrixXd::Zero(3, 3);

    weights_    = VectorXd::Zero(2*n_aug_ + 1);
    Xsig_pred_  = MatrixXd::Zero(n_x_,   2*n_aug_ + 1);     // initial Sigma Points Matrix
}
/**
 * Initializes Unscented Kalman filter
 */
void UKF::initialize(const Sensor &new_input) {

    // Initialize Process Covariance and Noise Matrices
    P_.diagonal()      << 0.01, 0.01, 0.01, 0.001, 0.01;
    Q_.diagonal()      << pow(std_a_, 2), pow(std_yawdd_, 2);
    R_lidar.diagonal() << pow(std_lasx, 2), pow(std_lasy, 2);
    R_radar.diagonal() << pow(std_radr, 2), pow(std_radphi, 2), pow(std_radrd, 2);

    // Weight for Sigma Points Calculation
    weights_(0) = lambda_/(lambda_ + n_aug_);

    for (int i = 1; i< 2*n_aug_ + 1; i++) {
        weights_(i) = 0.5/(n_aug_ + lambda_);
    }

    // Initialize the First State
    VectorXd measurement = new_input.data_;
    switch(new_input.sensor_type_)
    {
        case SensorType::RADAR: {
            double rho = measurement(0);
            double phi = measurement(1);
            x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
        }
            break;
        case SensorType::LASER:
            x_ << measurement(0), measurement(1), 0, 0, 0;
            break;
    }

}
void UKF::Predict(double delta_t){

    /*****************************************************************************
    *  1. Generate Augmented Sigma Points
    ****************************************************************************/
    MatrixXd SigmaPts = GenerateSigmaPoints();

    /*****************************************************************************
    *  2. Predict Sigma Points
    ****************************************************************************/
    for (int i = 0; i < 2*n_aug_ + 1; i++){        // How to Parallel?
        Xsig_pred_.col(i) = PredictSigmaPoint(SigmaPts.col(i), delta_t);
    }

    /*****************************************************************************
    *  3. Predict State Mean and Covariance
    ****************************************************************************/
    CalculateMeanAndCovariance();

}

void UKF::linear_update(const Sensor &new_input, const VectorXd z_mean){

    int size = new_input.data_.rows();
    /**
   * *******************************************************************************
   * Calculate Kalman Gain
   * *******************************************************************************
   */
    VectorXd z_diff = new_input.data_ - z_mean;
    MatrixXd S = MatrixXd::Zero(size, size);
    MatrixXd H = MatrixXd::Zero(size, n_x_);
    H << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0;

    S = H*P_*H.transpose() + R_lidar;


    Eigen::MatrixXd K = (P_*H.transpose())*S.inverse();

    /**
    * *******************************************************************************
    * Update State
    * *******************************************************************************
    */
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_x_, n_x_);
    x_ = x_ + K*z_diff;
    P_ = (I - K*H)*P_;

    /*****************************************************************************
     *  Update Normalized Innovation Squared error
     ****************************************************************************/
    nis_ = z_diff.transpose()*S.inverse()*z_diff;
}
void UKF::Update(const Sensor& new_input){

    int size        = new_input.data_.rows();
    SensorType type = new_input.sensor_type_;
    /**
     * *******************************************************************************
     * Transform to Measurement Predicted Mean & Covariance
     * *******************************************************************************
     */
    VectorXd z_mean = ConvertToMeasurement(x_, new_input.sensor_type_);

    if (type == SensorType::LASER) {
        linear_update(new_input, z_mean);       // In order to improve performance
        return;
    }

    MatrixXd Z_sigma_pred(size, 2*n_aug_ + 1);
    for( int i = 0; i < Xsig_pred_.cols(); i++){
        Z_sigma_pred.col(i) = ConvertToMeasurement(Xsig_pred_.col(i), new_input.sensor_type_);
    }

    /**
     * *****************************************************************************
     * Calculate Cross-Translation Matrix (upper top of Kalman Gain)
     * *****************************************************************************
     */
    // Sensor Noise Covariance Matrix
    MatrixXd S  = MatrixXd::Zero(size, size);
    MatrixXd Tc = MatrixXd::Zero(n_x_, size);

    // Calculate Cross-Correlation matrix
    for (int i = 0; i < 2*n_aug_ + 1; i++) {
        VectorXd z_diff = Z_sigma_pred.col(i) - z_mean;        // Residual
        VectorXd x_diff = Xsig_pred_.col(i) - x_;              // State difference

        if (type == SensorType::RADAR)
            normalize_angle(z_diff(1));
        normalize_angle(x_diff(1));

        S  += weights_(i) * z_diff * z_diff.transpose();
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    S = S + ((type==SensorType::RADAR)? R_radar : R_lidar);     // Add sensor noise

    /**
     * *****************************************************************************
     * Calculate Kalman Gain
     * *****************************************************************************
     */
    MatrixXd K = Tc * S.inverse();

    /**
     * *****************************************************************************
     * Update State and Covariance Matrix
     * *****************************************************************************
     */
    VectorXd measurement = new_input.data_;

    // Residual
    VectorXd z_diff = measurement - z_mean;
    if (new_input.sensor_type_ == SensorType::RADAR){
        normalize_angle(z_diff(1));
    }

    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

    normalize_angle(x_(3));


    /*****************************************************************************
    *  Update Normalized Innovation Squared error
    ****************************************************************************/
    nis_ = z_diff.transpose()*S.inverse()*z_diff;
}


/**
 * ----------------------------------------------------------------------------------------------------
 * GenerateSigmaPoints()
 * ---------------------------------------------------------------------------------------------------
 * Helper function for Prediction Step
 *      Number of sigma points = 2 * state_dimension + 1

 * */
MatrixXd UKF::GenerateSigmaPoints(){

    MatrixXd SigmaPts = MatrixXd::Zero(n_aug_, 2*n_aug_ + 1);
    VectorXd x_aug    = VectorXd::Zero(n_aug_);
    MatrixXd P_aug    = MatrixXd::Zero(n_aug_, n_aug_);

    // Initialize  Augmented State & Covariance matrix
    P_aug.topLeftCorner(P_.cols(), P_.rows()) = P_;
    P_aug.bottomRightCorner(Q_.cols(), Q_.rows()) = Q_;

    // Square root of P
    MatrixXd L = P_aug.llt().matrixL();

    x_aug.head(n_x_) = x_;
    SigmaPts.col(0)  = x_aug;

    // Calculate sigma points
    for (int i = 0; i < n_aug_; i++){
        SigmaPts.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_)*L.col(i);
        SigmaPts.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*L.col(i);
    }

    return SigmaPts;
}

/**
 * ----------------------------------------------------------------------------------------------------
 * PredictSigmaPoint()
 * ---------------------------------------------------------------------------------------------------
 * Idea: Linearize Non-linear model.
 *      Instead of converting a whole covariance shape.
 *      We only predict based on the current sigma points.
 *      new_sigma = current_sigma + rate_of_change + noise

 * */
VectorXd UKF::PredictSigmaPoint(const VectorXd &sigma_pt, const double &delta_t){

    // Predicted Sigma Point
    VectorXd x_sigma_pts = VectorXd::Zero(n_x_);

    // Generate curr state values for read-ability
    double px_rate, py_rate;
    double dt2 = delta_t*delta_t;

    const double px   = sigma_pt(0);
    const double py   = sigma_pt(1);
    const double vel  = sigma_pt(2);
    const double yaw  = sigma_pt(3);
    const double yawd = sigma_pt(4);
    const double nu_a = sigma_pt(5);
    const double nu_yawdd = sigma_pt(6);

    if(fabs(yawd) > 0.0001) {                                    //avoid division by zero
        px_rate = vel/yawd*(sin(yaw + yawd*delta_t) - sin(yaw));
        py_rate = vel/yawd*(-cos(yaw + yawd*delta_t) + cos(yaw));
    }
    else{
        px_rate = vel*cos(yaw)*delta_t;
        py_rate = vel*sin(yaw)*delta_t;
    }

    //CRTV Model
    x_sigma_pts << \
        px   +    px_rate      + 0.5  * nu_a*cos(yaw) * dt2,
        py   +    py_rate      + 0.5  * nu_a*sin(yaw) * dt2,
        vel  +    0            + nu_a * delta_t,
        yaw  +    yawd*delta_t + 0.5  * nu_yawdd * dt2,
        yawd +    0            + nu_yawdd * delta_t;

    // Angle normalization
    normalize_angle(x_sigma_pts[3]);

    return x_sigma_pts;
}


/**
* Helper function in Prediction Step
*/
void  UKF::CalculateMeanAndCovariance(){

    VectorXd x_mean = VectorXd::Zero(n_x_);
    MatrixXd P      = MatrixXd::Zero(n_x_, n_x_);

    /******************************************
     * Calculate Mean State
     ******************************************/
    for(int i = 0; i < Xsig_pred_.cols(); i++){
        x_mean += weights_(i)*Xsig_pred_.col(i);
        normalize_angle(x_mean(3));
    }
    /******************************************
     * Calculate Process Covariance Matrix
     ******************************************/
    for(int i = 0; i < Xsig_pred_.cols(); i++){
        MatrixXd x_diff = Xsig_pred_.col(i) - x_mean; // <--- using this over x_mean makes code more robust.
        normalize_angle(x_diff(3));
        P += weights_(i)*x_diff*x_diff.transpose();
    }
    /******************************************
     * Update Predicted Mean & Covariance
     ******************************************/
    x_ = x_mean;
    P_ = P;

}


/**
 * Convert Sigma Points To Measurement Space
 * @param new_input
 * @return
 */
VectorXd UKF::ConvertToMeasurement(const VectorXd &sigma_pt,const SensorType &type){

    const double px   = sigma_pt(0);
    const double py   = sigma_pt(1);
    const double v    = sigma_pt(2);
    const double yaw  = sigma_pt(3);
    const double yawd = sigma_pt(4);

    VectorXd  result = (type==SensorType::LASER) ? VectorXd::Zero(2) : VectorXd::Zero(3);
    switch(type)
        {
            case SensorType::LASER:
                result << px, py;
                break;

            case SensorType::RADAR:
                double rho      = sqrt(px*px + py*py);
                double phi      = atan2(py, px);
                double rho_dot  =  0.0;

                if (fabs(rho) > 0.001)
                    rho_dot = (px*(v*cos(yaw)) + py*(v*sin(yaw)))/rho;

                result << rho, phi, rho_dot;
                break;
        }
    return result;
}


/**
 * Destructor
 */
UKF::~UKF() {}
KalmanFilterBase::~KalmanFilterBase(){

}

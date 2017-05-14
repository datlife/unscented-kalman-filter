#ifndef UKF_H
#define UKF_H

#include "Sensor_Input.h"
#include "Dense"
#include <vector>
#include <string>
#include <fstream>
#include "Tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Unscented Kalman Filter Object
 *
 * Fuse LiDar and RaDar Sensor Input with CRTV model (Constant Turn Rate and Velocity Driving Model)
 */
class UKF {
    public:
        int         n_x_;             ///* State dimension
        int         n_aug_;           ///* Augmented state dimension
        double      lambda_;          ///* Sigma point spreading parameter
        long long   time_us_;         ///* time when the state is true, in us

        VectorXd    x_;              ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        MatrixXd    P_;              ///* state covariance matrix
        MatrixXd    Xsig_pred_;      ///* predicted sigma points matrix
        VectorXd    weights_;        ///* Weights of sigma points


        double      std_a_;          ///* Process noise standard deviation longitudinal acceleration in m/s^2
        double      std_yawdd_;      ///* Process noise standard deviation yaw acceleration in rad/s^2

        double      std_laspx_;      ///* Laser measurement noise standard deviation position1 in m
        double      std_laspy_;      ///* Laser measurement noise standard deviation position2 in m

        double      std_radr_;      ///* Radar measurement noise standard deviation radius in m
        double      std_radphi_;    ///* Radar measurement noise standard deviation angle in rad
        double      std_radrd_ ;    ///* Radar measurement noise standard deviation radius change in m/s

        double      NIS_radar_;     ///* the current NIS for radar
        double      NIS_laser_;     ///* the current NIS for laser

        bool        is_initialized_;  ///* initially set to false, set to true in first call of ProcessMeasurement
        bool        use_laser_;       ///* if this is false, laser measurements will be ignored (except for init)
        bool        use_radar_;       ///* if this is false, radar measurements will be ignored (except for init)

    /**
         * Constructor
         */
        UKF();

        /**
         * Destructor
         */
        virtual ~UKF();

        /**
         * ProcessMeasurement
         *
         * Wrapper of Prediction and Update steps.
         * This function will decide to call UpdateLiDar or UpdateRaDar
         * @param  The latest measurement data of either radar or laser
         */
        void ProcessMeasurement(SensorInput);


        /**
         * Prediction
         *
         * Predicts sigma points, the state, and the state covariance matrix
         * @param delta_t Time between k and k+1 in s
         */
        void Prediction(double delta_t);

        /**
         * Updates the state and the state covariance matrix using a laser measurement
         */
        void UpdateLidar(SensorInput);

        /**
         * Updates the state and the state covariance matrix using a radar measurement
         * @param  The measurement at k+1
         */
        void UpdateRadar(SensorInput);
};

#endif /* UKF_H */

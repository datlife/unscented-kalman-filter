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

    private:

        int         n_x_;             ///* State dimension
        int         n_aug_;           ///* Augmented state dimension
        int         n_sig_pts;        ///* Number of Sigma Points
        double      lambda_;          ///* Sigma point spreading parameter
        long long   prev_time_us_;         ///* time when the state is true, in us

        VectorXd    x_;              ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
        MatrixXd    P_;              ///* state covariance matrix
        MatrixXd    P_aug_;          ///* state covariance matrix

        MatrixXd    Q_;              ///* Process Covariance Noise Matrix (P = P_old + Q)
        MatrixXd    R_LiDar_;        ///* LiDar Sensor Noise Covariance Matrix (Used to update Kalman Gain)
        MatrixXd    R_RaDar_;        ///* RaDar Sensor Noise Covariance Matrix (_)

        MatrixXd    Xsig_pred_;       ///* [State Space] predicted sigma points matrix
        MatrixXd    Z_radar_sig_pred_;///* [Radar Measurement Space] predicted sigma points matrix
        MatrixXd    Z_laser_sig_pred_;///* [LiDar Measurement Space] predicted sigma points matrix
        VectorXd    weights_;         ///* Weights of sigma points


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
        * Prediction
        *
        * Predicts sigma points, the state, and the state covariance matrix
        * @param delta_t Time between k and k+1 in s
        */
        void Predict(double delta_t, SensorType);

        /**
        * Updates the state and the state covariance matrix using a laser measurement
        */
        void UpdateLidar(SensorInput);

        /**
        * Updates the state and the state covariance matrix using a radar measurement
        * @param  The measurement at k+1
        */
        void UpdateRadar(SensorInput);

        /**
         * Helper functions
         * */
        void Initialize_State(const SensorInput &first_input);
        void PredictSigmaPoint(VectorXd &sigma_pt, const double &delta_t, int position);
        void ConvertStateToMeasurement(VectorXd &sigma_pt, SensorType);

    public:
        /**
        * Constructor
        */
        UKF();

        /**
        * Getter Functions
        */
        VectorXd  getState()          const {return x_;}
        float     getState(int n)     const {return float(x_(n));}
        double    getNIS(SensorType s)const {return (s==SensorType::LASER) ? NIS_laser_ : NIS_radar_;}

        /**
        * ProcessMeasurement
        *
        * Wrapper of Prediction and Update steps.
        * Decide to call UpdateLiDar or UpdateRaDar
        *
        * @param  The latest measurement data of either radar or laser
        */
        void ProcessMeasurement(SensorInput);


        /**
        * Destructor
        */
        virtual ~UKF();
};

#endif /* UKF_H */

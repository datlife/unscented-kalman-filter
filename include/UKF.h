#ifndef UKF_H
#define UKF_H

#include "Sensor.h"
#include "Dense"
#include <vector>
#include <string>
#include <fstream>
#include "KalmanFilter.h"



/*
 * Unscented Kalman Filter Object
 *
 * Fuse LiDar and RaDar Sensor Input with CRTV model (Constant Turn Rate and Velocity Driving Model)
 */
class UKF : public KalmanFilter{

    private:

        int                n_x_;             ///* State dimension
        int                n_aug_;           ///* Augmented state dimension
        double             lambda_;          ///* Sigma point spreading parameter

        double             std_a_;           ///* Process noise standard deviation longitudinal acceleration in m/s^2
        double             std_yawdd_;       ///* Process noise standard deviation yaw acceleration in rad/s^2

        double             std_lasx;
        double             std_lasy;

        double             std_radr;         ///* Radar measurement noise standard deviation radius in m
        double             std_radphi;       ///* radar measurement noise standard deviation angle in rad
        double             std_radrd;        ///* radar measurement noise standard deviation radius change in m/s

        Eigen::MatrixXd    Xsig_pred_;       ///* Predicted sigma points matrix
        Eigen::VectorXd    weights_;         ///* Weights of sigma points
        Eigen::MatrixXd    R_lidar;
        Eigen::MatrixXd    R_radar;


    protected:
        virtual void    Predict(double delta_t);
        virtual void    Update(const Sensor&);

        void            normalize_angle(double &angle);

        /**
         * Helper Functions for Prediction Step
         */
        Eigen::MatrixXd GenerateSigmaPoints();
        Eigen::VectorXd PredictSigmaPoint(const Eigen::VectorXd &sigma_pt, const double &delta_t);
        void            CalculateMeanAndCovariance();

        /**
         * Helper Functions for Update Step
         */
        Eigen::VectorXd ConvertToMeasurement(const Eigen::VectorXd &, const SensorType &);
    public:
        /**
        * Constructor
        */
        UKF();

        virtual void initialize(const Sensor&);
        /**
        * Getter Functions
        */
        float     getState(int n)     const {return float(x_(n));}

        /**
        * Destructor
        */
        virtual ~UKF();
};

#endif /* UKF_H */

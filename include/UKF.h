#ifndef UKF_H
#define UKF_H

#include "Sensor.h"
#include "Dense"
#include <vector>
#include <string>
#include <fstream>
#include "KalmanFilterBase.h"



/*
 * Unscented Kalman Filter Object
 *
 * Fuse LiDar and RaDar Sensor Input with CRTV model (Constant Turn Rate and Velocity Driving Model)
 */
class UKF : public KalmanFilterBase{

    public:
        /**
        * Constructor
        */
        UKF();

        virtual void initialize(const Sensor&);
        virtual void    Predict(double delta_t);
        virtual void    Update(const Sensor&);
        /**
        * Getter Functions
        */
        virtual double    getNIS()            const {return nis_;}
        /**
        * Destructor
        */
        virtual ~UKF();

    protected:
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

        double             nis_;

        Eigen::MatrixXd    Xsig_pred_;       ///* Predicted sigma points matrix
        Eigen::VectorXd    weights_;         ///* Weights of sigma points
        Eigen::MatrixXd    R_lidar;
        Eigen::MatrixXd    R_radar;

};

#endif /* UKF_H */

//
// Created by dat on 5/17/17.
//

#ifndef UNSCENTEDKF_KALMANFILTER_H
#define UNSCENTEDKF_KALMANFILTER_H

#include <Dense>
#include "Sensor.h"
/**
 * Abstract Class of Kalman Filter
 *
 */
class KalmanFilterBase{

    public:
        virtual void   initialize(const Sensor&) = 0;
        virtual void   Predict(double delta_time) = 0;
        virtual void   Update(const Sensor&) = 0;
        virtual double getNIS() const = 0;

        Eigen::VectorXd getState()   const {return x_;}

        virtual ~KalmanFilterBase() = 0;
    protected:
        /**
         * --------------------------------------------
         * State Vector - Depended on Driving Model
         * --------------------------------------------
         * Example: CRTV Model - Constant Turn Rate and Velocity Driving Model
         *  x = [px py vel yaw yaw_rate]
         */
        Eigen::VectorXd     x_;

        /**
         * ---------------------------------------------
         * State Covariance Matrix
         * ---------------------------------------------
         */
        Eigen::MatrixXd     P_;

        /**
         * ---------------------------------------------
         * Process Noise Covariance Matrix
         * ---------------------------------------------
         * Used in Prediction Step
         * (P_new = P_old + Q)
         */
        Eigen::MatrixXd     Q_;

        /**
         * --------------------------------------------
         *  Sensor Noise Covariance Matrix
         *  -------------------------------------------
         */
        Eigen::MatrixXd     R_;


};
#endif //UNSCENTEDKF_KALMANFILTER_H

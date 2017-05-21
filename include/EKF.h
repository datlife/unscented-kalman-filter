//
// Created by dat on 5/20/17.
//

#ifndef KALMANFILTER_EKF_H
#define KALMANFILTER_EKF_H

#include "Dense"
#include "KalmanFilterBase.h"

class EKF : public KalmanFilterBase
{
    /**
     * Extended Kalman Filter
     */
    public:
        EKF();

        virtual void   initialize(const Sensor&);
        virtual void   Predict(double delta_time);
        virtual void   Update(const Sensor&);
        virtual double getNIS() const;

        ~EKF();
    protected:
        void            updateTimePeriod(double &new_delta_t);
        void            calculateJacobian();
    private:
        int             nx_;
        double          noise_ax_;
        double          noise_ay_;

        Eigen::MatrixXd F_;
        Eigen::MatrixXd R_laser;
        Eigen::MatrixXd R_radar;

        Eigen::MatrixXd H_laser;
        Eigen::MatrixXd H_radar;
};


#endif //KALMANFILTER_EKF_H

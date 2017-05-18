//
// Created by dat on 5/17/17.
//

#ifndef UNSCENTEDKF_SENSORFUSION_H
#define UNSCENTEDKF_SENSORFUSION_H

#include "KalmanFilter.h"
#include "Sensor.h"

class SensorFusion{

    private:
        KalmanFilter                *filter_;
        Eigen::VectorXd             z_pred_;
        Eigen::MatrixXd             S_;
        double                      prev_time_step_;
        bool                        initialized_;

public:
        SensorFusion(KalmanFilter*);
        void            Process(const Sensor&);
        Eigen::VectorXd getState() const { return filter_->getState();}

        double          calculate_NIS(const Sensor &);
        Eigen::VectorXd calculate_RMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

};
#endif //UNSCENTEDKF_SENSORFUSION_H

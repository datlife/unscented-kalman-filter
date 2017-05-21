//
// Created by dat on 5/17/17.
//

#ifndef UNSCENTEDKF_SENSORFUSION_H
#define UNSCENTEDKF_SENSORFUSION_H

#include "KalmanFilterBase.h"
#include "Sensor.h"

class SensorFusion{

    private:
        KalmanFilterBase                *filter_;
        double                      nis_;
        bool                        initialized_;
        long long                   prev_time_us_;    ///* time when the state is true, in us

public:
        SensorFusion(KalmanFilterBase*);
        void            Process(const Sensor&);
        Eigen::VectorXd getState() const { return filter_->getState();}

        double          calculate_NIS() const;
        Eigen::VectorXd calculate_RMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

};
#endif //UNSCENTEDKF_SENSORFUSION_H

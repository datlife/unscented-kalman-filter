#include <iostream>
#include "ukf.h"

UKF::UKF() {
    //TODO Auto-generated constructor stub
    Init();
}

UKF::~UKF() {
    //TODO Auto-generated destructor stub
}

void UKF::Init() {

}
void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {
    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
              1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
              2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
              0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
              0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create vector for weights // 15
    VectorXd weights = VectorXd(2*n_aug+1);

    //create vector for predicted state
    VectorXd x = VectorXd(n_x);

    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x, n_x);


/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //set weights
    for (int i = 0; i < Xsig_pred.cols(); i++){
        if (i == 0)
            weights[i] = lambda/(lambda+n_aug);
        else
            weights[i] = 0.5/(lambda + n_aug);
        //predict state mean
        x += weights[i]*Xsig_pred.col(i);
    }

    //predict state covariance matrix
    for (int i = 0; i < Xsig_pred.cols(); i++) {
        MatrixXd x_diff = (Xsig_pred.col(i) - x);
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        P += weights[i] * x_diff * x_diff.transpose();
    }
/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Predicted state" << std::endl;
    std::cout << x << std::endl;
    std::cout << "Predicted covariance matrix" << std::endl;
    std::cout << P << std::endl;

    //write result
    *x_out = x;
    *P_out = P;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out){
    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //create empty sigma point matrix (7x15)
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

    // Generate Augmented Sigma Points
    AugmentedSigmaPoints(&Xsig_aug);

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    double delta_t = 0.1; //time diff in sec
    double dt2 = delta_t*delta_t;

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
    //predict sigma points
    for (int i = 0; i < Xsig_aug.cols(); i++){
        VectorXd sigma_pt = Xsig_aug.col(i);
        VectorXd sigma_pt_pred(5);
        VectorXd rate_of_change(5);
        VectorXd noise(5);

        // Generate curr state values for read-ability
        double p_x       = sigma_pt[0]; // position in x-plane
        double p_y       = sigma_pt[1]; // position in y-plane
        double velocity  = sigma_pt[2];
        double yaw       = sigma_pt[3]; // orientation (yaw)
        double yawd      = sigma_pt[4]; // orientation rate
        double nu_a      = sigma_pt[5]; // process noise of acceleration
        double nu_yawdd  = sigma_pt[6]; // process noise of yaw

        double px_rate, py_rate;
        //avoid division by zero
        if(fabs(yawd) > 0.001) {
            px_rate = velocity/yawd*(sin(yaw + yawd*delta_t) - sin(yaw));
            py_rate = velocity/yawd*(-cos(yaw + yawd*delta_t) + cos(yaw));
        }
        else{
            px_rate = velocity*cos(yaw)*delta_t;
            py_rate = velocity*sin(yaw)*delta_t;
        }

        rate_of_change << px_rate,
                          py_rate,
                          0,
                          yawd*delta_t,
                          0;

        noise << 0.5  * nu_a*cos(yaw) * dt2,
                 0.5  * nu_a*sin(yaw) * dt2,
                 nu_a * delta_t,
                 0.5  * nu_yawdd * dt2,
                 nu_yawdd * delta_t;

         sigma_pt_pred = sigma_pt.head(5) + rate_of_change + noise;
        //write predicted sigma points into right column
        Xsig_pred.col(i) = sigma_pt_pred;
    }



/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

    //write result
    *Xsig_out = Xsig_pred;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out){
    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a = 0.2;

    //Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd = 0.2;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //set example state
    VectorXd x = VectorXd(n_x);
    x <<    5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    //create example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
             0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix (7, 15)
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 *  Quickly set vector y as first n elements of vector x.
    x.head(n) = y, where n is the number of elements from first element, and y is an input vector of that size.
    Quickly set matrix y to top left corner of matrix x.
    x.topLeftCorner( (y_size, y_size) )
    x.llt().matrixL();
    Reminder of what function to use to take the square root of a matrix x,
 ******************************************************************************/

    //create augmented mean state (first sigma point)
    x_aug.head(5) = x;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // Create Process Noise Covariance Matrix
    MatrixXd Q = MatrixXd(2, 2);
    Q << std_a*std_a,        0.0,
         0.0, std_yawdd*std_yawdd;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(P.cols(), P.rows()) = P;
    P_aug.bottomRightCorner(Q.cols(), Q.rows()) = Q;

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();
    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;

    for (int i = 0; i < n_aug; i++){
        Xsig_aug.col(i + 1)         = x_aug + sqrt(lambda + n_aug) * A.col(i);
        Xsig_aug.col(i + 1 + n_aug) = x_aug - sqrt(lambda + n_aug) * A.col(i);
    }
/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

    //write result
    *Xsig_out = Xsig_aug;

}

void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

    //set state dimension
    int n_x = 5;

    //define spreading parameter
    double lambda = 3 - n_x;

    //set example state
    VectorXd x = VectorXd(n_x);
    x <<   5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    //set example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
            0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1); // Dim (5, 11)

    //calculate square root of P
    MatrixXd A = P.llt().matrixL(); // Squared root of Covariance Matrix P

    //set first column of sigma point matrix
    Xsig.col(0)  = x;

    //set remaining sigma points
    for (int i = 0; i < n_x; i++)
    {
        Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
        Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
    }

    //write result
    *Xsig_out = Xsig;
}
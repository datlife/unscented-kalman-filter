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

    //define spreading parameter ( how far from mean state value)
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
    weights[0] = lambda/(lambda+n_aug);
    for (int i = 1; i < Xsig_pred.cols(); i++){
        weights[i] = 0.5/(lambda + n_aug);
        x += weights[i]*Xsig_pred.col(i);          //predict state mean
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

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
    weights(0) = lambda/(lambda+n_aug);
    for (int i=1; i<2*n_aug+1; i++) {
        double weight = 0.5/(n_aug+lambda);
        weights(i) = weight;
    }

    //radar measurement noise standard deviation radius in m
    double std_radr = 0.3;

    //radar measurement noise standard deviation angle in rad
    double std_radphi = 0.0175;

    //radar measurement noise standard deviation radius change in m/s
    double std_radrd = 0.1;

    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
    z_pred.fill(0.0);

    //transform sigma points into measurement space
    for (int i = 0; i < Xsig_pred.cols(); i++){
        VectorXd x = Xsig_pred.col(i);
        double px = x[0];
        double py = x[1];
        double v  = x[2];
        double yaw = x[3];
        double yaw_rate = x[4];

        double dist = sqrt(px*px + py*py); // phi = sqrt(px^2 + py^2)
        double rho  = atan2(py, px);       // rho = arctan(py/px)
        double rho_rate = (px*cos(yaw)*v + py*sin(yaw)*v)/dist;
        // Update predicted sigma point in measurement space
        Zsig.col(i) << dist, rho, rho_rate;

        //calculate mean predicted measurement
        z_pred += weights[i]*Zsig.col(i);
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr*std_radr, 0, 0,
            0, std_radphi*std_radphi, 0,
            0, 0,std_radrd*std_radrd;
    //calculate measurement covariance matrix S
    for (int i = 0; i < Zsig.cols(); i++){
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S += weights[i]*z_diff*z_diff.transpose();
    }
    S += R;
/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "z_pred: " << std::endl << z_pred << std::endl;
    std::cout << "S: " << std::endl << S << std::endl;

    //write result
    *z_out = z_pred;
    *S_out = S;
}


void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out) {

    //set state dimension
    int n_x = 5;

    //set augmented dimension
    int n_aug = 7;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
    double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {
        double weight = 0.5/(n_aug+lambda);
        weights(i) = weight;
    }

    //create example matrix with predicted sigma points in state space
    MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
    Xsig_pred <<
              5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create example vector for predicted state mean
    VectorXd x = VectorXd(n_x);
    x <<
      5.93637,
            1.49035,
            2.20528,
            0.536853,
            0.353577;

    //create example matrix for predicted state covariance
    MatrixXd P = MatrixXd(n_x,n_x);
    P <<
      0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
            -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
            0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
            -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
            -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

    //create example matrix with sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
    Zsig <<
         6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
            0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
            2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

    //create example vector for mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred <<
           6.12155,
            0.245993,
            2.10313;

    //create example matrix for predicted measurement covariance
    MatrixXd S = MatrixXd(n_z,n_z);
    S <<
      0.0946171, -0.000139448,   0.00407016,
            -0.000139448,  0.000617548, -0.000770652,
            0.00407016, -0.000770652,    0.0180917;

    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z <<
      5.9214,   //rho in m
            0.2187,   //phi in rad
            2.0062;   //rho_dot in m/s

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x = x + K * z_diff;
    P = P - K*S*K.transpose();


/*******************************************************************************
 * Student part end
 ******************************************************************************/

    //print result
    std::cout << "Updated state x: " << std::endl << x << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

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

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Dense"
#include "UKF.h"
#include "Sensor_Input.h"
#include <stdlib.h>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

typedef SensorInput GroundTruthPackage;
typedef SensorInput MeasurementPackage;

//============================ Function Prototypes ============================================
void validate_arguments(int argc, char **argv);
void check_files       (ifstream &, ofstream& out_file);
void read_input        (ifstream &, vector<MeasurementPackage> &, vector<GroundTruthPackage> &);
void fuse_data_sensors (ofstream &, vector<MeasurementPackage> &, vector<GroundTruthPackage> &);
void write_output      (ofstream &, const UKF &, const MeasurementPackage &, const GroundTruthPackage &);
void convert_ukf_to_cartesian(const VectorXd &state, VectorXd &ukf_in_cartesian);

// Input format :
// -----------------------------------------------------------------------------------------------------------------
// | SENSOR    |                      RAW DATA                           |         GROUND TRUTH  FOR CRTV MODEL     |
// -----------------------------------------------------------------------------------------------------------------
// |   LASER   | X POS    | Y POS       | TIME STAMP        | N/A        |  X POS | Y POS | V   | YAW  |   YAW_RATE |<-- Cartesian Coordinate
// |   RADAR   | DIST RHO | ANGLE THETA | DIST_RATE RHO DOT | TIME STAMP | X POS  | Y POS | V   | YAW  |   YAW_RATE |<-- Polar Coordinate
// |----------------------------------------------------------------------------------------------------------------

// Result with obj_pose-laser-radar-synthetic-input.txt
//Accuracy - RMSE:


int main(int argc, char* argv[]) {
    // Validate arguments - For this project, it requires an input path and output path respectively.
    validate_arguments(argc, argv);
    ifstream in_file_(argv[1], ifstream::in);              // Create input/output stream
    ofstream out_file_(argv[2], ofstream::out);
    check_files(in_file_, out_file_);                      // Validate input/output file

    vector<MeasurementPackage> measurement_pack_list;     // Used to store data from input file
    vector<GroundTruthPackage> gt_pack_list;

    // Input is (Laser/Radar Measurement)
    read_input(in_file_, measurement_pack_list, gt_pack_list);
    fuse_data_sensors(out_file_,measurement_pack_list, gt_pack_list);

    // close files
    if (out_file_.is_open()) out_file_.close();
    if (in_file_.is_open()) in_file_.close();
    cout << "Done!" << endl;

    return 0;
}

void validate_arguments(int argc, char* argv[]) {
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1) {
        cerr << usage_instructions << endl;
    } else if (argc == 2) {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    } else if (argc == 3) {
        has_valid_args = true;
    } else if (argc > 3) {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args) {
        exit(EXIT_FAILURE);
    }
}

void check_files(ifstream& in_file,
                 ofstream& out_file) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: \n";
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
        cerr << "Cannot open output file: \n";
        exit(EXIT_FAILURE);
    }
}

void read_input(ifstream &in_file_, vector<MeasurementPackage> &measurement_pack_list, vector<GroundTruthPackage> &gt_pack_list){

    string line;
    // prep the measurement packages (each line represents a measurement at a timestamp)
    while (getline(in_file_, line)) {
        long long timestamp;
        string sensor_type;
        istringstream iss(line);
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;

        // Reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) {
            // LASER MEASUREMENT
            float x, y;
            iss >> x >> y;
            iss >> timestamp;

            // read measurements at this timestamp
            meas_package.sensor_type_ = SensorType::LASER;
            meas_package.data_ = VectorXd(2);
            meas_package.data_ << x, y;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        else if (sensor_type.compare("R") == 0) {
            // RADAR MEASUREMENT
            float ro, phi, ro_dot;
            iss >> ro >> phi >> ro_dot;
            iss >> timestamp;

            // read measurements at this timestamp
            meas_package.sensor_type_ = SensorType::RADAR;
            meas_package.data_ = VectorXd(3);
            meas_package.data_ << ro, phi, ro_dot;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        // Read ground truth data to compare later
        float x_gt, y_gt, vx_gt, vy_gt;
        gt_package.data_ = VectorXd(4);

        iss >> x_gt >> y_gt;    // ground truth of current Position
        iss >> vx_gt >>vy_gt;   // ground truth of current Velocity
        gt_package.data_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }
}

void fuse_data_sensors (ofstream &out_file_ ,
                        vector<MeasurementPackage> & measurement_pack_list,
                        vector<GroundTruthPackage> &gt_pack_list){
    // Create a UKF instance
    // @TODO: Restructure UKF
    UKF ukf;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // column names for output file
    out_file_ << "time_stamp" << "\t"
              << "px_state" << "\t"
              << "py_state" << "\t"
              << "v_state" << "\t"
              << "yaw_angle_state" << "\t"
              << "yaw_rate_state" << "\t"
              << "sensor_type" << "\t"
              << "NIS" << "\t";

    out_file_ << "px_measured" << "\t" << "py_measured" << "\t"
              << "px_ground_truth" << "\t" << "py_ground_truth" << "\t"
              << "vx_ground_truth" << "\t" << "vy_ground_truth" << "\n";


    // start filtering from the second frame (the speed is unknown in the first frame)
    size_t number_of_measurements = measurement_pack_list.size();

    // Convert ukf x vector to cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    for (size_t k = 0; k < number_of_measurements; ++k) {
        // Call the UKF-based fusion
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        convert_ukf_to_cartesian(ukf.getState(),ukf_x_cartesian_);

        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].data_);

        write_output(out_file_, ukf, measurement_pack_list[k], gt_pack_list[k]);
    }

    // compute the accuracy (RMSE)
    Tools tools;
    // cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
    // @TODO: Compute NIS
}

void write_output      (ofstream &out_file_,
                        const UKF &ukf,
                        const MeasurementPackage &measurement,
                        const GroundTruthPackage &ground_truth){
    // timestamp
    out_file_ << measurement.timestamp_ << "\t"; // pos1 - est

    // output the state vector
    out_file_ << ukf.getState(0) << "\t"; // pos1 - est
    out_file_ << ukf.getState(1) << "\t"; // pos2 - est
    out_file_ << ukf.getState(2) << "\t"; // vel_abs -est
    out_file_ << ukf.getState(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.getState(4) << "\t"; // yaw_rate -est

    // output LiDar and radar specific data
    if (measurement.sensor_type_ == SensorType::LASER) {
        // sensor type
        out_file_ << "LiDar" << "\t";

        // NIS value
        out_file_ << ukf.getNIS(SensorType::LASER) << "\t";

        // output the LiDar sensor measurement px and py
        out_file_ << measurement.data_(0) << "\t";
        out_file_ << measurement.data_(1) << "\t";

    }
    else if (measurement.sensor_type_ == SensorType::RADAR) {
        // sensor type
        out_file_ << "Radar" << "\t";

        // NIS value
        out_file_ << ukf.getNIS(SensorType::RADAR) << "\t";

        // output radar measurement in cartesian coordinates
        float ro   = float(measurement.data_(0));
        float phi  = float(measurement.data_(1));
        out_file_ << ro * cos(phi) << "\t"; // px measurement
        out_file_ << ro * sin(phi) << "\t"; // py measurement
    }

    // Output the ground truth
    out_file_ << ground_truth.data_(0) << "\t";
    out_file_ << ground_truth.data_(1) << "\t";
    out_file_ << ground_truth.data_(2) << "\t";
    out_file_ << ground_truth.data_(3) << "\n";
}

void convert_ukf_to_cartesian(const VectorXd &state, VectorXd &ukf_in_cartesian){

    float x_estimate_  = state(0);
    float y_estimate_  = state(1);
    float vx_estimate_ = state(2) * cos(state(3));
    float vy_estimate_ = state(2) * sin(state(3));

    ukf_in_cartesian << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

}
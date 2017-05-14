
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
void check_files       (ifstream &, string& in_name, ofstream& out_file, string& out_name);
void read_input        (ifstream &, vector<MeasurementPackage> &, vector<GroundTruthPackage> &);
void fuse_data_sensors (ofstream &, vector<MeasurementPackage> &, vector<GroundTruthPackage> &);
void write_output      (ofstream &, const UKF &, const MeasurementPackage &, const GroundTruthPackage &);

// Input format :
// ------------------------------------------------------------------------------------------
// |                   RAW DATA                             |         GROUND TRUTH           |
// ------------------------------------------------------------------------------------------|
// |   LASER   | X POS    | Y POS       | TIME STAMP        |  X POS | Y POS | X VEL | Y VEL | <-- Cartesian Coordinate
// |   RADAR   | DIST RHO | ANGLE THETA | DIST_RATE RHO DOT |  X POS | Y POS | X VEL | Y VEL | <-- Polar Coordinate
// |-----------------------------------------------------------------------------------------

// Result with obj_pose-laser-radar-synthetic-input.txt
//Accuracy - RMSE:


int main(int argc, char* argv[]) {
    // Validate arguments - For this project, it requires an input path and output path respectively.
    validate_arguments(argc, argv);

    // Create input/output stream
    string in_file_name_ = argv[1], out_file_name_ = argv[2];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    // Validate input/output file
    check_files(in_file_, in_file_name_, out_file_, out_file_name_);

    // Used to store data from input file
    vector<MeasurementPackage> measurement_pack_list;
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

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
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
            meas_package.sensor_type_ = MeasurementPackage::LASER;
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
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
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

    // start filtering from the second frame (the speed is unknown in the first frame)
    size_t number_of_measurements = measurement_pack_list.size();

    // column names for output file
    out_file_ << "time_stamp" << "\t";
    out_file_ << "px_state" << "\t";
    out_file_ << "py_state" << "\t";
    out_file_ << "v_state" << "\t";
    out_file_ << "yaw_angle_state" << "\t";
    out_file_ << "yaw_rate_state" << "\t";
    out_file_ << "sensor_type" << "\t";
    out_file_ << "NIS" << "\t";
    out_file_ << "px_measured" << "\t";
    out_file_ << "py_measured" << "\t";
    out_file_ << "px_ground_truth" << "\t";
    out_file_ << "py_ground_truth" << "\t";
    out_file_ << "vx_ground_truth" << "\t";
    out_file_ << "vy_ground_truth" << "\n";


    for (size_t k = 0; k < number_of_measurements; ++k) {
        // Call the UKF-based fusion
        ukf.ProcessMeasurement(measurement_pack_list[k]);
        // convert ukf x vector to cartesian to compare to ground truth
        VectorXd ukf_x_cartesian_ = VectorXd(4);

        float x_estimate_ = ukf.x_(0);
        float y_estimate_ = ukf.x_(1);
        float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
        float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

        ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].data_);

        write_output(out_file_, ukf, measurement_pack_list[k], gt_pack_list[k]);
    }

    // compute the accuracy (RMSE)
    Tools tools;
    cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
    // @TODO: Compute NIS
}
void write_output      (ofstream &out_file_, const UKF &ukf,
                        const MeasurementPackage &measurement,
                        const GroundTruthPackage &ground_truth){
    // timestamp
    out_file_ << measurement.timestamp_ << "\t"; // pos1 - est

    // output the state vector
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output lidar and radar specific data
    if (measurement.sensor_type_ == MeasurementPackage::LASER) {
        // sensor type
        out_file_ << "LiDar" << "\t";

        // NIS value
        out_file_ << ukf.NIS_laser_ << "\t";

        // output the Lidar sensor measurement px and py
        out_file_ << measurement.data_(0) << "\t";
        out_file_ << measurement.data_(1) << "\t";

    }
    else if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
        // sensor type
        out_file_ << "Radar" << "\t";

        // NIS value
        out_file_ << ukf.NIS_radar_ << "\t";

        // output radar measurement in cartesian coordinates
        float ro = measurement.data_(0);
        float phi = measurement.data_(1);
        out_file_ << ro * cos(phi) << "\t"; // px measurement
        out_file_ << ro * sin(phi) << "\t"; // py measurement
    }

    // output the ground truth
    out_file_ << ground_truth.data_(0) << "\t";
    out_file_ << ground_truth.data_(1) << "\t";
    out_file_ << ground_truth.data_(2) << "\t";
    out_file_ << ground_truth.data_(3) << "\n";
}
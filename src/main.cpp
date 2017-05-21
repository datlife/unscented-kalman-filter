
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Dense"
#include "Sensor.h"
#include "SensorFusion.h"
#include "EKF.h"
#include "UKF.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

typedef Sensor GroundTruthPackage;
typedef Sensor MeasurementPackage;



//============================ Function Prototypes ============================================
void            validate_arguments(int argc, char **argv);
void            check_files       (ifstream &, ofstream& out_file);
void            read_input        (ifstream &, vector<MeasurementPackage> &, vector<GroundTruthPackage> &);
void            FuseSensors (ofstream &, vector<MeasurementPackage> &, vector<GroundTruthPackage> &);
void            write_output_header(ofstream &out_file_);
void            write_output      (ofstream &, const SensorFusion &, const MeasurementPackage &, const GroundTruthPackage &);
Eigen::VectorXd convert_ukf_to_cartesian(const VectorXd &state);
Eigen::VectorXd convert_sensor_to_cartesian(const Sensor &sensor);

// Input format :
// ------------------------------------------------------------------------------------------------------------------------
// | SENSOR    |                      RAW DATA                           |         GROUND TRUTH  FOR CRTV MODEL           |
// -----------------------------------------------------------------------------------------------------------------------
// |   LASER   | X POS    | Y POS       | TIME STAMP        | N/A        |  X POS | Y POS | VX  | VY  | YAW  |   YAW_RATE |<-- Cartesian Coordinate
// |   RADAR   | DIST RHO | ANGLE THETA | DIST_RATE RHO DOT | TIME STAMP | X POS  | Y POS | V   | VY  | YAW  |   YAW_RATE |<-- Polar Coordinate
// |----------------------------------------------------------------------------------------------------------------

// Result with obj_pose-laser-radar-synthetic-input.txt
//Accuracy - RMSE:


int main(int argc, char* argv[]) {
    // Validate arguments - For this project, it requires an input path and output path respectively.
    validate_arguments(argc, argv);

    ifstream in_file_(argv[1], ifstream::in);              // Create input/output stream
    ofstream out_file_(argv[2], ofstream::out);
    check_files(in_file_, out_file_);                      // Validate input/output file

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;
    read_input(in_file_, measurement_pack_list, gt_pack_list);    // Input is (Laser/Radar Measurement)

    // Sensor Fusion
    FuseSensors(out_file_,measurement_pack_list, gt_pack_list);

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
        float x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yaw_rate_gt;
        gt_package.data_ = VectorXd(6);

        iss >> x_gt  >> y_gt;    // ground truth of current Position
        iss >> vx_gt >> vy_gt;   // ground truth of current Velocity
        iss >> yaw_gt>> yaw_rate_gt;
        gt_package.data_ << x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yaw_rate_gt;
        gt_pack_list.push_back(gt_package);
    }
}

void FuseSensors (ofstream &out_file_ ,
                        vector<MeasurementPackage> & measurement_pack_list,
                        vector<GroundTruthPackage> &gt_pack_list){

    // used to compute the RMSE
    vector<VectorXd> estimations, ground_truth;

   // Disabled so output could follow Mercedes Visualization Format
   //  write_output_header(out_file_);

    // Create Kalman Filters
    //EKF ekf;
    UKF ukf;
    //SensorFusion filter(&ekf);
    SensorFusion filter(&ukf);
    double prev_time = 0.0;
    for (size_t k = 0; k < measurement_pack_list.size(); ++k) {

        // Call the UKF-based fusion
        filter.Process(measurement_pack_list[k]);

        // Estimated state is converted to Cartesian Space before writing to output
        VectorXd x_cartesian_ =  convert_ukf_to_cartesian(filter.getState());

        // save Estimation &  ground truth value
        estimations.push_back(x_cartesian_);
        ground_truth.push_back(gt_pack_list[k].data_);

        // Save to 'output.
        write_output(out_file_, filter, measurement_pack_list[k], gt_pack_list[k]);


        std::cout << ((measurement_pack_list[k].sensor_type_ == SensorType::LASER)?"LIDAR" : "RADAR");
        std::cout<< "\n------------------------------------------\n"
                 <<"STEP "<< k
                 <<"\nNIS: " <<filter.calculate_NIS()
                 <<"\nDelta T: "<< (measurement_pack_list[k].timestamp_ - prev_time)/ 1000000.0<<" sec\n"
                 <<"\nMeasurement in Cartesian:\n"<<convert_sensor_to_cartesian(measurement_pack_list[k])<<"\n"
                 <<"\nState X: \n"<<filter.getState()<<"\n"
                 << "\n\nState X in Cartesian\n"  <<  x_cartesian_<<"\n"
                 << "\nGround Truth\n" << gt_pack_list[k].data_ <<"\n\n";

        prev_time = measurement_pack_list[k].timestamp_;
    }
    std::cout << "RMSE" << endl << filter.calculate_RMSE(estimations, ground_truth) << std::endl;
}
void write_output_header(ofstream &out_file_){
    // column names for output file
    out_file_ << "time_stamp" << "\t"
              << "px_state" << "\t"
              << "py_state" << "\t"
              << "v_state" << "\t"
              << "yaw_angle_state" << "\t"
              << "yaw_rate_state" << "\t"
              << "sensor_type" << "\t"
              << "NIS" << "\t";

    out_file_ << "px_measured" << "\t"
              << "py_measured" << "\t"

              << "px_ground_truth" << "\t"
              << "py_ground_truth" << "\t"
              << "vx_ground_truth" << "\t"
              << "vy_ground_truth" << "\n";
}
void write_output      (ofstream &out_file_,
                        const SensorFusion &f,
                        const MeasurementPackage &measurement,
                        const GroundTruthPackage &ground_truth){
    VectorXd x = f.getState();
    // timestamp
    out_file_ << measurement.timestamp_ << "\t"; //

    // Output state estimation vector
    for(int i = 0; i < x.rows();i++)
        out_file_ << x(i) << "\t"; //

    double nis_laser = 0., nis_radar = 0.;
    if (measurement.sensor_type_ == SensorType::LASER) {
        // sensor type
        // out_file_ << "LiDar" << "\t";
        // out_file_ << f.calculate_NIS() << "\t";

        // Sensor measurement px and py
        out_file_ << measurement.data_(0) << "\t";
        out_file_ << measurement.data_(1) << "\t";

        // Follow ipython format
        nis_laser = f.calculate_NIS();
    }
    else if (measurement.sensor_type_ == SensorType::RADAR) {
        // sensor type
        //out_file_ << "Radar" << "\t";
        // NIS value
       //out_file_ << f.calculate_NIS() << "\t";
        // Sensor radar measurement in cartesian coordinates
        float ro   = float(measurement.data_(0));
        float phi  = float(measurement.data_(1));

        out_file_ << ro * cos(phi) << "\t"; // px measurement
        out_file_ << ro * sin(phi) << "\t"; // py measurement
        nis_radar  = f.calculate_NIS();
    }
    // Output the ground truth
    float vx_gt = ground_truth.data_(2);
    float vy_gt = ground_truth.data_(3);
    float v     = sqrt(vx_gt*vx_gt + vy_gt*vy_gt);

    out_file_ << ground_truth.data_(0) << "\t"; //px
    out_file_ << ground_truth.data_(1) << "\t"; //py
    out_file_  << v << "\t";                     //v
    out_file_ << ground_truth.data_(4) << "\t"; //yaw
    out_file_ << ground_truth.data_(5) << "\t"; //yaw_rate
    out_file_ << vx_gt  << "\t"                 //vx_gt
              << vy_gt <<"\t"                   //vy_gt
              << nis_laser <<"\t"               //nis_laser
              << nis_radar <<"\n";              //nis_radar

}

Eigen::VectorXd convert_ukf_to_cartesian(const VectorXd &state){

    float px_estimate_   = state(0); //px
    float py_estimate_   = state(1); //py
    float vx_estimate_   = state(2) * cos(state(3)); //vx = v * cos(yaw)
    float vy_estimate_   = state(2) * sin(state(3)); //vy = v * sin(yaw)

    VectorXd x_ = VectorXd::Zero(4);
    x_ << px_estimate_, py_estimate_, vx_estimate_, vy_estimate_;
    return x_;
}

Eigen::VectorXd convert_sensor_to_cartesian(const Sensor &sensor){
    // Initialize the First State
    VectorXd x_ = VectorXd::Zero(4);
    VectorXd measurement = sensor.data_;
    switch(sensor.sensor_type_)
    {
        case SensorType::RADAR: {
            double rho = measurement(0);
            double phi = measurement(1);
            x_ << rho * cos(phi), rho * sin(phi), 0 , 0;
        }
            break;
        case SensorType::LASER:
            x_ << measurement(0), measurement(1), 0, 0;
            break;
    }
    return x_;
}
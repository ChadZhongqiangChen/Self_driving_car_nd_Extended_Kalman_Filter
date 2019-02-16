#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "measurement_package.h"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


int main() 
{

	/*******************************************************************************
	 *  Set Measurements															 *
	 *******************************************************************************/
	//vector<MeasurementPackage> measurement_pack_list;
    
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

	// hardcoded input file with laser and radar measurements
	string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	
  string line;
  
  int i = 0;
  // Converge time
  int converge_time = 0;
  //VectorXd RMSE(4);
  //RMSE <<0, 0, 0, 0;
  
	while(getline(in_file, line) && (i<600))
    {
        MeasurementPackage meas_package;


		istringstream iss(line);
		string sensor_type;
		iss >> sensor_type;	//reads first element from the current line
		int64_t timestamp;
		if(sensor_type.compare("L") == 0){	//laser measurement
			//read measurements
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x,y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;

		}else if(sensor_type.compare("R") == 0){
     
          meas_package.sensor_type_ = MeasurementPackage::RADAR;
          meas_package.raw_measurements_ = VectorXd(3);
          float ro;
      	  float theta;
      	  float ro_dot;
          iss >> ro;
          iss >> theta;
          iss >> ro_dot;
          meas_package.raw_measurements_ << ro,theta, ro_dot;
          iss >> timestamp;
          meas_package.timestamp_ = timestamp;
		}
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);
          
       		
      //cout << "Ground truth values " << gt_values << endl;
     
      fusionEKF.ProcessMeasurement(meas_package);
      
          	  
      VectorXd estimate(4);

    	  double p_x = fusionEKF.ekf_.x_(0);
    	  double p_y = fusionEKF.ekf_.x_(1);
    	  double v1  = fusionEKF.ekf_.x_(2);
    	  double v2 = fusionEKF.ekf_.x_(3);
          
    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
      
          cout << "x_ = " << estimate << endl;
    	  estimations.push_back(estimate);
      
      // Using Calculate RMSE function

    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
       
  		  cout << "P_ = " << fusionEKF.ekf_.P_ << endl;
          cout << "RMSE = " << RMSE << endl;
          //cout<< "estimations size = " << estimations.size() << endl;
          //cout<< "ground_truth size = "<< ground_truth.size() << endl;
      
      
      if( (RMSE(0) > 0.11) || (RMSE(1) > 0.11) || (RMSE(2) > 0.52) || (RMSE(3) > 0.52))
      {
        converge_time++;
      }

   i++;

	}
  cout << "Kalman Filter converged at timestamp " << (converge+1) << endl ;

	if(in_file.is_open()){
		in_file.close();
	}
  
	return 0;
}


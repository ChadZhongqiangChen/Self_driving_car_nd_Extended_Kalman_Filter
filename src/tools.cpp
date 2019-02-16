#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
//using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0.0,0.0,0.0,0.0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
    std::cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  
  for (unsigned int i = 0; i < estimations.size(); i++)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	
	//compute the Jacobian matrix
	float rho = pow(px, 2) + pow(py, 2);
	
  	if(fabs(rho) < 0.0001){
		std::cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}
	else
	{
	    Hj(0,0) = px / pow(rho, 0.5);
	    Hj(0,1) = py / pow(rho, 0.5);
	    Hj(0,2) = 0;
	    Hj(0,3) = 0;
	    
	    Hj(1,0) = -py/rho;
	    Hj(1,1) = px/rho;
	    Hj(1,2) = 0;
	    Hj(1,3) = 0;
	    
	    Hj(2,0) = py*(vx*py - vy*px)/ pow(rho, 1.5);
	    Hj(2,1) = px*(vy*px - vx*py)/ pow(rho, 1.5);
	    Hj(2,2) = px/pow(rho, 0.5);
	    Hj(2,3) = py/pow(rho, 0.5);
 	}

	return Hj;
}

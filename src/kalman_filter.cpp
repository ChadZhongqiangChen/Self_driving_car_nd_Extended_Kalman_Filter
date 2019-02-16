#include "kalman_filter.h"
#include "tools.h"
//#define M_PI 3.1415926

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_* Ht * S.inverse();
  
  x_ = x_ + K * y; 
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float c0 = pow(px,2) + pow(py,2);
  float c1 = pow(c0, 0.5);

  float c2 = atan2(py, px);
  
  float c3 = px*vx + py*vy;
  
  float c4;
  
  //if (c1 < 0.0001)
  //{
  //  c4 = c3/0.0001;
  //}
  //else
  //{
  c4 = c3/c1;
  //}
  VectorXd xh(3);
  xh(0) = c1;
  xh(1) = c2;
  xh(2) = c4;
  
  VectorXd y = z - xh;
  //y(1) = y(1) % pi;
  
  if ( (y(1) < - M_PI) || (y(1) > M_PI) )
  {
  	while ( y(1) < - M_PI)
  	{
    	y(1) += 2* M_PI;
  	}
  
  	while ( y(1) > M_PI)
  	{
    	y(1) -= 2 * M_PI;
  	}
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_* Ht * S.inverse();
  
  x_ = x_ + K * y; 
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateIEKF(const VectorXd &z) {
  
  int count = 0;
  float diff = 1.0;
  VectorXd x_temp;
  VectorXd xs = x_;
  MatrixXd Ht;  
  MatrixXd S; 
  MatrixXd K;    

  while ((diff>0.1) || (count < 9))
  {
    VectorXd xh(3);  
    xh = Cart2Polar(x_);  
    Ht = H_.transpose(); 
    S = H_*P_*Ht + R_;  
    K = P_* Ht * S.inverse();
    VectorXd y;
    y = z - xh - H_ * (xs - x_) ;
  
    if ( (y(1) < - M_PI) || (y(1) > M_PI) )  
    {  	
      while ( y(1) < - M_PI)  	
      {   
        y(1) += 2* M_PI;  	
      } 	
      while ( y(1) > M_PI)  	
      {   	
        y(1) -= 2 * M_PI; 	
      } 
    }
    x_temp = xs + K * y; 
    diff = (x_temp-x_).norm();
    x_ = x_temp;
    H_ = tools.CalculateJacobian(x_);
    count++;
  }
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
}

VectorXd KalmanFilter::Cart2Polar( VectorXd &x_)
{
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float c0 = pow(px,2) + pow(py,2);
  float c1 = pow(c0, 0.5);

  float c2 = atan2(py, px);
  
  float c3 = px*vx + py*vy;
  
  float c4;
  
  //if (c1 < 0.0001)
  //{
  //  c4 = c3/0.0001;
  //}
  //else
  //{
  c4 = c3/c1;
  //}
  VectorXd xh(3);
  xh(0) = c1;
  xh(1) = c2;
  xh(2) = c4;
  
  return xh;
}
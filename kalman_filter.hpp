#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter{
public:
	KalmanFilter(){
		is_initialized_ = false;
	}
	~KalmanFilter(){}

	Eigen::VectorXd GetX(){
		return x_;
	}

	bool isInitialized(){
		return is_initialized_;
	}

	void Initializetion(Eigen::VectorXd x_in){
		x_ = x_in;
		is_initialized_ = true;
	}

	void SetF(Eigen::MatrixXd F_in){
		F_ = F_in;
	}

	void SetP(Eigen::MatrixXd P_in){
		P_ = P_in;
	}

	void SetQ(Eigen::MatrixXd Q_in){
		Q_ = Q_in;
	}

	void SetH(Eigen::MatrixXd H_in){
		H_ = H_in;
	}

	void SetR(Eigen::MatrixXd R_in){
		R_ = R_in;
	}

	void Prediction(){
		x_ = F_ * x_;
		Eigen::MatrixXd Ft = F_.transpose();
		P_ = F_ * P_ * Ft + Q_;
	}

	void MeasurementUpdate(const Eigen::VectorXd &z){
		Eigen::VectorXd y = z - H_ * x_;
		Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
		Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
		x_ = x_ + (K * y);
		int size = x_.size();
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
		P_ = (I - K * H_) * P_;
	}

private:

	bool is_initialized_;

	// state vector
	Eigen::VectorXd x_;

	// state transistion matrix
	Eigen::VectorXd F_;

	// state covariance matrix
	Eigen::VectorXd P_;

	// process coveriance matrix
	Eigen::VectorXd Q_;

	// measurement matrix
	Eigen::VectorXd H_;

	// measurement covariance matrix
	Eigen::VectorXd R_;

};
#endif
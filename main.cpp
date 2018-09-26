#include "kalman_filter.hpp"
#include <iostream>

bool GetLidarData(double mx, double my, double timestamp)
{
	return true;
}
int main()
{
	double m_x = 0.0, m_y = 0.0;
	double last_timestamp = 0.0, now_timestamp = 0.0;
	KalmanFilter kf;
	while(GetLidarData(m_x, m_y, now_timestamp)){
		if(!kf.isInitialized()){
			last_timestamp = now_timestamp;
			Eigen::VectorXd x_in(4,1);
			x_in << m_x, m_y, 0.0, 0.0;
			kf.Initializetion(x_in);

			Eigen::MatrixXd P_in(4, 4);
			P_in << 1.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 100.0, 0.0,
					0.0, 0.0, 0.0, 100.0,
			kf.SetP(P_in);

			Eigen::MatrixXd Q_in(4, 4);
			Q_in << 1.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0,
					0.0, 0.0, 0.0, 1.0,
			kf.SetQ(Q_in);

			Eigen::MatrixXd H_in(2, 4);
			H_in << 1.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0,
			kf.SetH(H_in);

			Eigen::MatrixXd R_in(2, 4);
			R_in << 1.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0,
			kf.SetR(R_in);
			continue;
		}

		double delta_t = now_timestamp - last_timestamp;
		last_timestamp = now_timestamp;
		Eigen::MatrixXd F_in(4, 4);
		F_in << 1.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, delta_t, 0.0,
				0.0, 0.0, 0.0, delta_t,
		kf.SetF(F_in);
		kf.Prediction();

		Eigen::VectorXd z(2, 1);
		z << m_x, m_y;
		kf.MeasurementUpdate(z);

		Eigen::VectorXd x_out = kf.GetX();
		std::cout << "kalman output x : " << x_out(0) <<
					" y : " << x_out(1) << std::endl;
	}
	return 0;
}

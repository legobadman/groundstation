#ifndef __LEVENBERG_MARQUARDT_H__
#define __LEVENBERG_MARQUARDT_H__

#include <Eigen/Eigen>

class LevenbergMarquardt
{
public:
	LevenbergMarquardt(double* a, double* b, double* c);
	void setParameters(double epsilon_1, double epsilon_2, int max_iter, bool is_out);
	void addObservation(const double& x, const double& y);
	void calcJ_fx();
	void calcH_g();
	double getCost();
	double F(double a, double b, double c);
	double L0_L(Eigen::Vector3d& h);
	void solve();

private:
	Eigen::MatrixXd fx_;
	Eigen::MatrixXd J_;	// 雅克比矩阵
	Eigen::Matrix3d H_; // H矩阵
	Eigen::Vector3d g_;
	
	std::vector< Eigen::Vector2d> obs_; // 观测
	/* 要求的三个参数 */
	double* a_, * b_, * c_;
	/* parameters */
	double epsilon_1_, epsilon_2_;
	int max_iter_;
	bool is_out_;
};

#endif

#include "LevenbergMarquardt.h"
#include <iostream>
#include <iomanip>
#include <functional>
#include <chrono>

/* ��ʱ�� */
class Runtimer {
public:
	inline void start()
	{
		t_s_ = std::chrono::steady_clock::now();
	}

	inline void stop()
	{
		t_e_ = std::chrono::steady_clock::now();
	}

	inline double duration()
	{
		return std::chrono::duration_cast<std::chrono::duration<double>>(t_e_ - t_s_).count() * 1000.0;
	}

private:
	std::chrono::steady_clock::time_point t_s_; //start time ponit
	std::chrono::steady_clock::time_point t_e_; //stop time point
};


LevenbergMarquardt::LevenbergMarquardt(double* a, double* b, double* c) :
	a_(a), b_(b), c_(c)
{
	epsilon_1_ = 1e-6;
	epsilon_2_ = 1e-6;
	max_iter_ = 50;
	is_out_ = true;
}

void LevenbergMarquardt::setParameters(double epsilon_1, double epsilon_2, int max_iter, bool is_out)
{
	epsilon_1_ = epsilon_1;
	epsilon_2_ = epsilon_2;
	max_iter_ = max_iter;
	is_out_ = is_out;
}

void LevenbergMarquardt::addObservation(const double& x, const double& y)
{
	obs_.push_back(Eigen::Vector2d(x, y));
}

void LevenbergMarquardt::calcJ_fx()
{
	J_.resize(obs_.size(), 3);
	fx_.resize(obs_.size(), 1);

	for (size_t i = 0; i < obs_.size(); i++)
	{
		const Eigen::Vector2d& ob = obs_.at(i);
		const double& x = ob(0);
		const double& y = ob(1);
		double j1 = -x * x * exp(*a_ * x * x + *b_ * x + *c_);
		double j2 = -x * exp(*a_ * x * x + *b_ * x + *c_);
		double j3 = -exp(*a_ * x * x + *b_ * x + *c_);
		J_(i, 0) = j1;
		J_(i, 1) = j2;
		J_(i, 2) = j3;
		fx_(i, 0) = y - exp(*a_ * x * x + *b_ * x + *c_);
	}
}

void LevenbergMarquardt::calcH_g()
{
	H_ = J_.transpose() * J_;
	g_ = -J_.transpose() * fx_;
}

double LevenbergMarquardt::getCost()
{
	Eigen::MatrixXd cost = fx_.transpose() * fx_;
	return cost(0, 0);
}

double LevenbergMarquardt::F(double a, double b, double c)
{
	Eigen::MatrixXd fx;
	fx.resize(obs_.size(), 1);

	for (size_t i = 0; i < obs_.size(); i++)
	{
		const Eigen::Vector2d& ob = obs_.at(i);
		const double& x = ob(0);
		const double& y = ob(1);
		fx(i, 0) = y - exp(a * x * x + b * x + c);
	}
	Eigen::MatrixXd F = 0.5 * fx.transpose() * fx;
	return F(0, 0);
}

double LevenbergMarquardt::L0_L(Eigen::Vector3d& h)
{
	Eigen::MatrixXd L = -h.transpose() * J_.transpose() * fx_ - 0.5 * h.transpose() * J_.transpose() * J_ * h;
	return L(0, 0);
}

void LevenbergMarquardt::solve()
{
	int k = 0;
	double nu = 2.0;
	calcJ_fx();
	calcH_g();
	bool found = (g_.lpNorm<Eigen::Infinity>() < epsilon_1_);

	std::vector<double> A;
	A.push_back(H_(0, 0));
	A.push_back(H_(1, 1));
	A.push_back(H_(2, 2));
	auto max_p = std::max_element(A.begin(), A.end());
	double mu = *max_p;

	double sumt = 0;

	while (!found && k < max_iter_)
	{
		Runtimer t;
		t.start();

		k = k + 1;
		Eigen::Matrix3d G = H_ + mu * Eigen::Matrix3d::Identity();
		Eigen::Vector3d h = G.ldlt().solve(g_);

		if (h.norm() <= epsilon_2_ * (sqrt(*a_ * *a_ + *b_ * *b_ + *c_ * *c_) + epsilon_2_))
			found = true;
		else
		{
			double na = *a_ + h(0);
			double nb = *b_ + h(1);
			double nc = *c_ + h(2);

			double rho = (F(*a_, *b_, *c_) - F(na, nb, nc)) / L0_L(h);

			if (rho > 0)
			{
				*a_ = na;
				*b_ = nb;
				*c_ = nc;
				calcJ_fx();
				calcH_g();

				found = (g_.lpNorm<Eigen::Infinity>() < epsilon_1_);
				mu = mu * std::max<double>(0.33, 1 - std::pow(2 * rho - 1, 3));
				nu = 2.0;
			}
			else
			{
				mu = mu * nu;
				nu = 2 * nu;
			}// if rho > 0
		}// if step is too small

		t.stop();
		if (is_out_)
		{
			std::cout << "Iter: " << std::left << std::setw(3) << k << " Result: " << std::left << std::setw(10) << *a_ << " " << std::left << std::setw(10) << *b_ << " " << std::left << std::setw(10) << *c_ <<
				" step: " << std::left << std::setw(14) << h.norm() << " cost: " << std::left << std::setw(14) << getCost() << " time: " << std::left << std::setw(14) << t.duration() <<
				" total_time: " << std::left << std::setw(14) << (sumt += t.duration()) << std::endl;
		}
	} // while

	if (found == true)
		std::cout << "\nConverged\n\n";
	else
		std::cout << "\nDiverged\n\n";

}//function 
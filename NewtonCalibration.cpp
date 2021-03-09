#include "NewtonCalibration.h"

NewtonCalibration::NewtonCalibration(const Eigen::MatrixXf& X_)
{
	epsilon = 0.005f;
	method = NEWTON;
	X = X_;
}

Vector3f NewtonCalibration::f(const VEC_THETA_TYPE& Theta, const Vector3f& a)
{
	float psi_a = Theta(0), theta_a = Theta(1), phi_a = Theta(2),
		Sax = Theta(3), Say = Theta(4), Saz = Theta(5), bax = Theta(6),
		bay = Theta(7), baz = Theta(8);
	Vector3f ba(bax, bay, baz);
	Vector3f A = a + ba;
	Matrix3f TK;
	TK << Sax, psi_a, -theta_a,
		-psi_a, Say, phi_a,
		theta_a, -phi_a, Saz;
	Vector3f result = TK * A;
	return result;
}

//MatrixXd NewtonCalibration::f(const VectorXd& Theta)
//{
//	float psi_a = Theta(0), theta_a = Theta(1), phi_a = Theta(2),
//		Sax = Theta(3), Say = Theta(4), Saz = Theta(5), bax = Theta(6),
//		bay = Theta(7), baz = Theta(8);
//	Vector3f ba(bax, bay, baz);
//	MatrixXd X_ = X + ba;
//	Matrix3f TK;
//	TK << Sax, psi_a, -theta_a,
//		-psi_a, Say, phi_a,
//		theta_a, -phi_a, Saz;
//	MatrixXd result = TK * X_;
//	return result;
//}

float NewtonCalibration::J(const VEC_THETA_TYPE& Theta)
{
	float Jtheta = 0;
	for (int i = 0; i < X.rows(); i++)
	{
		Vector3f a = Vector3f(X(i, 0), X(i, 1), X(i, 2));
		Vector3f Ha_i = f(Theta, a);
		Jtheta += 1.0f / 2 * std::pow((Ha_i.norm() - 1), 2);

		grad_f(Theta, a);
	}
	return Jtheta;
}

Matrix<float, 9, 3> NewtonCalibration::grad_f(const VEC_THETA_TYPE& Theta, const Vector3f& a)
{
	float ax = a(0), ay = a(1), az = a(2);
	float d_psi = Theta(0), d_theta = Theta(1), d_phi = Theta(2),
		Sx = Theta(3), Sy = Theta(4), Sz = Theta(5),
		bx = Theta(6), by = Theta(7), bz = Theta(8);
	Matrix<float, 9, 3> grad_H_to_theta;
	grad_H_to_theta << ay + by, -(ax + bx), 0,
		-(az + bz), 0, ax + bx,
		0, az + bz, -(ay + by),
		ax + bx, 0, 0,
		0, ay + by, 0,
		0, 0, az + bz,
		Sx, -d_psi, d_theta,
		d_psi, Sy, -d_phi,
		-d_theta, d_phi, Sz;
	return grad_H_to_theta;
}

VEC_THETA_TYPE NewtonCalibration::grad_J(const VEC_THETA_TYPE& Theta)
{
	VEC_THETA_TYPE sum;
	sum << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	int m = X.rows();
	for (int i = 0; i < m; i++)
	{
		Vector3f a = Vector3f(X(i, 0), X(i, 1), X(i, 2));
		Vector3f Ha_i = f(Theta, a);
		float ha_norm = Ha_i.norm();

		Matrix<float, 9, 3> grad_H_to_theta = grad_f(Theta, a);
		VEC_THETA_TYPE der = ((ha_norm - 1) / ha_norm) * grad_H_to_theta * Ha_i;
		sum = sum + der;
	}
	return sum;
}

HESSIAN_TYPE NewtonCalibration::Hessian_J(const VEC_THETA_TYPE& Theta)
{
	HESSIAN_TYPE sum = HESSIAN_TYPE::Zero();
	int m = X.rows();
	for (int i = 0; i < m; i++)
	{
		Vector3f a = Vector3f(X(i, 0), X(i, 1), X(i, 2));
		Vector3f ha_i = f(Theta, a);
		Matrix<float, 1, 3> ha_i_(ha_i[0], ha_i[1], ha_i[2]);
		float ha_norm = ha_i_.norm();
		Matrix<float, 9, 3> grad_H_to_theta = grad_f(Theta, a);
		Matrix<float, 9, 1> gH = grad_H_to_theta * ha_i;
		Matrix<float, 1, 9> hG = ha_i_ * grad_H_to_theta.transpose();

		HESSIAN_TYPE part1 = (1. / std::pow(ha_norm, 3)) * gH * hG;
		HESSIAN_TYPE part2 = grad_H_to_theta * grad_H_to_theta.transpose();
		HESSIAN_TYPE H_;
		H_ << 0, 0, 0, 0, 0, 0, -ha_i[1], ha_i[0], 0,	//[ay + by, -(ax + bx), 0]
			0, 0, 0, 0, 0, 0, ha_i[2], 0, -ha_i[0],		//[-(az + bz), 0, ax + bx]
			0, 0, 0, 0, 0, 0, 0, -ha_i[2], ha_i[1],		//[0, az + bz, -(ay + by)]
			0, 0, 0, 0, 0, 0, ha_i[0], 0, 0,			//[ax + bx, 0, 0]
			0, 0, 0, 0, 0, 0, 0, ha_i[1], 0,			//[0, ay + by, 0]
			0, 0, 0, 0, 0, 0, 0, 0, ha_i[2],				//[0, 0, az + bz]
			-ha_i[1], ha_i[2], 0, ha_i[0], 0, 0, 0, 0, 0,	//[Sx, -d_psi, d_theta]
			ha_i[0], 0, -ha_i[2], 0, ha_i[1], 0, 0, 0, 0,	//[d_psi, Sy, -d_phi]
			0, -ha_i[0], ha_i[1], 0, 0, ha_i[2], 0, 0, 0;	//[-d_theta, d_phi, Sz]
		HESSIAN_TYPE der = part1 + part2 + H_;
		sum = sum + der;
	}
	return sum;
}

VEC_THETA_TYPE NewtonCalibration::get_search_direction(const VEC_THETA_TYPE& Theta, float& lamba_square)
{
	lamba_square = 0;
	VEC_THETA_TYPE delta_theta;
	VEC_THETA_TYPE grad_theta = grad_J(Theta);
	if (method == NEWTON) {
		HESSIAN_TYPE H_theta = Hessian_J(Theta);
		HESSIAN_TYPE inv_H_theta = H_theta.inverse();
		delta_theta = -1 * inv_H_theta * grad_theta;
		lamba_square = (grad_theta.transpose() * inv_H_theta * grad_theta)(0);
	}
	else {
		delta_theta = -1 * grad_theta;
	}
	return delta_theta;
}

bool NewtonCalibration::stop_criteria(const VEC_THETA_TYPE delta_theta, float lambda_square)
{
	if (method == NEWTON) {
		return lambda_square / 2.0 <= epsilon;
	}
	else {
		return delta_theta.norm() <= epsilon;
	}
}

void NewtonCalibration::solve() {
	VEC_THETA_TYPE Theta;
	Theta << 0, 0, 0, 1, 1, 1, 0, 0, 0;

	float epsilon = 0.005f;
	while (true) {
		float lambda_square = 0;
		VEC_THETA_TYPE delta_Theta = get_search_direction(Theta, lambda_square);
		if (stop_criteria(delta_Theta, lambda_square)) {
			Theta_start = Theta;
			break;
		}
		float t = get_step();
		Theta = Theta + t * delta_Theta;
		float cost = J(Theta);
		printf("%f\n", cost);
	}
}

void NewtonCalibration::get_optimized_result(Matrix3f& TK, Vector3f& b)
{
	//Theta = [psi_a, theta_a, phi_a,Sax, Say, Saz, bax, bay, baz]
	//TK = np.array([[Sax, psi_a, -theta_a],
	//	[-psi_a, Say, phi_a],
	//	[theta_a, -phi_a, Saz]] )
	TK << Theta_start(3), Theta_start(0), -Theta_start(1),
		-Theta_start(0), Theta_start(4), Theta_start(2),
		Theta_start(1), -Theta_start(2), Theta_start(5);
	b << Theta_start(6), Theta_start(7), Theta_start(8);
}

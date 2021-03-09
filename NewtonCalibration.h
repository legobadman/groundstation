#ifndef __NEWTON_CALIBRATION_H__
#define __NEWTON_CALIBRATION_H__

#include <Eigen/Eigen>

using namespace Eigen;

typedef Matrix<float, 9, 1> VEC_THETA_TYPE;
typedef Matrix<float, 9, 9> HESSIAN_TYPE;

class NewtonCalibration
{
	enum DESCENT_METHOD
	{
		GRADIENT_DESCENT,
		NEWTON
	};

public:
	NewtonCalibration(const Eigen::MatrixXf& X_);
	Vector3f f(const VEC_THETA_TYPE& Theta, const Vector3f& a);	//�������յ�Ŀ�꺯����ֻ�Ǽ���У׼�������ĺ�����
	float J(const VEC_THETA_TYPE& Theta);		//���Ż���Ŀ�꺯����
	Matrix<float, 9, 3> grad_f(const VEC_THETA_TYPE& Theta, const Vector3f& a);
	VEC_THETA_TYPE grad_J(const VEC_THETA_TYPE& Theta);
	HESSIAN_TYPE Hessian_J(const VEC_THETA_TYPE& Theta);
	VEC_THETA_TYPE get_search_direction(const VEC_THETA_TYPE& Theta, float& lamba_square);
	float get_step() {
		if (method == NEWTON) {
			return 0.1f;
		}
		else {
			return 0.0001f;
		}
	}
	bool stop_criteria(const VEC_THETA_TYPE delta_theta, float lambda_square);
	void solve();
	void get_optimized_result(Matrix3f& TK, Vector3f& b);

private:
	MatrixXf X;
	VEC_THETA_TYPE Theta_start;
	float epsilon;
	DESCENT_METHOD method;
};

#endif // __NEWTON_CALIBRATION_H__


/****************************************************************************
 *
 *   Flatness Feedforward for Morphing Quadrotor
 *
 ****************************************************************************/

#include "FlatnessFeedforward.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

bool FlatnessFeedforward::computeSimple(const FlatOutput &xi, float mass, Feedforward &out)
{
	Vector3f epsilon(xi.acc(0), xi.acc(1), xi.acc(2) + _g);
	float epsilon_norm = epsilon.norm();

	if (epsilon_norm < 1e-3f) {
		return false;
	}

	out.collective_thrust = mass * epsilon_norm;
	out.body_z = epsilon / epsilon_norm;

	if (!_computeAttitude(out.body_z, xi.yaw, out.R)) {
		return false;
	}

	out.angular_velocity.zero();
	out.angular_acceleration.zero();
	out.servo_angle = xi.arm_angle;
	out.servo_rate = xi.arm_angle_dot;
	return true;
}

bool FlatnessFeedforward::compute(const FlatOutput &xi, float mass,
                                  const SquareMatrix3f &J, float thrust_k,
                                  Feedforward &out)
{
	if (!computeSimple(xi, mass, out)) {
		return false;
	}

	Vector3f epsilon(xi.acc(0), xi.acc(1), xi.acc(2) + _g);
	float epsilon_norm = epsilon.norm();
	float thrust = mass * epsilon_norm;

	if (!_computeAngularVelocity(out.body_z, xi.acc, xi.jerk,
				     thrust, mass, xi.yaw_dot, out.R,
				     out.angular_velocity)) {
		out.angular_velocity.zero();
	}

	float thrust_dot = mass * (xi.jerk.dot(out.body_z));

	if (!_computeAngularAcceleration(out.body_z, xi.acc, xi.jerk,
						 xi.snap, thrust, thrust_dot,
						 mass, xi.yaw_dot, xi.yaw_ddot, out.R,
						 out.angular_velocity,
						 out.angular_acceleration)) {
		out.angular_acceleration.zero();
	}

	return true;
}

bool FlatnessFeedforward::_computeAttitude(const Vector3f &z_b, float yaw, Matrix3f &R) const
{
	// x_c = [cos(yaw), sin(yaw), 0] in world frame
	Vector3f x_c(cosf(yaw), sinf(yaw), 0.0f);

	// y_b = normalize(z_b x x_c)
	Vector3f y_b = z_b.cross(x_c);
	float y_b_norm = y_b.norm();

	if (y_b_norm < 1e-4f) {
		// z_b is parallel to z_w (yaw irrelevant); pick arbitrary x_b
		// This happens at hover or pure vertical acceleration
		Vector3f x_b(1.0f, 0.0f, 0.0f);
		if (fabsf(z_b(2) - 1.0f) < 1e-3f) {
			// z_b points up: x_b along world x
			x_b = Vector3f(cosf(yaw), sinf(yaw), 0.0f);
		} else if (fabsf(z_b(2) + 1.0f) < 1e-3f) {
			// z_b points down (inverted flight)
			x_b = Vector3f(cosf(yaw), -sinf(yaw), 0.0f);
		} else {
			// General case: find orthogonal vector
			Vector3f ref(0.0f, 0.0f, 1.0f);
			x_b = ref.cross(z_b);
			x_b = x_b.normalized();
		}
		y_b = z_b.cross(x_b);
		R.setCol(0, x_b);
		R.setCol(1, y_b);
		R.setCol(2, z_b);
		return true;
	}

	y_b = y_b / y_b_norm;
	Vector3f x_b = y_b.cross(z_b);

	R.setCol(0, x_b);
	R.setCol(1, y_b);
	R.setCol(2, z_b);
	return true;
}

bool FlatnessFeedforward::_computeAngularVelocity(
	const Vector3f &z_b, const Vector3f &acc,
	const Vector3f &jerk, float thrust, float mass,
	float yaw_dot, const Matrix3f &R, Vector3f &omega) const
{
	if (thrust < 1e-3f) {
		return false;
	}

	// Gamma = (m/T) * [a_dot - (a_dot . z_b) * z_b]
	Vector3f a_dot = jerk;
	float adot_zb = a_dot.dot(z_b);
	Vector3f Gamma = (mass / thrust) * (a_dot - z_b * adot_zb);

	// Body axes
	Vector3f x_b = R.col(0);
	Vector3f y_b = R.col(1);

	// omega x z_b = -p*y_b + q*x_b = Gamma
	// => p = -Gamma . y_b,  q = Gamma . x_b
	float p = -Gamma.dot(y_b);
	float q = Gamma.dot(x_b);

	// r = yaw_dot * (z_w . z_b)
	Vector3f z_w(0.0f, 0.0f, 1.0f);
	float r = yaw_dot * z_w.dot(z_b);

	omega = Vector3f(p, q, r);
	return true;
}

bool FlatnessFeedforward::_computeAngularAcceleration(
	const Vector3f &z_b, const Vector3f &acc,
	const Vector3f &jerk, const Vector3f &snap,
	float thrust, float thrust_dot, float mass,
	float yaw_dot, float yaw_ddot, const Matrix3f &R,
	const Vector3f &omega, Vector3f &alpha) const
{
	if (thrust < 1e-3f) {
		return false;
	}

	// Compute Gamma = (m/T) * [jerk - (jerk.z_b)*z_b]
	float jz = jerk.dot(z_b);
	Vector3f Gamma = (mass / thrust) * (jerk - z_b * jz);

	// Gamma_dot = -(m*T_dot/T^2)*[jerk - (jerk.z_b)*z_b]
	//           + (m/T)*[snap - (snap.z_b + jerk.(omega x z_b))*z_b - (jerk.z_b)*(omega x z_b)]
	Vector3f omega_world = R * omega;
	Vector3f omega_cross_zb = omega_world.cross(z_b);
	float sz = snap.dot(z_b);
	float j_oz = jerk.dot(omega_cross_zb);
	Vector3f Gamma_dot = -(mass * thrust_dot / (thrust * thrust)) * (jerk - z_b * jz)
			   + (mass / thrust) * (snap - z_b * (sz + j_oz) - omega_cross_zb * jz);

	// alpha x z_b = Gamma_dot - (omega x Gamma)
	Vector3f rhs = Gamma_dot - omega_world.cross(Gamma);

	Vector3f x_b = R.col(0);
	Vector3f y_b = R.col(1);

	float p_dot = -rhs.dot(y_b);
	float q_dot = rhs.dot(x_b);

	// r_dot from yaw_ddot: differentiate r = yaw_dot * (z_w . z_b)
	Vector3f z_w(0.0f, 0.0f, 1.0f);
	float r_dot = yaw_ddot * z_w.dot(z_b) + yaw_dot * z_w.dot(omega_cross_zb);

	alpha = Vector3f(p_dot, q_dot, r_dot);
	return true;
}

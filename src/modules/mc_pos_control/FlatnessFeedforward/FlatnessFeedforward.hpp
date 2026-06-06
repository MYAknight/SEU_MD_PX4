/****************************************************************************
 *
 *   Flatness Feedforward for Morphing Quadrotor
 *   Computes feedforward thrust, attitude, and angular rates from
 *   differential flatness of the morphing quadrotor system.
 *
 *   Flat output: ξ = [x, y, z, ψ, arm_angle]
 *   Reference: Cui et al. "Motion Planning and Control of A Morphing
 *   Quadrotor in Restricted Scenarios" (arXiv:2312.07075)
 *
 ****************************************************************************/

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;

/**
 * FlatnessFeedforward computes feedforward control terms from a
 * differentially-flat trajectory for a morphing quadrotor.
 *
 * Assumptions:
 *  - Arms are inertialess (inertia matrix is a function of arm_angle only)
 *  - Thrust vector is aligned with body z-axis
 *  - Flat outputs: position (x,y,z), yaw (ψ), arm_angle
 */
class FlatnessFeedforward
{
public:
	FlatnessFeedforward() = default;
	~FlatnessFeedforward() = default;

	/**
	 * Flat output and its time derivatives up to snap (4th order).
	 * For minimum-snap trajectory tracking, jerk and snap are used
	 * to compute angular velocity and angular acceleration feedforward.
	 */
	struct FlatOutput {
		// Position and derivatives (NED frame)
		Vector3f pos{0.f, 0.f, 0.f};
		Vector3f vel{0.f, 0.f, 0.f};
		Vector3f acc{0.f, 0.f, 0.f};
		Vector3f jerk{0.f, 0.f, 0.f};
		Vector3f snap{0.f, 0.f, 0.f};

		// Yaw and derivatives
		float yaw{0.f};
		float yaw_dot{0.f};
		float yaw_ddot{0.f};

		// Morphing angle and derivatives
		float arm_angle{0.f};
		float arm_angle_dot{0.f};
		float arm_angle_ddot{0.f};
	};

	/**
	 * Feedforward control outputs.
	 * These are added to the feedback controller outputs.
	 */
	struct Feedforward {
		// Collective thrust [N] (positive = upward)
		float collective_thrust{0.f};

		// Body z-axis in world frame (unit vector)
		Vector3f body_z{0.f, 0.f, 1.f};

		// Desired attitude as rotation matrix (body frame in world)
		// R = [x_b, y_b, z_b]
		Matrix3f R;

		// Body-frame angular velocity feedforward [rad/s]
		Vector3f angular_velocity{0.f, 0.f, 0.f};

		// Body-frame angular acceleration feedforward [rad/s^2]
		Vector3f angular_acceleration{0.f, 0.f, 0.f};

		// Servo motor angle for morphing [rad]
		float servo_angle{0.f};

		// Servo motor rate [rad/s]
		float servo_rate{0.f};
	};

	/**
	 * Compute feedforward from flat output.
	 *
	 * @param xi        flat output and derivatives
	 * @param mass      vehicle mass [kg]
	 * @param J         inertia tensor [kg·m^2] at current arm_angle
	 * @param thrust_k  thrust coefficient (rotor drag model)
	 * @param out       feedforward outputs
	 * @return true if computation succeeded
	 */
	bool compute(const FlatOutput &xi, float mass,
		     const SquareMatrix3f &J, float thrust_k,
		     Feedforward &out);

	/**
	 * Simplified compute: only position-level flatness (acc → attitude + thrust).
	 * Does not compute angular velocity/acceleration feedforward.
	 * Use this when jerk/snap are not available.
	 */
	bool computeSimple(const FlatOutput &xi, float mass, Feedforward &out);

	/**
	 * Set the gravity constant (default 9.80665)
	 */
	void setGravity(float g) { _g = g; }

private:
	float _g{9.80665f};

	// Helper: reconstruct rotation from z_b and yaw
	bool _computeAttitude(const Vector3f &z_b, float yaw, Matrix3f &R) const;

	// Helper: compute angular velocity from flat output derivatives
	bool _computeAngularVelocity(const Vector3f &z_b, const Vector3f &acc,
				     const Vector3f &jerk, float thrust,
				     float mass, float yaw_dot,
				     const Matrix3f &R, Vector3f &omega) const;

	// Helper: compute angular acceleration from snap
	bool _computeAngularAcceleration(const Vector3f &z_b, const Vector3f &acc,
					 const Vector3f &jerk, const Vector3f &snap,
					 float thrust, float thrust_dot,
					 float mass, float yaw_dot, float yaw_ddot,
					 const Matrix3f &R, const Vector3f &omega,
					 Vector3f &alpha) const;
};

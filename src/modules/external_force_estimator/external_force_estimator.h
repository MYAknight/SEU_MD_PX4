#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/external_force_estimate.h>
#include <uORB/topics/contact_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

using namespace time_literals;

extern "C" __EXPORT int external_force_estimator_main(int argc, char *argv[]);

/**
 * @brief IMU-Based Impulsive Contact Detector (IMU-ICD)
 *
 * Detects contact/collision events using raw IMU accelerometer data
 * without external sensors. Based on the observation that physical
 * contact introduces high-frequency transient accelerations that are
 * visible in raw IMU measurements before EKF smoothing.
 *
 * Academic basis:
 *  - Haddadin et al., "External Wrench Estimation, Collision Detection,
 *    and Reflex Reaction for Flying Robots", IEEE T-RO 2017
 *  - Air Bumper (arXiv 2307.06101): IMU-only collision detection via
 *    acceleration thresholding and sliding-window analysis
 *
 * Implementation:
 *  - Subscribes to sensor_combined (raw accelerometer, body frame)
 *  - Applies high-pass filter to remove gravity DC component
 *  - Computes short-term variance over a sliding window
 *  - Triggers contact FSM when variance/impulse exceeds threshold
 *  - Publishes external_force_estimate and contact_state uORB topics
 */
class ExternalForceEstimator : public ModuleBase<ExternalForceEstimator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	ExternalForceEstimator();
	virtual ~ExternalForceEstimator() = default;

	static int task_spawn(int argc, char *argv[]);
	static ExternalForceEstimator *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

	void Run() override;

private:
	// Contact state machine states
	static constexpr uint8_t STATE_NO_CONTACT = 0;
	static constexpr uint8_t STATE_IMPACT = 1;
	static constexpr uint8_t STATE_CONFIRMED = 2;
	static constexpr uint8_t STATE_STABLE = 3;
	static constexpr uint8_t STATE_SLIPPING = 4;

	static constexpr int BUF_SIZE = 20;  // 20ms window @ 1kHz

	void parameters_update(bool force = false);

	// Core IMU-ICD detection
	void updateImuIcd(float a_mag, float a_hpf, float gyro_mag, float dt);

	// Contact detection FSM
	void updateContactFsm(float impact_metric, float gyro_mag, float t);

	// Publications
	uORB::Publication<external_force_estimate_s> _external_force_pub{ORB_ID(external_force_estimate)};
	uORB::Publication<contact_state_s> _contact_state_pub{ORB_ID(contact_state)};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	// === IMU-ICD internal state ===
	// HPF state for gravity removal
	float _a_mag_prev{0.f};
	float _a_hpf_prev{0.f};
	static constexpr float HPF_ALPHA = 0.90f;

	// Sliding window for variance computation
	float _acc_buffer[BUF_SIZE] {};
	int _buf_head{0};
	int _buf_count{0};

	// Contact detector state
	uint8_t _contact_state{STATE_NO_CONTACT};
	float _contact_confidence{0.f};
	float _contact_start_time{0.f};
	bool _should_close{false};

	// Last impact metric (published as "force magnitude")
	float _impact_metric{0.f};
	float _impact_metric_lpf{0.f};

	// Last timestamp
	hrt_abstime _last_update_time{0};

	// Mavlink log publisher
	orb_advert_t _mavlink_log_pub{nullptr};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EFO_ENABLE>) _param_enable,
		(ParamFloat<px4::params::EFO_MASS>) _param_mass,
		(ParamFloat<px4::params::EFO_FTHR>) _param_force_thr,
		(ParamFloat<px4::params::EFO_TTHR>) _param_time_thr,
		(ParamFloat<px4::params::EFO_GTHR>) _param_gyro_thr,
		(ParamFloat<px4::params::EFO_LWIN>) _param_long_win,
		(ParamFloat<px4::params::EFO_SWIN>) _param_short_win,
		(ParamFloat<px4::params::EFO_LPF>) _param_lpf_alpha
	)
};

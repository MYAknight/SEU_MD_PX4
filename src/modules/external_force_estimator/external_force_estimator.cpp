#include "external_force_estimator.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <cmath>

using matrix::Vector3f;

ExternalForceEstimator::ExternalForceEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

void ExternalForceEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	if (_param_enable.get() <= 0) {
		// Module disabled: publish no-contact state and skip processing
		const hrt_abstime now = hrt_absolute_time();
		contact_state_s cs{};
		cs.timestamp = now;
		cs.state = STATE_NO_CONTACT;
		cs.should_close = false;
		cs.contact_duration = 0.f;
		cs.contact_force[0] = 0.f;
		cs.contact_force[1] = 0.f;
		cs.contact_force[2] = 0.f;
		_contact_state_pub.publish(cs);
		return;
	}

	sensor_combined_s imu{};
	vehicle_angular_velocity_s ang_vel{};

	bool imu_updated = _sensor_combined_sub.update(&imu);
	bool ang_updated = _vehicle_angular_velocity_sub.update(&ang_vel);

	if (!imu_updated && !ang_updated) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.001f;  // default 1kHz assumption

	if (_last_update_time > 0) {
		dt = math::constrain((now - _last_update_time) * 1e-6f, 0.0002f, 0.005f);
	}

	_last_update_time = now;

	// === IMU-Based Impulsive Contact Detection (IMU-ICD) ===
	// Use raw accelerometer data from sensor_combined (body frame, includes gravity)
	float ax = imu.accelerometer_m_s2[0];
	float ay = imu.accelerometer_m_s2[1];
	float az = imu.accelerometer_m_s2[2];
	float a_mag = sqrtf(ax * ax + ay * ay + az * az);

	// High-pass filter to remove gravity DC component
	// y[n] = alpha * (y[n-1] + x[n] - x[n-1])
	float a_hpf = HPF_ALPHA * (_a_hpf_prev + a_mag - _a_mag_prev);
	_a_mag_prev = a_mag;
	_a_hpf_prev = a_hpf;

	// Gyro magnitude
	float gyro_mag = 0.f;
	if (ang_updated) {
		gyro_mag = sqrtf(ang_vel.xyz[0] * ang_vel.xyz[0]
				 + ang_vel.xyz[1] * ang_vel.xyz[1]
				 + ang_vel.xyz[2] * ang_vel.xyz[2]);
	}

	// Run core detector
	updateImuIcd(a_mag, a_hpf, gyro_mag, dt);

	// Publish external force estimate (use impact metric as "force")
	external_force_estimate_s efo{};
	efo.timestamp = now;
	efo.force[0] = ax;  // raw accel x (for diagnostics)
	efo.force[1] = ay;  // raw accel y
	efo.force[2] = az;  // raw accel z
	efo.force_magnitude = _impact_metric_lpf;
	efo.confidence = _contact_confidence;
	_external_force_pub.publish(efo);

	// Publish contact state
	contact_state_s cs{};
	cs.timestamp = now;
	cs.state = _contact_state;
	cs.should_close = _should_close;
	cs.contact_duration = (_contact_state != STATE_NO_CONTACT) ? ((now * 1e-6f) - _contact_start_time) : 0.f;
	cs.contact_force[0] = _impact_metric_lpf;
	cs.contact_force[1] = a_hpf;
	cs.contact_force[2] = gyro_mag;
	_contact_state_pub.publish(cs);

	// Debug output every ~1s
	static int dbg_cnt = 0;
	if (++dbg_cnt % 1000 == 0) {
		PX4_INFO("IMU-ICD: state=%d impact=%.2f a_mag=%.2f a_hpf=%.2f gyro=%.2f",
			 _contact_state, (double)_impact_metric_lpf, (double)a_mag,
			 (double)a_hpf, (double)gyro_mag);
	}
}

void ExternalForceEstimator::updateImuIcd(float a_mag, float a_hpf, float gyro_mag, float dt)
{
	// Store in sliding window
	_acc_buffer[_buf_head] = a_mag;
	_buf_head = (_buf_head + 1) % BUF_SIZE;
	if (_buf_count < BUF_SIZE) {
		_buf_count++;
	}

	// Compute mean and standard deviation
	float mean = 0.f;
	for (int i = 0; i < _buf_count; i++) {
		mean += _acc_buffer[i];
	}
	mean /= _buf_count;

	float var = 0.f;
	for (int i = 0; i < _buf_count; i++) {
		float d = _acc_buffer[i] - mean;
		var += d * d;
	}
	var /= _buf_count;
	float std = sqrtf(var);

	// Impact metric: combines HPF amplitude and short-term variance
	// HPF captures the impulsive peak; variance captures the disturbance energy
	float alpha_imp = 0.8f;
	_impact_metric = fabsf(a_hpf) + 2.0f * std;
	_impact_metric_lpf = alpha_imp * _impact_metric_lpf + (1.0f - alpha_imp) * _impact_metric;

	// Run contact FSM
	float t = hrt_absolute_time() * 1e-6f;
	updateContactFsm(_impact_metric_lpf, gyro_mag, t);
}

void ExternalForceEstimator::updateContactFsm(float impact_metric, float gyro_mag, float t)
{
	// Parameters (re-purposed from original GMO params):
	// EFO_FTHR -> impact threshold for contact detection
	// EFO_TTHR -> minimum duration to confirm contact
	// EFO_GTHR -> gyro threshold for stable detection
	float impact_thr = _param_force_thr.get();   // default 2.0 -> mapped to impact metric
	float t_thr = _param_time_thr.get();         // default 0.03s
	float g_thr = _param_gyro_thr.get();         // default 0.15 rad/s

	switch (_contact_state) {
	case STATE_NO_CONTACT:
		if (impact_metric > impact_thr) {
			_contact_start_time = t;
			_contact_state = STATE_IMPACT;
			_contact_confidence = 0.3f;
			_should_close = false;
			mavlink_log_info(&_mavlink_log_pub, "IMU-ICD: IMPACT detected");
			PX4_INFO("IMU-ICD: IMPACT! metric=%.2f thr=%.2f", (double)impact_metric, (double)impact_thr);
		}
		break;

	case STATE_IMPACT:
		if (impact_metric < impact_thr * 0.3f) {
			_contact_state = STATE_NO_CONTACT;
			_contact_confidence = 0.0f;
			_should_close = false;
		} else if ((t - _contact_start_time) > t_thr) {
			_contact_state = STATE_CONFIRMED;
			_contact_confidence = 0.7f;
			mavlink_log_info(&_mavlink_log_pub, "IMU-ICD: contact CONFIRMED");
			PX4_INFO("IMU-ICD: CONFIRMED");
		}
		break;

	case STATE_CONFIRMED:
		if (gyro_mag < g_thr && impact_metric > impact_thr * 0.5f) {
			_contact_state = STATE_STABLE;
			_contact_confidence = 0.9f;
			mavlink_log_info(&_mavlink_log_pub, "IMU-ICD: contact STABLE");
			PX4_INFO("IMU-ICD: STABLE");
		} else if (impact_metric < impact_thr * 0.2f) {
			_contact_state = STATE_NO_CONTACT;
			_contact_confidence = 0.0f;
			_should_close = false;
		}
		break;

	case STATE_STABLE:
		if (gyro_mag > g_thr * 2.5f) {
			_contact_state = STATE_SLIPPING;
			_contact_confidence = 0.4f;
			_should_close = false;
		} else if (impact_metric < impact_thr * 0.15f) {
			_contact_state = STATE_NO_CONTACT;
			_contact_confidence = 0.0f;
			_should_close = false;
			mavlink_log_info(&_mavlink_log_pub, "IMU-ICD: contact LOST");
		} else if (_contact_confidence >= 0.85f) {
			_should_close = true;
		}
		break;

	case STATE_SLIPPING:
		if (gyro_mag < g_thr && impact_metric > impact_thr * 0.5f) {
			_contact_state = STATE_STABLE;
		} else if (impact_metric < impact_thr * 0.15f) {
			_contact_state = STATE_NO_CONTACT;
			_contact_confidence = 0.0f;
			_should_close = false;
		}
		break;
	}
}

void ExternalForceEstimator::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

int ExternalForceEstimator::print_status()
{
	PX4_INFO("IMU-Based Impulsive Contact Detector (IMU-ICD)");
	PX4_INFO("  State: %d, Confidence: %.2f, Close: %s",
		 _contact_state, (double)_contact_confidence, _should_close ? "true" : "false");
	PX4_INFO("  Impact metric: %.2f (lpf: %.2f)",
		 (double)_impact_metric, (double)_impact_metric_lpf);
	return 0;
}

int ExternalForceEstimator::task_spawn(int argc, char *argv[])
{
	ExternalForceEstimator *instance = new ExternalForceEstimator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

bool ExternalForceEstimator::init()
{
	ScheduleOnInterval(1000_us);  // 1kHz to match IMU rate
	return true;
}

ExternalForceEstimator *ExternalForceEstimator::instantiate(int argc, char *argv[])
{
	ExternalForceEstimator *instance = new ExternalForceEstimator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int ExternalForceEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ExternalForceEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
IMU-Based Impulsive Contact Detector (IMU-ICD) for sensorless
contact detection on morphing quadrotors.

Uses raw accelerometer data from sensor_combined to detect
contact events via high-frequency transient analysis, bypassing
EKF smoothing that attenuates collision impulses.

Academic basis:
- Haddadin et al., "External Wrench Estimation, Collision Detection,
  and Reflex Reaction for Flying Robots", IEEE T-RO 2017
- Air Bumper (arXiv 2307.06101): sliding-window IMU thresholding

### Implementation
- High-pass filter removes gravity DC component
- Sliding-window variance captures disturbance energy
- Impact metric = |HPF accel| + 2*std(accel)
- Publishes external_force_estimate and contact_state uORB topics

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("external_force_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int external_force_estimator_main(int argc, char *argv[])
{
	return ExternalForceEstimator::main(argc, argv);
}

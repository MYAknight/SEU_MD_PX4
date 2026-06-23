#include "HuaqicccMorphControl.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <lib/parameters/param.h>
#include <mathlib/math/Limits.hpp>

using namespace time_literals;

HuaqicccMorphControl::HuaqicccMorphControl(int i2c_bus) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_i2c_bus(i2c_bus)
{
}

HuaqicccMorphControl::~HuaqicccMorphControl()
{
	if (should_exit()) {
		ScheduleClear();
	}

#if defined(CONFIG_I2C)

	if (_i2c_dev != nullptr) {
		px4_i2cbus_uninitialize(_i2c_dev);
		_i2c_dev = nullptr;
	}

#endif
}

bool HuaqicccMorphControl::init()
{
	if (_param_morph_en.get() == 0) {
		PX4_INFO("huaqiccc_morph_control disabled (MORPH_EN=0)");
		return false;
	}

#if defined(CONFIG_I2C)
	_i2c_dev = px4_i2cbus_initialize(_i2c_bus);

	if (_i2c_dev == nullptr) {
		PX4_WARN("failed to initialize I2C bus %d, entering SITL simulation mode", _i2c_bus);
		_sim_mode = true;
	}

#endif
	// Verify encoder is present (skip in simulation mode)
	uint16_t raw = 0;

	if (!_sim_mode && read_as5600(raw) != PX4_OK) {
		PX4_WARN("AS5600 not responding on I2C bus %d, entering SITL simulation mode", _i2c_bus);
		_sim_mode = true;
	}

	if (_sim_mode) {
		PX4_INFO("huaqiccc_morph_control started in SITL simulation mode");
	} else {
		PX4_INFO("huaqiccc_morph_control started on I2C bus %d, AS5600 raw=%d", _i2c_bus, raw);
	}

	// Explicitly advertise to ensure publication works after restart
	_actuator_servos_pub.advertise();
	_morph_angle_pub.advertise();

	// Schedule first run
	ScheduleOnInterval(1_s / math::max(_param_morph_rate.get(), (int32_t)10), 1000);
	return true;
}

int HuaqicccMorphControl::read_as5600(uint16_t &raw_angle)
{
#if !defined(CONFIG_I2C)
	return PX4_ERROR;
#else

	if (_i2c_dev == nullptr) {
		return PX4_ERROR;
	}

	// AS5600 angle register 0x0E: 2 bytes, high byte first
	uint8_t reg = AS5600_REG_ANGLE;
	uint8_t buf[2] = {0, 0};

	i2c_msg_s msgv[2] {};

	// Write register address
	msgv[0].frequency = 100000;
	msgv[0].addr = AS5600_ADDR;
	msgv[0].flags = 0;
	msgv[0].buffer = &reg;
	msgv[0].length = 1;

	// Read 2 bytes
	msgv[1].frequency = 100000;
	msgv[1].addr = AS5600_ADDR;
	msgv[1].flags = I2C_M_READ;
	msgv[1].buffer = buf;
	msgv[1].length = 2;

	int ret = I2C_TRANSFER(_i2c_dev, &msgv[0], 2);

	if (ret != PX4_OK) {
		return ret;
	}

	raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
	return PX4_OK;
#endif
}

float HuaqicccMorphControl::raw_to_normalized(uint16_t raw) const
{
	int emin = _param_morph_emin.get();
	int emax = _param_morph_emax.get();

	if (emax == emin) {
		return 0.0f;
	}

	// AS5600 angle wraps at 0/4095. Compute signed shortest deltas
	// so that endpoints on opposite sides of the wrap are handled correctly.
	auto signed_delta = [](int value, int ref) {
		int d = value - ref;
		while (d >= 2048) { d -= 4096; }
		while (d < -2048) { d += 4096; }
		return d;
	};

	int d_raw = signed_delta((int)raw, emin);
	int d_max = signed_delta(emax, emin);

	if (d_max == 0) {
		return 0.0f;
	}

	float t = (float)d_raw / (float)d_max;
	return math::constrain(t, 0.0f, 1.0f);
}

float HuaqicccMorphControl::normalized_to_angle(float t) const
{
	return t * MAX_ARM_ANGLE_RAD;  // 0 -> 0, 1 -> -0.5
}

float HuaqicccMorphControl::angle_to_normalized(float angle) const
{
	float t = angle / MAX_ARM_ANGLE_RAD;  // 0 -> 0, -0.5 -> 1
	return math::constrain(t, 0.0f, 1.0f);
}

void HuaqicccMorphControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check for new morph commands (target angle from MAVLink / QGC)
	huaqiccc_morph_cmd_s morph_cmd{};

	while (_morph_cmd_sub.update(&morph_cmd)) {
		float arm_angle = morph_cmd.arm_angle;

		// Validate range (0 = closed, negative = expanded)
		if (arm_angle <= 0.05f && arm_angle >= -0.55f) {
			_target_arm_angle = arm_angle;
			_last_cmd_time = hrt_absolute_time();
			PX4_INFO("morph target set to %.3f rad", (double)_target_arm_angle);
		}
	}

	float current_angle = 0.0f;

	if (_sim_mode) {
		// SITL simulation: first-order lag toward target angle
		float dt = 0.0f;
		if (_last_run != 0) {
			dt = (hrt_absolute_time() - _last_run) * 1e-6f;
		}
		_last_run = hrt_absolute_time();
		static constexpr float TAU = 0.3f;  // ~0.3s time constant
		if (dt > 0.0f && dt < 1.0f) {
			float alpha = math::constrain(dt / TAU, 0.0f, 1.0f);
			_sim_current_angle += (_target_arm_angle - _sim_current_angle) * alpha;
		} else {
			_sim_current_angle = _target_arm_angle;
		}
		current_angle = _sim_current_angle;

	} else {
		// Read current angle from AS5600
		uint16_t raw_angle = 0;
		int ret = read_as5600(raw_angle);

		if (ret == PX4_OK) {
			_last_raw_angle = raw_angle;
		} else {
			if (!_as5600_error_printed) {
				PX4_ERR("AS5600 read failed");
				_as5600_error_printed = true;
			}

			// Continue with last known values, servo goes to neutral
		}

		float current_t = raw_to_normalized(_last_raw_angle);
		current_angle = normalized_to_angle(current_t);

		// Bang-Bang closed-loop control for linear actuator
		// The actuator only has 3 states: extend (2000us), retract (1000us), stop (1500us)
		// Proportional PWM values in between are INVALID (treated as stop)
		float error = _target_arm_angle - current_angle;  // >0: need to retract/close, <0: need to expand/open
		float servo_output = 0.0f;  // default: stop (1500us)
		static constexpr float DEADBAND_RAD = 0.01f;  // ~0.6 deg deadband to avoid jitter

		if (error < -DEADBAND_RAD) {
			// Target is more expanded than current -> need to EXTEND actuator
			servo_output = 1.0f;  // +1.0 -> 2000us (extend)
		} else if (error > DEADBAND_RAD) {
			// Target is more closed than current -> need to RETRACT actuator
			servo_output = -1.0f;  // -1.0 -> 1000us (retract)
		} else {
			// Within deadband -> STOP
			servo_output = 0.0f;  // 0.0 -> 1500us (stop)
		}

		// Use actuator_test to bypass arm/disarm check (allows servo control while disarmed)
		actuator_test_s test{};
		test.timestamp = hrt_absolute_time();
		test.action = actuator_test_s::ACTION_DO_CONTROL;
		test.function = actuator_test_s::FUNCTION_SERVO1;  // AUX1
		test.value = servo_output;
		test.timeout_ms = 200;  // Refresh frequently for closed-loop control
		_actuator_test_pub.publish(test);

		// Also publish actuator_servos for compatibility with flight control
		actuator_servos_s servos{};
		servos.timestamp = hrt_absolute_time();

		for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
			servos.control[i] = NAN;  // Disarm all by default
		}

		servos.control[0] = servo_output;
		_actuator_servos_pub.publish(servos);
	}

	// Publish actual angle for ControlAllocator and other modules
	huaqiccc_morph_angle_s morph_msg{};
	morph_msg.timestamp = hrt_absolute_time();
	morph_msg.arm_angle = current_angle;
	_morph_angle_pub.publish(morph_msg);

	// Periodically print angle for ground station display via MAVLink STATUSTEXT
	if (++_status_print_count >= 50) {
		_status_print_count = 0;
		PX4_INFO("morph angle=%.4f rad raw=%u target=%.4f rad%s", (double)current_angle,
			 (unsigned)_last_raw_angle, (double)_target_arm_angle, _sim_mode ? " (sim)" : "");
	}
}

int HuaqicccMorphControl::task_spawn(int argc, char *argv[])
{
	int i2c_bus = 2;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			print_usage("unrecognized option");
			return PX4_ERROR;
		}
	}

	HuaqicccMorphControl *dev = new HuaqicccMorphControl(i2c_bus);

	if (dev == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!dev->init()) {
		delete dev;
		return PX4_ERROR;
	}

	_object.store(dev);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int HuaqicccMorphControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HuaqicccMorphControl::print_status()
{
	if (_object.load() != nullptr) {
		HuaqicccMorphControl *dev = static_cast<HuaqicccMorphControl *>(_object.load());
		PX4_INFO("target: %.3f rad, I2C bus: %d, raw: %u", (double)dev->_target_arm_angle, dev->_i2c_bus,
			 (unsigned)dev->_last_raw_angle);
	}

	return 0;
}

int HuaqicccMorphControl::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("huaqiccc Morphing Arm Controller\n"
				 "Controls a single linear actuator with AS5600 encoder feedback.\n"
				 "Outputs to AUX1 (actuator_servos control[0]).");
	PRINT_MODULE_USAGE_NAME("huaqiccc_morph_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('b', "I2C bus (default: 2)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int huaqiccc_morph_control_main(int argc, char *argv[]);

int huaqiccc_morph_control_main(int argc, char *argv[])
{
	return HuaqicccMorphControl::main(argc, argv);
}

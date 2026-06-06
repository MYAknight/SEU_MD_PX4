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
		PX4_ERR("failed to initialize I2C bus %d", _i2c_bus);
		return false;
	}

#endif
	// Verify encoder is present
	uint16_t raw = 0;

	if (read_as5600(raw) != PX4_OK) {
		PX4_ERR("AS5600 not responding on I2C bus %d", _i2c_bus);
		return false;
	}

	PX4_INFO("huaqiccc_morph_control started on I2C bus %d, AS5600 raw=%d", _i2c_bus, raw);

	// Explicitly advertise to ensure publication works after restart
	_actuator_servos_pub.advertise();

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

	if (emax <= emin) {
		return 0.0f;
	}

	float t = (float)((int)raw - emin) / (float)(emax - emin);
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

	// Check for new vehicle commands (target angle from MAVLink / QGC)
	vehicle_command_s vcmd{};

	while (_vehicle_command_sub.update(&vcmd)) {
		if (vcmd.command == 31440) {  // MAV_CMD_HUAQICCC_SET_ARM_ANGLE
			float arm_angle = vcmd.param1;

			// Validate range (0 = closed, negative = expanded)
			if (arm_angle <= 0.05f && arm_angle >= -0.55f) {
				_target_arm_angle = arm_angle;
				_last_cmd_time = hrt_absolute_time();
				PX4_INFO("morph target set to %.3f rad", (double)_target_arm_angle);
			}
		}
	}

	// Read current angle from AS5600
	uint16_t raw_angle = 0;
	int ret = read_as5600(raw_angle);

	if (ret != PX4_OK) {
		PX4_ERR("AS5600 read failed");
		// Continue with last known values, servo goes to neutral
	}

	float current_t = raw_to_normalized(raw_angle);
	float current_angle = normalized_to_angle(current_t);

	// Publish actual angle for ControlAllocator and other modules
	huaqiccc_morph_angle_s morph_msg{};
	morph_msg.timestamp = hrt_absolute_time();
	morph_msg.arm_angle = current_angle;
	_morph_angle_pub.publish(morph_msg);

	// Direct position mapping (with REV=1: -1.0=extend, +1.0=retract)
	// t=0 (closed) -> output=-1.0 -> REV=1 -> +1.0 -> MAX=1950us (retract/closed)
	// t=1 (open)   -> output=+1.0 -> REV=1 -> -1.0 -> MIN=1050us (extend/open)
	float target_t = angle_to_normalized(_target_arm_angle);
	float servo_output = 2.0f * target_t - 1.0f;
	servo_output = math::constrain(servo_output, -1.0f, 1.0f);

	// Publish actuator output on AUX1 (servo index 0)
	actuator_servos_s servos{};
	servos.timestamp = hrt_absolute_time();

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
		servos.control[i] = NAN;  // Disarm all by default
	}

	servos.control[0] = servo_output;
	_actuator_servos_pub.publish(servos);
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
		PX4_INFO("target: %.3f rad, I2C bus: %d", (double)dev->_target_arm_angle, dev->_i2c_bus);
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

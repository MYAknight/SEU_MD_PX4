#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/huaqiccc_morph_angle.h>
#include <uORB/topics/vehicle_command.h>

#if defined(CONFIG_I2C)
#include <nuttx/i2c/i2c_master.h>
#endif

class HuaqicccMorphControl : public ModuleBase<HuaqicccMorphControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HuaqicccMorphControl(int i2c_bus = 2);
	~HuaqicccMorphControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	/**
	 * Read AS5600 magnetic encoder raw angle via I2C.
	 * @param raw_angle 12-bit raw value (0-4095)
	 * @return PX4_OK on success
	 */
	int read_as5600(uint16_t &raw_angle);

	/**
	 * Convert raw encoder value to normalized position t in [0, 1].
	 * t=0 means fully closed, t=1 means fully open.
	 */
	float raw_to_normalized(uint16_t raw) const;

	/**
	 * Convert normalized position t to arm_angle in rad (SITL semantic).
	 * arm_angle=0 means closed, arm_angle=-0.5 means fully open.
	 */
	float normalized_to_angle(float t) const;

	/**
	 * Convert target arm_angle (rad) to normalized position t.
	 */
	float angle_to_normalized(float angle) const;

	// I2C
	int _i2c_bus{2};
#if defined(CONFIG_I2C)
	struct i2c_master_s *_i2c_dev{nullptr};
#endif

	// uORB
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<huaqiccc_morph_angle_s> _morph_angle_pub{ORB_ID(huaqiccc_morph_angle)};

	// State
	float _target_arm_angle{0.0f};  ///< Target angle in rad (SITL semantic)
	uint64_t _last_cmd_time{0};     ///< Last time a valid command was received
	hrt_abstime _last_run{0};

	// Constants
	static constexpr uint8_t AS5600_ADDR = 0x36;
	static constexpr uint8_t AS5600_REG_ANGLE = 0x0E;
	static constexpr float MAX_ARM_ANGLE_RAD = -0.4f;  // Fully open (~23 deg)

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MORPH_EN>) _param_morph_en,
		(ParamFloat<px4::params::MORPH_KP>) _param_morph_kp,
		(ParamFloat<px4::params::MORPH_DB>) _param_morph_db,
		(ParamInt<px4::params::MORPH_EMIN>) _param_morph_emin,
		(ParamInt<px4::params::MORPH_EMAX>) _param_morph_emax,
		(ParamInt<px4::params::MORPH_RATE>) _param_morph_rate
	)
};

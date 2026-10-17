#include <drivers/drv_hrt.h>
#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class HAUVWaterControl final : public ModuleBase<HAUVWaterControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HAUVWaterControl() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
	}

	static int task_spawn(int argc, char *argv[])
	{
		HAUVWaterControl *instance = new HAUVWaterControl();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(20_ms);
		return PX4_OK;
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s\n", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
Manual surface-water controller for the HAUV 4+1 vehicle.

In Water mode the centered throttle stick commands bidirectional T500 thrust,
right-stick horizontal commands car-like steering through differential
quad-motor torque, and a small collective quad thrust keeps the four aerial
motors ready. Steering reverses while backing up. Roll and pitch stabilization
are intentionally not implemented yet.
)DESCR_STR");
		PRINT_MODULE_USAGE_NAME("hauv_water_control", "controller");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("Water mode: %s", _active ? "active" : "inactive");
		PX4_INFO("Manual input: %s", _input_fresh ? "fresh" : "stale");
		PX4_INFO("Controls: %s", _controls_ready ? "ready" : "waiting for neutral sticks");
		return 0;
	}

private:
	static float slew(float current, float target, float rate, float dt)
	{
		const float maximum_change = rate * dt;
		return current + math::constrain(target - current, -maximum_change, maximum_change);
	}

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		if (_parameter_update_sub.updated()) {
			parameter_update_s parameter_update;
			_parameter_update_sub.copy(&parameter_update);
			updateParams();
		}

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_status_sub.update(&_vehicle_status);
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		const hrt_abstime now = hrt_absolute_time();

		_active = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_WATER
			  && _vehicle_control_mode.flag_control_manual_enabled
			  && _vehicle_control_mode.flag_control_allocation_enabled;

		if (!_active) {
			_input_fresh = false;
			_controls_ready = false;
			_was_armed = false;
			_forward_thrust_setpoint = 0.f;
			_yaw_torque_setpoint = 0.f;
			_last_run = now;
			return;
		}

		const hrt_abstime input_timeout = static_cast<hrt_abstime>(_param_input_timeout.get() * 1_s);
		_input_fresh = _manual_control_setpoint.valid
			       && _manual_control_setpoint.timestamp != 0
			       && now < _manual_control_setpoint.timestamp + input_timeout;

		const bool armed = _vehicle_control_mode.flag_armed;
		const bool newly_armed = armed && !_was_armed;

		if (!armed) {
			_controls_ready = false;

		} else if (!_controls_ready && _input_fresh
			   && fabsf(_manual_control_setpoint.throttle) <= 0.10f
			   && fabsf(_manual_control_setpoint.roll) <= 0.10f) {
			_controls_ready = true;
		}

		if (newly_armed && !_controls_ready) {
			PX4_WARN("center throttle and right stick to enable Water controls");
		}

		float forward_thrust_target = 0.f;
		float yaw_torque_target = 0.f;

		if (_input_fresh && _controls_ready) {
			const float throttle_input = math::constrain(_manual_control_setpoint.throttle, -1.f, 1.f);
			const float steering_input = math::constrain(_manual_control_setpoint.roll, -1.f, 1.f);
			forward_thrust_target = throttle_input * _param_thrust_max.get();
			yaw_torque_target = steering_input * throttle_input * _param_yaw_max.get();
		}

		const float dt = (_last_run != 0) ? math::constrain((now - _last_run) * 1e-6f, 0.f, 0.1f) : 0.f;

		if (armed) {
			_forward_thrust_setpoint = slew(_forward_thrust_setpoint, forward_thrust_target,
						_param_thrust_slew.get(), dt);
			_yaw_torque_setpoint = slew(_yaw_torque_setpoint, yaw_torque_target,
					     _param_yaw_slew.get(), dt);

		} else {
			_forward_thrust_setpoint = 0.f;
			_yaw_torque_setpoint = 0.f;
		}

		const hrt_abstime timestamp_sample = _input_fresh ? _manual_control_setpoint.timestamp_sample : now;

		vehicle_thrust_setpoint_s thrust_setpoint{};
		thrust_setpoint.timestamp = now;
		thrust_setpoint.timestamp_sample = timestamp_sample;
		thrust_setpoint.xyz[0] = _forward_thrust_setpoint;
		thrust_setpoint.xyz[2] = armed ? -_param_quad_idle.get() : 0.f;
		_vehicle_thrust_setpoint_pub.publish(thrust_setpoint);

		vehicle_torque_setpoint_s torque_setpoint{};
		torque_setpoint.timestamp = now;
		torque_setpoint.timestamp_sample = timestamp_sample;
		torque_setpoint.xyz[2] = _yaw_torque_setpoint;
		_vehicle_torque_setpoint_pub.publish(torque_setpoint);

		_was_armed = armed;
		_last_run = now;
	}

	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_status_s _vehicle_status{};
	bool _active{false};
	bool _input_fresh{false};
	bool _controls_ready{false};
	bool _was_armed{false};
	float _forward_thrust_setpoint{0.f};
	float _yaw_torque_setpoint{0.f};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HAUV_W_THR_MAX>) _param_thrust_max,
		(ParamFloat<px4::params::HAUV_W_YAW_MAX>) _param_yaw_max,
		(ParamFloat<px4::params::HAUV_W_Q_IDLE>) _param_quad_idle,
		(ParamFloat<px4::params::HAUV_W_IN_T>) _param_input_timeout,
		(ParamFloat<px4::params::HAUV_W_THR_SLEW>) _param_thrust_slew,
		(ParamFloat<px4::params::HAUV_W_YAW_SLEW>) _param_yaw_slew
	)
};

extern "C" __EXPORT int hauv_water_control_main(int argc, char *argv[])
{
	return HAUVWaterControl::main(argc, argv);
}

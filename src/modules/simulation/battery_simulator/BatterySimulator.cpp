/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "BatterySimulator.hpp"

namespace
{
// EOLO 15x8 @ ~21 V pack (datasheet): RPM → current [A]
constexpr float kAerialRpm[] = {
	2847.f, 3377.f, 3918.f, 4380.f, 4880.f, 4984.f, 5335.f, 5726.f,
	6114.f, 6283.f, 6623.f, 7030.f, 7539.f, 8315.f, 9319.f
};
constexpr float kAerialAmp[] = {
	2.80f, 4.26f, 6.06f, 8.26f, 10.13f, 12.78f, 15.20f, 18.65f,
	21.52f, 24.24f, 28.44f, 34.98f, 43.08f, 63.62f, 106.88f
};
static_assert(sizeof(kAerialRpm) / sizeof(kAerialRpm[0]) == sizeof(kAerialAmp) / sizeof(kAerialAmp[0]),
	      "aerial LUT size");

// T200 power vs |thrust| (from BR chart @16 V; P≈same vs T at 12/16 V). Thrust [N], Power [W].
constexpr float kT200ThrustN[] = {
	0.f, 8.9f, 17.8f, 26.7f, 35.6f, 44.5f, 51.5f, 65.7f
};
constexpr float kT200PowerW[] = {
	0.f, 25.f, 75.f, 140.f, 210.f, 290.f, 390.f, 645.f
};
static_assert(sizeof(kT200ThrustN) / sizeof(kT200ThrustN[0]) == sizeof(kT200PowerW) / sizeof(kT200PowerW[0]),
	      "T200 LUT size");

// Match gz_standalone hauv_4plus1 MulticopterMotorModel (T200).
constexpr float kT200OmegaMax = 425.9f;		// rad/s
constexpr float kT200MotorConstant = 3.623815e-4f; // T = k·ω²
constexpr float kPackIContMaxX4120 = 150.f;	// 6S10P
constexpr float kPackIContMaxX6215 = 150.f;	// 12S6P soft clamp

// X6215S + 22x6.6: thrust [N] → current [A] at 44.4 V and 50 V.
constexpr float kX6215Thrust44N[] = {
	9.81f, 19.61f, 29.42f, 39.23f, 49.03f, 58.84f, 68.65f, 79.73f
};
constexpr float kX6215Amp44[] = {
	2.1f, 4.9f, 8.5f, 12.8f, 17.4f, 22.7f, 28.1f, 35.2f
};
constexpr float kX6215Thrust50N[] = {
	9.81f, 19.61f, 29.42f, 39.23f, 49.03f, 58.84f, 68.65f, 78.46f, 88.26f, 98.56f
};
constexpr float kX6215Amp50[] = {
	1.8f, 4.5f, 7.7f, 11.7f, 15.9f, 20.5f, 25.6f, 31.0f, 37.1f, 43.5f
};
static_assert(sizeof(kX6215Thrust44N) / sizeof(kX6215Thrust44N[0]) == sizeof(kX6215Amp44) / sizeof(kX6215Amp44[0]),
	      "X6215 44V LUT");
static_assert(sizeof(kX6215Thrust50N) / sizeof(kX6215Thrust50N[0]) == sizeof(kX6215Amp50) / sizeof(kX6215Amp50[0]),
	      "X6215 50V LUT");
constexpr float kX6215MotorConstant = 1.410483e-4f; // T=k·ω² @ 50 V ref
constexpr float kX6215VRef = 50.f;
constexpr float kX6215VLow = 44.4f;

// Typical Li-ion 18650 rest OCV shape (flat mid, steep ends). Values are
// absolute V/cell; mapped onto BAT1_V_EMPTY…BAT1_V_CHARGED at runtime.
constexpr float kSocLut[] = {
	0.00f, 0.05f, 0.10f, 0.15f, 0.20f, 0.30f, 0.40f,
	0.50f, 0.60f, 0.70f, 0.80f, 0.90f, 0.95f, 1.00f
};
constexpr float kOcvShapeV[] = {
	3.50f, 3.60f, 3.70f, 3.74f, 3.77f, 3.80f, 3.83f,
	3.86f, 3.90f, 3.95f, 4.02f, 4.10f, 4.16f, 4.20f
};
static_assert(sizeof(kSocLut) / sizeof(kSocLut[0]) == sizeof(kOcvShapeV) / sizeof(kOcvShapeV[0]),
	      "OCV LUT size");
} // namespace

BatterySimulator::BatterySimulator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_battery(1, this, BATTERY_SIMLATOR_SAMPLE_INTERVAL_US, battery_status_s::SOURCE_POWER_MODULE)
{
}

BatterySimulator::~BatterySimulator()
{
	perf_free(_loop_perf);
}

bool BatterySimulator::init()
{
	ScheduleOnInterval(BATTERY_SIMLATOR_SAMPLE_INTERVAL_US);
	return true;
}

float BatterySimulator::interpolateLut(const float *x, const float *y, unsigned n, float xq)
{
	if (n == 0) {
		return 0.f;
	}

	if (xq <= x[0]) {
		return y[0];
	}

	if (xq >= x[n - 1]) {
		return y[n - 1];
	}

	for (unsigned i = 1; i < n; i++) {
		if (xq <= x[i]) {
			const float u = (xq - x[i - 1]) / (x[i] - x[i - 1]);
			return y[i - 1] + u * (y[i] - y[i - 1]);
		}
	}

	return y[n - 1];
}

float BatterySimulator::cellOcvFromSoc(float soc)
{
	soc = math::constrain(soc, 0.f, 1.f);

	const float v_shape = interpolateLut(kSocLut, kOcvShapeV,
					     sizeof(kSocLut) / sizeof(kSocLut[0]), soc);
	const float v_lo = kOcvShapeV[0];
	const float v_hi = kOcvShapeV[sizeof(kOcvShapeV) / sizeof(kOcvShapeV[0]) - 1];
	const float u = (v_shape - v_lo) / (v_hi - v_lo); // 0…1 along Li-ion shape

	const float v_empty = _battery.empty_cell_voltage();
	const float v_full = _battery.full_cell_voltage();
	return v_empty + u * (v_full - v_empty);
}

float BatterySimulator::packVoltageFromSoc(float soc, float current_a)
{
	const int n_cells = _battery.cell_count();

	if (n_cells <= 0) {
		return 0.f;
	}

	float v_pack = cellOcvFromSoc(soc) * static_cast<float>(n_cells);

	// IR sag: BAT1_R_INTERNAL is per series cell (set effective R_cell/P for nP packs).
	const float r_cell = _param_bat1_r_internal.get();

	if (r_cell > 0.f && current_a > 0.f) {
		v_pack -= current_a * r_cell * static_cast<float>(n_cells);
	}

	// Never report below empty pack (failsafe floor, e.g. 21 V).
	const float v_floor = _battery.empty_cell_voltage() * static_cast<float>(n_cells);
	return math::max(v_pack, v_floor);
}

float BatterySimulator::estimateLoadCurrentA(float bus_voltage_v)
{
	actuator_outputs_s outputs{};

	if (!_actuator_outputs_sub.copy(&outputs)) {
		return _param_sim_bat_idle_a.get();
	}

	const float v = math::max(bus_voltage_v, 1.f);
	float i_motors = 0.f;

	const unsigned n = math::min(static_cast<unsigned>(outputs.noutputs),
				     static_cast<unsigned>(actuator_outputs_s::NUM_ACTUATOR_OUTPUTS));

	const bool x6215 = (_param_sim_bat_prop.get() == 1);

	// Effective ω after SIM_GZ_VREF scale (ESC applies same factor to Gazebo).
	float v_scale = 1.f;
	const float vref = _param_sim_gz_vref.get();

	if (vref > 1.f) {
		v_scale = math::constrain(v / vref, 0.5f, 1.f);
	}

	// Motors 0–3: aerial
	for (unsigned i = 0; i < math::min(n, 4u); i++) {
		const float omega = fabsf(outputs.output[i]) * v_scale;

		if (x6215) {
			const float thrust_n = kX6215MotorConstant * omega * omega;
			const float i44 = interpolateLut(kX6215Thrust44N, kX6215Amp44,
							 sizeof(kX6215Thrust44N) / sizeof(kX6215Thrust44N[0]), thrust_n);
			const float i50 = interpolateLut(kX6215Thrust50N, kX6215Amp50,
							 sizeof(kX6215Thrust50N) / sizeof(kX6215Thrust50N[0]), thrust_n);
			const float u = math::constrain((v - kX6215VLow) / (kX6215VRef - kX6215VLow), 0.f, 1.f);
			i_motors += i44 + u * (i50 - i44);

		} else {
			const float rpm = omega * (60.f / (2.f * M_PI_F));
			i_motors += interpolateLut(kAerialRpm, kAerialAmp,
						   sizeof(kAerialRpm) / sizeof(kAerialRpm[0]), rpm);
		}
	}

	// Motor 4: T200 — T=k·ω², P(T) from datasheet, I=P/V.
	if (n >= 5) {
		const float omega = math::constrain(fabsf(outputs.output[4]), 0.f, kT200OmegaMax);
		const float thrust_n = kT200MotorConstant * omega * omega;
		const float power_w = interpolateLut(kT200ThrustN, kT200PowerW,
						     sizeof(kT200ThrustN) / sizeof(kT200ThrustN[0]), thrust_n);
		i_motors += power_w / v;
	}

	const float i_max = x6215 ? kPackIContMaxX6215 : kPackIContMaxX4120;
	float i_total = i_motors + _param_sim_bat_idle_a.get();
	return math::constrain(i_total, 0.f, i_max);
}

void BatterySimulator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	updateCommands();

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const hrt_abstime now_us = hrt_absolute_time();

	float ibatt = -1.0f; // no current sensor in time-based mode

	if (_armed) {
		if (_last_integration_us != 0) {
			const float dt_s = (now_us - _last_integration_us) * 1e-6f;

			if (_param_sim_bat_src.get() == 1) {
				// Bus ≈ OCV for P/V (before this-step sag).
				const float v_oc = packVoltageFromSoc(_battery_percentage, 0.f);
				ibatt = estimateLoadCurrentA(v_oc);

				const float capacity_mah = _param_bat1_capacity.get();

				if (capacity_mah > 1.f && dt_s > 0.f) {
					// Ah discharged = I * dt / 3600; SoC uses capacity in Ah.
					_battery_percentage -= (ibatt * dt_s / 3600.f) / (capacity_mah * 1e-3f);
				}

			} else {
				const float discharge_interval_us = _param_sim_bat_drain.get() * 1000 * 1000;
				_battery_percentage -= (now_us - _last_integration_us) / discharge_interval_us;
			}
		}

		_last_integration_us = now_us;

	} else {
		// PX4 default SITL behaviour: full pack again when disarmed.
		_battery_percentage = 1.f;
		_last_integration_us = 0;
		ibatt = -1.0f;
	}

	_battery_percentage = math::max(_battery_percentage, _param_bat_min_pct.get() / 100.f);

	float vbatt = packVoltageFromSoc(_battery_percentage, (ibatt > 0.f) ? ibatt : 0.f);

	if (_force_empty_battery) {
		vbatt = _battery.empty_cell_voltage() * math::max(_battery.cell_count(), 1);
	}

	_battery.setConnected(true);
	_battery.updateVoltage(vbatt);
	_battery.updateCurrent(ibatt);
	_battery.updateAndPublishBatteryStatus(now_us);

	perf_end(_loop_perf);
}

void BatterySimulator::updateCommands()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);
		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SYSTEM_BATTERY) {

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				handled = true;
				PX4_INFO("CMD_INJECT_FAILURE, battery ok");
				supported = false;

				if (instance == 0) {
					supported = true;
					_force_empty_battery = false;
				}

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				// Force battery empty for FAILURE_TYPE_OFF - not perfectly accurate, but what we want to achieve
				handled = true;
				PX4_WARN("CMD_INJECT_FAILURE, battery empty");
				supported = false;

				if (instance == 0) {
					supported = true;
					_force_empty_battery = true;
				}
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}
}

int BatterySimulator::task_spawn(int argc, char *argv[])
{
	BatterySimulator *instance = new BatterySimulator();

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

int BatterySimulator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BatterySimulator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SITL battery simulation. SIM_BAT_SRC=0 drains by time (SIM_BAT_DRAIN);
SIM_BAT_SRC=1 estimates current from actuator rad/s using motor datasheet
curves (aerial EOLO 15x8 + T200) plus SIM_BAT_IDLE_A.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_simulator", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_simulator_main(int argc, char *argv[])
{
	return BatterySimulator::main(argc, argv);
}

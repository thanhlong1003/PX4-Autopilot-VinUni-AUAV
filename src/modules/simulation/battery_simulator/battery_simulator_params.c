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

/**
 * Simulator Battery enabled
 *
 * Enable or disable the internal battery simulation. This is useful
 * when the battery is simulated externally and interfaced with PX4
 * through MAVLink for example.
 *
 * @boolean
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_BAT_ENABLE, 1);

/**
 * Simulator Battery drain interval
 *
 * @min 1
 * @max 86400
 * @increment 1
 * @unit s
 *
 * @group SITL
 */
PARAM_DEFINE_FLOAT(SIM_BAT_DRAIN, 60);

/**
 * Simulator Battery minimal percentage.
 *
 * Can be used to alter the battery level during SITL- or HITL-simulation on the fly.
 * Particularly useful for testing different low-battery behaviour.
 *
 * @min 0
 * @max 100
 * @increment 0.1
 * @unit %
 *
 * @group SITL
 */
PARAM_DEFINE_FLOAT(SIM_BAT_MIN_PCT, 50.0f);

/**
 * Simulator battery drain source
 *
 * 0: time-based (SIM_BAT_DRAIN seconds for 100%→0% while armed)
 * 1: power-based from actuator outputs (motor datasheet I(ω) + SIM_BAT_IDLE_A)
 *
 * @value 0 Time
 * @value 1 Actuator power
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_BAT_SRC, 0);

/**
 * Simulator avionics / idle current (power mode)
 *
 * Added to estimated motor current when SIM_BAT_SRC=1.
 *
 * @min 0
 * @max 50
 * @increment 0.1
 * @unit A
 * @group SITL
 */
PARAM_DEFINE_FLOAT(SIM_BAT_IDLE_A, 3.0f);

/**
 * Simulator aerial propulsion current model
 *
 * 0: SunnySky X4120 + EOLO 15x8 (I vs RPM @ ~21 V)
 * 1: SunnySky X6215S + 22x6.6 (I vs thrust, V-interpolated 44.4/50 V)
 *
 * @value 0 X4120 / 15x8
 * @value 1 X6215S / 22x6.6
 * @group SITL
 */
PARAM_DEFINE_INT32(SIM_BAT_PROP, 0);

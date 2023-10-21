/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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
 * @file TECS.cpp
 *
 * @author Paul Riseborough
 */

#include "TECS.hpp"

#include <lib/geo/geo.h>

#include <px4_platform_common/defines.h>

#include "matrix/Matrix.hpp"
#include "matrix/Vector2.hpp"

using math::constrain;
using math::max;
using math::min;
using namespace time_literals;

static inline constexpr bool TIMESTAMP_VALID(float dt) { return (PX4_ISFINITE(dt) && dt > FLT_EPSILON);}

void TECSAirspeedFilter::initialize(const float equivalent_airspeed)
{
	_airspeed_state.speed = equivalent_airspeed;
	_airspeed_state.speed_rate = 0.0f;
}

void TECSAirspeedFilter::update(const float dt, const Input &input, const Param &param,
				const bool airspeed_sensor_available)
{
	// Input checking
	if (!TIMESTAMP_VALID(dt)) {
		// Do not update the states.
		PX4_WARN("Time intervall is not valid.");
		return;
	}

	float airspeed;

	if (PX4_ISFINITE(input.equivalent_airspeed) && airspeed_sensor_available) {
		airspeed = input.equivalent_airspeed;

	} else {
		airspeed = param.equivalent_airspeed_trim;
	}

	float airspeed_derivative;

	if (PX4_ISFINITE(input.equivalent_airspeed_rate) && airspeed_sensor_available) {
		airspeed_derivative = input.equivalent_airspeed_rate;

	} else {
		airspeed_derivative = 0.0f;
	}

	/* Filter airspeed and rate using a constant airspeed rate model in a steady state Kalman Filter.
	 We use the gains of the continuous Kalman filter Kc and approximate the discrete version Kalman gain Kd =dt*Kc,
	 since the continuous algebraic Riccatti equation is easier to solve.
	*/

	matrix::Vector2f new_state_predicted;

	new_state_predicted(0) = _airspeed_state.speed + dt * _airspeed_state.speed_rate;
	new_state_predicted(1) = _airspeed_state.speed_rate;

	const float airspeed_noise_inv{1.0f / param.airspeed_measurement_std_dev};
	const float airspeed_rate_noise_inv{1.0f / param.airspeed_rate_measurement_std_dev};
	const float airspeed_rate_noise_inv_squared_process_noise{airspeed_rate_noise_inv *airspeed_rate_noise_inv * param.airspeed_rate_noise_std_dev};
	const float denom{airspeed_noise_inv + airspeed_rate_noise_inv_squared_process_noise};
	const float common_nom{std::sqrt(param.airspeed_rate_noise_std_dev * (2.0f * airspeed_noise_inv + airspeed_rate_noise_inv_squared_process_noise))};

	matrix::Matrix<float, 2, 2> kalman_gain;
	kalman_gain(0, 0) = airspeed_noise_inv * common_nom / denom;
	kalman_gain(0, 1) = airspeed_rate_noise_inv_squared_process_noise / denom;
	kalman_gain(1, 0) = airspeed_noise_inv * airspeed_noise_inv * param.airspeed_rate_noise_std_dev / denom;
	kalman_gain(1, 1) = airspeed_rate_noise_inv_squared_process_noise * common_nom / denom;

	const matrix::Vector2f innovation{(airspeed - new_state_predicted(0)), (airspeed_derivative - new_state_predicted(1))};
	matrix::Vector2f new_state;
	new_state = new_state_predicted + dt * (kalman_gain * (innovation));

	// Clip airspeed at zero
	if (new_state(0) < FLT_EPSILON) {
		new_state(0) = 0.0f;
		// calculate input that would result in zero speed.
		const float desired_airspeed_innovation = (-new_state_predicted(0) / dt - kalman_gain(0,
				1) * innovation(1)) / kalman_gain(0,
						0);
		new_state(1) = new_state_predicted(1) + dt * (kalman_gain(1, 0) * desired_airspeed_innovation + kalman_gain(1,
				1) * innovation(1));
	}

	// Update states
	_airspeed_state.speed = new_state(0);
	_airspeed_state.speed_rate = new_state(1);
}

TECSAirspeedFilter::AirspeedFilterState TECSAirspeedFilter::getState() const
{
	return _airspeed_state;
}

void TECSAltitudeReferenceModel::update(const float dt, const AltitudeReferenceState &setpoint, float altitude,
					float height_rate, const Param &param)
{
	// Input checks
	if (!TIMESTAMP_VALID(dt)) {
		// Do not update the states.
		PX4_WARN("Time intervall is not valid.");
		return;
	}

	const float current_alt = PX4_ISFINITE(altitude) ? altitude : 0.f;

	_velocity_control_traj_generator.setMaxJerk(param.jerk_max);
	_velocity_control_traj_generator.setMaxAccelUp(param.vert_accel_limit);
	_velocity_control_traj_generator.setMaxAccelDown(param.vert_accel_limit);
	_velocity_control_traj_generator.setMaxVelUp(param.max_sink_rate); // different convention for FW than for MC
	_velocity_control_traj_generator.setMaxVelDown(param.max_climb_rate); // different convention for FW than for MC

	// Altitude setpoint reference
	_alt_control_traj_generator.setMaxJerk(param.jerk_max);
	_alt_control_traj_generator.setMaxAccel(param.vert_accel_limit);
	_alt_control_traj_generator.setMaxVel(fmax(param.max_climb_rate, param.max_sink_rate));

	_velocity_control_traj_generator.setVelSpFeedback(setpoint.alt_rate);

	bool control_altitude = true;
	float altitude_setpoint = setpoint.alt;

	if (PX4_ISFINITE(setpoint.alt_rate)) {
		// input is height rate (not altitude)
		_velocity_control_traj_generator.setCurrentPositionEstimate(current_alt);
		_velocity_control_traj_generator.update(dt, setpoint.alt_rate);
		altitude_setpoint = _velocity_control_traj_generator.getCurrentPosition();
		control_altitude = PX4_ISFINITE(altitude_setpoint); // returns true if altitude is locked

	} else {
		_velocity_control_traj_generator.reset(0, height_rate, altitude_setpoint);
	}

	if (control_altitude) {
		const float target_climbrate_m_s = math::min(param.target_climbrate, param.max_climb_rate);
		const float target_sinkrate_m_s = math::min(param.target_sinkrate, param.max_sink_rate);

		const float delta_trajectory_to_target_m = altitude_setpoint - _alt_control_traj_generator.getCurrentPosition();

		float height_rate_target = math::signNoZero<float>(delta_trajectory_to_target_m) *
					   math::trajectory::computeMaxSpeedFromDistance(
						   param.jerk_max, param.vert_accel_limit, fabsf(delta_trajectory_to_target_m), 0.f);

		height_rate_target = math::constrain(height_rate_target, -target_sinkrate_m_s, target_climbrate_m_s);

		_alt_control_traj_generator.updateDurations(height_rate_target);
		_alt_control_traj_generator.updateTraj(dt);
		_height_rate_setpoint_direct = NAN;

	} else {
		_alt_control_traj_generator.setCurrentVelocity(_velocity_control_traj_generator.getCurrentVelocity());
		_alt_control_traj_generator.setCurrentPosition(current_alt);
		_height_rate_setpoint_direct = _velocity_control_traj_generator.getCurrentVelocity();
	}
}

TECSAltitudeReferenceModel::AltitudeReferenceState TECSAltitudeReferenceModel::getAltitudeReference() const
{
	TECSAltitudeReferenceModel::AltitudeReferenceState ref{
		.alt = _alt_control_traj_generator.getCurrentPosition(),
		.alt_rate = _alt_control_traj_generator.getCurrentVelocity(),
	};

	return ref;
}

void TECSAltitudeReferenceModel::initialize(const AltitudeReferenceState &state)
{
	const float init_state_alt = PX4_ISFINITE(state.alt) ? state.alt : 0.f;
	const float init_state_alt_rate = PX4_ISFINITE(state.alt_rate) ? state.alt_rate : 0.f;

	_alt_control_traj_generator.reset(0.0f, init_state_alt_rate, init_state_alt);
	_velocity_control_traj_generator.reset(0.f, init_state_alt_rate, init_state_alt);
}

void TECSControl::initialize(const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag)
{
	resetIntegrals();

	AltitudePitchControl control_setpoint;

	control_setpoint.tas_rate_setpoint = _calcAirspeedControlOutput(setpoint, input, param, flag);

	control_setpoint.altitude_rate_setpoint = _calcAltitudeControlOutput(setpoint, input, param);

	SpecificEnergyRates specific_energy_rate{_calcSpecificEnergyRates(control_setpoint, input)};

	_detectUnderspeed(input, param, flag);

	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
	ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rate)};

	_pitch_setpoint = _calcPitchControlOutput(input, seb_rate, param, flag);

	const STERateLimit limit{_calculateTotalEnergyRateLimit(input, param)};

	_ste_rate_estimate_filter.reset(specific_energy_rate.spe_rate.estimate + specific_energy_rate.ske_rate.estimate);

	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rate, input, param, flag)};

	_throttle_setpoint = _calcThrottleControlOutput(0.0f, limit, ste_rate, input, param, flag);

	//Calculate drag model params

	
	if (param.use_dynamic_throttle_calculation) {
		/* Airspeed-dependent drag coefficients */

		const float drag_trim_clean = math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G / param.equivalent_airspeed_trim * param.weight_gross;
		const float drag_trim_flaps = math::max(param.min_sink_rate_flaps, FLT_EPSILON) * CONSTANTS_ONE_G / param.equivalent_airspeed_trim * param.weight_gross;

		const float lift = param.weight_gross * CONSTANTS_ONE_G;
		
		//Induced drag = L^2/(0.5 * rho0 * eas^2 * pi * wingspan^2 * efficiency factor)
		// at trim airspeed the induced drag is 
		float D_i_trim = lift * lift / (0.5f * M_PI_F * CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C * param.wingspan * param.wingspan * param.wing_efficiency * param.equivalent_airspeed_trim * param.equivalent_airspeed_trim);

		// parasitic drag at trim airspeed
		float D_p_trim_clean = drag_trim_clean - D_i_trim;
		float D_p_trim_flaps = drag_trim_flaps - D_i_trim;

		// Cd_specific: Vehicle specific parasitic drag coefficient, which equals to 1/2*A*rho0*Cd_o
		param.Cd_specific_clean = D_p_trim_clean / (param.equivalent_airspeed_trim * param.equivalent_airspeed_trim);
		param.Cd_specific_flaps = D_p_trim_flaps / (param.equivalent_airspeed_trim * param.equivalent_airspeed_trim);
		
	}


	// Debug output
	_debug_output.total_energy_rate_estimate = ste_rate.estimate;
	_debug_output.total_energy_rate_sp = ste_rate.setpoint;
	_debug_output.throttle_integrator = _STE_rate_integ_state;
	_debug_output.energy_balance_rate_estimate = seb_rate.estimate;
	_debug_output.energy_balance_rate_sp = seb_rate.setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
	_debug_output.altitude_rate_control = control_setpoint.altitude_rate_setpoint;
	_debug_output.true_airspeed_derivative_control = control_setpoint.tas_rate_setpoint;
	_debug_output.ste_rate_min = limit.STE_rate_min;
	_debug_output.ste_rate_max = limit.STE_rate_max;
}

void TECSControl::update(const float dt, const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag)
{
	// Input checking
	if (!TIMESTAMP_VALID(dt)) {
		// Do not update the states and output.
		PX4_WARN("Time intervall is not valid.");
		return;
	}

	AltitudePitchControl control_setpoint;

	control_setpoint.tas_rate_setpoint = _calcAirspeedControlOutput(setpoint, input, param, flag);

	if (PX4_ISFINITE(setpoint.altitude_rate_setpoint_direct)) {
		// direct height rate control
		control_setpoint.altitude_rate_setpoint = setpoint.altitude_rate_setpoint_direct;

	} else {
		// altitude is locked, go through altitude outer loop
		control_setpoint.altitude_rate_setpoint = _calcAltitudeControlOutput(setpoint, input, param);
	}

	SpecificEnergyRates specific_energy_rate{_calcSpecificEnergyRates(control_setpoint, input)};

	_detectUnderspeed(input, param, flag);

	_calcPitchControl(dt, input, specific_energy_rate, param, flag);

	_calcThrottleControl(dt, specific_energy_rate, input, param, flag);

	_debug_output.altitude_rate_control = control_setpoint.altitude_rate_setpoint;
	_debug_output.true_airspeed_derivative_control = control_setpoint.tas_rate_setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
	_debug_output.throttle_integrator = _STE_rate_integ_state;

}

TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Input &input, const Param &param) const
{
	TECSControl::STERateLimit limit;

	// Calculate the specific total energy upper rate limits from the max throttle climb rate
	const float rate_max = math::max(param.max_climb_rate, FLT_EPSILON) * CONSTANTS_ONE_G;;

	// Calculate the specific total energy lower rate limits from the min throttle sink rate
	const float rate_min = - math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G;

	if (param.use_dynamic_throttle_calculation) {
		/* Airspeed-dependent drag coefficients */

		// The additional normal load factor is given by 1/cos(bank angle) = load_factor
		const float lift = param.weight_gross * CONSTANTS_ONE_G * constrain(param.load_factor, 0.1f, 1.f);

		// _STE_rate_min equals to the sum of parasitic and induced drag power.
		// Drag force = Induced drag + Cd_specific * EAS * EAS;
		// Drag power = Drag force * _tas_state
		float eas_sq = constrain(input.tas / input.eas_to_tas * input.tas / input.eas_to_tas, param.equivalent_airspeed_min * param.equivalent_airspeed_min, param.equivalent_airspeed_max * param.equivalent_airspeed_max);
		
		const float drag_clean = lift * lift / (0.5f * M_PI_F * CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C * param.wingspan * param.wingspan * param.wing_efficiency * eas_sq) + param.Cd_specific_clean * eas_sq;
		const float drag_flaps = lift * lift / (0.5f * M_PI_F * CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C * param.wingspan * param.wingspan * param.wing_efficiency * eas_sq) + param.Cd_specific_flaps * eas_sq;
		const float drag_combined = input.flaps_setpoint * drag_flaps + (1.0f - input.flaps_setpoint) * drag_clean;
		limit.STE_rate_min = - drag_combined * input.tas / param.weight_gross;


		float max_thrust = 0.f;
		
		//currently only supports electric motor with propeller / ducted fan 
		if(param.propulsion_type == 0){
			max_thrust = _calcMaxThrust(input, param);
		}
		else{
			goto default_STE_rate_limit_calculation;
		}
		limit.STE_rate_max = limit.STE_rate_min + max_thrust * input.tas / param.weight_gross;

	}
	else {
		default_STE_rate_limit_calculation:
		// Calculate the specific total energy rate limits from the max throttle limits
		limit.STE_rate_max = rate_max;
		limit.STE_rate_min = rate_min;
	}
	
	return limit;
}

float TECSControl::_calcMaxThrust(const Input &input, const Param &param) const
{
	float max_thrust;

	if(input.air_density > param.ref_air_density[1]){
		max_thrust = _interpolateMaxThrustBetweenRho(0, 1, 4, 4, input, param); //rpm index 4 for max thrust
	} else {
		max_thrust = _interpolateMaxThrustBetweenRho(1, 2, 4, 4, input, param);
	}

	return max_thrust;
}

float TECSControl::_calcThrustAtConstantRho(const float *thrust_data_lower_airspeed, const float *thrust_data_upper_airspeed, const float airspeed_lower, const float airspeed_upper, const int rpm_index, const Input &input, const Param &param) const
{
	float eas = constrain(input.tas / input.eas_to_tas, airspeed_lower, airspeed_upper);
	float scaler = (eas - airspeed_lower) / (airspeed_upper - airspeed_lower);
	if(!PX4_ISFINITE(scaler)){
		scaler = 0;
	}
	return scaler * (thrust_data_upper_airspeed[rpm_index] - thrust_data_lower_airspeed[rpm_index]) + thrust_data_lower_airspeed[rpm_index];
}

float TECSControl::_interpolateMaxThrustBetweenRho(const int lower_rho_index, const int upper_rho_index, const int rpm_index_upper_rho, const int rpm_index_lower_rho, const Input &input, const Param &param) const
{
	float max_thrust_upper_rho;
	float max_thrust_lower_rho;
	bool min_as_is_land = (param.equivalent_airspeed_land < param.equivalent_airspeed_min);

	float eas = input.tas / input.eas_to_tas;
	
	if(eas < param.equivalent_airspeed_min && min_as_is_land){
		max_thrust_lower_rho = _calcThrustAtConstantRho(param.land_eas_thrust_rpm[lower_rho_index], param.min_eas_thrust_rpm[lower_rho_index], param.equivalent_airspeed_land, param.equivalent_airspeed_min, rpm_index_lower_rho, input, param);
		max_thrust_upper_rho = _calcThrustAtConstantRho(param.land_eas_thrust_rpm[upper_rho_index], param.min_eas_thrust_rpm[upper_rho_index], param.equivalent_airspeed_land, param.equivalent_airspeed_min, rpm_index_upper_rho, input, param);
	} else if(eas < param.equivalent_airspeed_land && !min_as_is_land){
		max_thrust_lower_rho = _calcThrustAtConstantRho(param.min_eas_thrust_rpm[lower_rho_index], param.land_eas_thrust_rpm[lower_rho_index], param.equivalent_airspeed_min, param.equivalent_airspeed_land, rpm_index_lower_rho, input, param);
		max_thrust_upper_rho = _calcThrustAtConstantRho(param.min_eas_thrust_rpm[upper_rho_index], param.land_eas_thrust_rpm[upper_rho_index], param.equivalent_airspeed_min, param.equivalent_airspeed_land, rpm_index_upper_rho, input, param);
	} else if(eas < param.equivalent_airspeed_trim){
		max_thrust_lower_rho = _calcThrustAtConstantRho(param.min_eas_thrust_rpm[lower_rho_index], param.trim_eas_thrust_rpm[lower_rho_index], param.equivalent_airspeed_min, param.equivalent_airspeed_trim, rpm_index_lower_rho, input, param);
		max_thrust_upper_rho = _calcThrustAtConstantRho(param.min_eas_thrust_rpm[upper_rho_index], param.trim_eas_thrust_rpm[upper_rho_index], param.equivalent_airspeed_min, param.equivalent_airspeed_trim, rpm_index_upper_rho, input, param);
	} else {
		max_thrust_lower_rho = _calcThrustAtConstantRho(param.trim_eas_thrust_rpm[lower_rho_index], param.max_eas_thrust_rpm[lower_rho_index], param.equivalent_airspeed_trim, param.equivalent_airspeed_max, rpm_index_lower_rho, input, param);
		max_thrust_upper_rho = _calcThrustAtConstantRho(param.trim_eas_thrust_rpm[upper_rho_index], param.max_eas_thrust_rpm[upper_rho_index], param.equivalent_airspeed_trim, param.equivalent_airspeed_max, rpm_index_upper_rho, input, param);
	}
	
	float rho = constrain(input.air_density, param.ref_air_density[upper_rho_index], param.ref_air_density[lower_rho_index]);
	float rho_scaler = (rho - param.ref_air_density[lower_rho_index]) / (param.ref_air_density[upper_rho_index] - param.ref_air_density[lower_rho_index]);
	if(!PX4_ISFINITE(rho_scaler)){
		rho_scaler = 0;
	}
	return rho_scaler * (max_thrust_upper_rho - max_thrust_lower_rho) + max_thrust_lower_rho;
}

float TECSControl::_calcMaxRPM(const Input &input, const Param &param) const
{
	float max_rpm;

	if(input.air_density > param.ref_air_density[1]){
		max_rpm = _interpolateMaxRPMBetweenRho(0, 1, input, param);
	} else {
		max_rpm = _interpolateMaxRPMBetweenRho(1, 2, input, param);
	}

	return max_rpm;
}

float TECSControl::_calcMaxRPMtAtConstantRho(const float max_rpm_lower_airspeed, const float max_rpm_upper_airspeed, const float airspeed_lower, const float airspeed_upper, const Input &input, const Param &param) const
{
	float eas = constrain(input.tas / input.eas_to_tas, airspeed_lower, airspeed_upper);
	float scaler = (eas - airspeed_lower) / (airspeed_upper - airspeed_lower);
	if(!PX4_ISFINITE(scaler)){
		scaler = 0;
	}
	return scaler * (max_rpm_upper_airspeed - max_rpm_lower_airspeed) + max_rpm_lower_airspeed;
}

float TECSControl::_interpolateMaxRPMBetweenRho(const int lower_rho_index, const int upper_rho_index, const Input &input, const Param &param) const
{
	float max_rpm_upper_rho;
	float max_rpm_lower_rho;
	bool min_as_is_land = (param.equivalent_airspeed_land < param.equivalent_airspeed_min);

	float eas = input.tas / input.eas_to_tas;
	
	if(eas < param.equivalent_airspeed_min && min_as_is_land){
		max_rpm_lower_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[lower_rho_index][0], param.rpm_max_dynamic[lower_rho_index][1], param.equivalent_airspeed_land, param.equivalent_airspeed_min, input, param);
		max_rpm_upper_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[upper_rho_index][0], param.rpm_max_dynamic[upper_rho_index][1], param.equivalent_airspeed_land, param.equivalent_airspeed_min, input, param);
	} else if(eas < param.equivalent_airspeed_land && !min_as_is_land){
		max_rpm_lower_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[lower_rho_index][1], param.rpm_max_dynamic[lower_rho_index][0], param.equivalent_airspeed_min, param.equivalent_airspeed_land, input, param);
		max_rpm_upper_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[upper_rho_index][1], param.rpm_max_dynamic[upper_rho_index][0], param.equivalent_airspeed_min, param.equivalent_airspeed_land, input, param);
	} else if(eas < param.equivalent_airspeed_trim){
		max_rpm_lower_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[lower_rho_index][1], param.rpm_max_dynamic[lower_rho_index][2], param.equivalent_airspeed_min, param.equivalent_airspeed_trim, input, param);
		max_rpm_upper_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[upper_rho_index][1], param.rpm_max_dynamic[upper_rho_index][2], param.equivalent_airspeed_min, param.equivalent_airspeed_trim, input, param);
	} else {
		max_rpm_lower_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[lower_rho_index][2], param.rpm_max_dynamic[lower_rho_index][3], param.equivalent_airspeed_trim, param.equivalent_airspeed_max, input, param);
		max_rpm_upper_rho = _calcMaxRPMtAtConstantRho(param.rpm_max_dynamic[upper_rho_index][2], param.rpm_max_dynamic[upper_rho_index][3], param.equivalent_airspeed_trim, param.equivalent_airspeed_max, input, param);
	}
	
	float rho = constrain(input.air_density, param.ref_air_density[upper_rho_index], param.ref_air_density[lower_rho_index]);
	float rho_scaler = (rho - param.ref_air_density[lower_rho_index]) / (param.ref_air_density[upper_rho_index] - param.ref_air_density[lower_rho_index]);
	if(!PX4_ISFINITE(rho_scaler)){
		rho_scaler = 0;
	}
	return rho_scaler * (max_rpm_upper_rho - max_rpm_lower_rho) + max_rpm_lower_rho;
}

float TECSControl::_calcRequiredRPMForThrust(const float desired_thrust, const Input &input, const Param &param) const
{
	float rpm_rho_index_0, rpm_rho_index_1;
	int rho_index_0, rho_index_1;
	float eas = input.tas / input.eas_to_tas;
	bool min_as_is_land = (param.equivalent_airspeed_land < param.equivalent_airspeed_min);
	if(input.air_density > param.ref_air_density[1]){
		rho_index_0 = 0;
		rho_index_1 = 1;
	} else {
		rho_index_0 = 1;
		rho_index_1 = 2;
	}

	if(eas < param.equivalent_airspeed_min && min_as_is_land){
		rpm_rho_index_0 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_min, param.equivalent_airspeed_land, param.min_eas_thrust_rpm[rho_index_0], param.land_eas_thrust_rpm[rho_index_0], eas, param.rpm_max_dynamic[0][1], param.rpm_max_dynamic[0][0]);
		rpm_rho_index_1 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_min, param.equivalent_airspeed_land, param.min_eas_thrust_rpm[rho_index_1], param.land_eas_thrust_rpm[rho_index_1], eas, param.rpm_max_dynamic[1][1], param.rpm_max_dynamic[1][0]);
		
	}else if(eas < param.equivalent_airspeed_land && !min_as_is_land){
		rpm_rho_index_0 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_land, param.equivalent_airspeed_min, param.land_eas_thrust_rpm[rho_index_0], param.min_eas_thrust_rpm[rho_index_0], eas, param.rpm_max_dynamic[0][0], param.rpm_max_dynamic[0][1]);
		rpm_rho_index_1 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_land, param.equivalent_airspeed_min, param.land_eas_thrust_rpm[rho_index_1], param.min_eas_thrust_rpm[rho_index_1], eas, param.rpm_max_dynamic[1][0], param.rpm_max_dynamic[1][1]);
	
	}else if(eas < param.equivalent_airspeed_trim) {
		rpm_rho_index_0 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_trim, param.equivalent_airspeed_min, param.trim_eas_thrust_rpm[rho_index_0], param.min_eas_thrust_rpm[rho_index_0], eas, param.rpm_max_dynamic[0][2], param.rpm_max_dynamic[0][1]);
		rpm_rho_index_1 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_trim, param.equivalent_airspeed_min, param.trim_eas_thrust_rpm[rho_index_1], param.min_eas_thrust_rpm[rho_index_1], eas, param.rpm_max_dynamic[1][2], param.rpm_max_dynamic[1][1]);

	} else{
		rpm_rho_index_0 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_max, param.equivalent_airspeed_trim, param.max_eas_thrust_rpm[rho_index_0], param.trim_eas_thrust_rpm[rho_index_0], eas, param.rpm_max_dynamic[0][3], param.rpm_max_dynamic[0][2]);
		rpm_rho_index_1 = _calcRPMAtConstantRho(desired_thrust, param.equivalent_airspeed_max, param.equivalent_airspeed_trim, param.max_eas_thrust_rpm[rho_index_1], param.trim_eas_thrust_rpm[rho_index_1], eas, param.rpm_max_dynamic[1][3], param.rpm_max_dynamic[1][2]);

	}

	float rho = constrain(input.air_density, param.ref_air_density[rho_index_1], param.ref_air_density[rho_index_0]);
	float rho_scaler = (rho - param.ref_air_density[rho_index_0]) / (param.ref_air_density[rho_index_1] - param.ref_air_density[rho_index_0]);
	if(!PX4_ISFINITE(rho_scaler)){
		rho_scaler = 0;
	}
	return rho_scaler * (rpm_rho_index_1 - rpm_rho_index_0) + rpm_rho_index_0;

}

float TECSControl::_calcRPMAtConstantRho(const float desired_thrust, const float upper_airspeed, const float lower_airspeed, const float *upper_eas_thrust_data, const float *lower_eas_thrust_data, const float eas, const float max_rpm_upper_as, const float max_rpm_lower_as) const
{
	float eas_adj = constrain(eas, lower_airspeed, upper_airspeed);
	float scaler = (eas_adj - lower_airspeed) / (upper_airspeed - lower_airspeed);
	if(!PX4_ISFINITE(scaler)){
		scaler = 0;
	}
	float rpm_lower_as = _calcRPMAtConstantAirspeedAndRho(desired_thrust, lower_eas_thrust_data, max_rpm_lower_as, 5);
	float rpm_upper_as = _calcRPMAtConstantAirspeedAndRho(desired_thrust, upper_eas_thrust_data, max_rpm_upper_as, 5);

	return scaler * (rpm_upper_as - rpm_lower_as) + rpm_lower_as;
}

float TECSControl::_calcRPMAtConstantAirspeedAndRho(const float desired_thrust, const float *thrust_data, const float max_rpm, const int data_length) const
{
	float rpm;
	int i;
	for(i = 1; i < data_length; i++){
		if(desired_thrust < thrust_data[i]){
			break;
		}
	}

	if(i < data_length){
		float scaler_thrust = (desired_thrust - thrust_data[i-1])/(thrust_data[i] - thrust_data[i-1]);
		rpm = (i - 1 + scaler_thrust)/(data_length-1) * max_rpm;
	}
	else {
		rpm = max_rpm;
	}

	return rpm;
}

float TECSControl::_control_RPM(const float dt, ControlValues rpm, const float max_rpm, const Param &param)
{
	float throttle_setpoint = 0.0f;

	float rpm_error = _getControlError(rpm);

	//FF term (very rough)
	throttle_setpoint += rpm.setpoint/max_rpm;

	//P term
	throttle_setpoint += param.rpm_error_gain * rpm_error;

	//D term
	throttle_setpoint += (rpm_error - _rpm_error_prev) / dt * param.rpm_damping_gain;
	_rpm_error_prev = rpm_error;

	//I term
	float rpm_integ_add = rpm_error * dt * param.rpm_integrator_gain;

	//limit the integrator windup
	if((throttle_setpoint + _rpm_integ_state + rpm_integ_add <= 1.0f) && (throttle_setpoint + _rpm_integ_state + rpm_integ_add >= 0.0f)){
		_rpm_integ_state = constrain(_rpm_integ_state + rpm_integ_add, -1.0f, 1.0f);
		throttle_setpoint += _rpm_integ_state;
	}


	return throttle_setpoint * param.throttle_max;
}

float TECSControl::_calcAirspeedControlOutput(const Setpoint &setpoint, const Input &input, const Param &param,
		const Flag &flag) const
{
	float airspeed_rate_output{0.0f};

	const STERateLimit limit{_calculateTotalEnergyRateLimit(input, param)};

	// calculate the demanded true airspeed rate of change based on first order response of true airspeed error
	// if airspeed measurement is not enabled then always set the rate setpoint to zero in order to avoid constant rate setpoints
	if (flag.airspeed_enabled) {
		// Calculate limits for the demanded rate of change of speed based on physical performance limits
		// with a 50% margin to allow the total energy controller to correct for errors.
		const float max_tas_rate_sp = limit.STE_rate_max / math::max(input.tas, FLT_EPSILON);
		const float min_tas_rate_sp = limit.STE_rate_min / math::max(input.tas, FLT_EPSILON);
		airspeed_rate_output = constrain((setpoint.tas_setpoint - input.tas) * param.airspeed_error_gain, min_tas_rate_sp,
						 max_tas_rate_sp);
	}

	return airspeed_rate_output;
}

float TECSControl::_calcAltitudeControlOutput(const Setpoint &setpoint, const Input &input, const Param &param) const
{
	float altitude_rate_output;
	altitude_rate_output = (setpoint.altitude_reference.alt - input.altitude) * param.altitude_error_gain
			       + param.altitude_setpoint_gain_ff * setpoint.altitude_reference.alt_rate;

	altitude_rate_output = math::constrain(altitude_rate_output, -param.max_sink_rate, param.max_climb_rate);

	return altitude_rate_output;
}

TECSControl::SpecificEnergyRates TECSControl::_calcSpecificEnergyRates(const AltitudePitchControl &control_setpoint,
		const Input &input) const
{
	SpecificEnergyRates specific_energy_rates;
	// Calculate specific energy rate demands in units of (m**2/sec**3)
	specific_energy_rates.spe_rate.setpoint = control_setpoint.altitude_rate_setpoint *
			CONSTANTS_ONE_G; // potential energy rate of change
	specific_energy_rates.ske_rate.setpoint = input.tas *
			control_setpoint.tas_rate_setpoint; // kinetic energy rate of change

	// Calculate specific energy rates in units of (m**2/sec**3)
	specific_energy_rates.spe_rate.estimate = input.altitude_rate * CONSTANTS_ONE_G; // potential energy rate of change
	specific_energy_rates.ske_rate.estimate = input.tas * input.tas_rate;// kinetic energy rate of change

	return specific_energy_rates;
}

void TECSControl::_detectUnderspeed(const Input &input, const Param &param, const Flag &flag)
{
	if (!flag.detect_underspeed_enabled) {
		_ratio_undersped = 0.0f;
		return;
	}

	// this is the expected (something like standard) deviation from the airspeed setpoint that we allow the airspeed
	// to vary in before ramping in underspeed mitigation
	const float tas_error_bound = param.tas_error_percentage * param.equivalent_airspeed_trim;

	// this is the soft boundary where underspeed mitigation is ramped in
	// NOTE: it's currently the same as the error bound, but separated here to indicate these values do not in general
	// need to be the same
	const float tas_underspeed_soft_bound = param.tas_error_percentage * param.equivalent_airspeed_trim;

	const float tas_fully_undersped = math::max(param.tas_min - tas_error_bound - tas_underspeed_soft_bound, 0.0f);
	const float tas_starting_to_underspeed = math::max(param.tas_min - tas_error_bound, tas_fully_undersped);

	_ratio_undersped = 1.0f - math::constrain((input.tas - tas_fully_undersped) /
			   math::max(tas_starting_to_underspeed - tas_fully_undersped, FLT_EPSILON), 0.0f, 1.0f);
}

TECSControl::SpecificEnergyWeighting TECSControl::_updateSpeedAltitudeWeights(const Param &param, const Flag &flag)
{

	SpecificEnergyWeighting weight;
	// Calculate the weight applied to control of specific kinetic energy error
	float pitch_speed_weight = constrain(param.pitch_speed_weight, 0.0f, 2.0f);

	if (_ratio_undersped > FLT_EPSILON && flag.airspeed_enabled) {
		pitch_speed_weight = 2.0f * _ratio_undersped + (1.0f - _ratio_undersped) * pitch_speed_weight;

	} else if (!flag.airspeed_enabled) {
		pitch_speed_weight = 0.0f;

	}

	// don't allow any weight to be larger than one, as it has the same effect as reducing the control
	// loop time constant and therefore can lead to a destabilization of that control loop
	weight.spe_weighting = constrain(2.0f - pitch_speed_weight, 0.f, 1.f);
	weight.ske_weighting = constrain(pitch_speed_weight, 0.f, 1.f);

	return weight;
}

void TECSControl::_calcPitchControl(float dt, const Input &input, const SpecificEnergyRates &specific_energy_rates,
				    const Param &param,
				    const Flag &flag)
{
	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
	ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rates)};

	_calcPitchControlUpdate(dt, input, seb_rate, param);
	const float pitch_setpoint{_calcPitchControlOutput(input, seb_rate, param, flag)};

	// Comply with the specified vertical acceleration limit by applying a pitch rate limit
	// NOTE: at zero airspeed, the pitch increment is unbounded
	const float pitch_increment = dt * param.vert_accel_limit / math::max(input.tas, FLT_EPSILON);
	_pitch_setpoint = constrain(pitch_setpoint, _pitch_setpoint - pitch_increment,
				    _pitch_setpoint + pitch_increment);

	//Debug Output
	_debug_output.energy_balance_rate_estimate = seb_rate.estimate;
	_debug_output.energy_balance_rate_sp = seb_rate.setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
}

TECSControl::ControlValues TECSControl::_calcPitchControlSebRate(const SpecificEnergyWeighting &weight,
		const SpecificEnergyRates &specific_energy_rates) const
{
	ControlValues seb_rate;
	/*
	 * The SKE_weighting variable controls how speed and altitude control are prioritized by the pitch demand calculation.
	 * A weighting of 1 gives equal speed and altitude priority
	 * A weighting of 0 gives 100% priority to altitude control and must be used when no airspeed measurement is available.
	 * A weighting of 2 provides 100% priority to speed control and is used when:
	 * a) an underspeed condition is detected.
	 * b) during climbout where a minimum pitch angle has been set to ensure altitude is gained. If the airspeed
	 * rises above the demanded value, the pitch angle demand is increased by the TECS controller to prevent the vehicle overspeeding.
	 * The weighting can be adjusted between 0 and 2 depending on speed and altitude accuracy requirements.
	*/
	seb_rate.setpoint = specific_energy_rates.spe_rate.setpoint * weight.spe_weighting -
			    specific_energy_rates.ske_rate.setpoint *
			    weight.ske_weighting;

	seb_rate.estimate = (specific_energy_rates.spe_rate.estimate * weight.spe_weighting) -
			    (specific_energy_rates.ske_rate.estimate * weight.ske_weighting);

	return seb_rate;
}

void TECSControl::_calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate,
		const Param &param)
{
	if (param.integrator_gain_pitch > FLT_EPSILON) {

		// Calculate derivative from change in climb angle to rate of change of specific energy balance
		const float climb_angle_to_SEB_rate = input.tas * CONSTANTS_ONE_G;

		// Calculate pitch integrator input term
		float pitch_integ_input = _getControlError(seb_rate) * param.integrator_gain_pitch / climb_angle_to_SEB_rate;

		// Prevent the integrator changing in a direction that will increase pitch demand saturation
		if (_pitch_setpoint >= param.pitch_max) {
			pitch_integ_input = min(pitch_integ_input, 0.f);

		} else if (_pitch_setpoint <= param.pitch_min) {
			pitch_integ_input = max(pitch_integ_input, 0.f);
		}

		// Update the pitch integrator state.
		_pitch_integ_state = _pitch_integ_state + pitch_integ_input * dt;

	} else {
		_pitch_integ_state = 0.0f;
	}
}

float TECSControl::_calcPitchControlOutput(const Input &input, const ControlValues &seb_rate, const Param &param,
		const Flag &flag) const
{
	// Calculate derivative from change in climb angle to rate of change of specific energy balance
	const float climb_angle_to_SEB_rate = input.tas * CONSTANTS_ONE_G;

	// Calculate a specific energy correction that doesn't include the integrator contribution
	float SEB_rate_correction = _getControlError(seb_rate) * param.pitch_damping_gain +
				    param.seb_rate_ff *
				    seb_rate.setpoint;

	// Convert the specific energy balance rate correction to a target pitch angle. This calculation assumes:
	// a) The climb angle follows pitch angle with a lag that is small enough not to destabilise the control loop.
	// b) The offset between climb angle and pitch angle (angle of attack) is constant, excluding the effect of
	// pitch transients due to control action or turbulence.
	const float pitch_setpoint_unc = SEB_rate_correction / climb_angle_to_SEB_rate + _pitch_integ_state;

	return constrain(pitch_setpoint_unc, param.pitch_min, param.pitch_max);
}

void TECSControl::_calcThrottleControl(float dt, const SpecificEnergyRates &specific_energy_rates, const Input &input, const Param &param,
				       const Flag &flag)
{
	const STERateLimit limit{_calculateTotalEnergyRateLimit(input, param)};

	// Update STE rate estimate LP filter
	const float STE_rate_estimate_raw = specific_energy_rates.spe_rate.estimate + specific_energy_rates.ske_rate.estimate;
	_ste_rate_estimate_filter.setParameters(dt, param.ste_rate_time_const);
	_ste_rate_estimate_filter.update(STE_rate_estimate_raw);

	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rates, input, param, flag)};
	_calcThrottleControlUpdate(dt, limit, ste_rate, param, flag);
	float throttle_setpoint{_calcThrottleControlOutput(dt, limit, ste_rate, input, param, flag)};

	// Rate limit the throttle demand
	if (fabsf(param.throttle_slewrate) > FLT_EPSILON) {
		const float throttle_increment_limit = dt * (param.throttle_max - param.throttle_min) * param.throttle_slewrate;
		throttle_setpoint = constrain(throttle_setpoint, _throttle_setpoint - throttle_increment_limit,
					      _throttle_setpoint + throttle_increment_limit);
	}

	_throttle_setpoint = constrain(throttle_setpoint, param.throttle_min, param.throttle_max);

	// Debug output
	_debug_output.total_energy_rate_estimate = ste_rate.estimate;
	_debug_output.total_energy_rate_sp = ste_rate.setpoint;
	_debug_output.throttle_integrator = _STE_rate_integ_state;
	_debug_output.ste_rate_min = limit.STE_rate_min;
	_debug_output.ste_rate_max = limit.STE_rate_max;
}

TECSControl::ControlValues TECSControl::_calcThrottleControlSteRate(const STERateLimit &limit,
		const SpecificEnergyRates &specific_energy_rates,
		const Input &input, const Param &param, const Flag &flag)
{
	// Output ste rate values
	ControlValues ste_rate;

	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
	
	// The STE rate setpoint must now be constrained so that we will never end up in a situation where
	// the total energy is OK, but the airspeed is dangerously low because we have too much potential energy.
	// This is prevented by first bleeding off any excess potential energy into kinetic energy and then reducing the excess
	// airspeed. However, if the pitch is controlling also the airspeed, the risk of this underspeeding is reduced.
	float STE_rate_min_adj = (0.5f * weight.ske_weighting) * limit.STE_rate_min + (1.0f - 0.5f * weight.ske_weighting) * specific_energy_rates.ske_rate.setpoint;

	//Also adjust the safety margin to the current airspeed. Full effect at min airspeed, no effect at trim airspeed.
	float scaler = constrain((input.tas / input.eas_to_tas - param.equivalent_airspeed_min) / (max(0.1f, param.equivalent_airspeed_trim - param.equivalent_airspeed_min)), 0.0f, 1.0f);
	STE_rate_min_adj = (1.0f - scaler) * STE_rate_min_adj + scaler * limit.STE_rate_min;

	
	ste_rate.setpoint = specific_energy_rates.spe_rate.setpoint + specific_energy_rates.ske_rate.setpoint;

	ste_rate.setpoint = constrain(ste_rate.setpoint, STE_rate_min_adj, limit.STE_rate_max);
	ste_rate.estimate = _ste_rate_estimate_filter.getState();

	return ste_rate;
}

void TECSControl::_calcThrottleControlUpdate(float dt, const STERateLimit &limit, const ControlValues &ste_rate,
		const Param &param, const Flag &flag)
{

	// Integral handling
	if (flag.airspeed_enabled) {
		if (param.integrator_gain_throttle > FLT_EPSILON) {
			// underspeed conditions zero out integration
			_STE_rate_integ_state = _STE_rate_integ_state + (_getControlError(ste_rate) * param.integrator_gain_throttle) * dt *
						     (1.0f - _ratio_undersped);

			const float integ_state_max = limit.STE_rate_max - ste_rate.setpoint;
			const float integ_state_min = limit.STE_rate_min - ste_rate.setpoint;
			
			_STE_rate_integ_state = constrain(_STE_rate_integ_state, integ_state_min, integ_state_max);
		} else {
			_STE_rate_integ_state = 0.0f;
		}
	}
}

float TECSControl::_calcThrottleControlOutput(const float dt, const STERateLimit &limit, const ControlValues &ste_rate,
		const Input &input,
		const Param &param,
		const Flag &flag )
{
	
	float throttle_setpoint = 0.0f;
	if (param.use_dynamic_throttle_calculation && input.rpm > 0) {

		// Throttle calculations...
		if(param.propulsion_type == 0){ // electric motor with propeller / ducted fan

			_thrust_setpoint = (ste_rate.setpoint + _STE_rate_integ_state + _getControlError(ste_rate) * param.throttle_damping_gain - limit.STE_rate_min) / input.tas * param.weight_gross;

			float rpm_setpoint = max(0.0f, _calcRequiredRPMForThrust(_thrust_setpoint, input, param));
			rpm_control.setpoint = rpm_setpoint;
			rpm_control.estimate = input.rpm;

			float max_rpm = _calcMaxRPM(input, param);
			throttle_setpoint = _control_RPM(dt, rpm_control, max_rpm, param);

			_debug_output.thrust_setpoint = _thrust_setpoint;
			_debug_output.rpm_setpoint = rpm_setpoint;
			_debug_output.max_rpm = max_rpm;

			_thrust_setpoint_valid = true;


		} else{// thrust/rpm control is not supported with ICE or jet engine yet.
			goto default_throttle_calculation;
		}

	}
	else{
		
		default_throttle_calculation:
		//The traditional way without dynamic throttle calculation (but with integrated STE rate setpoint calc for I and D terms)
		_thrust_setpoint_valid = false;

		// Calculate a predicted throttle from the demanded rate of change of energy, using the cruise throttle
		// as the starting point. Assume:
		// Specific total energy rate = _STE_rate_max is achieved when throttle is set to _throttle_setpoint_max
		// Specific total energy rate = 0 at cruise throttle
		// Specific total energy rate = _STE_rate_min is achieved when throttle is set to _throttle_setpoint_min

		// assume airspeed and density-independent delta_throttle to sink/climb rate mapping
		// TODO: include air density for thrust mappings
		const float throttle_above_trim_per_ste_rate = (param.throttle_max - param.throttle_trim) / limit.STE_rate_max;
		const float throttle_below_trim_per_ste_rate = (param.throttle_trim - param.throttle_min) / limit.STE_rate_min;

		float STE_rate_setpoint_adj =  ste_rate.setpoint + _STE_rate_integ_state + _getControlError(ste_rate) * param.throttle_damping_gain;
		// Adjust the demanded total energy rate to compensate for induced drag rise in turns.
		// Assume induced drag scales linearly with normal load factor.
		// The additional normal load factor is given by (1/cos(bank angle) - 1)
		
		STE_rate_setpoint_adj = STE_rate_setpoint_adj + param.load_factor_correction * (param.load_factor - 1.f);

		if (ste_rate.setpoint >= FLT_EPSILON) {
			// throttle is between trim and maximum
			throttle_setpoint = param.throttle_trim_adjusted + STE_rate_setpoint_adj * throttle_above_trim_per_ste_rate;

		} else {
			// throttle is between trim and minimum
			throttle_setpoint = param.throttle_trim_adjusted - STE_rate_setpoint_adj * throttle_below_trim_per_ste_rate;
		}

	}


	// ramp in max throttle setting with underspeediness value
	throttle_setpoint = _ratio_undersped * param.throttle_max + (1.0f - _ratio_undersped) * throttle_setpoint;

	return constrain(throttle_setpoint, param.throttle_min, param.throttle_max);
}

void TECSControl::resetIntegrals()
{
	_pitch_integ_state = 0.0f;
	_STE_rate_integ_state = 0.0f;
	_rpm_integ_state = 0.0f;
}

float TECS::_update_speed_setpoint(const float tas_min, const float tas_max, const float tas_setpoint, const float tas)
{
	float new_setpoint{tas_setpoint};
	const float percent_undersped = _control.getRatioUndersped();

	// Set the TAS demand to the minimum value if an underspeed condition exists to maximise climb rate
	if (percent_undersped > FLT_EPSILON) {
		// TAS setpoint is reset from external setpoint every time tecs is called, so the interpolation is still
		// between current setpoint and mininimum airspeed here (it's not feeding the newly adjusted setpoint
		// from this line back into this method each time).
		// TODO: WOULD BE GOOD to "functionalize" this library a bit and remove many of these internal states to
		// avoid the fear of side effects in simple operations like this.
		new_setpoint = tas_min * percent_undersped + (1.0f - percent_undersped) * tas_setpoint;
	}

	new_setpoint = constrain(new_setpoint, tas_min, tas_max);

	return new_setpoint;
}

void TECS::initialize(const float altitude, const float altitude_rate, const float equivalent_airspeed,
		      const float eas_to_tas, const float rpm)
{
	// Init subclasses
	TECSAltitudeReferenceModel::AltitudeReferenceState current_state{.alt = altitude,
			.alt_rate = altitude_rate};
	_altitude_reference_model.initialize(current_state);
	_airspeed_filter.initialize(equivalent_airspeed);

	TECSControl::Setpoint control_setpoint;
	control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
	control_setpoint.altitude_rate_setpoint_direct =
		_altitude_reference_model.getAltitudeReference().alt_rate; // init to reference altitude rate
	control_setpoint.tas_setpoint = equivalent_airspeed * eas_to_tas;

	const TECSControl::Input control_input{ .altitude = altitude,
						.altitude_rate = altitude_rate,
						.tas = eas_to_tas * equivalent_airspeed,
						.tas_rate = 0.0f,
						.flaps_setpoint= 0.0f,
						.rpm = rpm};

	_control.initialize(control_setpoint, control_input, _control_param, _control_flag);

	_debug_status.tecs_mode = _tecs_mode;
	_debug_status.control = _control.getDebugOutput();
	_debug_status.true_airspeed_filtered = eas_to_tas * _airspeed_filter.getState().speed;
	_debug_status.true_airspeed_derivative = eas_to_tas * _airspeed_filter.getState().speed_rate;
	_debug_status.altitude_reference = _altitude_reference_model.getAltitudeReference().alt;
	_debug_status.height_rate_reference = _altitude_reference_model.getAltitudeReference().alt_rate;
	_debug_status.height_rate_direct = _altitude_reference_model.getHeightRateSetpointDirect();

	_update_timestamp = hrt_absolute_time();
}

void TECS::update(float pitch, float altitude, float hgt_setpoint, float EAS_setpoint, float equivalent_airspeed,
		  float eas_to_tas, float throttle_min, float throttle_setpoint_max,
		  float throttle_trim, float throttle_trim_adjusted, float pitch_limit_min, float pitch_limit_max, float target_climbrate,
		  float target_sinkrate, const float speed_deriv_forward, float hgt_rate, float flaps_setpoint, float air_density, float rpm,
		  float hgt_rate_sp)
{

	// Calculate the time since last update (seconds)
	const hrt_abstime now(hrt_absolute_time());
	const float dt = static_cast<float>((now - _update_timestamp)) / 1_s;

	// Update parameters from input
	// Reference model
	_reference_param.target_climbrate = target_climbrate;
	_reference_param.target_sinkrate = target_sinkrate;
	// Control
	_control_param.tas_min = eas_to_tas * _equivalent_airspeed_min;
	_control_param.pitch_max = pitch_limit_max;
	_control_param.pitch_min = pitch_limit_min;
	_control_param.throttle_trim = throttle_trim;
	_control_param.throttle_trim_adjusted = throttle_trim_adjusted;
	_control_param.throttle_max = throttle_setpoint_max;
	_control_param.throttle_min = throttle_min;

	if (dt < DT_MIN) {
		// Update intervall too small, do not update. Assume constant states/output in this case.
		return;
	}

	if (dt > DT_MAX || _update_timestamp == 0UL) {
		// Update time intervall too large, can't guarantee sanity of state updates anymore. reset the control loop.
		initialize(altitude, hgt_rate, equivalent_airspeed, eas_to_tas, rpm);

	} else {
		// Update airspeedfilter submodule
		const TECSAirspeedFilter::Input airspeed_input{ .equivalent_airspeed = equivalent_airspeed,
				.equivalent_airspeed_rate = speed_deriv_forward / eas_to_tas};

		_airspeed_filter.update(dt, airspeed_input, _airspeed_filter_param, _control_flag.airspeed_enabled);
		const TECSAirspeedFilter::AirspeedFilterState eas = _airspeed_filter.getState();

		// Update Reference model submodule
		const TECSAltitudeReferenceModel::AltitudeReferenceState setpoint{ .alt = hgt_setpoint,
				.alt_rate = hgt_rate_sp};

		_altitude_reference_model.update(dt, setpoint, altitude, hgt_rate, _reference_param);

		TECSControl::Setpoint control_setpoint;
		control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
		control_setpoint.altitude_rate_setpoint_direct = _altitude_reference_model.getHeightRateSetpointDirect();

		// Calculate the demanded true airspeed
		// TODO this function should not be in the module. Only give feedback that the airspeed can't be achieved.
		control_setpoint.tas_setpoint = _update_speed_setpoint(eas_to_tas * _equivalent_airspeed_min,
						eas_to_tas * _equivalent_airspeed_max, EAS_setpoint * eas_to_tas, eas_to_tas * eas.speed);

		const TECSControl::Input control_input{ .altitude = altitude,
							.altitude_rate = hgt_rate,
							.tas = eas_to_tas * eas.speed,
							.tas_rate = eas_to_tas * eas.speed_rate,
							.flaps_setpoint = flaps_setpoint,
							.air_density = air_density,
							.eas_to_tas = eas_to_tas,
							.rpm = rpm};

		_control.update(dt, control_setpoint, control_input, _control_param, _control_flag);

		// Update time stamps
		_update_timestamp = now;


		// Set TECS mode for next frame
		if (_control.getRatioUndersped() > FLT_EPSILON) {
			_tecs_mode = ECL_TECS_MODE_UNDERSPEED;

		} else {
			// This is the default operation mode
			_tecs_mode = ECL_TECS_MODE_NORMAL;
		}

		_debug_status.tecs_mode = _tecs_mode;
		_debug_status.control = _control.getDebugOutput();
		_debug_status.true_airspeed_filtered = eas_to_tas * eas.speed;
		_debug_status.true_airspeed_derivative = eas_to_tas * eas.speed_rate;
		_debug_status.altitude_reference = control_setpoint.altitude_reference.alt;
		_debug_status.height_rate_reference = control_setpoint.altitude_reference.alt_rate;
		_debug_status.height_rate_direct = _altitude_reference_model.getHeightRateSetpointDirect();
	}
}


/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "generic_i2c_rpm.hpp"

generic_i2c_rpm::generic_i2c_rpm(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

int generic_i2c_rpm::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: %" PRId8 ", pool: %" PRId32 ", magenet: %" PRId32, get_device_address(),
		  _param_generic_i2c_rpm_pool.get(),
		  _param_generic_i2c_rpm_magnet.get());

	initCounter();
	ScheduleOnInterval(_param_generic_i2c_rpm_pool.get());
	_rpm_pub.advertise();

	return PX4_OK;
}

int generic_i2c_rpm::probe()
{
	setRegister('p', 0b00100000);

	uint8_t s = readRegister('p', 1);
	PX4_DEBUG("status register: %" PRId8 " fail_count: %" PRId8, s, _tranfer_fail_count);

	if (s != 0b00100000) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

uint32_t generic_i2c_rpm::getRPM()
{
	// the rpm is stored in 3 bytes
	uint8_t a[3] = readRegister('r', 3);

	return uint32_t(a[0] << 16 + a[2] << 8 + a[2]);
}

uint8_t generic_i2c_rpm::readRegister(uint8_t reg, unit8_t cnt)
{
	uint8_t rcv[3]{};
	int ret = transfer(&reg, 1, &rcv, cnt);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
	}

	return rcv;
}

// Configure generic_i2c_rpm
void generic_i2c_rpm::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
	}
}

void generic_i2c_rpm::RunImpl()
{

	int32_t diffTime = hrt_elapsed_time(&_last_measurement_time);

	// check if delay is enought
	if (diffTime < _param_generic_i2c_rpm_pool.get() / 2) {
		PX4_ERR("generic_i2c_rpm loop called too early");
		return;
	}

	_last_measurement_time = hrt_absolute_time();

	uint32_t rpm = getRPM();

	//check if device failed or reset
	uint8_t s = readRegister('p', 1);

	if (_tranfer_fail_count > 0 || s != 0b00100000) {
		PX4_ERR("generic_i2c_rpm RPM sensor restart: fail count %d, status: %d",
			_tranfer_fail_count, s);
		return;
	}

	// Calculate RPM and accuracy estimation
	float indicated_rpm = (float)rpm / _param_generic_i2c_rpm_magnet.get();
	float estimated_accurancy = 1.0f;

	// publish data to uorb
	rpm_s msg{};
	msg.indicated_frequency_rpm = indicated_rpm;
	msg.estimated_accurancy_rpm = estimated_accurancy;
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);

}

void generic_i2c_rpm::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_generic_i2c_rpm_pool.get());
}

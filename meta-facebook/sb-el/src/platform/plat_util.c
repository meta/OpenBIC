/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "libutil.h"
#include "plat_util.h"
#include "plat_i2c.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_util);

bool plat_i2c_read(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	memset(data, 0, len);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = offset;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, offset);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

bool plat_i2c_write(uint8_t bus, uint8_t addr, uint8_t offset, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = len + 1;
	i2c_msg.data[0] = offset;

	if (len > 0)
		memcpy(&i2c_msg.data[1], data, len);

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write reg, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, offset);
		return false;
	}
	return true;
}

bool linear11_from_scaled(uint16_t val, uint16_t scale, uint16_t *raw)
{
	CHECK_NULL_ARG_WITH_RETURN(raw, false);
	if (!scale)
		return false;

	for (int8_t exponent = -16; exponent <= 15; exponent++) {
		uint32_t numerator = val;
		uint32_t denominator = scale;

		if (exponent < 0)
			numerator <<= -exponent;
		else
			denominator <<= exponent;

		uint32_t mantissa = (numerator + denominator / 2) / denominator;
		if (mantissa <= 0x3ff) {
			*raw = (((uint16_t)exponent & 0x1f) << 11) | (uint16_t)mantissa;
			return true;
		}
	}

	return false;
}

uint16_t linear11_to_scaled(uint16_t raw, uint16_t scale)
{
	if (!scale)
		return 0;

	int8_t exponent = raw >> 11;
	int16_t mantissa = raw & 0x7ff;

	if (exponent & 0x10)
		exponent |= 0xe0;
	if (mantissa & 0x400)
		mantissa |= 0xf800;
	if (mantissa <= 0)
		return 0;

	uint32_t numerator = (uint32_t)mantissa * scale;
	uint32_t denominator = 1;
	if (exponent < 0)
		denominator <<= -exponent;
	else
		numerator <<= exponent;

	return (numerator + denominator / 2) / denominator;
}

uint16_t linear16u_from_scaled(uint16_t val, uint16_t scale, uint8_t fractional_bits)
{
	if (!scale || fractional_bits > 16)
		return 0;

	uint32_t multiplier = 1U << fractional_bits;
	return ((uint32_t)val * multiplier + scale / 2) / scale;
}

uint16_t linear16u_to_scaled(uint16_t raw, uint16_t scale, uint8_t fractional_bits)
{
	if (!scale || fractional_bits > 16)
		return 0;

	uint32_t divisor = 1U << fractional_bits;
	return ((uint32_t)raw * scale + divisor / 2) / divisor;
}

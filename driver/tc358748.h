/*
 * tc358748_.h - tc358748 sensor header
 *
 * Copyright (c) 2020, RidgeRun. All rights reserved.
 *
 * Contact us: support@ridgerun.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __tc358748_H__
#define __tc358748_H__

/* tc358748 - sensor parameters */
#define tc358748_MIN_GAIN		                (0)
#define tc358748_MAX_GAIN		                (978)
#define tc358748_ANALOG_GAIN_C0		        (1024)
#define tc358748_SHIFT_8_BITS			(8)
#define tc358748_MIN_FRAME_LENGTH		        (256)
#define tc358748_MAX_FRAME_LENGTH		        (65535)
#define tc358748_MIN_COARSE_EXPOSURE	        (1)
#define tc358748_MAX_COARSE_DIFF		        (10)
#define tc358748_MASK_LSB_2_BITS			0x0003
#define tc358748_MASK_LSB_8_BITS			0x00ff

/* tc358748 sensor register address */
#define tc358748_MODEL_ID_ADDR_MSB		0x0000
#define tc358748_MODEL_ID_ADDR_LSB		0x0001
#define tc358748_ANALOG_GAIN_ADDR_MSB		0x0204
#define tc358748_ANALOG_GAIN_ADDR_LSB		0x0205
#define tc358748_DIGITAL_GAIN_ADDR_MSB		0x020e
#define tc358748_DIGITAL_GAIN_ADDR_LSB		0x020f
#define tc358748_FRAME_LENGTH_ADDR_MSB		0x0340
#define tc358748_FRAME_LENGTH_ADDR_LSB		0x0341
#define tc358748_COARSE_INTEG_TIME_ADDR_MSB	0x0202
#define tc358748_COARSE_INTEG_TIME_ADDR_LSB	0x0203
#define tc358748_FINE_INTEG_TIME_ADDR_MSB		0x0200
#define tc358748_FINE_INTEG_TIME_ADDR_LSB		0x0201
#define tc358748_GROUP_HOLD_ADDR		        0x0104

#endif /* __tc358748_H__ */

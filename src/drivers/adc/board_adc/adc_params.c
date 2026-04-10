/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * ADC samples discarded before reading.
 *
 * Number of ADC samples to discard per channel before taking
 * the real measurement. Discarded samples allow the sample-and-hold
 * capacitor to settle, mitigating crosstalk from previously sampled
 * channels. A value of 0 gives the original behavior (no discards).
 *
 * @min 0
 * @max 100
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(ADC_DIS_CNT, 0);

/**
 * ADC discard channel bitmask.
 *
 * Bitmask selecting which ADC channels have samples discarded
 * before the real measurement (controlled by ADC_DIS_CNT).
 * Each bit corresponds to an ADC channel number (bit 0 = channel 0,
 * bit 4 = channel 4, etc.). Only channels with their bit set will
 * perform discard reads; all other channels are sampled directly.
 * A value of 0 disables discarding on all channels.
 *
 * @min 0
 * @max 2147483647
 * @group Sensors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(ADC_DIS_CHN, 0);

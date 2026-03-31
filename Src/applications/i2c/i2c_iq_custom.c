// Copyright (c) Acconeer AB, 2023-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "acc_config.h"
#include "acc_definitions_a121.h"
#include "acc_detector_distance.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_processing.h"

#include "distance_reg_protocol.h"
#include "i2c_distance_detector_core.h"

#define SENSOR_ID          (1U)
#define SENSOR_TIMEOUT_MS  (10000U)
#define IQ_MAX_POINTS      (1024U)
#define IQ_CMD_BASE        ((uint16_t)6U)
#define IQ_STATUS_READY_BIT (1U << 16U)

static float    iq_real_buffer[IQ_MAX_POINTS] = {0.0f};
static float    iq_imag_buffer[IQ_MAX_POINTS] = {0.0f};
static bool     iq_valid                      = false;
static uint16_t iq_num_points                 = 0U;
static uint16_t iq_current_index              = 0U;

static bool my_capture_iq_frame(distance_detector_resources_t *resources,
                                float                        *out_real,
                                float                        *out_imag,
                                uint16_t                     *out_num_points);

static bool core_detector_is_ready(void)
{
	uint32_t status = i2c_distance_detector_core_get_status_default();
	uint32_t mask   = DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK |
	                  DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_OK_MASK |
	                  DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_OK_MASK;
	return (status & mask) == mask;
}

void i2c_distance_detector_core_on_measurement_done(const acc_detector_distance_result_t *result)
{
	(void)result;
}

void command_handler(uint32_t command)
{
	uint16_t op    = (uint16_t)(command >> 16);
	uint16_t index = (uint16_t)(command & 0xFFFFU);

	if (op == IQ_CMD_BASE)
	{
		acc_integration_critical_section_enter();
		iq_valid         = false;
		iq_num_points    = 0U;
		iq_current_index = 0U;
		acc_integration_critical_section_exit();

		if (!core_detector_is_ready())
		{
			return;
		}

		uint16_t num_points = 0U;
		if (my_capture_iq_frame(i2c_distance_detector_core_get_resources(), iq_real_buffer, iq_imag_buffer, &num_points) &&
		    num_points > 0U)
		{
			uint16_t requested_index = index;
			if (requested_index >= num_points)
			{
				requested_index = (uint16_t)(num_points - 1U);
			}

			acc_integration_critical_section_enter();
			iq_num_points    = num_points;
			iq_current_index = requested_index;
			iq_valid         = true;
			acc_integration_critical_section_exit();
		}

		return;
	}

	if (command == DISTANCE_REG_COMMAND_ENUM_APPLY_CONFIGURATION ||
	    command == DISTANCE_REG_COMMAND_ENUM_CALIBRATE ||
	    command == DISTANCE_REG_COMMAND_ENUM_RECALIBRATE ||
	    command == DISTANCE_REG_COMMAND_ENUM_APPLY_CONFIG_AND_CALIBRATE ||
	    command == DISTANCE_REG_COMMAND_ENUM_MEASURE_DISTANCE)
	{
		acc_integration_critical_section_enter();
		iq_valid         = false;
		iq_num_points    = 0U;
		iq_current_index = 0U;
		acc_integration_critical_section_exit();
	}

	i2c_distance_detector_core_command_handler(command);
}

uint32_t i2c_distance_detector_get_status(void)
{
	uint32_t status = i2c_distance_detector_core_get_status_default();

	acc_integration_critical_section_enter();
	bool iq_is_valid = iq_valid;
	acc_integration_critical_section_exit();

	if (iq_is_valid)
	{
		status |= IQ_STATUS_READY_BIT;
	}

	return status;
}

uint32_t i2c_distance_detector_get_counter(void)
{
	acc_integration_critical_section_enter();
	bool     iq_is_valid   = iq_valid;
	uint16_t iq_points_len = iq_num_points;
	acc_integration_critical_section_exit();

	if (iq_is_valid)
	{
		return (uint32_t)iq_points_len;
	}

	return i2c_distance_detector_core_get_counter_default();
}

float i2c_distance_detector_get_peak_distance(uint16_t peak_id)
{
	acc_integration_critical_section_enter();
	bool     iq_is_valid   = iq_valid;
	uint16_t idx           = iq_current_index;
	uint16_t num_points    = iq_num_points;
	float    iq_real_value = 0.0f;

	if (iq_is_valid && peak_id == 0U && idx < num_points && idx < IQ_MAX_POINTS)
	{
		iq_real_value = iq_real_buffer[idx];
		acc_integration_critical_section_exit();
		return iq_real_value;
	}

	acc_integration_critical_section_exit();
	return i2c_distance_detector_core_get_peak_distance_default(peak_id);
}

float i2c_distance_detector_get_peak_strength(uint16_t peak_id)
{
	acc_integration_critical_section_enter();
	bool     iq_is_valid   = iq_valid;
	uint16_t idx           = iq_current_index;
	uint16_t num_points    = iq_num_points;
	float    iq_imag_value = 0.0f;

	if (iq_is_valid && peak_id == 0U && idx < num_points && idx < IQ_MAX_POINTS)
	{
		iq_imag_value = iq_imag_buffer[idx];
		acc_integration_critical_section_exit();
		return iq_imag_value;
	}
	acc_integration_critical_section_exit();

	float peak_strength = 0.0f;
	if (peak_id < ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES)
	{
		distance_detector_resources_t *resources = i2c_distance_detector_core_get_resources();
		peak_strength = resources->result.strengths[peak_id];
	}
	return peak_strength;
}

int acconeer_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	i2c_distance_detector_core_run();
	return EXIT_FAILURE;
}

static bool my_capture_iq_frame(distance_detector_resources_t *resources,
                                float                        *out_real,
                                float                        *out_imag,
                                uint16_t                     *out_num_points)
{
	if (resources == NULL || out_real == NULL || out_imag == NULL || out_num_points == NULL)
	{
		return false;
	}

	*out_num_points = 0U;

	bool result_available = false;
	bool status           = true;
	acc_hal_integration_sensor_enable(SENSOR_ID);

	do
	{
		if (!acc_detector_distance_prepare(resources->handle,
		                                   resources->config,
		                                   resources->sensor,
		                                   &resources->sensor_cal_result,
		                                   resources->buffer,
		                                   resources->buffer_size))
		{
			status = false;
			break;
		}

		if (!acc_sensor_measure(resources->sensor))
		{
			status = false;
			break;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			status = false;
			break;
		}

		if (!acc_sensor_read(resources->sensor, resources->buffer, resources->buffer_size))
		{
			status = false;
			break;
		}

		if (!acc_detector_distance_process(resources->handle,
		                                   resources->buffer,
		                                   resources->detector_cal_result_static,
		                                   &resources->detector_cal_result_dynamic,
		                                   &result_available,
		                                   &resources->result))
		{
			status = false;
			break;
		}

		if (!result_available)
		{
			status = false;
			break;
		}

		acc_processing_result_t   *proc_result = resources->result.processing_result;
		acc_processing_metadata_t *proc_meta   = resources->result.processing_metadata;
		const acc_config_t        *sensor_cfg  = resources->result.sensor_config;

		if (proc_result == NULL || proc_meta == NULL || sensor_cfg == NULL || proc_result->frame == NULL)
		{
			status = false;
			break;
		}

		uint16_t num_bins         = proc_meta->sweep_data_length;
		uint16_t sweeps_per_frame = acc_config_sweeps_per_frame_get(sensor_cfg);

		if (num_bins == 0U || sweeps_per_frame == 0U)
		{
			status = false;
			break;
		}

		if (num_bins > IQ_MAX_POINTS)
		{
			num_bins = IQ_MAX_POINTS;
		}

		acc_int16_complex_t *frame = proc_result->frame;
		for (uint16_t bin = 0U; bin < num_bins; bin++)
		{
			float sum_real = 0.0f;
			float sum_imag = 0.0f;

			for (uint16_t sweep = 0U; sweep < sweeps_per_frame; sweep++)
			{
				uint32_t idx = (uint32_t)sweep * (uint32_t)num_bins + (uint32_t)bin;
				sum_real += frame[idx].real;
				sum_imag += frame[idx].imag;
			}

			float denom = (float)sweeps_per_frame;
			out_real[bin] = sum_real / denom;
			out_imag[bin] = sum_imag / denom;
		}

		*out_num_points = num_bins;
	} while (false);

	acc_hal_integration_sensor_disable(SENSOR_ID);
	return status;
}

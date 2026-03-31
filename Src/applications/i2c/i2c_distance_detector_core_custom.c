// Copyright (c) Acconeer AB, 2023-2025
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.
//
// Slim/custom core for I2C distance detector.
//
// Intentionally removes optional features (wakeup auto-measure, UART log
// toggling/config printing) and removes long printf/uart_log strings to save
// FLASH. The normal detector lifecycle is preserved.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "acc_definitions_a121.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_rss_a121.h"

#include "acc_reg_protocol.h"
#include "distance_reg_protocol.h"
#include "i2c_application_system.h"
#include "i2c_distance_detector.h"
#include "i2c_distance_detector_core.h"

#define SENSOR_ID         (1U)
#define SENSOR_TIMEOUT_MS (10000U)

static distance_detector_resources_t detector_resources       = {0};
static uint32_t                      i2c_detector_command     = 0U;
static uint32_t                      i2c_detector_status      = 0U;
static uint32_t                      measure_distance_counter = 0U;
static bool                          measure_distance_error   = false;

static uint32_t get_command(void);
static void     detector_status_set_bits(uint32_t bit_mask);
static void     detector_status_clr_bits(uint32_t bit_mask);
static bool     detector_status_test_bits(uint32_t bit_mask);
static void     create_sensor(distance_detector_resources_t *resources);
static void     apply_detector_config(distance_detector_resources_t *resources);
static void     calibrate_sensor(distance_detector_resources_t *resources);
static void     calibrate_detector(distance_detector_resources_t *resources, bool update_calibration);
static bool     is_detector_ready(void);
static bool     detector_get_next(distance_detector_resources_t *resources);
static void     print_distance_result(const acc_detector_distance_result_t *result);

/* Forward: default implementation in core; IQ stream overrides. */
void i2c_distance_detector_core_command_handler(uint32_t command);

/* Weak default: IQ stream overrides CAPTURE_IQ and delegates to core. */
__attribute__((weak)) void command_handler(uint32_t command)
{
	i2c_distance_detector_core_command_handler(command);
}

/* Optional callback after a successful distance measurement (IQ stream overrides). */
__attribute__((weak)) void i2c_distance_detector_core_on_measurement_done(const acc_detector_distance_result_t *result)
{
	(void)result;
}

void i2c_distance_detector_core_run(void)
{
	bool setup_status = true;

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (acc_rss_hal_register(hal))
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_RSS_REGISTER_OK_MASK);
	else
	{
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
		                         DISTANCE_REG_DETECTOR_STATUS_FIELD_RSS_REGISTER_ERROR_MASK);
		setup_status = false;
	}

	if (setup_status)
	{
		detector_resources.config = acc_detector_distance_config_create();
		if (detector_resources.config != NULL)
		{
			distance_reg_protocol_write_default();
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_CREATE_OK_MASK);
		}
		else
		{
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
			                         DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_CREATE_ERROR_MASK);
			setup_status = false;
		}
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);

	if (setup_status)
		create_sensor(&detector_resources);

	i2c_application_system_init();
	distance_reg_protocol_setup();

	while (true)
	{
		uint32_t command = get_command();

		if (command == 0)
		{
			// If we woke up due to interrupt, the command will be fetched next loop.
			if (i2c_application_system_test_wakeup_pin())
			{
				i2c_application_system_wait_for_interrupt();
			}
			else
			{
				// Enter low power state; no auto-measure on wakeup in custom core.
				i2c_application_system_set_ready_pin(false);
				i2c_application_enter_low_power_state();
			}

			if (command == 0)
			{
				i2c_application_system_set_ready_pin(true);
				continue;
			}
		}

		if (command == DISTANCE_REG_COMMAND_ENUM_RESET_MODULE)
		{
			i2c_application_system_reset();
			continue;
		}

		if (detector_status_test_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK))
			continue;

		command_handler(command);

		detector_status_clr_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_BUSY_MASK);
		i2c_application_system_set_ready_pin(true);
	}
}

distance_detector_resources_t *i2c_distance_detector_core_get_resources(void)
{
	return &detector_resources;
}

uint32_t i2c_distance_detector_core_get_counter_default(void)
{
	acc_integration_critical_section_enter();
	uint32_t counter = measure_distance_counter;
	acc_integration_critical_section_exit();
	return counter;
}

float i2c_distance_detector_core_get_peak_distance_default(uint16_t peak_id)
{
	float peak_distance = 0.0f;
	if (peak_id < ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES)
		peak_distance = detector_resources.result.distances[peak_id];
	return peak_distance;
}

uint32_t i2c_distance_detector_core_get_status_default(void)
{
	acc_integration_critical_section_enter();
	uint32_t status = i2c_detector_status;
	acc_integration_critical_section_exit();
	return status;
}

/* Weak so i2c_iq_custom can override IQ endpoints without linker conflicts. */
__attribute__((weak)) acc_detector_distance_config_t *i2c_distance_detector_get_config(void)
{
	return detector_resources.config;
}

__attribute__((weak)) bool i2c_distance_detector_command(uint32_t command)
{
	bool status = false;
	acc_integration_critical_section_enter();
	if (i2c_detector_command == 0U)
	{
		i2c_application_system_set_ready_pin(false);
		i2c_detector_status  |= DISTANCE_REG_DETECTOR_STATUS_FIELD_BUSY_MASK;
		i2c_detector_command  = command;
		status                = true;
	}
	acc_integration_critical_section_exit();
	return status;
}

__attribute__((weak)) uint32_t i2c_distance_detector_get_status(void)
{
	return i2c_distance_detector_core_get_status_default();
}

__attribute__((weak)) uint32_t i2c_distance_detector_get_result(void)
{
	uint32_t value = 0;
	value = (detector_resources.result.num_distances << DISTANCE_REG_DISTANCE_RESULT_FIELD_NUM_DISTANCES_POS) &
	        DISTANCE_REG_DISTANCE_RESULT_FIELD_NUM_DISTANCES_MASK;
	if (detector_resources.result.near_start_edge_status)
		value |= DISTANCE_REG_DISTANCE_RESULT_FIELD_NEAR_START_EDGE_MASK;
	if (detector_resources.result.calibration_needed)
		value |= DISTANCE_REG_DISTANCE_RESULT_FIELD_CALIBRATION_NEEDED_MASK;
	if (measure_distance_error)
		value |= DISTANCE_REG_DISTANCE_RESULT_FIELD_MEASURE_DISTANCE_ERROR_MASK;

	// Temperature packing preserved for protocol compatibility.
	uint32_t temp = (uint32_t)detector_resources.result.temperature;
	temp   = (temp << DISTANCE_REG_DISTANCE_RESULT_FIELD_TEMPERATURE_POS) & DISTANCE_REG_DISTANCE_RESULT_FIELD_TEMPERATURE_MASK;
	value |= temp;
	return value;
}

__attribute__((weak)) uint32_t i2c_distance_detector_get_counter(void)
{
	return i2c_distance_detector_core_get_counter_default();
}

__attribute__((weak)) float i2c_distance_detector_get_peak_distance(uint16_t peak_id)
{
	return i2c_distance_detector_core_get_peak_distance_default(peak_id);
}

__attribute__((weak)) float i2c_distance_detector_get_peak_strength(uint16_t peak_id)
{
	float peak_strength = 0.0f;
	if (peak_id < ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES)
		peak_strength = detector_resources.result.strengths[peak_id];
	return peak_strength;
}

// measure_on_wakeup feature removed from custom core:
// keep symbols for protocol access; always return false.
__attribute__((weak)) void i2c_distance_detector_measure_on_wakeup(bool enable)
{
	(void)enable;
}

__attribute__((weak)) bool i2c_distance_detector_get_measure_on_wakeup(void)
{
	return false;
}

void i2c_distance_detector_core_command_handler(uint32_t command)
{
	bool do_apply_config = false;
	bool do_calibrate    = false;
	bool do_recalibrate  = false;

	switch (command)
	{
		case DISTANCE_REG_COMMAND_ENUM_APPLY_CONFIGURATION:
			if (!detector_status_test_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK))
				do_apply_config = true;
			break;
		case DISTANCE_REG_COMMAND_ENUM_CALIBRATE:
			if (detector_status_test_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK))
				do_calibrate = true;
			break;
		case DISTANCE_REG_COMMAND_ENUM_RECALIBRATE:
			if (detector_status_test_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK))
				do_recalibrate = true;
			break;
		case DISTANCE_REG_COMMAND_ENUM_APPLY_CONFIG_AND_CALIBRATE:
			if (!detector_status_test_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK))
			{
				do_apply_config = true;
				do_calibrate    = true;
			}
			break;
		case DISTANCE_REG_COMMAND_ENUM_MEASURE_DISTANCE:
			memset(&detector_resources.result, 0, sizeof(detector_resources.result));
			acc_integration_critical_section_enter();
			measure_distance_counter++;
			acc_integration_critical_section_exit();

			if (detector_get_next(&detector_resources))
			{
				print_distance_result(&detector_resources.result);
				measure_distance_error = false;
			}
			else
			{
				measure_distance_error = true;
			}
			break;
		default:
			// Unsupported/ignored command.
			break;
	}

	if (do_apply_config)
		apply_detector_config(&detector_resources);

	if (do_calibrate || do_recalibrate)
	{
		calibrate_sensor(&detector_resources);
		calibrate_detector(&detector_resources, do_recalibrate);
	}
}

static uint32_t get_command(void)
{
	acc_integration_critical_section_enter();
	uint32_t command = i2c_detector_command;
	i2c_detector_command = 0U;
	acc_integration_critical_section_exit();
	return command;
}

static void detector_status_set_bits(uint32_t bit_mask)
{
	acc_integration_critical_section_enter();
	i2c_detector_status |= bit_mask;
	acc_integration_critical_section_exit();
}

static void detector_status_clr_bits(uint32_t bit_mask)
{
	acc_integration_critical_section_enter();
	i2c_detector_status &= ~bit_mask;
	acc_integration_critical_section_exit();
}

static bool detector_status_test_bits(uint32_t bit_mask)
{
	acc_integration_critical_section_enter();
	bool status = (i2c_detector_status & bit_mask) == bit_mask;
	acc_integration_critical_section_exit();
	return status;
}

static void create_sensor(distance_detector_resources_t *resources)
{
	acc_hal_integration_sensor_enable(SENSOR_ID);
	resources->sensor = acc_sensor_create(SENSOR_ID);
	acc_hal_integration_sensor_disable(SENSOR_ID);

	if (resources->sensor != NULL)
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CREATE_OK_MASK);
	else
	{
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
		                         DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CREATE_ERROR_MASK);
	}
}

static void apply_detector_config(distance_detector_resources_t *resources)
{
	bool status = true;

	resources->handle = acc_detector_distance_create(resources->config);
	if (resources->handle != NULL)
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CREATE_OK_MASK);
	else
	{
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
		                         DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CREATE_ERROR_MASK);
		status = false;
	}

	if (status)
	{
		if (acc_detector_distance_get_sizes(resources->handle, &(resources->buffer_size), &(resources->detector_cal_result_static_size)))
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_BUFFER_OK_MASK);
		else
		{
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
			                         DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_BUFFER_ERROR_MASK);
			status = false;
		}
	}

	if (status)
	{
		resources->buffer = acc_integration_mem_alloc(resources->buffer_size);
		if (resources->buffer != NULL)
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_BUFFER_OK_MASK);
		else
		{
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
			                         DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_BUFFER_ERROR_MASK);
			status = false;
		}
	}

	if (status)
	{
		resources->detector_cal_result_static = acc_integration_mem_alloc(resources->detector_cal_result_static_size);
		if (resources->detector_cal_result_static != NULL)
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CALIBRATION_BUFFER_OK_MASK);
		else
		{
			detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK |
			                         DISTANCE_REG_DETECTOR_STATUS_FIELD_CALIBRATION_BUFFER_ERROR_MASK);
			status = false;
		}
	}

	if (status)
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK);
	else
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_ERROR_MASK | DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_ERROR_MASK);
}

static void calibrate_sensor(distance_detector_resources_t *resources)
{
	acc_hal_integration_sensor_enable(SENSOR_ID);

	detector_status_clr_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_OK_MASK |
	                         DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_ERROR_MASK);

	bool status;
	bool cal_complete = false;

	do
	{
		status = acc_sensor_calibrate(resources->sensor, &cal_complete, &resources->sensor_cal_result, resources->buffer, resources->buffer_size);
		if (cal_complete)
			break;
		if (status)
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
	} while (status);

	if (status)
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_OK_MASK);
	else
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_ERROR_MASK);

	acc_hal_integration_sensor_disable(SENSOR_ID);
}

static void calibrate_detector(distance_detector_resources_t *resources, bool update_calibration)
{
	bool done = false;
	bool status;

	acc_hal_integration_sensor_enable(SENSOR_ID);

	detector_status_clr_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_OK_MASK |
	                         DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_ERROR_MASK);

	do
	{
		if (update_calibration)
			status = acc_detector_distance_update_calibration(resources->sensor,
			                                                  resources->handle,
			                                                  &resources->sensor_cal_result,
			                                                  resources->buffer,
			                                                  resources->buffer_size,
			                                                  &resources->detector_cal_result_dynamic,
			                                                  &done);
		else
			status = acc_detector_distance_calibrate(resources->sensor,
			                                         resources->handle,
			                                         &resources->sensor_cal_result,
			                                         resources->buffer,
			                                         resources->buffer_size,
			                                         resources->detector_cal_result_static,
			                                         resources->detector_cal_result_static_size,
			                                         &resources->detector_cal_result_dynamic,
			                                         &done);

		if (done)
			break;
		if (status)
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
	} while (status);

	if (status)
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_OK_MASK);
	else
		detector_status_set_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_ERROR_MASK);

	acc_hal_integration_sensor_disable(SENSOR_ID);
}

static bool is_detector_ready(void)
{
	return detector_status_test_bits(DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK |
	                                 DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_OK_MASK |
	                                 DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_OK_MASK);
}

static bool detector_get_next(distance_detector_resources_t *resources)
{
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

		if (result_available)
			i2c_distance_detector_core_on_measurement_done(&resources->result);
	} while (!result_available);

	acc_hal_integration_sensor_disable(SENSOR_ID);
	return status;
}

static void print_distance_result(const acc_detector_distance_result_t *result)
{
	// Custom core removes verbose UART output to save FLASH.
	(void)result;
}


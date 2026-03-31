// Copyright (c) Acconeer AB, 2023-2025
// All rights reserved

#ifndef I2C_DISTANCE_DETECTOR_CORE_H_
#define I2C_DISTANCE_DETECTOR_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#include "acc_detector_distance.h"
#include "acc_sensor.h"

/**
 * @brief Detector resources (buffer, config, sensor, calibration) used by core and IQ stream.
 */
typedef struct distance_detector_resources
{
	acc_sensor_t                     *sensor;
	acc_cal_result_t                  sensor_cal_result;
	acc_detector_distance_config_t   *config;
	acc_detector_distance_handle_t   *handle;
	acc_detector_distance_result_t    result;
	void                             *buffer;
	uint32_t                          buffer_size;
	uint8_t                          *detector_cal_result_static;
	uint32_t                          detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
} distance_detector_resources_t;

/**
 * @brief Run the distance detector main loop (setup + command loop).
 *        Does not return. Called from acconeer_main() in the app layer.
 */
void i2c_distance_detector_core_run(void);

/**
 * @brief Default command handler (APPLY_CONFIG, CALIBRATE, MEASURE_DISTANCE, etc.).
 *        IQ stream app calls this for commands it does not handle.
 */
void i2c_distance_detector_core_command_handler(uint32_t command);

/**
 * @brief Default measure counter value (for IQ stream get_counter when no IQ frame active).
 */
uint32_t i2c_distance_detector_core_get_counter_default(void);

/**
 * @brief Default peak distance (for IQ stream get_peak_distance when no IQ frame or peak_id != 0).
 */
float i2c_distance_detector_core_get_peak_distance_default(uint16_t peak_id);

/**
 * @brief Default status value (for IQ stream get_status to OR in IQ_READY_STATUS_BIT).
 */
uint32_t i2c_distance_detector_core_get_status_default(void);

/**
 * @brief Get pointer to core detector resources (for IQ stream capture_iq_raw_frame).
 */
distance_detector_resources_t *i2c_distance_detector_core_get_resources(void);

/**
 * @brief Optional callback after a successful distance measurement (weak in core).
 *        IQ stream overrides to cache sweep_data_length and sweeps_per_frame.
 */
void i2c_distance_detector_core_on_measurement_done(const acc_detector_distance_result_t *result);

#endif

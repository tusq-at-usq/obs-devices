/******************************************************************************/
/*                                                                            */
/*                Advanced Navigation Packet Protocol Library                 */
/*             C Language Dynamic Certus Mini D SDK, Version 7.3              */
/*                    Copyright 2024, Advanced Navigation                     */
/*                                                                            */
/******************************************************************************/
/*
 * Copyright (C) 2024 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef INS_PACKETS_H_
#define INS_PACKETS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define MAXIMUM_EXTENDED_SATELLITES 28
#define MAXIMUM_PACKET_PERIODS 50

#define START_SYSTEM_PACKETS 0
#define START_STATE_PACKETS 20
#define START_CONFIGURATION_PACKETS 180

typedef enum
{
	packet_id_acknowledge,
	packet_id_request,
	packet_id_boot_mode,
	packet_id_device_information,
	packet_id_restore_factory_settings,
	packet_id_reset,
	packet_id_6_reserved,
	packet_id_file_transfer_request,
	packet_id_file_transfer_acknowledge,
	packet_id_file_transfer,
	packet_id_serial_port_passthrough,
	packet_id_ip_configuration,
	packet_id_12_reserved,
	packet_id_extended_device_information,
	packet_id_subcomponent_information,
	end_system_packets,

	packet_id_system_state = START_STATE_PACKETS,
	packet_id_unix_time,
	packet_id_formatted_time,
	packet_id_status,
	packet_id_position_standard_deviation,
	packet_id_velocity_standard_deviation,
	packet_id_euler_orientation_standard_deviation,
	packet_id_quaternion_orientation_standard_deviation,
	packet_id_raw_sensors,
	packet_id_raw_gnss,
	packet_id_satellites,
	packet_id_satellites_detailed,
	packet_id_geodetic_position,
	packet_id_ecef_position,
	packet_id_utm_position,
	packet_id_ned_velocity,
	packet_id_body_velocity,
	packet_id_acceleration,
	packet_id_body_acceleration,
	packet_id_euler_orientation,
	packet_id_quaternion_orientation,
	packet_id_dcm_orientation,
	packet_id_angular_velocity,
	packet_id_angular_acceleration,
	packet_id_external_position_velocity,
	packet_id_external_position,
	packet_id_external_velocity,
	packet_id_external_body_velocity,
	packet_id_external_heading,
	packet_id_running_time,
	packet_id_local_magnetics,
	packet_id_odometer_state,
	packet_id_external_time,
	packet_id_external_depth,
	packet_id_geoid_height,
	packet_id_rtcm_corrections,
	packet_id_56_reserved,
	packet_id_wind,
	packet_id_heave,
	packet_id_59_reserved,
	packet_id_raw_satellite_data,
	packet_id_raw_satellite_ephemeris,
	packet_id_62_reserved,
	packet_id_63_reserved,
	packet_id_64_reserved,
	packet_id_65_reserved,
	packet_id_gnss_summary,
	packet_id_external_odometer,
	packet_id_external_air_data,
	packet_id_gnss_receiver_information,
	packet_id_raw_dvl_data,
	packet_id_north_seeking_status,
	packet_id_gimbal_state,
	packet_id_automotive,
	packet_id_74_reserved,
	packet_id_external_magnetometers,
	packet_id_76_reserved,
	packet_id_77_reserved,
	packet_id_78_reserved,
	packet_id_79_reserved,
	packet_id_basestation,
	packet_id_81_reserved,
	packet_id_82_reserved,
	packet_id_zero_angular_velocity,
	packet_id_extended_satellites,
	packet_id_sensor_temperatures,
	packet_id_system_temperature,
	packet_id_87_reserved,
	packet_id_88_reserved,
	packet_id_vessel_motion,
	packet_id_automatic_magnetometer_calibration_status,
	end_state_packets,

	packet_id_packet_timer_period = START_CONFIGURATION_PACKETS,
	packet_id_packet_periods,
	packet_id_baud_rates,
	packet_id_183_reserved,
	packet_id_sensor_ranges,
	packet_id_installation_alignment,
	packet_id_filter_options,
	packet_id_187_reserved,
	packet_id_gpio_configuration,
	packet_id_magnetic_calibration_values,
	packet_id_magnetic_calibration_configuration,
	packet_id_magnetic_calibration_status,
	packet_id_odometer_configuration,
	packet_id_zero_alignment,
	packet_id_reference_offsets,
	packet_id_gpio_output_configuration,
	packet_id_dual_antenna_configuration,
	packet_id_gnss_configuration,
	packet_id_user_data,
	packet_id_gpio_input_configuration,
	packet_id_200_reserved,
	packet_id_201_reserved,
	packet_id_ip_dataports_configuration,
	packet_id_can_configuration,
	packet_id_device_name,
	end_configuration_packets
} packet_id_e;

/* start of system packets typedef structs */

typedef enum
{
	acknowledge_success,
	acknowledge_failure_crc,
	acknowledge_failure_length,
	acknowledge_failure_range,
	acknowledge_failure_flash,
	acknowledge_failure_not_ready,
	acknowledge_failure_unknown_packet,
	acknowledge_failure_mode
} acknowledge_result_e;

typedef struct
{
	uint8_t packet_id;
	uint16_t packet_crc;
	uint8_t acknowledge_result;
} acknowledge_packet_t;

typedef enum
{
	boot_mode_bootloader,
	boot_mode_main_program
} boot_mode_e;

typedef struct
{
	uint8_t boot_mode;
} boot_mode_packet_t;

typedef enum
{
	device_id_spatial = 1,
	device_id_orientus = 3,
	device_id_spatial_fog,
	device_id_spatial_dual,
	device_id_obdii_odometer = 10,
	device_id_orientus_v3,
	device_id_ilu,
	device_id_air_data_unit,
	device_id_spatial_fog_dual = 16,
	device_id_motus,
	device_id_gnss_compass = 19,
	device_id_certus = 26,
	device_id_aries,
	device_id_boreas_d90,
	device_id_boreas_90_fpga = 35,
	device_id_boreas_90_coil,
	device_id_boreas_70_coil = 40,
	device_id_boreas_d70,
	device_id_boreas_70_fpga,
	device_id_boreas_a90,
	device_id_boreas_a70,
	device_id_certus_mini_a = 49,
	device_id_certus_mini_n,
	device_id_certus_mini_d
} device_id_e;

typedef struct
{
	uint32_t software_version;
	uint32_t device_id;
	uint32_t hardware_revision;
	uint32_t serial_number[3];
} device_information_packet_t;

typedef enum
{
	file_transfer_response_completed_successfully,
	file_transfer_response_ready,
	file_transfer_response_index_mismatch,
	file_transfer_response_refused = 64,
	file_transfer_response_bad_metadata,
	file_transfer_response_timeout,
	file_transfer_response_retry_error,
	file_transfer_response_storage_error,
	file_transfer_response_data_invalid,
	file_transfer_response_packet_length_invalid,
	file_transfer_response_total_size_invalid,
	file_transfer_response_overflow_error,
	file_transfer_response_busy,
	file_transfer_response_cancelled,
	file_transfer_response_incorrect_device,
	file_transfer_response_file_not_found = 128,
	file_transfer_response_access_denied
} file_transfer_response_e;

typedef struct
{
	uint32_t unique_id;
	uint32_t data_index;
	uint8_t response_code;
} file_transfer_acknowledge_packet_t;

typedef enum
{
	data_encoding_binary,
	data_encoding_aes256
} data_encoding_e;

typedef enum
{
	file_transfer_metadata_none,
	file_transfer_metadata_extended_anpp,
	file_transfer_metadata_utf8_filename,
	file_transfer_metadata_an_firmware
} file_transfer_metadata_e;

typedef struct
{
	uint32_t unique_id;
	uint32_t data_index;
	uint32_t total_size;
	uint8_t data_encoding;
	uint8_t metadata_type;
	uint16_t metadata_length;
	uint8_t* metadata;
	uint8_t* packet_data;
} file_transfer_packet_t;

typedef enum
{
	passthrough_route_gpio,
	passthrough_route_auxiliary
} passthrough_route_e;

typedef struct
{
	uint8_t passthrough_route;
	uint8_t* passthrough_data;
} serial_port_passthrough_packet_t;

typedef enum
{
	device_subtype_boreas_acc_15g_v1,
	device_subtype_boreas_acc_15g_v2,
	device_subtype_boreas_acc_30g,
	device_subtype_certus = 0,
	device_subtype_certus_evo,
	device_subtype_certus_oem,
	device_subtype_certus_evo_oem,
	device_subtype_certus_mini = 0,
	device_subtype_certus_mini_oem = 2
} device_subtype_e;

typedef struct
{
	uint32_t software_version;
	uint32_t device_id;
	uint32_t hardware_revision;
	uint32_t serial_number[3];
	uint32_t device_subtype;
} extended_device_information_packet_t;

/* start of state packets typedef structs */

typedef enum
{
	gnss_fix_none,
	gnss_fix_2d,
	gnss_fix_3d,
	gnss_fix_sbas,
	gnss_fix_differential,
	gnss_fix_omnistar,
	gnss_fix_rtk_float,
	gnss_fix_rtk_fixed
} gnss_fix_type_e;

typedef struct
{
	union
	{
		uint16_t r;
		struct
		{
			uint16_t system_failure :1;
			uint16_t accelerometer_sensor_failure :1;
			uint16_t gyroscope_sensor_failure :1;
			uint16_t magnetometer_sensor_failure :1;
			uint16_t pressure_sensor_failure :1;
			uint16_t gnss_failure :1;
			uint16_t accelerometer_over_range :1;
			uint16_t gyroscope_over_range :1;
			uint16_t magnetometer_over_range :1;
			uint16_t pressure_over_range :1;
			uint16_t minimum_temperature_alarm :1;
			uint16_t maximum_temperature_alarm :1;
			uint16_t low_voltage_alarm :1;
			uint16_t high_voltage_alarm :1;
			uint16_t gnss_antenna_fault :1;
			uint16_t serial_port_overflow_alarm :1;
		} b;
	} system_status;
	union
	{
		uint16_t r;
		struct
		{
			uint16_t orientation_filter_initialised :1;
			uint16_t ins_filter_initialised :1;
			uint16_t heading_initialised :1;
			uint16_t utc_time_initialised :1;
			uint16_t gnss_fix_type :3;
			uint16_t event1_flag :1;
			uint16_t event2_flag :1;
			uint16_t internal_gnss_enabled :1;
			uint16_t dual_antenna_heading_active :1;
			uint16_t velocity_heading_enabled :1;
			uint16_t atmospheric_altitude_enabled :1;
			uint16_t external_position_active :1;
			uint16_t external_velocity_active :1;
			uint16_t external_heading_active :1;
		} b;
	} filter_status;
	uint32_t unix_time_seconds;
	uint32_t microseconds;
	double latitude;
	double longitude;
	double height;
	float velocity[3];
	float body_acceleration[3];
	float g_force;
	float orientation[3];
	float angular_velocity[3];
	float standard_deviation[3];
} system_state_packet_t;

typedef struct
{
	uint32_t unix_time_seconds;
	uint32_t microseconds;
} unix_time_packet_t;

typedef struct
{
	uint32_t microseconds;
	uint16_t year;
	uint16_t year_day;
	uint8_t month;
	uint8_t month_day;
	uint8_t week_day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} formatted_time_packet_t;

typedef struct
{
	union
	{
		uint16_t r;
		struct
		{
			uint16_t system_failure :1;
			uint16_t accelerometer_sensor_failure :1;
			uint16_t gyroscope_sensor_failure :1;
			uint16_t magnetometer_sensor_failure :1;
			uint16_t pressure_sensor_failure :1;
			uint16_t gnss_failure :1;
			uint16_t accelerometer_over_range :1;
			uint16_t gyroscope_over_range :1;
			uint16_t magnetometer_over_range :1;
			uint16_t pressure_over_range :1;
			uint16_t minimum_temperature_alarm :1;
			uint16_t maximum_temperature_alarm :1;
			uint16_t low_voltage_alarm :1;
			uint16_t high_voltage_alarm :1;
			uint16_t gnss_antenna_disconnected :1;
			uint16_t serial_port_overflow_alarm :1;
		} b;
	} system_status;
	union
	{
		uint16_t r;
		struct
		{
			uint16_t orientation_filter_initialised :1;
			uint16_t ins_filter_initialised :1;
			uint16_t heading_initialised :1;
			uint16_t utc_time_initialised :1;
			uint16_t gnss_fix_type :3;
			uint16_t event1_flag :1;
			uint16_t event2_flag :1;
			uint16_t internal_gnss_enabled :1;
			uint16_t dual_antenna_heading_active :1;
			uint16_t velocity_heading_enabled :1;
			uint16_t atmospheric_altitude_enabled :1;
			uint16_t external_position_active :1;
			uint16_t external_velocity_active :1;
			uint16_t external_heading_active :1;
		} b;
	} filter_status;
} status_packet_t;

typedef struct
{
	float standard_deviation[3];
} position_standard_deviation_packet_t;

typedef struct
{
	float standard_deviation[3];
} velocity_standard_deviation_packet_t;

typedef struct
{
	float standard_deviation[3];
} euler_orientation_standard_deviation_packet_t;

typedef struct
{
	float standard_deviation[4];
} quaternion_orientation_standard_deviation_packet_t;

typedef struct
{
	float accelerometers[3];
	float gyroscopes[3];
	float magnetometers[3];
	float imu_temperature;
	float pressure;
	float pressure_temperature;
} raw_sensors_packet_t;

typedef struct
{
	uint32_t unix_time_seconds;
	uint32_t microseconds;
	double position[3];
	float velocity[3];
	float position_standard_deviation[3];
	float tilt; /* This field will only be valid if an external dual antenna GNSS system is connected */
	float heading; /* This field will only be valid if an external dual antenna GNSS system is connected */
	float tilt_standard_deviation; /* This field will only be valid if an external dual antenna GNSS system is connected */
	float heading_standard_deviation; /* This field will only be valid if an external dual antenna GNSS system is connected */
	union
	{
		uint16_t r;
		struct
		{
			uint16_t fix_type :3;
			uint16_t velocity_valid :1;
			uint16_t time_valid :1;
			uint16_t external_gnss :1;
			uint16_t tilt_valid :1; /* This field will only be valid if an external dual antenna GNSS system is connected */
			uint16_t heading_valid :1; /* This field will only be valid if an external dual antenna GNSS system is connected */
		} b;
	} flags;
} raw_gnss_packet_t;

typedef struct
{
	float hdop;
	float vdop;
	uint8_t gps_satellites;
	uint8_t glonass_satellites;
	uint8_t beidou_satellites;
	uint8_t galileo_satellites;
	uint8_t sbas_satellites;
} satellites_packet_t;

typedef enum
{
	satellite_system_unknown,
	satellite_system_gps,
	satellite_system_glonass,
	satellite_system_beidou,
	satellite_system_galileo,
	satellite_system_sbas,
	satellite_system_qzss,
	satellite_system_starfire,
	satellite_system_omnistar,
	satellite_system_imes,
	satellite_system_irnss
} satellite_system_e;

typedef struct
{
	double position[3];
} geodetic_position_packet_t;

typedef struct
{
	double position[3];
} ecef_position_packet_t;

typedef struct
{
	double position[3];
	uint8_t zone_number;
	char zone_char;
} utm_position_packet_t;

typedef struct
{
	float velocity[3];
} ned_velocity_packet_t;

typedef struct
{
	float velocity[3];
} body_velocity_packet_t;

typedef struct
{
	float acceleration[3];
} acceleration_packet_t;

typedef struct
{
	float acceleration[3];
	float g_force;
} body_acceleration_packet_t;

typedef struct
{
	float orientation[3];
} euler_orientation_packet_t;

typedef struct
{
	float orientation[4];
} quaternion_orientation_packet_t;

typedef struct
{
	float orientation[3][3];
} dcm_orientation_packet_t;

typedef struct
{
	float angular_velocity[3];
} angular_velocity_packet_t;

typedef struct
{
	float angular_acceleration[3];
} angular_acceleration_packet_t;

typedef struct
{
	double position[3];
	float velocity[3];
	float position_standard_deviation[3];
	float velocity_standard_deviation[3];
} external_position_velocity_packet_t;

typedef struct
{
	double position[3];
	float standard_deviation[3];
} external_position_packet_t;

typedef struct
{
	float velocity[3];
	float standard_deviation[3];
} external_velocity_packet_t;

typedef struct
{
	float velocity[3];
	float standard_deviation;
} external_body_velocity_packet_t;

typedef struct
{
	float heading;
	float standard_deviation;
} external_heading_packet_t;

typedef struct
{
	uint32_t seconds;
	uint32_t microseconds;
} running_time_packet_t;

typedef struct
{
	float magnetic_field[3];
} local_magnetics_packet_t;

typedef struct
{
	int32_t pulse_count;
	float distance;
	float speed;
	float slip;
	uint8_t active;
} odometer_state_packet_t;

typedef struct
{
	uint32_t unix_time_seconds;
	uint32_t microseconds;
} external_time_packet_t;

typedef struct
{
	float depth;
	float standard_deviation;
} external_depth_packet_t;

typedef struct
{
	float geoid_height;
} geoid_height_packet_t;

typedef struct
{
	uint8_t* packet_data;
} rtcm_corrections_packet_t;

typedef struct
{
	float wind_velocity[2];
	float wind_standard_deviation;
} wind_packet_t;

typedef struct
{
	float heave_point_1;
	float heave_point_2;
	float heave_point_3;
	float heave_point_4;
} heave_packet_t;

typedef enum
{
	satellite_frequency_unknown,
	satellite_frequency_l1_ca,
	satellite_frequency_l1_c,
	satellite_frequency_l1_p,
	satellite_frequency_l1_m,
	satellite_frequency_l2_c,
	satellite_frequency_l2_p,
	satellite_frequency_l2_m,
	satellite_frequency_l5,
	satellite_frequency_l3
} satellite_frequencies_e;

typedef enum
{
	tracking_status_carrier_phase_valid,
	tracking_status_carrier_phase_cycle_slip_detected,
	tracking_status_carrier_phase_half_cycle_ambiguity,
	tracking_status_pseudo_range_valid,
	tracking_status_doppler_valid,
	tracking_status_snr_valid
} tracking_status_e;

typedef struct
{
	uint8_t satellite_frequency;
	union
	{
		uint8_t r;
		struct
		{
			uint8_t carrier_phase_valid :1;
			uint8_t carrier_phase_cycle_slip_detected :1;
			uint8_t carrier_phase_half_cycle_ambiguity :1;
			uint8_t pseudo_range_valid :1;
			uint8_t doppler_valid :1;
			uint8_t signal_to_noise_ratio_valid :1;
		} b;
	} tracking_status;
	double carrier_phase;
	double pseudo_range;
	float doppler_frequency;
	float signal_to_noise_ratio;
} raw_frequency_t;

typedef struct
{
	uint8_t satellite_system;
	uint8_t prn;
	uint8_t elevation;
	uint16_t azimuth;
	uint8_t number_of_frequencies;
	raw_frequency_t* frequency;
} raw_satellite_t;

typedef struct
{
	uint32_t unix_time_seconds;
	uint32_t nanoseconds;
	int32_t receiver_clock_offset;
	uint8_t receiver_number;
	uint8_t packet_number;
	uint8_t total_packets;
	uint8_t number_of_satellites;
	raw_satellite_t* satellite;
} raw_satellite_data_packet_t;

typedef struct
{
	float delay;
	float speed;
	float distance_travelled;
	union
	{
		uint8_t r;
		struct
		{
			uint8_t reverse_detection_supported :1;
		} b;
	} flags;
} odometer_packet_t;

typedef struct
{
	float barometric_altitude_delay;
	float airspeed_delay;
	float barometric_altitude;
	float airspeed;
	float barometric_altitude_standard_deviation;
	float airspeed_standard_deviation;
	union
	{
		uint8_t r;
		struct
		{
			uint8_t barometric_altitude_valid :1;
			uint8_t airspeed_valid :1;
			uint8_t barometric_altitude_reference_reset :1;
		} b;
	} flags;
} external_air_data_packet_t;

typedef enum
{
	gnss_manufacturer_unknown,
	gnss_manufacturer_trimble,
	gnss_manufacturer_ublox,
	gnss_manufacturer_advanced_navigation
} gnss_manufacturers_e;

typedef enum
{
	gnss_receiver_model_unknown,
	gnss_receiver_model_trimble_bd920,
	gnss_receiver_model_trimble_bd930,
	gnss_receiver_model_trimble_bd982,
	gnss_receiver_model_trimble_mbone,
	gnss_receiver_model_trimble_mbtwo,
	gnss_receiver_model_trimble_bd940,
	gnss_receiver_model_trimble_bd992,
	gnss_receiver_model_ublox_m8 = 1,
	gnss_receiver_model_ublox_m8t,
	gnss_receiver_model_ublox_m8p,
	gnss_receiver_model_ublox_zedf9p,
	gnss_receiver_model_ublox_neof9p,
	gnss_receiver_model_advanced_navigation_aries = 1,
	gnss_receiver_model_advanced_navigation_aries_gc2
} gnss_receiver_models_e;

typedef enum
{
	omnistar_engine_mode_inactive,
	omnistar_engine_mode_hp,
	omnistar_engine_mode_xp,
	omnistar_engine_mode_g2,
	omnistar_engine_mode_hp_g2,
	omnistar_engine_mode_hp_xp
} omnistar_engine_modes_e;

typedef struct
{
	uint8_t gnss_manufacturer_id;
	uint8_t gnss_receiver_model;
	char serial_number[11];
	uint32_t firmware_version;
	uint32_t software_license[3];
	uint32_t omnistar_serial_number;
	uint32_t omnistar_subscription_start_unix_time;
	uint32_t omnistar_subscription_expiry_unix_time;
	uint8_t omnistar_engine_mode;
	uint8_t rtk_accuracy;
} gnss_receiver_information_packet_t;

typedef struct
{
	float current_angle;
	uint32_t reserved;
} gimbal_state_packet_t;

typedef struct
{
	float virtual_odometer_distance;
	float slip_angle;
	float velocity_x;
	float velocity_y;
	float distance_standard_deviation;
} automotive_packet_t;

typedef struct
{
	float duration;
} zero_angular_velocity_packet_t;

typedef struct
{
	uint8_t satellite_system;
	uint8_t number;
	union
	{
		uint8_t r;
		struct
		{
			uint8_t l1_ca :1;
			uint8_t l1_c :1;
			uint8_t l1_p :1;
			uint8_t l1_m :1;
			uint8_t l2_c :1;
			uint8_t l2_p :1;
			uint8_t l2_m :1;
			uint8_t l5 :1;
		} b;
	} frequencies;
	uint8_t elevation;
	uint16_t azimuth;
	uint8_t snr1;
	uint8_t snr2;
	union
	{
		uint8_t r;
		struct
		{
			uint8_t visible_by_receiver_1 :1;
			uint8_t visible_by_receiver_2 :1;
			uint8_t used_in_primary_position_solution :1;
			uint8_t used_in_moving_baseline_solution :1;
		} b;
	} flags;
} extended_satellite_t;

typedef struct
{
	uint8_t total_number_of_packets;
	uint8_t packet_number;
	extended_satellite_t extended_satellites[MAXIMUM_EXTENDED_SATELLITES];
} extended_satellites_packet_t;

typedef struct
{
	float accelerometer_temperature[3];
	float gyroscope_temperature[3];
	float magnetometer_temperature;
	float pressure_sensor_temperature;
} sensor_temperature_packet_t;

typedef enum
{
	automatic_magnetometer_calibration_method_disabled,
	automatic_magnetometer_calibration_method_gnss_aided,
	automatic_magnetometer_calibration_method_online
} automatic_magnetometer_calibration_method_e;

typedef struct
{
	uint8_t automagcal_method;
	union
	{
		uint8_t r;
		struct
		{
			uint8_t ready							:1;
			uint8_t coarse_complete					:1;
			uint8_t existing_calibration			:1;
			uint8_t awaiting_filters 				:1;
			uint8_t manual_calibration_in_progress 	:1;
			uint8_t invalid_vehicle_type			:1;
			uint8_t magnetic_heading_disabled 		:1;
			uint8_t reserved_flag					:1;
		} b;
	} flags;
	float convergence;
	float scale_factor[3];
	float soft_iron[3];
	float hard_iron[3];
	float scale_factor_standard_deviation[3];
	float soft_iron_standard_deviation[3];
	float hard_iron_standard_deviation[3];
} automatic_magnetometer_calibration_status_packet_t;

/* start of configuration packets typedef structs */

typedef struct
{
	uint8_t permanent;
	uint8_t utc_synchronisation;
	uint16_t packet_timer_period;
} packet_timer_period_packet_t;

typedef struct
{
	uint8_t packet_id;
	uint32_t period;
} packet_period_t;

typedef struct
{
	uint8_t permanent;
	uint8_t clear_existing_packets;
	packet_period_t packet_periods[MAXIMUM_PACKET_PERIODS];
} packet_periods_packet_t;

typedef struct
{
	uint8_t permanent;
	uint32_t primary_baud_rate;
	uint32_t gpio_1_2_baud_rate;
	uint32_t auxiliary_baud_rate;
	uint32_t reserved;
} baud_rates_packet_t;

typedef enum
{
	accelerometer_range_2g,
	accelerometer_range_4g,
	accelerometer_range_16g
} accelerometer_range_e;

typedef enum
{
	gyroscope_range_250dps,
	gyroscope_range_500dps,
	gyroscope_range_2000dps
} gyroscope_range_e;

typedef enum
{
	magnetometer_range_2g,
	magnetometer_range_4g,
	magnetometer_range_8g
} magnetometer_range_e;

typedef struct
{
	uint8_t permanent;
	uint8_t accelerometers_range;
	uint8_t gyroscopes_range;
	uint8_t magnetometers_range;
} sensor_ranges_packet_t;

typedef struct
{
	uint8_t permanent;
	float alignment_dcm[3][3];
	float gnss_antenna_offset[3];
	float odometer_offset[3];
	float external_data_offset[3];
} installation_alignment_packet_t;

typedef enum
{
	vehicle_type_unlimited,
	vehicle_type_bicycle,
	vehicle_type_car,
	vehicle_type_hovercraft,
	vehicle_type_submarine,
	vehicle_type_3d_underwater,
	vehicle_type_fixed_wing_plane,
	vehicle_type_3d_aircraft,
	vehicle_type_human,
	vehicle_type_small_boat,
	vehicle_type_ship,
	vehicle_type_stationary,
	vehicle_type_stunt_plane,
	vehicle_type_race_car,
	vehicle_type_train
} vehicle_type_e;

typedef struct
{
	uint8_t permanent;
	uint8_t vehicle_type;
	uint8_t internal_gnss_enabled;
	uint8_t reserved3;
	uint8_t atmospheric_altitude_enabled;
	uint8_t velocity_heading_enabled;
	uint8_t reversing_detection_enabled;
	uint8_t motion_analysis_enabled;
	uint8_t reserved8;
	uint8_t disable_dual_antenna_heading;
	uint8_t disable_navigation;
} filter_options_packet_t;

typedef enum
{
	gpio_function_inactive,
	gpio_function_1pps_output,
	gpio_function_gnss_fix_output,
	gpio_function_odometer_input,
	gpio_function_stationary_input,
	gpio_function_pitot_tube_input,
	gpio_function_nmea_input,
	gpio_function_nmea_output,
	gpio_function_novatel_gnss_input,
	gpio_function_topcon_gnss_input,
	gpio_function_motec_output,
	gpio_function_anpp_input,
	gpio_function_anpp_output,
	gpio_function_disable_magnetometers,
	gpio_function_disable_gnss,
	gpio_function_disable_pressure,
	gpio_function_set_zero_alignment,
	gpio_function_packet_trigger_system_state,
	gpio_function_packet_trigger_raw_sensors,
	gpio_function_rtcm_corrections_input,
	gpio_function_trimble_gnss_input,
	gpio_function_ublox_gnss_input,
	gpio_function_hemisphere_gnss_input,
	gpio_function_teledyne_dvl_input,
	gpio_function_tritech_usbl_input,
	gpio_function_linkquest_dvl_input,
	gpio_function_pressure_depth_sensor,
	gpio_function_left_wheel_speed_sensor,
	gpio_function_right_wheel_speed_sensor,
	gpio_function_pps_input,
	gpio_function_wheel_speed_sensor,
	gpio_function_wheel_encoder_phase_a,
	gpio_function_wheel_encoder_phase_b,
	gpio_function_event1_input,
	gpio_function_event2_input,
	gpio_function_linkquest_usbl_input,
	gpio_function_ixblue_input,
	gpio_function_sonardyne_input,
	gpio_function_gnss_receiver_passthrough,
	gpio_function_tss1_output,
	gpio_function_simrad_1000_output,
	gpio_function_simrad_3000_output,
	gpio_function_serial_port_passthrough,
	gpio_function_gimbal_encoder_phase_a,
	gpio_function_gimbal_encoder_phase_b,
	gpio_function_odometer_direction_forward_low,
	gpio_function_odometer_direction_forward_high,
	gpio_function_virtual_odometer_output,
	gpio_function_novatel_mark_output,
	gpio_function_gpio2_8mhz_output,
	gpio_function_ng206_output,
	gpio_function_nortek_dvl_input,
	gpio_function_moving_base_corrections_output,
	gpio_function_reverse_alignment_forward_low,
	gpio_function_reverse_alignment_forward_high,
	gpio_function_zero_angular_velocity_input,
	gpio_function_mavlink_output,
	gpio_function_gnss_receiver2_passthrough,
	gpio_function_water_linked_dvl_input = 59,
	gpio_function_nortek_nucleus_dvl_input = 63,
	gpio_function_nortek_nucleus_dvl_output
} gpio_function_e;

typedef enum
{
	gpio_index_gpio1,
	gpio_index_gpio2,
	gpio_index_auxiliary_tx,
	gpio_index_auxiliary_rx
} gpio_index_e;


typedef struct
{
	uint8_t permanent;
	uint8_t gpio_function[4];
} gpio_configuration_packet_t;

typedef struct
{
	uint8_t permanent;
	float hard_iron[3];
	float soft_iron[3][3];
} magnetic_calibration_values_packet_t;

typedef enum
{
	magnetic_calibration_action_cancel,
	magnetic_calibration_action_stabilise,
	magnetic_calibration_action_start_2d,
	magnetic_calibration_action_start_3d
} magnetic_calibration_action_e;

typedef struct
{
	uint8_t magnetic_calibration_action;
} magnetic_calibration_configuration_packet_t;

typedef enum
{
	magnetic_calibration_status_not_completed,
	magnetic_calibration_status_completed_2d,
	magnetic_calibration_status_completed_3d,
	magnetic_calibration_status_completed_user,
	magnetic_calibration_status_stabilizing,
	magnetic_calibration_status_in_progress_2d,
	magnetic_calibration_status_in_progress_3d,
	magnetic_calibration_status_error_excessive_roll,
	magnetic_calibration_status_error_excessive_pitch,
	magnetic_calibration_status_error_overrange_event,
	magnetic_calibration_status_error_timeout,
	magnetic_calibration_status_error_system,
	magnetic_calibration_status_error_interference
} magnetic_calibration_status_e;

typedef struct
{
	uint8_t magnetic_calibration_status;
	uint8_t magnetic_calibration_progress_percentage;
	uint8_t local_magnetic_error_percentage;
} magnetic_calibration_status_packet_t;

typedef struct
{
	uint8_t permanent;
	uint8_t automatic_calibration;
	float pulse_length;
} odometer_configuration_packet_t;

typedef struct
{
	uint8_t permanent;
} zero_alignment_packet_t;

typedef struct
{
	uint8_t permanent;
	float heave_point_1_offset[3];
	float heave_point_2_offset[3];
	float heave_point_3_offset[3];
	float heave_point_4_offset[3];
} heave_offset_packet_t;

typedef enum
{
	gpio_rate_disabled,
	gpio_rate_0o1hz,
	gpio_rate_0o2hz,
	gpio_rate_0o5hz,
	gpio_rate_1hz,
	gpio_rate_2hz,
	gpio_rate_5hz,
	gpio_rate_10hz,
	gpio_rate_25hz,
	gpio_rate_50hz,
	gpio_rate_8hz
} gpio_rate_e;

typedef enum
{
	nmea_fix_behaviour_normal,
	nmea_fix_behaviour_always_3d
} nmea_fix_behaviour_e;

typedef union
{
	uint16_t r;
	struct
	{
		uint16_t gpio1_rate :4;
		uint16_t auxiliary_rate :4;
	} b;
} gpio_output_rate_u;
typedef struct
{
	uint8_t permanent;
	uint8_t nmea_fix_behaviour;
	gpio_output_rate_u gpzda_rate;
	gpio_output_rate_u gpgga_rate;
	gpio_output_rate_u gpvtg_rate;
	gpio_output_rate_u gprmc_rate;
	gpio_output_rate_u gphdt_rate;
	gpio_output_rate_u gpgll_rate;
	gpio_output_rate_u pashr_rate;
	gpio_output_rate_u tss1_rate;
	gpio_output_rate_u simrad_rate;
	gpio_output_rate_u gprot_rate;
	gpio_output_rate_u gphev_rate;
	gpio_output_rate_u gpgsv_rate;
	gpio_output_rate_u pfecatt_rate;
	gpio_output_rate_u pfechve_rate;
	gpio_output_rate_u gpgst_rate;
} gpio_output_configuration_packet_t;

typedef enum
{
	dual_antenna_offset_type_manual,
	dual_antenna_offset_type_automatic
} dual_antenna_offset_type_e;

typedef enum
{
	dual_antenna_automatic_offset_primary_front_secondary_rear,
	dual_antenna_automatic_offset_primary_rear_secondary_front,
	dual_antenna_automatic_offset_primary_right_secondary_left,
	dual_antenna_automatic_offset_primary_left_secondary_right
} dual_antenna_automatic_offset_orientation_e;

typedef struct
{
	uint8_t permanent;
	union
	{
		uint16_t r;
		struct
		{
			uint16_t automatic_offset_enabled :1;
		} b;
	} options;
	uint8_t automatic_offset_orientation;
	uint8_t reserved;
	float manual_offset[3];
} dual_antenna_configuration_packet_t;

typedef enum
{
	lband_mode_disabled,
	lband_mode_omnistar_auto = 2,
	lband_mode_omnistar_hp,
	lband_mode_omnistar_xp,
	lband_mode_omnistar_vbs,
	lband_mode_omnistar_g2 = 8,
	lband_mode_omnistar_hp_g2,
	lband_mode_omnistar_hp_xp,
	lband_mode_trimble_rtx,
	lband_mode_omnistar_l1_only = 13,
	lband_mode_omnistar_g4_only,
	lband_mode_omnistar_g2plus_only,
	lband_mode_omnistar_g4plus_only
} lband_mode_e;

typedef enum
{
	lband_satellite_id_iosat = 2,
	lband_satellite_id_ior,
	lband_satellite_id_sasat,
	lband_satellite_id_por,
	lband_satellite_id_easat,
	lband_satellite_id_irsat,
	lband_satellite_id_aoret,
	lband_satellite_id_esat,
	lband_satellite_id_aore,
	lband_satellite_id_aorw,
	lband_satellite_id_asat,
	lband_satellite_id_arsat,
	lband_satellite_id_ausat,
	lband_satellite_id_amsat = 16,
	lband_satellite_id_ocsat = 18,
	lband_satellite_id_rtxal,
	lband_satellite_id_ersat,
	lband_satellite_id_rtxeu,
	lband_satellite_id_rtxen,
	lband_satellite_id_rtxna,
	lband_satellite_id_rtxaf,
	lband_satellite_id_rtxir,
	lband_satellite_id_rtxme,
	lband_satellite_id_rtxpa,
	lband_satellite_id_rtxap,
	lband_satellite_id_rtxsa = 30,
	lband_satellite_id_rtxcn,
	lband_satellite_id_custom = 100,
	lband_satellite_id_auto = 110
} lband_satellite_id_e;

typedef struct
{
	uint8_t permanent;
	union
	{
		uint64_t r;
		struct
		{
			uint64_t gps_l1ca :1;
			uint64_t gps_l1c :1;
			uint64_t gps_l1p :1;
			uint64_t gps_l2c :1;
			uint64_t gps_l2p :1;
			uint64_t gps_l2m :1;
			uint64_t gps_l5 :1;
			uint64_t glonass_g1ca :1;
			uint64_t glonass_g1p :1;
			uint64_t glonass_l1oc :1;
			uint64_t glonass_l1sc :1;
			uint64_t glonass_g2ca :1;
			uint64_t glonass_g2p :1;
			uint64_t glonass_l2oc :1;
			uint64_t glonass_l2sc :1;
			uint64_t glonass_l3oc :1;
			uint64_t glonass_l3sc :1;
			uint64_t beidou_b1 :1;
			uint64_t beidou_b2 :1;
			uint64_t beidou_b3 :1;
			uint64_t galileo_e1 :1;
			uint64_t galileo_e5a :1;
			uint64_t galileo_e5b :1;
			uint64_t galileo_e5ab :1;
			uint64_t galileo_e6 :1;
			uint64_t qzss_l1ca :1;
			uint64_t qzss_l1saif :1;
			uint64_t qzss_l1c :1;
			uint64_t qzss_l2c :1;
			uint64_t qzss_l5 :1;
			uint64_t qzss_lex :1;
			uint64_t sbas_l1ca :1;
		} b;
	} gnss_frequencies;
	float pdop;
	float tdop;
	uint8_t elevation_mask;
	uint8_t snr_mask;
	uint8_t sbas_corrections_enabled;
	uint8_t lband_mode;
	uint32_t lband_frequency;
	uint32_t lband_baud;
	uint32_t primary_antenna_type;
	uint32_t secondary_antenna_type;
	uint8_t lband_satellite_id;
} gnss_configuration_packet_t;

typedef struct
{
	uint8_t user_data[64];
} user_data_packet_t;

typedef struct
{
	uint8_t permanent;
	float gimbal_radians_per_encoder_tick;
} gpio_input_configuration_packet_t;

typedef enum
{
	can_protocol_canopen
} can_protocol_e;

typedef struct
{
	uint8_t permanent;
	uint8_t enabled;
	uint32_t baud_rate;
	uint8_t can_protocol;
} can_configuration_packet_t;


int decode_acknowledge_packet(acknowledge_packet_t* acknowledge_packet, an_packet_t* an_packet);
an_packet_t* encode_request_packet(uint8_t requested_packet_id);
int decode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet, an_packet_t* an_packet);
an_packet_t* encode_boot_mode_packet(boot_mode_packet_t* boot_mode_packet);
int decode_device_information_packet(device_information_packet_t* device_information_packet, an_packet_t* an_packet);
an_packet_t* encode_restore_factory_settings_packet();
an_packet_t* encode_reset_packet();
int decode_file_transfer_acknowledge_packet(file_transfer_acknowledge_packet_t* file_transfer_acknowledge_packet, an_packet_t* an_packet);
an_packet_t* encode_file_transfer_packet(file_transfer_packet_t* file_transfer_packet, int data_size);
int decode_serial_port_passthrough_packet(serial_port_passthrough_packet_t* serial_port_passthrough_packet, an_packet_t* an_packet);
an_packet_t* encode_serial_port_passthrough_packet(serial_port_passthrough_packet_t* serial_port_passthrough_packet, int data_size);
int decode_extended_device_information_packet(extended_device_information_packet_t* extended_device_information_packet, an_packet_t* an_packet);
int decode_system_state_packet(system_state_packet_t* system_state_packet, an_packet_t* an_packet);
int decode_unix_time_packet(unix_time_packet_t* unix_time_packet, an_packet_t* an_packet);
int decode_formatted_time_packet(formatted_time_packet_t* formatted_time_packet, an_packet_t* an_packet);
int decode_status_packet(status_packet_t* status_packet, an_packet_t* an_packet);
int decode_position_standard_deviation_packet(position_standard_deviation_packet_t* position_standard_deviation_packet, an_packet_t* an_packet);
int decode_velocity_standard_deviation_packet(velocity_standard_deviation_packet_t* velocity_standard_deviation_packet, an_packet_t* an_packet);
int decode_euler_orientation_standard_deviation_packet(euler_orientation_standard_deviation_packet_t* euler_orientation_standard_deviation, an_packet_t* an_packet);
int decode_quaternion_orientation_standard_deviation_packet(quaternion_orientation_standard_deviation_packet_t* quaternion_orientation_standard_deviation_packet, an_packet_t* an_packet);
int decode_raw_sensors_packet(raw_sensors_packet_t* raw_sensors_packet, an_packet_t* an_packet);
int decode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet, an_packet_t* an_packet);
an_packet_t* encode_raw_gnss_packet(raw_gnss_packet_t* raw_gnss_packet);
int decode_satellites_packet(satellites_packet_t* satellites_packet, an_packet_t* an_packet);
int decode_geodetic_position_packet(geodetic_position_packet_t* geodetic_position_packet, an_packet_t* an_packet);
int decode_ecef_position_packet(ecef_position_packet_t* ecef_position_packet, an_packet_t* an_packet);
int decode_utm_position_packet(utm_position_packet_t* utm_position_packet, an_packet_t* an_packet);
int decode_ned_velocity_packet(ned_velocity_packet_t* ned_velocity_packet, an_packet_t* an_packet);
int decode_body_velocity_packet(body_velocity_packet_t* body_velocity_packet, an_packet_t* an_packet);
int decode_acceleration_packet(acceleration_packet_t* acceleration, an_packet_t* an_packet);
int decode_body_acceleration_packet(body_acceleration_packet_t* body_acceleration, an_packet_t* an_packet);
int decode_euler_orientation_packet(euler_orientation_packet_t* euler_orientation_packet, an_packet_t* an_packet);
int decode_quaternion_orientation_packet(quaternion_orientation_packet_t* quaternion_orientation_packet, an_packet_t* an_packet);
int decode_dcm_orientation_packet(dcm_orientation_packet_t* dcm_orientation_packet, an_packet_t* an_packet);
int decode_angular_velocity_packet(angular_velocity_packet_t* angular_velocity_packet, an_packet_t* an_packet);
int decode_angular_acceleration_packet(angular_acceleration_packet_t* angular_acceleration_packet, an_packet_t* an_packet);
int decode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet, an_packet_t* an_packet);
an_packet_t* encode_external_position_velocity_packet(external_position_velocity_packet_t* external_position_velocity_packet);
int decode_external_position_packet(external_position_packet_t* external_position_packet, an_packet_t* an_packet);
an_packet_t* encode_external_position_packet(external_position_packet_t* external_position_packet);
int decode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet, an_packet_t* an_packet);
an_packet_t* encode_external_velocity_packet(external_velocity_packet_t* external_velocity_packet);
int decode_external_body_velocity_packet(external_body_velocity_packet_t* external_body_velocity_packet, an_packet_t* an_packet);
an_packet_t* encode_external_body_velocity_packet(external_body_velocity_packet_t* external_body_velocity_packet);
int decode_external_heading_packet(external_heading_packet_t* external_heading_packet, an_packet_t* an_packet);
an_packet_t* encode_external_heading_packet(external_heading_packet_t* external_heading_packet);
int decode_running_time_packet(running_time_packet_t* running_time_packet, an_packet_t* an_packet);
int decode_local_magnetics_packet(local_magnetics_packet_t* local_magnetics_packet, an_packet_t* an_packet);
int decode_odometer_state_packet(odometer_state_packet_t* odometer_state_packet, an_packet_t* an_packet);
int decode_external_time_packet(external_time_packet_t* external_time_packet, an_packet_t* an_packet);
an_packet_t* encode_external_time_packet(external_time_packet_t* external_time_packet);
int decode_external_depth_packet(external_depth_packet_t* external_depth_packet, an_packet_t* an_packet);
an_packet_t* encode_external_depth_packet(external_depth_packet_t* external_depth_packet);
int decode_geoid_height_packet(geoid_height_packet_t* geoid_height_packet, an_packet_t* an_packet);
an_packet_t* encode_rtcm_corrections_packet(rtcm_corrections_packet_t* rtcm_corrections_packet, uint8_t data_size);
int decode_wind_packet(wind_packet_t* wind_packet, an_packet_t* an_packet);
an_packet_t* encode_wind_packet(wind_packet_t* wind_packet);
int decode_heave_packet(heave_packet_t* heave_packet, an_packet_t* an_packet);
int decode_raw_satellite_data_packet(raw_satellite_data_packet_t* raw_satellite_data_packet, an_packet_t* an_packet);
int decode_external_odometer_packet(odometer_packet_t* external_odometer_packet, an_packet_t* an_packet);
an_packet_t* encode_external_odometer_packet(odometer_packet_t* external_odometer_packet);
int decode_external_air_data_packet(external_air_data_packet_t* external_air_data_packet, an_packet_t* an_packet);
an_packet_t* encode_external_air_data_packet(external_air_data_packet_t* external_air_data_packet);
int decode_gnss_information_packet(gnss_receiver_information_packet_t* gnss_information_packet, an_packet_t* an_packet);
int decode_gimbal_state_packet(gimbal_state_packet_t* gimbal_state_packet, an_packet_t* an_packet);
an_packet_t* encode_gimbal_state_packet(gimbal_state_packet_t* gimbal_state_packet);
int decode_automotive_packet(automotive_packet_t* automotive_packet, an_packet_t* an_packet);
an_packet_t* encode_zero_angular_velocity_packet(zero_angular_velocity_packet_t* zero_angular_velocity_packet_t);
int decode_extended_satellites_packet(extended_satellites_packet_t* extended_satellites_packet, an_packet_t* an_packet);
int decode_sensor_temperatures_packet(sensor_temperature_packet_t* sensor_temperature_packet, an_packet_t* an_packet);
int decode_automatic_magnetometer_calibration_status_packet(automatic_magnetometer_calibration_status_packet_t* automatic_magnetometer_calibration_status_packet, an_packet_t* an_packet);
int decode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet, an_packet_t* an_packet);
an_packet_t* encode_packet_timer_period_packet(packet_timer_period_packet_t* packet_timer_period_packet);
int decode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet, an_packet_t* an_packet);
an_packet_t* encode_packet_periods_packet(packet_periods_packet_t* packet_periods_packet);
int decode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet, an_packet_t* an_packet);
an_packet_t* encode_baud_rates_packet(baud_rates_packet_t* baud_rates_packet);
int decode_sensor_ranges_packet(sensor_ranges_packet_t* sensor_ranges_packet, an_packet_t* an_packet);
an_packet_t* encode_sensor_ranges_packet(sensor_ranges_packet_t* sensor_ranges_packet);
int decode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet, an_packet_t* an_packet);
an_packet_t* encode_installation_alignment_packet(installation_alignment_packet_t* installation_alignment_packet);
int decode_filter_options_packet(filter_options_packet_t* filter_options_packet, an_packet_t* an_packet);
an_packet_t* encode_filter_options_packet(filter_options_packet_t* filter_options_packet);
int decode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_gpio_configuration_packet(gpio_configuration_packet_t* gpio_configuration_packet);
int decode_magnetic_calibration_values_packet(magnetic_calibration_values_packet_t* magnetic_calibration_values_packet, an_packet_t* an_packet);
an_packet_t* encode_magnetic_calibration_values_packet(magnetic_calibration_values_packet_t* magnetic_calibration_values_packet);
an_packet_t* encode_magnetic_calibration_configuration_packet(magnetic_calibration_configuration_packet_t* magnetic_calibration_configuration_packet);
int decode_magnetic_calibration_status_packet(magnetic_calibration_status_packet_t* magnetic_calibration_status_packet, an_packet_t* an_packet);
int decode_odometer_configuration_packet(odometer_configuration_packet_t* odometer_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_odometer_configuration_packet(odometer_configuration_packet_t* odometer_configuration_packet);
an_packet_t* encode_zero_alignment_packet(zero_alignment_packet_t* zero_alignment_packet);
int decode_heave_offset_packet(heave_offset_packet_t* heave_offset_packet, an_packet_t* an_packet);
an_packet_t* encode_heave_offset_packet(heave_offset_packet_t* heave_offset_packet);
int decode_gpio_output_configuration_packet(gpio_output_configuration_packet_t* gpio_output_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_gpio_output_configuration_packet(gpio_output_configuration_packet_t* gpio_output_configuration_packet);
int decode_dual_antenna_configuration_packet(dual_antenna_configuration_packet_t* dual_antenna_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_dual_antenna_configuration_packet(dual_antenna_configuration_packet_t* dual_antenna_configuration_packet);
int decode_gnss_configuration_packet(gnss_configuration_packet_t* gnss_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_gnss_configuration_packet(gnss_configuration_packet_t* gnss_configuration_packet);
int decode_user_data_packet(user_data_packet_t* user_data_packet, an_packet_t* an_packet);
an_packet_t* encode_user_data_packet(user_data_packet_t* user_data_packet);
int decode_gpio_input_configuration_packet(gpio_input_configuration_packet_t* gpio_input_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_gpio_input_configuration_packet(gpio_input_configuration_packet_t* gpio_input_configuration_packet);
int decode_can_configuration_packet(can_configuration_packet_t* can_configuration_packet, an_packet_t* an_packet);
an_packet_t* encode_can_configuration_packet(can_configuration_packet_t* can_configuration_packet);

#ifdef __cplusplus
}
#endif

#endif

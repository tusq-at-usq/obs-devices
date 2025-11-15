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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <jansson.h>
#include <locale.h>
#include <math.h>
#include <time.h>

#include "rs232/rs232.h"
#include <unistd.h>

#include "an_packet_protocol.h"
#include "ins_packets.h"

#define RADIANS_TO_DEGREES (180.0 / M_PI)

static unsigned char request_all_configuration[] = {
    0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA,
    0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7};
int comPortIndex = -1;
int socket_fd = -1;
int logFlag = 0;

int transmit(const unsigned char *data, int length);
int receive(unsigned char *data, int length);
int an_packet_transmit(an_packet_t *an_packet);
void set_filter_options();

int main(int argc, char *argv[]) {
  an_decoder_t an_decoder;
  an_packet_t *an_packet;

  system_state_packet_t system_state_packet;
  raw_sensors_packet_t raw_sensors_packet;

  FILE *anpp_log_file;

  char filename[64];
  time_t rawtime;
  struct tm *timeinfo;
  int write_counter = 0;
  int bytes_received;

  if (argc != 4) {
    printf("Incorrect number of arguments supplied\n");
    printf("Usage - %s com_port baud_rate log_flag\n", argv[0]);
  }

  logFlag = atoi(argv[3]);
  if ((logFlag < 0) || (logFlag > 1)) {
    printf("Log flag must be 0 or 1\n");
    exit(EXIT_FAILURE);
  }

  /* Find the serial port */
  comEnumerate();
  comPortIndex = comFindPort(argv[1]);
  if (comPortIndex == -1) {
    printf("Serial port not available\n");
    exit(EXIT_FAILURE);
  }
  /* Open the serial port */
  if (comOpen(comPortIndex, atoi(argv[2])) == 0) {
    printf("Could not open serial port\n");
    exit(EXIT_FAILURE);
  }

  if (logFlag == 1) {
    printf("Logging enabled\n");
    /* Create a Log file */
    rawtime = time(NULL);
    timeinfo = localtime(&rawtime);
    sprintf(filename, "ANLog_%02d-%02d-%02d_%02d-%02d-%02d.anpp",
            timeinfo->tm_year - 100, timeinfo->tm_mon + 1, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    anpp_log_file = fopen(filename, "wb");
  } else {
    printf("Logging disabled\n");
  }

  /* Request all the configuration and the device information from the unit */
  transmit(request_all_configuration, sizeof(request_all_configuration));

  an_decoder_initialise(&an_decoder);

  /* Set stdout to be line buffered */
  setvbuf(stdout, NULL, _IOLBF, 0);

  /* Ensure that decimal points are used for floating point numbers */
  setlocale(LC_NUMERIC, "C");

  while (1) {
    if ((bytes_received = receive(an_decoder_pointer(&an_decoder),
                                  an_decoder_size(&an_decoder))) > 0) {
      if (logFlag == 1) {
        /* Log all binary data coming from the sensor, this can be converted to
         * CSV using the manager software */
        fwrite(an_decoder_pointer(&an_decoder), sizeof(uint8_t), bytes_received,
               anpp_log_file);
        if (write_counter++ >= 100) {
          fflush(anpp_log_file);
          write_counter = 0;
        }
      }

      /* increment the decode buffer length by the number of bytes received */
      an_decoder_increment(&an_decoder, bytes_received);

      /* decode all the packets in the buffer */
      while ((an_packet = an_packet_decode(&an_decoder)) != NULL) {
        if (an_packet->id == packet_id_system_state) /* system state packet */
        {
          /* copy all the binary data into the typedef struct for the packet */
          /* this allows easy access to all the different values             */
          if (decode_system_state_packet(&system_state_packet, an_packet) ==
              0) {
            json_t *obj = json_object();
            json_object_set_new(
                obj, "Sec",
                json_real(system_state_packet.unix_time_seconds +
                          system_state_packet.microseconds * 1e-6));
            json_object_set_new(
                obj, "Lat",
                json_real(system_state_packet.latitude * RADIANS_TO_DEGREES));
            json_object_set_new(
                obj, "Lon",
                json_real(system_state_packet.longitude * RADIANS_TO_DEGREES));
            json_object_set_new(obj, "Alt",
                                json_real(system_state_packet.height));
            json_object_set_new(obj, "Roll",
                                json_real(system_state_packet.orientation[0] *
                                          RADIANS_TO_DEGREES));
            json_object_set_new(obj, "Pitch",
                                json_real(system_state_packet.orientation[1] *
                                          RADIANS_TO_DEGREES));
            json_object_set_new(obj, "Head",
                                json_real(system_state_packet.orientation[2] *
                                          RADIANS_TO_DEGREES));
            json_object_set_new(obj, "w_Roll",
                                json_real(system_state_packet.angular_velocity[0] *
                                          RADIANS_TO_DEGREES));
            json_object_set_new(obj, "w_Pitch",
                                json_real(system_state_packet.angular_velocity[1] *
                                          RADIANS_TO_DEGREES));
            json_object_set_new(obj, "w_Head",
                                json_real(system_state_packet.angular_velocity[2] *
                                          RADIANS_TO_DEGREES));
            json_object_set_new(
                obj, "DGNSS_heading_active",
                json_integer((system_state_packet.filter_status.b.dual_antenna_heading_active)));
            //   json_object_set_new(
            //       obj, "Filter_status",
            //       json_integer(system_state_packet.status.filter_status));
            // }
            // json_object_set_new(obj, "Flter_init",
            //                     json_integer(system_state_packet.filter_status &
            //                                  0x0001));
            // // Differential GNSS is active if bit 6 of filter_status is set
            // json_object_set_new(obj, "Differential GNSS status",
            //                     json_integer((system_state_packet.filter_status >> 6) & 0x01));

            // dump compact (no spaces), then newline
            char *line = json_dumps(obj, JSON_COMPACT);
            puts(line); // adds '\n'
            fflush(stdout);

            free(line);
            json_decref(obj);

            // printf("System State Packet:\n");
            // printf("\tLatitude = %f, Longitude = %f, Height = %f\n",
            //        system_state_packet.latitude * RADIANS_TO_DEGREES,
            //        system_state_packet.longitude * RADIANS_TO_DEGREES,
            //        system_state_packet.height);
            // printf("\tRoll = %f, Pitch = %f, Heading = %f\n",
            //        system_state_packet.orientation[0] * RADIANS_TO_DEGREES,
            //        system_state_packet.orientation[1] * RADIANS_TO_DEGREES,
            //        system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
          }
        } else if (an_packet->id ==
                   packet_id_raw_sensors) /* raw sensors packet */
        {
          /* copy all the binary data into the typedef struct for the packet */
          /* this allows easy access to all the different values             */
          if (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0) {
            // printf("Raw Sensors Packet:\n");
            // printf("\tAccelerometers X: %f Y: %f Z: %f\n",
            //        raw_sensors_packet.accelerometers[0],
            //        raw_sensors_packet.accelerometers[1],
            //        raw_sensors_packet.accelerometers[2]);
            // printf("\tGyroscopes X: %f Y: %f Z: %f\n",
            //        raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES,
            //        raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES,
            //        raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
          }
        } else {
          printf("Packet ID %u of Length %u\n", an_packet->id,
                 an_packet->length);
        }

        /* Ensure that you free the an_packet when your done with it or you will
         * leak memory */
        an_packet_free(&an_packet);
      }
    }
    usleep(10000);
  }
}

int transmit(const unsigned char *data, int length) {
  return comWrite(comPortIndex, data, length);
}

int receive(unsigned char *data, int length) {
  return comRead(comPortIndex, data, length);
}

int an_packet_transmit(an_packet_t *an_packet) {
  an_packet_encode(an_packet);
  return transmit(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

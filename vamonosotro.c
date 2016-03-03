/*
 * Copyright (c) 2011, Zolertia(TM) is a trademark of Advancare,SL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 *         A quick program for testing the light ziglet driver in the Z1 platform
 * \author
 *         Antonio Lignan <alinan@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include "contiki.h"
#include "vamonosotro.h"

#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "sys/ctimer.h"


#include "dev/i2cmaster.h"
#include "dev/adxl345.h"
#include "dev/light-ziglet.h"

// ALTA MACRO para ALTA MOTA
#define DELTA(X, Y) (X-Y < 0 ? -(X-Y) : X-Y)

/*---------------------------------------------------------------------------*/
#define SENSOR_READ_INTERVAL_LIGHT (CLOCK_SECOND / 2)

// LIGHT EVENTS ...
#define LIGHT_TRESH 300

static struct etimer et_light;

process_event_t event_light_detected;


/*---- MOTION ----*/
static struct etimer et_motion;
process_event_t event_motion_detected;

#define SENSOR_READ_INTERVAL_MOTION (CLOCK_SECOND / 4)
#define MOTION_TRESH 120

/*----NETWORK----*/

static struct my_msg_t msg;
static struct my_msg_t *msgPtr = &msg;

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static void send_packet()
{

  /* Print the sensor data */
  printf("key: %u, value: %u\n", msg.key, msg.value);
  /* Print the sensor data */

  /* Convert to network byte order as expected by the UDPServer application */
  msg.value = UIP_HTONS(msg.value);

  printf("Send readings to");
  PRINT6ADDR(&server_ipaddr);
  printf("\n");

  uip_udp_packet_sendto(client_conn, msgPtr, sizeof(msg), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}

static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  printf("Client IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}


PROCESS(light_event_handler, "Light event handler");
PROCESS(motion_event_handler, "Motion  event handler");
PROCESS(consumer, "Consumer");

//AUTOSTART_PROCESSES(&light_event_handler, &consumer);
AUTOSTART_PROCESSES(&motion_event_handler, &light_event_handler, &consumer);

PROCESS_THREAD(motion_event_handler, ev, data)
{
  PROCESS_BEGIN();

  event_motion_detected = process_alloc_event();

  static int8_t x_axis;
  static int8_t y_axis;
  static int8_t z_axis;

  static int8_t last_x_axis;
  static int8_t last_y_axis;
  static int8_t last_z_axis;

  static int8_t motion_detected = 0;
  static int8_t counter = 0;

  SENSORS_ACTIVATE(adxl345);

  etimer_set(&et_motion, SENSOR_READ_INTERVAL_MOTION);

  last_x_axis = adxl345.value(X_AXIS);
  last_y_axis = adxl345.value(Y_AXIS);
  last_z_axis = adxl345.value(Z_AXIS);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_motion));

    /* Read the sensors */
    x_axis = adxl345.value(X_AXIS);
    y_axis = adxl345.value(Y_AXIS);
    z_axis = adxl345.value(Z_AXIS);

    if (!motion_detected && ((DELTA(x_axis, last_x_axis) +
                              DELTA(y_axis, last_y_axis) +
                              DELTA(z_axis, last_z_axis)) > MOTION_TRESH)) {
      //emit
      process_post(PROCESS_BROADCAST, event_motion_detected, 0);
      counter = 20; // ~ 5s
      motion_detected = 1;
    }

    if (motion_detected && --counter <= 0) {
      motion_detected = 0;
    }

    etimer_reset(&et_motion);
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(light_event_handler, ev, data)
{
  PROCESS_BEGIN();

  event_light_detected = process_alloc_event();

  /* Initialize driver and set a slower data rate */
  light_ziglet_init();

  static uint16_t last_light;
  static uint16_t light;

  last_light = light_ziglet_read();

  etimer_set(&et_light, SENSOR_READ_INTERVAL_LIGHT);


  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_light));

    light = light_ziglet_read();

    printf("Light = %u\n", light);

    if (DELTA(light, last_light) > LIGHT_TRESH) {
      process_post(PROCESS_BROADCAST, event_light_detected, &light);
      last_light = light;
    }

    etimer_reset(&et_light);

  }
  PROCESS_END();
}


PROCESS_THREAD(consumer, ev, data)
{
  PROCESS_BEGIN();

  msg.id = 0x42;
  msg.key = 0;
  msg.value = 0;

  printf("Process consumer started\n");

  printf("UDP client process started\n");

  /* Set the server address here */
  uip_ip6addr(&server_ipaddr, 0x2800, 0x0340, 0x0052, 0x0066, 0x021e, 0x68ff, 0xfe45, 0xfec4);

  printf("Server address: ");
  PRINT6ADDR(&server_ipaddr);
  printf("\n");

  /* Print the node's addresses */
  print_local_addresses();

  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);

  if(client_conn == NULL) {
    printf("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }


  /* This function binds a UDP connection to a specified local por */
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  printf("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  printf(" local/remote port %u/%u\n", UIP_HTONS(client_conn->lport),
                                       UIP_HTONS(client_conn->rport));


  while(1) {


    PROCESS_WAIT_EVENT_UNTIL((ev == event_light_detected) ||
                             (ev == event_motion_detected));

    if (ev == event_light_detected) {
      printf("LEVEL LIGHT CHANGE\n");
      msg.key = 1;
      msg.value = *((uint8_t *)data);
    }

    if (ev == event_motion_detected) {
      printf("MOTION DETECTED\n");
      msg.key = 2;
      msg.value = 0;
    }

    send_packet();

  }

  /* This is the end of the process, we tell the system we are done.  Even if
   * we won't reach this due to the "while(...)" we need to include it
   */
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

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
#include "dev/leds.h"

#include "dev/i2cmaster.h"
#include "dev/adxl345.h"
#include "dev/light-ziglet.h"

/* libraries for ubidots works to works */
//#include "sys/process.h"
//#include "sys/etimer.h"
//#include "ubidots.h"
//#include "dev/tmp102.h"
//#include <string.h>

/*------------- UBIDOTS PART -------------------*/
/*---------------------------------------------------------------------------*/
/* Sanity check */
#if !defined(UBIDOTS_DEMO_CONF_LIGHT) || !defined(UBIDOTS_DEMO_CONF_MOTION)
#error "UBIDOTS_DEMO_CONF_LIGHT or UBIDOTS_DEMO_CONF_MOTION undefined."
/*#if !defined(UBIDOTS_DEMO_CONF_TEMPERATURE) || !defined(UBIDOTS_DEMO_CONF_SEQUENCE)
#error "UBIDOTS_DEMO_CONF_TEMPERATURE or UBIDOTS_DEMO_CONF_SEQUENCE undefined."*/
#error "Make sure you have followed the steps in the README"
#endif
/*---------------------------------------------------------------------------*/
/* POST period */
#define POST_PERIOD (CLOCK_SECOND * 30)
static struct etimer et;
/*---------------------------------------------------------------------------*/
#define VARIABLE_BUF_LEN 16
static unsigned int sequence;

static char variable_buffer[VARIABLE_BUF_LEN];
/*---------------------------------------------------------------------------*/
/*
 * 'List' of HTTP reply headers that we want to be notified about.
 * Terminate with NULL
 */
static const char *headers[] = {
  "Vary",
  NULL
};
/*---------------------------------------------------------------------------*/
/*
 * An example of how to POST one more more values to the same variable. This
 * primarily shows how to use the value argument depending on whether you
 * want to send a JSON string, number or boolean.
 */
static void post_sequence_number(void){
  if(ubidots_prepare_post(UBIDOTS_DEMO_CONF_SEQUENCE) == UBIDOTS_ERROR) {
    printf("post_variable: ubidots_prepare_post failed\n");
  }

  memset(variable_buffer, 0, VARIABLE_BUF_LEN);

  /*
   * Write your value to the buffer. The contents of the buffer will be used
   * verbatim as the value of the variable in your JSON string. So, if you
   * store a number in the buffer this will become a JSON number. If you
   * enclose the value in double quotes, this will essentially be a JSON string
   *
   * Some examples
   * To send your value as a JSON number:
   * snprintf(variable_buffer, VARIABLE_BUF_LEN, "%u", sequence);
   *
   * To send your value as a JSON string:
   * snprintf(variable_buffer, VARIABLE_BUF_LEN, "\"%u\"", sequence);
   *
   * To send a JSON boolean:
   * ubidots_enqueue_value(NULL, "true");
   */
  snprintf(variable_buffer, VARIABLE_BUF_LEN, "%u", sequence);

  /* Append the contents of the buffer to your HTTP POST's payload */
  if(ubidots_enqueue_value(NULL, variable_buffer) == UBIDOTS_ERROR) {
    printf("post_variable (string): ubidots_enqueue_value failed\n");
  }

  /*
   * You can make a series of calls to ubidots_enqueue_value() here, as long
   * as they all have NULL as the first argument. In doing so, you can send
   * multiple values for the same variable
   */
  if(ubidots_post() == UBIDOTS_ERROR) {
    printf("post_variable: ubidots_post failed\n");
  }
}
/*---------------------------------------------------------------------------*/
/*
 * An example of how to post a collection: multiple different variables in
 * a single HTTP POST using {"variable":k,"value":v} pairs
 */
static void post_collection(void){
  uint16_t temp;
  if(ubidots_prepare_post(NULL) == UBIDOTS_ERROR) {
    printf("post_collection: ubidots_prepare_post failed\n");
  }

  /* Encode and enqueue the uptime as a JSON number */
  memset(variable_buffer, 0, VARIABLE_BUF_LEN);
  temp = tmp102_read_temp_x100();  
  snprintf(variable_buffer, VARIABLE_BUF_LEN, "%u", temp);

  if(ubidots_enqueue_value(UBIDOTS_DEMO_CONF_TEMPERATURE, variable_buffer) == UBIDOTS_ERROR) {
    printf("post_collection: ubidots_prepare_post failed\n");
  }

  /* And the sequence counter, again as a JSON number */
  memset(variable_buffer, 0, VARIABLE_BUF_LEN);
  snprintf(variable_buffer, VARIABLE_BUF_LEN, "%u", sequence);

  if(ubidots_enqueue_value(UBIDOTS_DEMO_CONF_SEQUENCE, variable_buffer) == UBIDOTS_ERROR) {
    printf("post_collection: ubidots_prepare_post failed\n");
  }

  if(ubidots_post() == UBIDOTS_ERROR) {
    printf("post_collection: ubidots_prepare_post failed\n");
  }
}
/*---------------------------------------------------------------------------*/
/*
 * This is how to process the HTTP reply from the Ubidots server. In a real
 * scenario, we may wish to do something useful here, e.g. to test whether
 * the POST succeeded.
 *
 * This function here simply prints the entire thing, demonstrating how to use
 * the engine's API.
 */
static void print_reply(ubidots_reply_part_t *r) {
  switch(r->type) {
  case UBIDOTS_REPLY_TYPE_HTTP_STATUS:
    printf("HTTP Status: %ld\n", *((long int *)r->content));
    break;
  case UBIDOTS_REPLY_TYPE_HTTP_HEADER:
    printf("H: '%s'\n", (char *)r->content);
    break;
  case UBIDOTS_REPLY_TYPE_PAYLOAD:
    printf("P: '%s'\n", (char *)r->content);
    break;
  default:
    printf("Unknown reply type\n");
    break;
  }
}
/*------------------------------ auxiliars functions for UBIDOTS ENDS ---------------------------------------------*/

// ALTA MACRO para ALTA MOTA
#define DELTA(X, Y) (X-Y < 0 ? -(X-Y) : X-Y)

/*---------------------------------------------------------------------------*/
#define SENSOR_READ_INTERVAL_LIGHT (CLOCK_SECOND / 2)
#define SENSOR_READ_INTERVAL_MOTION (CLOCK_SECOND / 4)

/*---- LIGHT ----*/
#define LIGHT_TRESH 300
static struct etimer et_light;
process_event_t event_light_detected;

/*---- MOTION ----*/
#define MOTION_TRESH 224
static struct etimer et_motion;
process_event_t event_motion_detected;

/*----NETWORK----*/
static struct my_msg_t msg;
static struct my_msg_t *msgPtr = &msg;

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;


static void send_packet(){
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

static void print_local_addresses(void) {
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




/*---- PROCESS ----*/
PROCESS(light_event_handler, "Light event handler");
PROCESS(motion_event_handler, "Motion  event handler");
PROCESS(consumer, "Consumer");
PROCESS(ubidots_demo_process, "Ubidots demo process"); // PROCESO PARA UBIDOTS

/*---- START PROCESSES ----*/
//AUTOSTART_PROCESSES(&light_event_handler, &consumer);
AUTOSTART_PROCESSES(&motion_event_handler, &consumer);
//AUTOSTART_PROCESSES(&ubidots_demo_process); // PROCESO PARA UBIDOTS
//AUTOSTART_PROCESSES(&motion_event_handler, &light_event_handler, &consumer);


/*---- THREAD MOTION SENSOR ----*/
PROCESS_THREAD(motion_event_handler, ev, data) {

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

    printf("%d %d %d\n%d %d %d\n%d\n\n\n",
           x_axis, y_axis, z_axis,
           last_x_axis, last_y_axis, last_z_axis,
           DELTA(last_x_axis, x_axis) + DELTA(last_y_axis, y_axis) + DELTA(last_z_axis,z_axis));

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

    last_x_axis = x_axis;
    last_y_axis = y_axis;
    last_z_axis = z_axis;

    etimer_reset(&et_motion);
  }

  PROCESS_END();
}

/*---- THREAD LIGHT SENSOR ----*/
PROCESS_THREAD(light_event_handler, ev, data) {
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

/*---- THREAD CONSUMER ----*/
PROCESS_THREAD(consumer_mqtt, ev, data)
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
      leds_toggle(LEDS_BLUE);
    }
    if (ev == event_motion_detected) {
      printf("MOTION DETECTED\n");
      msg.key = 2;
      msg.value = 0;
      leds_toggle(LEDS_RED);
    }

    send_packet();

  }
  /* This is the end of the process, we tell the system we are done.  Even if
   * we won't reach this due to the "while(...)" we need to include it
   */
  PROCESS_END();
}

/*----------------------------- THREAD THAT LOAD DATA TO UBIDOTS ----------------------------------------------*/
PROCESS_THREAD(ubidots_demo_process, ev, data)
{
  PROCESS_BEGIN();

  tmp102_init();
  ubidots_init(&ubidots_demo_process, headers);

  sequence = 0;

  while(1) {

    PROCESS_YIELD();

    if(ev == ubidots_event_established ||
       (ev == PROCESS_EVENT_TIMER && data == &et)) {
      leds_on(LEDS_GREEN);
      sequence++;

      if(sequence & 1) {
        post_sequence_number();
      } else {
        post_collection();
      }
    } else if(ev == ubidots_event_post_sent) {
      leds_off(LEDS_GREEN);
      etimer_set(&et, POST_PERIOD);
    } else if(ev == ubidots_event_post_reply_received) {
      print_reply((ubidots_reply_part_t *)data);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

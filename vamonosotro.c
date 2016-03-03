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
#include "dev/light-ziglet.h"
/*---------------------------------------------------------------------------*/
#define SENSOR_READ_INTERVAL (CLOCK_SECOND / 2)

// LIGHT EVENTS ...
#define LOW_LIGHT 1
#define MID_LIGHT 2
#define HIGH_LIGHT 3

#define LOW_TRESH 50
#define HIGH_TRESH 300

/*
   low   |      med     | high
        050            300
 */

/*----NETWORK----*/

static struct my_msg_t msg;
static struct my_msg_t *msgPtr = &msg;

static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static void send_packet()
{

  /* Print the sensor data */
  printf("ID: %u, light: %u\n", msg.id, msg.light);
  /* Print the sensor data */

  /* Convert to network byte order as expected by the UDPServer application */
  //  msg.light = UIP_HTONS(msg.light);

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
PROCESS(consumer, "Consumer");

AUTOSTART_PROCESSES(&light_event_handler, &consumer);

static struct etimer et;

process_event_t event_light_low;
process_event_t event_light_mid;
process_event_t event_light_high;

uint8_t to_range(uint16_t value) {

  if (value <= LOW_TRESH) {
    return LOW_LIGHT;
  } else if (value <= HIGH_TRESH) {
    return MID_LIGHT;
  }

  return HIGH_LIGHT;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(light_event_handler, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  static uint16_t last_value = -1;

  event_light_low = process_alloc_event();
  event_light_mid = process_alloc_event();
  event_light_high = process_alloc_event();

  /* Initialize driver and set a slower data rate */
  light_ziglet_init();
  i2c_setrate(I2C_PRESC_100KHZ_LSB, I2C_PRESC_100KHZ_MSB);

  while(1) {
    etimer_set(&et, SENSOR_READ_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    uint16_t light = light_ziglet_read();
    uint16_t value = to_range(light);

    printf("Light = %u\n", light);

    if (value != last_value) {
      last_value = value;
      switch(value) {
      case LOW_LIGHT:
        process_post(PROCESS_BROADCAST, event_light_low, 0);
        break;
      case MID_LIGHT:
        process_post(PROCESS_BROADCAST, event_light_mid, 0);
        break;
      case HIGH_LIGHT:
        process_post(PROCESS_BROADCAST, event_light_high, 0);
        break;
      }
    }

  }
  PROCESS_END();
}


PROCESS_THREAD(consumer, ev, data)
{
  PROCESS_BEGIN();

  msg.id      = 0xAB;
  msg.light   = 0x42;

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
    PROCESS_WAIT_EVENT_UNTIL((ev == event_light_low) || (ev == event_light_mid) || (ev == event_light_high));

    if (ev == event_light_low) {
      printf("LOW\n");
      msg.light = 10;

    }

    if (ev == event_light_mid) {
      printf("MID\n");
      msg.light = 20;
    }

    if (ev == event_light_high) {
      printf("HIGH\n");
      msg.light = 30;
    }
    send_packet();

  }

  /* This is the end of the process, we tell the system we are done.  Even if
   * we won't reach this due to the "while(...)" we need to include it
   */
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

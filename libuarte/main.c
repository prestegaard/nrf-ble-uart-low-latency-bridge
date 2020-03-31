/**
 * Copyright (c) 2018 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup libuarte_example_main main.c
 * @{
 * @ingroup libuarte_example
 * @brief Libuarte Example Application main file.
 *
 * This file contains the source code for a sample application using libuarte.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_libuarte_async.h"
#include "nrf_drv_clock.h"
#include <bsp.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_queue.h"
#include "nrfx_timer.h"

#define TX_PIN_NUMBER_PERIPHERAL 30
#define RX_PIN_NUMBER_PERIPHERAL 31
#define TX_PIN_NUMBER_CENTRAL    6
#define RX_PIN_NUMBER_CENTRAL    8

#define MESSAGE_FREQUENCY_MS     10


NRF_LIBUARTE_ASYNC_DEFINE(libuarte_peripheral, 0, 0, 0, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 100, 3);
NRF_LIBUARTE_ASYNC_DEFINE(libuarte_central,    1, 1, 2, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 100, 3);

const nrfx_timer_t timestamp_timer = NRFX_TIMER_INSTANCE(2);   
const nrfx_timer_t inactivity_timer = NRFX_TIMER_INSTANCE(3);

static uint8_t text[] = "UART example started.\r\n Loopback:\r\n";
static uint8_t text_size = sizeof(text);
static volatile bool m_loopback_phase;
static uint8_t msg_cnt;
static volatile bool inactivity_timeout;
static volatile uint32_t messages_received_from_peripheral;
static volatile uint32_t messages_received_from_central;
static volatile uint8_t message_peripheral[5][100];
static volatile uint8_t message_central[5][100];
static volatile uint8_t rtt_peripheral[100];
static volatile uint8_t rtt_central[100];

typedef struct {
    uint8_t * p_data;
    uint32_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue_peripheral, 10, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(buffer_t, m_buf_queue_central, 10, NRF_QUEUE_MODE_NO_OVERFLOW);

static void dummy_handler(nrf_timer_event_t event_type, void* p_context){
    //printf("dummy_handler");
}

void timestamp_timer_config(void)
{ 
    uint32_t err_code = NRF_SUCCESS;
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    
    err_code = nrfx_timer_init(&timestamp_timer, &timer_cfg, dummy_handler);
    APP_ERROR_CHECK(err_code);
    
    nrfx_timer_enable(&timestamp_timer);
}

// Timestamp function for logging
uint32_t timestamp_timer_cnt(void)
{
    uint32_t time = nrfx_timer_capture(&timestamp_timer, NRF_TIMER_CC_CHANNEL0);
    return time;
}

uint32_t timestamp_ms()
{
    timestamp_timer_cnt()/125;
}
static void inactivity_timer_handler(nrf_timer_event_t event_type, void* p_context){
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            nrfx_timer_clear(&inactivity_timer);
            if(msg_cnt < 100)
            {
                msg_cnt++;
                inactivity_timeout = true; 
            }
    }
}

void inactivity_timer_config(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    
    err_code = nrfx_timer_init(&inactivity_timer, &timer_cfg, inactivity_timer_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_timer_compare(&inactivity_timer, NRF_TIMER_CC_CHANNEL0, MESSAGE_FREQUENCY_MS*125, true);
    
    nrfx_timer_enable(&inactivity_timer);
    
    nrfx_timer_pause(&inactivity_timer);

    nrfx_timer_clear(&inactivity_timer);
    
}

void uart_event_handler_peripheral(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    nrf_libuarte_async_t * p_libuarte = (nrf_libuarte_async_t *)context;
    ret_code_t ret;
    static uint32_t current_time;
    static uint32_t time_diff;
    static char time_str[20];
    static int64_t time_from_message;

    switch (p_evt->type)
    {
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:
            bsp_board_led_invert(0);
            bsp_board_led_invert(1);
            bsp_board_led_invert(2);
            bsp_board_led_invert(3);
            NRF_LOG_ERROR("%d, LIBUARTE_ASYNC_EVT_ERROR", msg_cnt);
            break;
        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
            current_time = timestamp_timer_cnt();
            strncpy(time_str, p_evt->data.rxtx.p_data, 10);
            time_from_message = atoi(time_str);
            if(time_from_message)
            {
                time_diff = current_time - time_from_message;
                time_diff = time_diff/125;
                uint8_t _err = 0;
                uint8_t _msgGood = 0;
                if(p_evt->data.rxtx.length != 100)
                {
                    NRF_LOG_ERROR("Received %d bytes from Peripheral UART", p_evt->data.rxtx.length);
                    _err = 1;
                }
                
                if(!_err)
                {
                    for(uint8_t j=0; j<5; j++)
                    {
                        _msgGood = 1;
                        for(uint8_t i=0; i<100; i++)
                        {
                            // Verify message originally sent to central over UART.
                            if(p_evt->data.rxtx.p_data[i] != message_central[j][i])
                            {
                                _msgGood = 0;
                                // NRF_LOG_ERROR("Message0 error, bad byte at: %d", i);
                                break;
                            }                    
                        }    
                        if(_msgGood)
                        {
                            rtt_peripheral[messages_received_from_peripheral++] = time_diff;
                            NRF_LOG_INFO("#msg: %d, rtt: %d [ms]", messages_received_from_peripheral, time_diff);
                            break;
                        }
                    }                
                    if(!_msgGood)
                    {
                        NRF_LOG_ERROR("Payload does not match message history");
                    }
                }                
            }
            else
            {
                NRF_LOG_ERROR("Received message without timestamp, length: %d", p_evt->data.rxtx.length);
            }                
            bsp_board_led_invert(2); 
            nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
            break;
        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:    
            bsp_board_led_invert(3);
            break;
        default:
            break;
    }
}

void uart_event_handler_central(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    nrf_libuarte_async_t * p_libuarte = (nrf_libuarte_async_t *)context;
    ret_code_t ret;
    static uint32_t current_time;
    static uint32_t time_diff;
    static char time_str[20];
    static int64_t time_from_message;

    switch (p_evt->type)
    {
        case NRF_LIBUARTE_ASYNC_EVT_ERROR:
            bsp_board_led_invert(0);
            bsp_board_led_invert(1);
            bsp_board_led_invert(2);
            bsp_board_led_invert(3);
            NRF_LOG_ERROR("%d, LIBUARTE_ASYNC_EVT_ERROR", msg_cnt);
            break;
        case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
            current_time = timestamp_timer_cnt();
            strncpy(time_str, p_evt->data.rxtx.p_data, 10);
            time_from_message = atoi(time_str);
            if(time_from_message)
            {
                time_diff = current_time - time_from_message;
                time_diff = time_diff/125;
                uint8_t _err = 0;
                uint8_t _msgGood = 0;
                if(p_evt->data.rxtx.length != 100)
                {
                    NRF_LOG_ERROR("Received %d bytes from Central UART", p_evt->data.rxtx.length);
                    _err = 1;
                }
                
                if(!_err)
                {
                    for(uint8_t j=0; j<5; j++)
                    {
                        _msgGood = 1;
                        for(uint8_t i=0; i<100; i++)
                        {
                            // Verify message originally sent to central over UART.
                            if(p_evt->data.rxtx.p_data[i] != message_peripheral[j][i])
                            {
                                _msgGood = 0;
                                // NRF_LOG_ERROR("Message0 error, bad byte at: %d", i);
                                break;
                            }                    
                        }    
                        if(_msgGood)
                        {
                            rtt_central[messages_received_from_central++] = time_diff;
                            NRF_LOG_INFO("#msg: %d, rtt: %d [ms]", messages_received_from_central, time_diff);
                            break;
                        }
                    }                
                    if(!_msgGood)
                    {
                        NRF_LOG_ERROR("Payload does not match message history");
                    }
                }                
            }
            else
            {
                NRF_LOG_ERROR("Received message without timestamp, length: %d", p_evt->data.rxtx.length);
            }                
            bsp_board_led_invert(2); 
            nrf_libuarte_async_rx_free(p_libuarte, p_evt->data.rxtx.p_data, p_evt->data.rxtx.length);
            break;
        case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:    
            bsp_board_led_invert(3);
            break;
        default:
            break;
    }
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    messages_received_from_central = 0;
    msg_cnt = 0;
    bsp_board_init(BSP_INIT_LEDS);
    
    ret_code_t ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
  
    nrf_drv_clock_lfclk_request(NULL);

    timestamp_timer_config();

    inactivity_timer_config();
    

    // ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
    ret_code_t err_code = NRF_LOG_INIT(timestamp_timer_cnt);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();


    nrf_libuarte_async_config_t nrf_libuarte_async_config_peripheral = {
            .tx_pin     = TX_PIN_NUMBER_PERIPHERAL,
            .rx_pin     = RX_PIN_NUMBER_PERIPHERAL,
            .baudrate   = UARTE_BAUDRATE_BAUDRATE_Baud250000,
            .parity     = NRF_UARTE_PARITY_EXCLUDED,
            .hwfc       = NRF_UARTE_HWFC_DISABLED,
            .timeout_us = 100,
            .int_prio   = APP_IRQ_PRIORITY_LOW
            // .pullup_rx  = 1;
    };

    nrf_libuarte_async_config_t nrf_libuarte_async_config_central = {
            .tx_pin     = TX_PIN_NUMBER_CENTRAL,
            .rx_pin     = RX_PIN_NUMBER_CENTRAL,
            .baudrate   = UARTE_BAUDRATE_BAUDRATE_Baud250000,
            .parity     = NRF_UARTE_PARITY_EXCLUDED,
            .hwfc       = NRF_UARTE_HWFC_DISABLED,
            .timeout_us = 100,
            .int_prio   = APP_IRQ_PRIORITY_LOW
            // .pullup_rx  = 1;
    };

    err_code = nrf_libuarte_async_init(&libuarte_peripheral, &nrf_libuarte_async_config_peripheral, uart_event_handler_peripheral, (void *)&libuarte_peripheral);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_libuarte_async_init(&libuarte_central, &nrf_libuarte_async_config_central, uart_event_handler_central, (void *)&libuarte_central);
    APP_ERROR_CHECK(err_code);  

    nrf_libuarte_async_enable(&libuarte_peripheral);
    nrf_libuarte_async_enable(&libuarte_central);

    // err_code = nrf_libuarte_async_tx(&libuarte_peripheral, text, text_size);
    // APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("*****************************************");
    NRF_LOG_INFO("Start of UART round trip measurement demo");
    for(uint8_t i=0; i<100; i++)
    {
        message_peripheral[0][i] = 65; // Initialize message with char A to show in message
        message_peripheral[1][i] = 66; // Initialize message with char B to show in message
        message_peripheral[2][i] = 67; // Initialize message with char C to show in message
        message_peripheral[3][i] = 68; // Initialize message with char D to show in message
        message_peripheral[4][i] = 69; // Initialize message with char E to show in message
        message_central[0][i] = 65; // Initialize message with char A to show in message
        message_central[1][i] = 66; // Initialize message with char B to show in message
        message_central[2][i] = 67; // Initialize message with char C to show in message
        message_central[3][i] = 68; // Initialize message with char D to show in message
        message_central[4][i] = 69; // Initialize message with char E to show in message
    }
    uint32_t current_time = timestamp_timer_cnt();
    uint8_t current_time_str[20];
    uint8_t msg_cnt_str[20] = {0};

    uint8_t direction = 0;
    nrfx_timer_resume(&inactivity_timer);
    while (true)
    {
        NRF_LOG_FLUSH();
        if(inactivity_timeout)
        {
            inactivity_timeout = 0;
            current_time = timestamp_timer_cnt();
            itoa(current_time, current_time_str, 10);
            itoa(msg_cnt, msg_cnt_str, 10);
            uint8_t msg_number;
            if(direction == 0)
            {
                msg_number = msg_cnt % 5;
                // sprintf(message_peripheral[msg_cnt % 5], "%06d, %03d, This message is made to test round trip time. This message needs some length %d%d%d%d%d%d%d%d\r\n", current_time, msg_cnt, msg_number, msg_number, msg_number, msg_number, msg_number, msg_number, msg_number, msg_number);
                
                sprintf(message_peripheral[msg_cnt % 5], "\r\n%07d, %03d, ", current_time, msg_cnt);
                for(uint8_t j=16; j<100; j++)
                {
                    if(msg_cnt < 35)
                    {
                        message_peripheral[msg_cnt % 5][j] = msg_cnt + 35;
                    }                        
                    else
                    {
                        message_peripheral[msg_cnt % 5][j] = msg_cnt;
                    }
                    
                }

                while (nrf_libuarte_async_tx(&libuarte_peripheral, message_peripheral[msg_cnt % 5], 100) != NRF_ERROR_BUSY);
                if(msg_cnt % 10 == 0)
                {
                    NRF_LOG_INFO("%s, Sent message, timestamp: %s", msg_cnt_str, current_time_str);
                }
                // Change direction of message flow
                if (msg_cnt >= 100)
                {
                    direction = 1;
                    msg_cnt = 0;
                }
            }
            else if(direction == 1)
            {
                msg_number = msg_cnt % 5;
                sprintf(message_central[msg_cnt % 5], "\r\n%07d, %03d, ", current_time, msg_cnt);
                for(uint8_t j=16; j<100; j++)
                {
                    if(msg_cnt < 35)
                    {
                        message_central[msg_cnt % 5][j] = msg_cnt + 35;
                    }
                    else
                    {
                        message_central[msg_cnt % 5][j] = msg_cnt;
                    }
                    
                }
                while (nrf_libuarte_async_tx(&libuarte_central, message_central[msg_cnt % 5], 100) != NRF_ERROR_BUSY);
                if(msg_cnt % 10 == 0)
                {
                    NRF_LOG_INFO("%s, Sent message, timestamp: %s", msg_cnt_str, current_time_str);
                }
                if(msg_cnt >= 100)
                {
                    direction = 2;
                    msg_cnt = 0;
                }

                
            }
            // Calculate average round trip times for both directions
            else if(direction == 2)
            {
                // Add delay to make sure all messages are done
                if (msg_cnt >= 100)
                {   
                    uint32_t total_rtt_peripheral = 0;
                    float avg_peripheral;
                    for(uint8_t peripheral_msg=0; peripheral_msg<messages_received_from_peripheral; peripheral_msg++)
                    {
                        total_rtt_peripheral += rtt_peripheral[peripheral_msg];
                    }
                    avg_peripheral = total_rtt_peripheral/messages_received_from_peripheral;
                    

                    uint32_t total_rtt_central = 0;
                    float avg_central;
                    for(uint8_t central_msg=0; central_msg<messages_received_from_central; central_msg++)
                    {
                        total_rtt_central += rtt_central[central_msg];
                    }
                    avg_central = total_rtt_central/messages_received_from_central;
                    NRF_LOG_FLUSH();
                    NRF_LOG_FLUSH();
                    NRF_LOG_INFO("Messages peripheral-central direction %d of %d OK", messages_received_from_central, 100);
                    NRF_LOG_INFO("Messages central-peripheral direction %d of %d OK", messages_received_from_peripheral, 100);
                    NRF_LOG_INFO("Average round trip time:");
                    NRF_LOG_INFO("Tester -> UART -> Peripheral -> BLE -> Central -> UART -> Tester: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(avg_peripheral));
                    NRF_LOG_INFO("Tester -> UART -> Central -> BLE -> Peripheral -> UART -> Tester: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(avg_central));
                    direction = 3;
                }
            }
            
            
        }
    }
}


/** @} */

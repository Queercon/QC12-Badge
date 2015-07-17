/*
 * qcxi.h
 *
 * (c) 2014 George Louthan
 * 3-clause BSD license; see license.md.
 */

#ifndef QC12_H_
#define QC12_H_

#include <stdint.h>
#include <driverlib/MSP430FR5xx_6xx/driverlib.h>
#include <msp430fr5949.h>
#include <grlib.h>

// CONFIGURATION SETTINGS ///////////////////////////////////////////

// System configuration
#define BADGES_IN_SYSTEM 200
#define SLEEP_BITS LPM1_bits // because we need SMCLK for the TLC.

// Name entry configuration parameters:
#define NAME_SEL_CHAR '*'
#define NAME_MAX_LEN 14
// this will break everything if it's 1 or possibly 2:
#define NAME_COMMIT_LOOPS 80

// Character name & title font:
#define NAME_FONT_HEIGHT 14
#define NAME_FONT g_sFontCmss12b

// Main system font:
#define SYS_FONT g_sFontCmss12b
#define SYS_FONT_HEIGHT 12

// Softkey label font:
#define SOFTKEY_LABEL_FONT g_sFontCmsc12
#define SOFTKEY_FONT_HEIGHT 10 // not 12 because we're using sm. caps only.

//////////////////////////
// Derived definitions ///

#define NAME_Y_OFFSET 10+SYS_FONT_HEIGHT*7 // Top of the name entry field.
#define SPRITE_Y 128 - 64 - SOFTKEY_FONT_HEIGHT // Top of the char sprite.

/////////////////////////////////////////////////////////////////////
// Hardware related defines /////////////////////////////////////////

// LED controller:
#define TLC_LATPORT     GPIO_PORT_P1
#define TLC_LATPIN      GPIO_PIN4

// Radio:
#define RFM_NSS_PORT_OUT P1OUT
#define RFM_NSS_PIN      GPIO_PIN3

/////////////////////////////////////////////////////////////////////
// State constants //////////////////////////////////////////////////

// Button events:
#define BUTTON_PRESS 1
#define BUTTON_RELEASE 2

// LED sending types:
#define TLC_SEND_IDLE     0
#define TLC_SEND_TYPE_GS  1
#define TLC_SEND_TYPE_FUN 2
#define TLC_SEND_TYPE_LB  3

//////////////////////////////////////////////////////////////////////
// Functions etc. ////////////////////////////////////////////////////

#define GPIO_pulse(port, pin) do { GPIO_setOutputHighOnPin(port, pin); GPIO_setOutputLowOnPin(port, pin); } while (0)

// The delay function, which we don't really want to use much, please.
void delay(unsigned int);

// Interrupt flags:
extern volatile uint8_t f_time_loop;
extern volatile uint8_t f_rfm_rx_done;
extern volatile uint8_t f_rfm_tx_done;

typedef struct {
	uint8_t to_addr, from_addr, base_id;
	uint8_t beacon;
	uint8_t padding[40];
} qc12payload;

extern qc12payload in_payload, out_payload;

extern const char titles[][10];

typedef struct {
    uint8_t badge_id;
    uint8_t mood;
    uint8_t title_index;
    uint16_t exp;
    char handle[NAME_MAX_LEN+1];
    uint16_t crc16;
} qc12conf;

extern qc12conf my_conf;
extern const qc12conf default_conf;

#endif /* QC12_H_ */

/*
 * leds.c
 * (c) 2015 George Louthan
 * 3-clause BSD license; see license.md.
 */

/*
 *   LED controller (TLC5948A)
 *        (write on rise, change on fall,
 *         clock inactive low, MSB first)
 *        eUSCI_A0 - LEDs  (shared)
 *        somi, miso, clk
 *        GSCLK     P1.2 (timer TA1.1)
 *        LAT       P1.4
 */

#include "qc12.h"
#include "leds.h"
#include <string.h>

#define TLC_THISISGS    0x00
#define TLC_THISISFUN   0x01

// Current TLC sending state:
uint8_t tlc_send_type = TLC_SEND_IDLE;

uint8_t tlc_tx_index = 0;   // Index of currently sending buffer
uint8_t tlc_ok_to_send = 1; // SPI bus busy or not?
uint8_t tlc_light_offset = 0;   // Number of channels to shift gs_data by

// GS entry transmission handling indices:
volatile uint8_t rgb_element_index = 14;
volatile uint8_t tlc_color_index = 0;

#define TLC_ANIM_MODE_IDLE  0
#define TLC_ANIM_MODE_SHIFT 1
#define TLC_ANIM_MODE_SAME  2

uint8_t led_anim_mode = TLC_ANIM_MODE_IDLE;
uint8_t led_anim_index = 0;

// Utility light:
uint16_t tlc_tx_light = 0xffff;

rgbcolor_t *tlc_curr_anim;
uint8_t tlc_curr_anim_len;

rgbcolor_t rainbow1[10] = {
        // rainbow colors
        { 0x0f00, 0x00, 0x00},
        { 0xff00, 0x00, 0x00 },
        { 0x0f00, 0x0f00, 0x00},
        { 0x00, 0xff00, 0x00 },
        { 0x00, 0x0f00, 0x0f00 },
        { 0x00, 0x00, 0xff00 },
        { 0x100, 0x100, 0x2000 },
        { 0xe00, 0xe00, 0xe00 },
        { 0x100, 0x100, 0x100 },
        { 0x00, 0x00, 0x000 },
};

rgbcolor_t rainbow2[5] = {
        { 0xff00, 0x00, 0x00 },
        { 0x00, 0xff00, 0x00 },
        { 0x00, 0x00, 0xff00 },
        { 0xff00, 0xff00, 0xff00 },
        { 0x00, 0x00, 0x000 },
};

rgbcolor_t test_colors[3] = {
        // rainbow colors
        { 0xff00, 0x00, 0x00},
        { 0x00, 0xff00, 0x00 },
        { 0x00, 0x00, 0xff00 },
};

uint8_t ring_fade_index = 0;
uint8_t ring_fade_steps = 0;

rgbcolor_t tlc_curr_colors[5] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};

rgbcolor_t rainbow_ring_dest[5] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};

rgbdelta_t rainbow_ring_step[5] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};

// Buffers containing actual data to send to the TLC:

uint8_t fun_base[] = {
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00, // ...reserved...
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        // B135 / PSM(D1)       0
        // B134 / PSM(D0)       0
        // B133 / OLDENA        0
        // B132 / IDMCUR(D1)    0
        // B131 / IDMCUR(D0)    0
        // B130 / IDMRPT(D0)    0
        // B129 / IDMENA        0
        // B128 / LATTMG(D1)    1:
        0x01,
        // B127 / LATTMG(D0)    1
        // B126 / LSDVLT(D1)    0
        // B125 / LSDVLT(D0)    0
        // B124 / LODVLT(D1)    0
        // B123 / LODVLT(D0)    0
        // B122 / ESPWM         1
        // B121 / TMGRST        1
        // B120 / DSPRPT        1:
        0x87,
        // B119 / BLANK
        // and 7 bits of global brightness correction:
        0x08,
        // HERE WE SWITCH TO 7-BIT SPI.
        // The following index is 18:
        0x4F, // rgb0
        0x7F,
        0x7F,
        0x4F, // rgb1
        0x7F,
        0x7F,
        0x4F, // rgb2
        0x7F,
        0x7F,
        0x4F, // rgb3
        0x7F,
        0x7F,
        0x4F, // rgb4
        0x7F,
        0x7F,
        0x7F, // tx
};

void tlc_set_gs() {
    if (tlc_send_type != TLC_SEND_IDLE)
        return; // TODO: let's try to make sure this never happens.
    tlc_send_type = TLC_SEND_TYPE_GS;
    tlc_tx_index = 0;
    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, TLC_THISISGS);
}

void tlc_set_fun() {
    while (tlc_send_type != TLC_SEND_IDLE); // shouldn't ever actually have to block on this please.
    tlc_send_type = TLC_SEND_TYPE_FUN;
    tlc_tx_index = 0;
    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, TLC_THISISFUN);
}

// Stage the blank bit:
void tlc_stage_blank(uint8_t blank) {
    if (blank) {
        fun_base[17] |= BIT7;
        fun_base[16] &= ~BIT1;
    } else {
        fun_base[17] &= ~BIT7;
        fun_base[16] |= BIT1;
    }
}

uint8_t tlc_loopback_data_out = 0x00;
uint8_t tlc_loopback_data_in = 0x00;

// Test the TLC chip with a shift-register loopback.
// Returns 0 for success and 1 for failure.
uint8_t tlc_test_loopback(uint8_t test_pattern) {
    // Send the test pattern 34 times, and expect to receive it shifted
    // a bit.
    tlc_loopback_data_out = test_pattern;
    while (tlc_send_type != TLC_SEND_IDLE); // I don't see this happening...

    EUSCI_A_SPI_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
    EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);

    tlc_send_type = TLC_SEND_TYPE_LB;
    tlc_tx_index = 0;
    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, test_pattern);
    // Spin while we send and receive:
    while (tlc_send_type != TLC_SEND_IDLE);

    EUSCI_A_SPI_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);

    return tlc_loopback_data_in != (uint8_t) ((test_pattern << 7) | (test_pattern >> 1));
}

// Stage global brightness if different from default:
void tlc_stage_bc(uint8_t bc) {
    bc = bc & 0b01111111; // Mask out BLANK just in case.
    fun_base[17] &= 0b10000000;
    fun_base[17] |= bc;
}

void init_tlc() {
    // TODO: Shouldn't really need to do this:
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 &= ~UC7BIT;
    UCA0CTLW0 &= ~UCSWRST;

    EUSCI_A_SPI_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
    EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);

    tlc_stage_blank(1);
    tlc_set_fun();
}

void tlc_load_colors() {
    // Load index 0 no matter what.
    memcpy(&tlc_curr_colors[0], &tlc_curr_anim[tlc_light_offset], sizeof(rgbcolor_t));
    memcpy(&rainbow_ring_dest[0], &tlc_curr_anim[(tlc_light_offset + 1) % tlc_curr_anim_len], sizeof(rgbcolor_t));

    rainbow_ring_step[0].red = ((int_fast32_t) rainbow_ring_dest[0].red - tlc_curr_colors[0].red) / ring_fade_steps;
    rainbow_ring_step[0].green = ((int_fast32_t) rainbow_ring_dest[0].green - tlc_curr_colors[0].green) / ring_fade_steps;
    rainbow_ring_step[0].blue = ((int_fast32_t) rainbow_ring_dest[0].blue - tlc_curr_colors[0].blue) / ring_fade_steps;

    if (led_anim_mode == TLC_ANIM_MODE_SHIFT) {
        for (uint8_t i=1; i<5; i++) {
            memcpy(&tlc_curr_colors[i], &tlc_curr_anim[(tlc_light_offset + 1) % tlc_curr_anim_len], sizeof(rgbcolor_t));
            memcpy(&rainbow_ring_dest[i], &tlc_curr_anim[(tlc_light_offset + i + 1) % tlc_curr_anim_len], sizeof(rgbcolor_t));
            rainbow_ring_step[i].red = ((int_fast32_t) rainbow_ring_dest[i].red - tlc_curr_colors[i].red);
            rainbow_ring_step[i].green = ((int_fast32_t) rainbow_ring_dest[i].green - tlc_curr_colors[i].green);
            rainbow_ring_step[i].blue = ((int_fast32_t) rainbow_ring_dest[i].blue - tlc_curr_colors[i].blue);
        }
    }
}

void tlc_fade_colors() {
    // TODO: there's a bit of resiliency we could add here to avoid the
    // potential for odd behavior caused by roundoff errors.

    // Load index 0 no matter what.
    tlc_curr_colors[0].red += rainbow_ring_step[0].red;
    tlc_curr_colors[0].green += rainbow_ring_step[0].green;
    tlc_curr_colors[0].blue += rainbow_ring_step[0].blue;

    // If we're shifting (i.e. using the rest of the buffer), then do the rest
    if (led_anim_mode == TLC_ANIM_MODE_SHIFT) {
        for (uint8_t i=1; i<5; i++) {
            tlc_curr_colors[i].red += rainbow_ring_step[i].red;
            tlc_curr_colors[i].green += rainbow_ring_step[i].green;
            tlc_curr_colors[i].blue += rainbow_ring_step[i].blue;
        }
    }
}

void tlc_start_anim(rgbcolor_t *anim, uint8_t anim_len, uint8_t fade_steps, uint8_t all_lights_same) {
    tlc_light_offset = 0;
    if (all_lights_same)
        led_anim_mode = TLC_ANIM_MODE_SAME;
    else
        led_anim_mode = TLC_ANIM_MODE_SHIFT;
    tlc_curr_anim = anim;
    tlc_curr_anim_len = anim_len;
    ring_fade_steps = fade_steps;
    ring_fade_index = 0;
    if (all_lights_same) {
        led_anim_mode = TLC_ANIM_MODE_SAME;
    } else {
        led_anim_mode = TLC_ANIM_MODE_SHIFT;
    }
    // TODO: looping?
    tlc_load_colors();
}

void tlc_timestep() {
    if (led_anim_mode == TLC_ANIM_MODE_IDLE) {
        return;
    }

    // Display currently loaded frame:
    tlc_set_gs();

    // Now compute the next frame:
    // If we're shifting, we use all of tlc_curr_colors;
    //  if they're all the same, we only care about index 0.

    // Start by incrementing the between-frame index.
    ring_fade_index++;

    if (ring_fade_index >= ring_fade_steps) {
        // If we're done cycling between frames, then we can increment the shift.
        ring_fade_index = 0;
        tlc_light_offset++;

        // If the shift will overflow, we're finished.
        // TODO: unless we're looping?
        if (tlc_light_offset == tlc_curr_anim_len) {
//            led_anim_mode = TLC_ANIM_MODE_IDLE;
            tlc_light_offset = 0;
        }
        // load the next color sets.
        tlc_load_colors();
    } else { // Otherwise, we're still fading...
        // Compute the next step fade.
        tlc_fade_colors();
    }
}

#pragma vector=USCI_A0_VECTOR
__interrupt void EUSCI_A0_ISR(void)
{
    switch (__even_in_range(UCA0IV, 4)) {
    //Vector 2 - RXIFG
    case 2:
        // We received some garbage sent to us while we were sending.
        if (tlc_send_type == TLC_SEND_TYPE_LB) {
            // We're only interested in it if we're doing a loopback test.
            tlc_loopback_data_in = EUSCI_B_SPI_receiveData(EUSCI_A0_BASE);
        } else {
            EUSCI_B_SPI_receiveData(EUSCI_A0_BASE); // Throw it away.
        }
        break; // End of RXIFG ///////////////////////////////////////////////////////

    case 4: // Vector 4 - TXIFG : I just sent a byte.
        if (tlc_send_type == TLC_SEND_TYPE_GS) {
            if (tlc_tx_index == 0) { // txl, msb
                EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_tx_light >> 8));
            } else if (tlc_tx_index == 1) { // txl, lsb
                EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_tx_light & 0x00ff));
                rgb_element_index = 0;
//                tlc_color_index = tlc_light_offset % tlc_curr_colors_len;
                tlc_color_index = 0;
            } else if (tlc_tx_index == 32) { // done
                GPIO_pulse(TLC_LATPORT, TLC_LATPIN); // LATCH.
                tlc_send_type = TLC_SEND_IDLE;
                break;
            } else { // gs
                switch(rgb_element_index % 6) { // WATCH OUT! Weird order below:
                case 1:
                    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_curr_colors[tlc_color_index].blue & 0x00ff));
                    break;
                case 0:
                    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_curr_colors[tlc_color_index].blue >> 8));
                    break;
                case 3:
                    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_curr_colors[tlc_color_index].green & 0x00ff));
                    break;
                case 2:
                    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_curr_colors[tlc_color_index].green >> 8));
                    break;
                case 4:
                    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_curr_colors[tlc_color_index].red >> 8));
                    break;
                case 5:
                    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, (uint8_t) (tlc_curr_colors[tlc_color_index].red & 0x00ff));

                    if (led_anim_mode == TLC_ANIM_MODE_SHIFT) {
                        tlc_color_index++;
                    } // otherwise we keep the same color for all the RGB LEDs.

                    break;
                }
                rgb_element_index++;
            }
            tlc_tx_index++;
        } else if (tlc_send_type == TLC_SEND_TYPE_FUN) {
            if (tlc_tx_index == 18) { // after 18 we have to switch to 7-bit mode.
                UCA0CTLW0 |= UCSWRST;
                UCA0CTLW0 |= UC7BIT;
                UCA0CTLW0 &= ~UCSWRST;
                EUSCI_A_SPI_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
                EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
            } else if (tlc_tx_index == 34) {
                GPIO_pulse(TLC_LATPORT, TLC_LATPIN); // LATCH.
                UCA0CTLW0 |= UCSWRST;
                UCA0CTLW0 &= ~UC7BIT;
                UCA0CTLW0 &= ~UCSWRST;
                EUSCI_A_SPI_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
                EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
                tlc_send_type = TLC_SEND_IDLE;
                break;
            }
            EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, fun_base[tlc_tx_index]);
            tlc_tx_index++;
        } else if (tlc_send_type == TLC_SEND_TYPE_LB) { // Loopback for POST
            if (tlc_tx_index == 33) { // TODO: make sure this is the right #.
                tlc_send_type = TLC_SEND_IDLE;
                break;
            }
            EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, tlc_loopback_data_out);
            tlc_tx_index++;
        } else {
            tlc_send_type = TLC_SEND_IDLE; // TODO: probably shouldn't reach.
        }
        break; // End of TXIFG /////////////////////////////////////////////////////

    default: break;
    } // End of ISR flag switch ////////////////////////////////////////////////////
}

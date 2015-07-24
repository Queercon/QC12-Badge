/*
 * leds.h
 * (c) 2014 George Louthan
 * 3-clause BSD license; see license.md.
 */

#ifndef LEDS_H_
#define LEDS_H_

typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} rgbcolor_t;

typedef struct {
    int_fast32_t red;
    int_fast32_t green;
    int_fast32_t blue;
} rgbdelta_t;

typedef struct {
    rgbcolor_t ** colors;
    uint8_t len;
} tlc_animation_t;

extern rgbcolor_t rainbow2[];

void init_tlc();

uint8_t tlc_test_loopback(uint8_t);

void tlc_set_gs();
void tlc_set_fun();
void tlc_stage_blank(uint8_t);
void tlc_start_anim(rgbcolor_t *, uint8_t, uint8_t, uint8_t, uint8_t);
void tlc_stop_anim(uint8_t blank);

void tlc_timestep();

#endif /* LEDS_H_ */

// System includes:
#include <stdint.h>
#include <grlib.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Grace includes:
#include <ti/mcu/msp430/Grace.h>

// Project includes:
#include <qc12_oled.h>
#include "img.h"
#include "qc12.h"
#include "radio.h"
#include "leds.h"
#include "oled.h"
#include "flash.h"

// Interrupt flags:
volatile uint8_t f_bl = 0;
volatile uint8_t f_br = 0;
volatile uint8_t f_bs = 0;
volatile uint8_t f_time_loop = 0;
volatile uint8_t f_new_second = 0;
volatile uint8_t f_rfm_rx_done = 0;
volatile uint8_t f_rfm_tx_done = 0;
volatile uint8_t s_new_minute = 0;

// Non-interrupt signal flags (no need to avoid optimization):
uint8_t s_default_conf_loaded = 0;
uint8_t s_need_rf_beacon = 0;
uint8_t s_newly_met = 0;

void poll_buttons();

#pragma DATA_SECTION (my_conf, ".infoA");
#pragma DATA_SECTION (default_conf, ".infoB");

qc12conf my_conf;
const qc12conf backup_conf;

const qc12conf default_conf = {
        0,     // id
        50,    // mood
        0,     // title
        0,     // flag
        0,     // flag_cooldown
        0,     // exp
        {0},   // achievements
        0,
};

// Gaydar:
uint8_t window_position = 0; // Currently only used for restarting radio & skipping windows.
uint8_t skip_window = 1;
uint8_t neighbor_count = 0;
uint8_t window_seconds = RECEIVE_WINDOW_LENGTH_SECONDS;
uint8_t neighbor_badges[BADGES_IN_SYSTEM] = {0};

const char titles[][10] = {
        "n00b",
        "UBER",
        "Spastic",
        "Bored",
        "Socialite",
};

// In the "modal" sense:
uint8_t op_mode = OP_MODE_IDLE;

uint8_t suppress_softkey = 0;
uint16_t softkey_en = BIT0 | BIT1 | BIT4 | BIT6 | BIT7;

const char sk_labels[SK_SEL_MAX+1][10] = {
       "Play",
       "ASL?",
       "Befriend",
       "Wave flag",
       "Demo anim",
//       "Pick flag",
       "RPS",
       "Set name",
       "Sleep"
};

uint16_t badges_seen[BADGES_IN_SYSTEM];
uint8_t fav_badges_ids[FAVORITE_COUNT] = {0};
char fav_badges_handles[FAVORITE_COUNT][NAME_MAX_LEN];
uint8_t fav_badge_minute_countdown = FAVORITE_COUNTDOWN_MINUTES;

void my_conf_write_crc() {
    CRC_setSeed(CRC_BASE, 0x0C12);
    for (uint8_t i = 0; i < sizeof(qc12conf) - 2; i++) {
        CRC_set8BitData(CRC_BASE, ((uint8_t *) &default_conf)[i]);
    }
    my_conf.crc16 = CRC_getResult(CRC_BASE);
}

void check_conf() {
    CRC_setSeed(CRC_BASE, 0x0C12);
    for (uint8_t i = 0; i < sizeof(qc12conf) - 2; i++) {
        CRC_set8BitData(CRC_BASE, ((uint8_t *) &default_conf)[i]);
    }

    if (my_conf.crc16 != CRC_getResult(CRC_BASE)) {
        memcpy(&my_conf, &default_conf, sizeof(qc12conf));
        my_conf_write_crc();
        memset(badges_seen, 0, sizeof(uint16_t) * BADGES_IN_SYSTEM);
        s_default_conf_loaded = 1;
        out_payload.handle[0] = 0;
    }
}

void set_badge_seen(uint8_t id) {
    if (id >= BADGES_IN_SYSTEM) {
        return;
    }

    if (!(BADGE_SEEN_BIT & badges_seen[id])) {
        s_newly_met = 1;
        badges_seen[id] |= BADGE_SEEN_BIT;
    }
}

void set_badge_friend(uint8_t id) {
    if (!(id < BADGES_IN_SYSTEM)) {
        return;
    }
    badges_seen[id] |= BADGE_FRIEND_BIT;
}

void tick_badge_seen(uint8_t id, char* handle) {
    if (!(id < BADGES_IN_SYSTEM)) {
        return;
    }
    if (badges_seen[id] & BADGE_TICKS_MASK < BADGE_TICKS_MASK) {
        badges_seen[id]++;
        // do top badges:
        for (uint8_t top_index=0; top_index<FAVORITE_COUNT; top_index++) {
            if (badges_seen[id] & BADGE_TICKS_MASK > badges_seen[fav_badges_ids[top_index]] & BADGE_TICKS_MASK) {
                // This is where it goes, and all the rest need to be demoted.

                // So we can start at the lowest ranked favorite, which is fav_badges_ids[FAVORITE_COUNT-1]
                //  and clobber it with its superior until one IS CLOBBERED BY top_index.
                //  Then we replace top_id's original spot with id.

                for (uint8_t index_to_clobber = FAVORITE_COUNT-1; index_to_clobber>top_index; index_to_clobber--) {
                    // index_to_clobber starts at the max value for fav_badges_ids
                    // and goes through top_index+1. (index_to_clobber will never be 0).
                    fav_badges_ids[index_to_clobber] = fav_badges_ids[index_to_clobber-1];
                    strcpy(fav_badges_handles[index_to_clobber], fav_badges_handles[index_to_clobber-1]);
                }

                // now clobber top_index with out new one.
                fav_badges_ids[top_index] = id;
                strcpy("???", handle);
            }
        }
    }
}

void init_rtc() {
    RTC_B_definePrescaleEvent(RTC_B_BASE, RTC_B_PRESCALE_1, RTC_B_PSEVENTDIVIDER_4); // 4 => 32 Hz
    RTC_B_clearInterrupt(RTC_B_BASE, RTC_B_CLOCK_READ_READY_INTERRUPT + RTC_B_TIME_EVENT_INTERRUPT + RTC_B_CLOCK_ALARM_INTERRUPT + RTC_B_PRESCALE_TIMER1_INTERRUPT);
    RTC_B_enableInterrupt(RTC_B_BASE, RTC_B_CLOCK_READ_READY_INTERRUPT + RTC_B_TIME_EVENT_INTERRUPT + RTC_B_CLOCK_ALARM_INTERRUPT + RTC_B_PRESCALE_TIMER1_INTERRUPT);
}

void init_payload() {
    out_payload.base_id = NOT_A_BASE;
    out_payload.beacon = 0;
    out_payload.from_addr = my_conf.badge_id;
    out_payload.to_addr = RFM_BROADCAST;
    strcpy(out_payload.handle, my_conf.handle);
}

void init() {
    Grace_init(); // Activate Grace-generated configuration

    check_conf();

    init_payload();
    init_tlc();
    init_radio();
    init_oled();
    init_rtc();
}

// Power-on self test
void post() {
    // If the radio is super-broken, we won't even get here.
    // Check the crystal.
    uint8_t crystal_error = CSCTL5 & LFXTOFFG;

    // Check the shift register of the TLC.
    uint8_t led_error = tlc_test_loopback(0b10101010);
    led_error = led_error || tlc_test_loopback(0b01010101);

    // Check the flash chip.
    tlc_set_fun();
    EUSCI_A_SPI_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
    EUSCI_A_SPI_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
    uint8_t flash_error = flash_post();
    EUSCI_A_SPI_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
    EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
    EUSCI_A_SPI_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
    EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);

    // If we detected no errors, we're done here.
    if (!(flash_error || led_error || crystal_error))
        return;

    // Otherwise, show those errors and then delay for a bit so we can
    // see them.
    GrClearDisplay(&g_sContext);
    GrContextFontSet(&g_sContext, &SYS_FONT);
    uint8_t print_loc = 32;

    GrStringDraw(&g_sContext, "- POST -", -1, 0, 5, 0);

    if (crystal_error) {
        GrStringDraw(&g_sContext, "Err: XTAL!", -1, 0, print_loc, 0);
        print_loc += 12;
    }

    if (led_error) {
        GrStringDraw(&g_sContext, "Err: LED!", -1, 0, print_loc, 0);
        print_loc += 12;
    }

    if (flash_error) {
        GrStringDraw(&g_sContext, "Err: Mem!", -1, 0, print_loc, 0);
        print_loc += 12;
    }

    GrFlush(&g_sContext);

    delay(5000);
}

// Play a cute animation when we first turn the badge on.
void intro() {
    GrImageDraw(&g_sContext, &fingerprint_1BPP_UNCOMP, 0, 0);
    GrStringDrawCentered(&g_sContext, "Queercon", -1, 31, 94 + SYS_FONT_HEIGHT/3, 0);
    GrStringDrawCentered(&g_sContext, "twelve", -1, 31, 94 + SYS_FONT_HEIGHT/3 + SYS_FONT_HEIGHT, 0);
    GrStringDrawCentered(&g_sContext, "- 2015 -", -1, 31, 94 + SYS_FONT_HEIGHT/3 + SYS_FONT_HEIGHT*2, 0);
    GrFlush(&g_sContext);
}

// Shared activities between modes are:
//   Infrastructure service
//   LED actions
//   Character actions

void handle_infrastructure_services() {

    uint8_t s_tick_next_window = 0;
    uint8_t ticking_this_window = 0;
    static uint8_t minute_secs = 60;

    // Handle inbound and outbound background radio functionality, and buttons.
    if (f_time_loop) {
        poll_buttons();
    }
    if (f_rfm_tx_done) {
        f_rfm_tx_done = 0;
        // And return to our normal receive automode:
        // RX->SB->RX on receive.
        mode_sync(RFM_MODE_RX);
        write_single_register(0x3b, RFM_AUTOMODE_RX);
    }

    // Radio RX tasks:
    //    Badge count incrementing
    //    Friendship requests
    //    Base check-ins
    //    Handle sharing (only for our top-3)
    //    Flag scheduling
    //    Animation (OLED play) scheduling
    //    Clock sync???

    if (f_rfm_rx_done) {
        f_rfm_rx_done = 0;
        in_payload.handle[NAME_MAX_LEN-1] = 0; // Make sure it's definitely null-terminated.

        // Increment the badge count if needed:
        if (in_payload.beacon && in_payload.from_addr < BADGES_IN_SYSTEM) {
            // It's a beacon (one per cycle).
            // Increment our beacon count in the current position in our
            // sliding window.
            neighbor_badges[in_payload.from_addr] = RECEIVE_WINDOW;
            set_badge_seen(in_payload.from_addr);
            // Every 10 minutes:
            if (ticking_this_window) {
                tick_badge_seen(in_payload.from_addr, in_payload.handle);
            }
        }

        // Resolve inbound or completed friendship requests:

        // Do base check-ins and related tasks:

        // Schedule and pass on flags:

        // Schedule OLED play animations:

        // Clock stuff???
    }

    if (f_new_second) {
        f_new_second = 0;

        minute_secs--;
        if (!minute_secs) {
            minute_secs = 60;
            s_new_minute = 1;
        }

        window_seconds--;
        if (!window_seconds) {
            window_seconds = RECEIVE_WINDOW_LENGTH_SECONDS;
            if (skip_window != window_position) {
                s_need_rf_beacon = 1;
            }

            if (s_tick_next_window) {
                s_tick_next_window = 0;
                ticking_this_window = 1;
            } else if (ticking_this_window) {
                ticking_this_window = 0;
            }

            neighbor_count = 0;
            for (uint8_t i=0; i<BADGES_IN_SYSTEM; i++) {
                if (neighbor_badges[i]) {
                    neighbor_count++;
                    neighbor_badges[i]--;
                }
            }

            window_position = (window_position + 1) % RECEIVE_WINDOW;
            if (!window_position) {
                skip_window = rand() % RECEIVE_WINDOW;
            }
            // If we're rolling over the window and have no neighbors,
            // try a radio reboot, in case that can gin up some neighbors
            // for some reason.
            if (!window_position && neighbor_count == 0) {
                init_radio();
            }
        }
    }

    if (s_new_minute) {
        s_new_minute = 0;
        if (my_conf.flag_cooldown) {
            my_conf.flag_cooldown--;
            my_conf_write_crc();
        }

        fav_badge_minute_countdown--;
        if (!fav_badge_minute_countdown) {
            fav_badge_minute_countdown = FAVORITE_COUNTDOWN_MINUTES;
            s_tick_next_window = 1;
        }
    }

    if (s_need_rf_beacon && rfm_reg_state == RFM_REG_IDLE) {
        // If we need to beacon, and we're not talking to the RFM module.
        // Note: Last year we also had a check for
        //  "!(read_single_register_sync(0x27) & (BIT1+BIT0))".
        // That is SyncAddressMatch and AutoMode (i.e. check whether we're
        // receiving something or are in our receive intermediate state.)
        // I'm not sure it added any robustness.
        s_need_rf_beacon = 0;

        out_payload.beacon = 1;
        radio_send_sync();
    }

}

void handle_led_actions() {
    if (f_time_loop) {
//        tlc_timestep();
    }
}

void handle_character_actions() {
    static uint8_t skip_frame = 20;
    if (f_time_loop) {
        skip_frame--;

        if (!skip_frame) {
            skip_frame = 20;
            oled_timestep();
        }
    }
}

void try_to_sleep() {
    // If there are no more flags left to service, go to sleep.
    if (!(f_bl || f_br || f_bs || f_new_second || f_rfm_rx_done || f_rfm_tx_done || f_time_loop)) {
        __bis_SR_register(SLEEP_BITS);
    }
}

const tRectangle name_erase_rect = {0, NAME_Y_OFFSET, 63, NAME_Y_OFFSET + NAME_FONT_HEIGHT + SYS_FONT_HEIGHT};

// Read the badgeholder's name if appropriate:
void handle_mode_name() {
    // Clear the screen and display the instructions.
    GrClearDisplay(&g_sContext);
    GrContextFontSet(&g_sContext, &SYS_FONT);
    GrStringDraw(&g_sContext, "Enter a", -1, 0, 5, 1);
    GrStringDraw(&g_sContext, "name.", -1, 0, 5+SYS_FONT_HEIGHT, 1);
    GrStringDraw(&g_sContext, "Hold", -1, 0, 5+SYS_FONT_HEIGHT*3, 1);
    GrStringDraw(&g_sContext, "middle", -1, 0, 5+SYS_FONT_HEIGHT*4, 1);
    GrStringDraw(&g_sContext, "button", -1, 0, 5+SYS_FONT_HEIGHT*5, 1);
    GrStringDraw(&g_sContext, "to finish.", -1, 0, 5+SYS_FONT_HEIGHT*6, 1);
    GrFlush(&g_sContext);

    // Switch to the NAME font so it's the expected width.
    GrContextFontSet(&g_sContext, &NAME_FONT);

    // Temporary buffer to hold the selected name.
    // (In the event of a power cycle we don't wand to be messing around
    //  with the actual config's handle)
    char name[NAME_MAX_LEN+1] = {' ', 0};
    uint8_t char_entry_index = 0;
    uint8_t curr_char = ' ';

    // String to display under the name; it's just the selection character,
    // configured in qc12.h
    const char undername[2] = {NAME_SEL_CHAR, 0};

    // For figuring out where to put the underline & selection character:
    uint8_t underchar_x = 0;
    uint8_t text_width = 0;
    uint8_t last_char_index = 0;

    // For determining whether name entry is complete:
    uint8_t bs_down_loops = 0;

    while (1) {
        handle_infrastructure_services();
        handle_led_actions();

        if (f_time_loop) {
            f_time_loop = 0;
            // Check for left/right buttons to change character slot
            text_width = GrStringWidthGet(&g_sContext, name, last_char_index+1);
            if (f_bl == BUTTON_RELEASE) {
                if (char_entry_index > 0) {
                    // check for deletion:
                    if (char_entry_index == last_char_index && curr_char == ' ')
                        last_char_index--;
                    char_entry_index--;
                    curr_char = name[char_entry_index];
                }
                f_bl = 0;
            }
            if (f_br == BUTTON_RELEASE) {
                if (char_entry_index < NAME_MAX_LEN && curr_char != ' ' && text_width < 58) {
                    char_entry_index++;
                    if (!name[char_entry_index])
                        name[char_entry_index] = ' ';
                    curr_char = name[char_entry_index];
                    if (char_entry_index > last_char_index)
                        last_char_index = char_entry_index;
                }
                f_br = 0;
            }
            if (f_bs == BUTTON_RELEASE) {
                // Softkey button cycles the current character.
                // This is a massive PITA for the person entering their name,
                // but they only have to do it once so whatever.
                if (curr_char == 'Z') // First comes capital letters
                    curr_char = 'a';
                else if (curr_char == 'z') // Then lower case
                    curr_char = '0';
                else if (curr_char == '9') // Then numbers
                    curr_char = ' ';
                else if (curr_char == ' ') // Then a space, and then we cycle.
                    curr_char = 'A';
                else
                    curr_char++;
                name[char_entry_index] = curr_char;
                f_bs = 0;
                // Since it's released, clear the depressed loop count.
                bs_down_loops = 0;
            } else if ((last_char_index || name[0] != ' ') && f_bs == BUTTON_PRESS) {
                // If we're in a valid state to complete the name entry, and
                // the softkey button is depressed, then it's time to start
                // counting the number of time loops for which it is depressed.
                bs_down_loops = 1;
                f_bs = 0;
            }

            // If we're counting the number of loops for which the softkey is
            // depressed, go ahead and increment it. This is going to do one
            // double-count at the beginning, but I don't care.
            if (bs_down_loops && bs_down_loops < NAME_COMMIT_LOOPS) {
                bs_down_loops++;
            } else if (bs_down_loops) {
                break;
            }

            underchar_x = GrStringWidthGet(&g_sContext, name, char_entry_index);

            // Clear the area:
            GrContextForegroundSet(&g_sContext, ClrBlack);
            GrRectFill(&g_sContext, &name_erase_rect);
            GrContextForegroundSet(&g_sContext, ClrWhite);

            // Rewrite it:
            GrStringDraw(&g_sContext, name, -1, 0, NAME_Y_OFFSET, 1);
            GrLineDrawH(&g_sContext, 0, text_width, NAME_Y_OFFSET+12);
            GrStringDraw(&g_sContext, undername, -1, underchar_x, NAME_Y_OFFSET+13, 1);
            GrFlush(&g_sContext);
        } // end if (f_time_loop)

        try_to_sleep();

    } // end while (1)

    // Commit the name with a correctly placed null termination character..
    uint8_t name_len = 0;
    while (name[name_len] && name[name_len] != ' ')
        name_len++;
    name[name_len] = 0; // null terminate.
    strcpy(my_conf.handle, name);
    strcpy(out_payload.handle, name);
    op_mode = OP_MODE_IDLE;
    suppress_softkey = 1; // And don't register the button release

    GrClearDisplay(&g_sContext);
    GrFlush(&g_sContext);
} // handle_mode_name

uint8_t softkey_enabled(uint8_t index) {
    return ((1<<index) & softkey_en)? 1 : 0;
}

void handle_mode_idle() {
    // Clear any outstanding stray flags asking the character to do stuff
    //    so we know we're in a consistent state when we enter this mode.
    static uint8_t softkey_sel;
    softkey_sel = 0;
    uint8_t s_new_pane = 0;

    oled_draw_pane(softkey_sel);
    // Pick our current appearance...
    oled_play_animation(&standing, 0);
    oled_anim_next_frame();

    while (1) {
        handle_infrastructure_services();
        handle_led_actions();
        handle_character_actions();

        if (f_time_loop) {
            f_time_loop = 0;
            if (f_br == BUTTON_PRESS) {
                // Left button
                do {
                    softkey_sel = (softkey_sel+1) % (SK_SEL_MAX+1);
                } while (!softkey_enabled(softkey_sel));
                s_new_pane = 1;
            }
            f_br = 0;

            if (f_bl == BUTTON_PRESS) {
                do {
                    softkey_sel = (softkey_sel+SK_SEL_MAX) % (SK_SEL_MAX+1);
                } while (!softkey_enabled(softkey_sel));
                s_new_pane = 1;
            }
            f_bl = 0;

            if (f_bs == BUTTON_RELEASE) {
                // Select button
                switch (softkey_sel) {
                case SK_SEL_ASL:
                    break;
                case SK_SEL_SETFLAG: // TEMPORARILY: anim demo:
                    op_mode = OP_MODE_SETFLAG;
                    break;
                case SK_SEL_NAME:
                    op_mode = OP_MODE_NAME;
                    break;
                case SK_SEL_PLAY:
                    tlc_start_anim(&flag_pink, 0, 3, 0, 3);
                    break;
                case SK_SEL_FLAG:
                    break;
                case SK_SEL_RPS:
                    break;
                case SK_SEL_FRIEND:
                    break;
                case SK_SEL_SLEEP:
                    op_mode = OP_MODE_SLEEP;
                    break;
                default:
                    __never_executed();
                }
            }
            f_bs = 0;
        }

        if (s_new_pane) {
            // Title or softkey or something changed:
            s_new_pane = 0;
            oled_draw_pane(softkey_sel);
        }

        if (op_mode != OP_MODE_IDLE) {
            break; // Escape the loop!
        }

        try_to_sleep();

    }

    f_bs = 0;

    // Cleanup:

    tlc_stop_anim(1);
}

void handle_mode_asl() {
    // Radio does background stuff but character actions ignored.
    // TLC continues to animate.
    while (1) {
        handle_infrastructure_services();
        handle_led_actions();
        try_to_sleep();
    }
}

void handle_mode_sleep() {
    // Sleep the radio.
    mode_sync(RFM_MODE_SL); // Going to sleep... mode...
    write_single_register(0x3b, RFM_AUTOMODE_OFF);
    f_rfm_rx_done = f_rfm_tx_done = 0;


    // Kill the LEDs.
    tlc_stage_blank(1);
    tlc_set_fun();

    // Clear the screen.
    GrClearDisplay(&g_sContext);
    GrFlush(&g_sContext);

    // Set up our Zzz animation.
    static uint8_t zzz_index = 0;

    while (1) {
        if (f_time_loop) {
            f_time_loop = 0;
            poll_buttons();

            if (f_bs == BUTTON_RELEASE) {
                f_bs = 0;
                break;
            }
            f_bs = f_br = f_bl = 0;

        }
        if (f_new_second) {
            f_new_second = 0;
            GrClearDisplay(&g_sContext);
            GrStringDraw(&g_sContext, "Zzz...", zzz_index, 10, 100, 1);
            GrFlush(&g_sContext);
            zzz_index = (zzz_index+1) % 7;
        }
        try_to_sleep(); // This one needs to be different...
    }

    init_radio();
    op_mode = OP_MODE_IDLE;
}

void handle_mode_setflag() {
    static uint8_t softkey_sel;
    softkey_sel = 0;
    uint8_t s_new_pane = 1;

    char buf[2] = "";

    oled_draw_pane(softkey_sel);
    // Pick our current appearance...
    oled_play_animation(&standing, 0);
    oled_anim_next_frame();

    while (1) {
        handle_infrastructure_services();
        handle_led_actions();
        handle_character_actions();

        if (f_time_loop) {
            f_time_loop = 0;
            if (f_br == BUTTON_PRESS) {
                softkey_sel = (softkey_sel+1) % (demo_anim_count+1);
                s_new_pane = 1;
            }
            f_br = 0;

            if (f_bl == BUTTON_PRESS) {
                softkey_sel = (softkey_sel+demo_anim_count) % (demo_anim_count+1);
                s_new_pane = 1;
            }
            f_bl = 0;

            if (f_bs == BUTTON_RELEASE) {
                f_bs = 0;
                // Select button
                if (softkey_sel == demo_anim_count) {
                    op_mode = OP_MODE_IDLE;
                    break;
                } else {
                    oled_play_animation(demo_anims[softkey_sel], 3);
                }
            }
            f_bs = 0;
        }

        if (s_new_pane) {
            // softkey or something changed:
            s_new_pane = 0;
            GrContextFontSet(&g_sContext, &SOFTKEY_LABEL_FONT);
            GrStringDrawCentered(&g_sContext, "                ", -1, 31, 127 - SOFTKEY_FONT_HEIGHT/2, 1);

            if (softkey_sel == demo_anim_count) {
                GrStringDrawCentered(&g_sContext, "Done", -1, 32, 127 - SOFTKEY_FONT_HEIGHT/2, 1);
            } else {
                buf[0] = 'A' + softkey_sel;
                buf[1] = 0;
                GrStringDrawCentered(&g_sContext, buf, 1, 32, 127 - SOFTKEY_FONT_HEIGHT/2, 1);
            }

            GrLineDrawH(&g_sContext, 0, 64, 116);
            GrFlush(&g_sContext);
        }

        try_to_sleep();

    }
    f_bs = 0;

    // Cleanup:

    tlc_stop_anim(1);
}

void handle_mode_rps() {
    while (1) {
        handle_infrastructure_services();
        try_to_sleep();
    }
}

int main(void)
{
    init();
    intro(); // Play a cute animation when we first turn the badge on.
    delay(1000);
    post();

    GrClearDisplay(&g_sContext);

    if (!my_conf.handle[0] || s_default_conf_loaded) { // Name is not set:
        op_mode = OP_MODE_NAME;
        s_default_conf_loaded = 0;
    }

    while (1) {
        switch(op_mode) {
        case OP_MODE_IDLE:
            handle_mode_idle();
            break;
        case OP_MODE_NAME:
            handle_mode_name(); // Learn the badge's name (if we don't have it already)
            break;
        case OP_MODE_ASL:
            handle_mode_asl();
            break;
        case OP_MODE_SETFLAG:
            handle_mode_setflag();
            break;
        case OP_MODE_BEFRIEND:
            break;
        case OP_MODE_SLEEP:
            handle_mode_sleep();
            break;
        }

        // Reset user-interactive flags after switching modes:
        f_bl = f_br = f_bs = 0;

    } // loop forever. Sleeping is done inside the mode handlers.
} // main

void poll_buttons() {

    static uint8_t bl_read_prev = 1;
    static uint8_t bl_read = 1;
    static uint8_t bl_state = 1;

    static uint8_t br_read_prev = 1;
    static uint8_t br_read = 1;
    static uint8_t br_state = 1;

    static uint8_t bs_read_prev = 1;
    static uint8_t bs_read = 1;
    static uint8_t bs_state = 1;

    // Poll the buttons two time loops in a row to debounce and
    // if there's a change, raise a flag.
    // Left button:
    bl_read = GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN6);
    if (bl_read == bl_read_prev && bl_read != bl_state) {
        f_bl = bl_read? BUTTON_RELEASE : BUTTON_PRESS; // active low
        bl_state = bl_read;
    }
    bl_read_prev = bl_read;

    // Softkey button:
    bs_read = GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN5);
    if (bs_read == bs_read_prev && bs_read != bs_state) {
        if (suppress_softkey) {
            // suppress_softkey means we don't generate a flag for the next
            // release (or press, I guess, but we mostly care about releases.)
            suppress_softkey = 0;
        } else {
            f_bs = bs_read? BUTTON_RELEASE : BUTTON_PRESS; // active low
        }
        bs_state = bs_read;
    }
    bs_read_prev = bs_read;

    // Right button:
    br_read = GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN4);
    if (br_read == br_read_prev && br_read != br_state) {
        f_br = br_read? BUTTON_RELEASE : BUTTON_PRESS; // active low
        br_state = br_read;
    }
    br_read_prev = br_read;
} // poll_buttons

#pragma vector=RTC_VECTOR
__interrupt
void RTC_A_ISR(void) {
    switch (__even_in_range(RTCIV, 16)) {
    case 0: break;  //No interrupts
    case 2:         //RTCRDYIFG
        f_new_second = 1;
        __bic_SR_register_on_exit(SLEEP_BITS);
        break;
    case 4:         //RTCEVIFG
        //Interrupts every minute - ignored.
        break;
    case 6:         //RTCAIFG
        // Alarm!
        break;
    case 8: break;  //RT0PSIFG
    case 10:		// Rollover of prescale counter
        f_time_loop = 1; // We know what it does! It's a TIME LOOP MACHINE.
        // ...who would build a device that loops time every 32 milliseconds?
        // WHO KNOWS. But that's what it does.
        tlc_timestep();
        __bic_SR_register_on_exit(SLEEP_BITS);
        break; //RT1PSIFG
    case 12: break; //Reserved
    case 14: break; //Reserved
    case 16: break; //Reserved
    default: break;
    }
} // RTC_A_ISR

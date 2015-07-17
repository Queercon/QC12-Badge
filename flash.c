/*
 * flash.c
 *
 *  Created on: Jun 25, 2015
 *      Author: George
 */

/*
 *   Flash chip (S25FL216K)
 *        (write on rise, change on fall,
 *         CS active low, clock inactive low, MSB first)
 *        eUSCI_A0 (shared)
 *        somi, miso, clk
 *        CS#       P1.1 (normally high - selected when low)
 *        WP#       P3.0 (normally high - write protect when low)
 *        HOLD#     P1.0 (normally high - held when low)
 *        TODO: DMA: https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/235583
 */

#include <stdint.h>
#include <qc12.h>

// todo: remove
#include <oled.h>
#include <stdio.h>

const uint8_t FLASH_STATE_WREN  = BIT1;
const uint8_t FLASH_STATE_SLEEP = BIT2;
const uint8_t FLASH_STATE_BUSY  = BIT3;
const uint8_t FLASH_STATE_HOLD  = BIT4;

uint8_t flash_state = 0;

uint8_t flash_status_register = 0;

const uint8_t FLASH_SR_WIP = BIT1;

const uint8_t FLASH_CMD_WREN = 0x06;
const uint8_t FLASH_CMD_WRDIS = 0x04;
const uint8_t FLASH_CMD_READ_STATUS = 0x05;
const uint8_t FLASH_CMD_WRITE_STATUS = 0x01;
const uint8_t FLASH_CMD_READ_DATA = 0x03;
const uint8_t FLASH_CMD_PAGE_PROGRAM = 0x02;
const uint8_t FLASH_CMD_BLOCK_ERASE = 0xD8;
const uint8_t FLASH_CMD_SECTOR_ERASE = 0x20;
const uint8_t FLASH_CMD_CHIP_ERASE = 0xC7; // TODO: delay after these three:
const uint8_t FLASH_CMD_POWER_DOWN = 0xB9;
const uint8_t FLASH_CMD_POWER_UP = 0xAB;

void usci_a0_send_sync(uint8_t data) {
    // If we're held, wait:
    // TODO: don't busy wait.
    while (flash_state & FLASH_STATE_HOLD);

//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT));
    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, data);
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
              EUSCI_A_SPI_TRANSMIT_INTERRUPT));
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
            EUSCI_A_SPI_RECEIVE_INTERRUPT));
    EUSCI_A_SPI_receiveData(EUSCI_A0_BASE); // Throw away the stale garbage we got while sending.
}

uint8_t usci_a0_recv_sync(uint8_t data) {
    // If we're held, wait:
    // TODO: don't busy wait.
    while (flash_state & FLASH_STATE_HOLD);
//    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
//              EUSCI_A_SPI_TRANSMIT_INTERRUPT));
    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, data);
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
              EUSCI_A_SPI_TRANSMIT_INTERRUPT));
    while (!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
            EUSCI_A_SPI_RECEIVE_INTERRUPT));
    return EUSCI_A_SPI_receiveData(EUSCI_A0_BASE);
}

void flash_begin() {
    // If we're held, wait:
    // TODO: don't busy wait.
    while (flash_state & FLASH_STATE_HOLD);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    P1OUT &= ~BIT1; // CS low, select.
}

void flash_end() {
    // TODO:
    while (flash_state & FLASH_STATE_HOLD);
    P1OUT |= BIT1; // CS high, deselect.
}

void flash_simple_cmd(uint8_t cmd) {
    flash_begin();
    usci_a0_send_sync(cmd);
    flash_end();
}

void flash_wr_en() {
    flash_simple_cmd(FLASH_CMD_WREN);
}

void flash_wr_dis() {
    flash_simple_cmd(FLASH_CMD_WRDIS);
}

uint8_t flash_get_status() {
    flash_begin();
    usci_a0_send_sync(FLASH_CMD_READ_STATUS);
    uint8_t retval = usci_a0_recv_sync(0xff);
    flash_end();
    flash_status_register = retval;
    return retval;
}

uint8_t flash_set_status(uint8_t status) {
    flash_begin();
    usci_a0_send_sync(FLASH_CMD_WRITE_STATUS);
    usci_a0_send_sync(status);
    flash_end();
    return flash_get_status() == status;
}

void flash_block_while_wip() {
    // Make sure nothing is in progress:
    while (flash_get_status() & FLASH_SR_WIP)
        __delay_cycles(100); // TODO: this number came from nowhere.
}

void flash_read_data(uint32_t address, uint32_t len_bytes, uint8_t* buffer) {
    flash_block_while_wip();
    flash_begin();
    usci_a0_send_sync(FLASH_CMD_READ_DATA);
    usci_a0_send_sync((address & 0x00FF0000) >> 16); // MSByte of address
    usci_a0_send_sync((address & 0x0000FF00) >> 8); // Middle byte of address
    usci_a0_send_sync((address & 0x000000FF)); // LSByte of address
    // TODO: should be done with DMA, probably.
    for (uint32_t i = 0; i < len_bytes; i++) {
        buffer[i] = usci_a0_recv_sync(0xff);
    }
    flash_end();
}

void flash_write_data(uint32_t address, uint8_t len_bytes, uint8_t* buffer) {
    // Length may not be any longer than 255.
    flash_block_while_wip();
    flash_begin();
    usci_a0_send_sync(FLASH_CMD_PAGE_PROGRAM);
    usci_a0_send_sync((address & 0x00FF0000) >> 16); // MSByte of address
    usci_a0_send_sync((address & 0x0000FF00) >> 8); // Middle byte of address
    usci_a0_send_sync((address & 0x000000FF)); // LSByte of address
    // TODO: should be done with DMA, probably.
    for (uint8_t i = 0; i < len_bytes; i++) {
        usci_a0_send_sync(buffer[i]);
    }
    flash_end();
}

void flash_erase_chip() {
    flash_block_while_wip();
    flash_simple_cmd(FLASH_CMD_CHIP_ERASE);
}

void flash_erase_block_64kb(uint32_t address) {
    flash_block_while_wip();
    flash_begin();
    usci_a0_send_sync(FLASH_CMD_BLOCK_ERASE);
    usci_a0_send_sync((address & 0x00FF0000) >> 16); // MSByte of address
    usci_a0_send_sync((address & 0x0000FF00) >> 8); // Middle byte of address
    usci_a0_send_sync((address & 0x000000FF)); // LSByte of address
    flash_end();
}

void flash_erase_sector_4kb(uint32_t address) {
    flash_block_while_wip();
    flash_begin();
    usci_a0_send_sync(FLASH_CMD_SECTOR_ERASE);
    usci_a0_send_sync((address & 0x00FF0000) >> 16); // MSByte of address
    usci_a0_send_sync((address & 0x0000FF00) >> 8); // Middle byte of address
    usci_a0_send_sync((address & 0x000000FF)); // LSByte of address
    flash_end();
}

void flash_sleep() {
    flash_block_while_wip();
    flash_simple_cmd(FLASH_CMD_POWER_DOWN);
}

void flash_wake() {
    flash_simple_cmd(FLASH_CMD_POWER_UP);
    // TODO: delay 3 us
    __delay_cycles(30);
}

void init_flash() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
    flash_state = 0;
    flash_get_status();
}

void usci_a_send(uint16_t base, uint8_t data) {
    EUSCI_A_SPI_transmitData(base, data);
    while (!EUSCI_A_SPI_getInterruptStatus(base,
            EUSCI_A_SPI_TRANSMIT_INTERRUPT));
    while (EUSCI_A_SPI_isBusy(base));
}

// Do a test of the flash.
// Returns 1 if a problem is detected.
uint8_t flash_post() {
    volatile uint8_t status;
    volatile uint8_t initial_status = flash_get_status(); // TODO...
    initial_status = flash_get_status();
    flash_wr_en();
    status = flash_get_status();
    if (status != (initial_status | BIT1))
        return 1;
    flash_wr_dis();
    status = flash_get_status();
    if (status != (initial_status & ~BIT1))
        return 1;
    return 0;
}

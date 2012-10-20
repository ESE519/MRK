/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Paul M. Gurniak
*******************************************************************************/

#ifndef MRF24J40
#define MRF24J40

#include "lpc17xx.h"

// ------ MRF24J40 Address Map ------

// Short (6-bit) register address map
#define RXMCR       0x00
#define PANIDL      0x01
#define PANIDH      0x02
#define SADRL       0x03
#define SADRH       0x04
#define EADR0       0x05
#define EADR1       0x06
#define EADR2       0x07
#define EADR3       0x08
#define EADR4       0x09
#define EADR5       0x0A
#define EADR6       0x0B
#define EADR7       0x0C
#define RXFLUSH     0x0D

#define ORDER       0x10
#define TXMCR       0x11
#define ACKTMOUT     0x12
#define ESLOTG1     0x13
#define SYMTICKL    0x14
#define SYMTICKH    0x15
#define PACON0      0x16
#define PACON1      0x17
#define PACON2      0x18
#define TXBCON0     0x1A
#define TXNCON      0x1B
#define TXG1CON     0x1C
#define TXG2CON     0x1D
#define ESLOTG23    0x1E
#define ESLOTG45    0x1F

#define ESLOTG67    0x20
#define TXPEND      0x21
#define WAKECON     0x22
#define FRMOFFSET   0x23
#define TXSTAT      0x24
#define TXBCON1     0x25
#define GATECLK     0x26
#define TXTIME      0x27
#define HSYMTMRL    0x28
#define HSYMTMRH    0x29
#define SOFTRST     0x2A
#define SECCON0     0x2C
#define SECCON1     0x2D
#define TXSTBL      0x2E

#define RXSR        0x30
#define INTSTAT     0x31
#define INTCON      0x32
#define GPIO        0x33
#define TRISGPIO    0x34
#define SLPACK      0x35
#define RFCTL       0x36
#define SECCR2      0x37
#define BBREG0      0x38
#define BBREG1      0x39
#define BBREG2      0x3A
#define BBREG3      0x3B
#define BBREG4      0x3C
#define BBREG6      0x3E
#define CCAEDTH     0x3F
// End short address map

// Long (10-bit) register address map
#define TXNFIFO     0x000
#define RXFIFO      0x300

#define RFCON0      0x200
#define RFCON1      0x201
#define RFCON2      0x202
#define RFCON3      0x203
#define RFCON5      0x205
#define RFCON6      0x206
#define RFCON7      0x207
#define RFCON8      0x208

#define SLPCON0     0x211
#define SLPCON1     0x220

// End long address map

// End address map

// ------ Helper defines ------
#define MRF_RSSI_THOLD	127			// TODO: Paul Gurniak, fix this to become dynamic
#define CCA_IS_1				(mrf_rssi() > MRF_RSSI_THOLD)

// End Helpers

// ------ SPI interface ------
void mrf_spi_init(uint8_t dataSize, uint8_t clkMode, int frequency);
uint16_t mrf_spi_write(uint16_t data);
// End SPI interface

// ------ GPIO interface ------
void mrf_gpio_init(void);
void mrf_cs_set(uint8_t val);
void mrf_reset_set(uint8_t val);
// End GPIO interface

// ------ Device interface ------
void mrf_write_short(uint8_t addr, uint8_t data);
uint8_t mrf_read_short(uint8_t addr);
void mrf_write_long(uint16_t addr, uint8_t data);
uint8_t mrf_read_long(uint16_t addr);
// End Device interface

// ------ Debug interface ------
// void mrf_debug(Serial &pc);
// End Debug interface

// ------ MRF24J40 Interface ------
void mrf_init(void);
void mrf_reset(void);
/*
uint8_t mrf_receive(uint8_t *buf, uint8_t max_length);
void mrf_send(uint8_t *data, uint8_t length);
uint8_t mrf_send_addr(uint8_t *data, uint8_t length, uint16_t destAddr, uint16_t destPanid);
*/

void mrf_test_mode(void);
void mrf_data_mode(void);

uint8_t mrf_rssi(void);

void mrf_set_addr(uint16_t addr);
void mrf_set_panid(uint16_t panid);

void mrf_sleep(void);
void mrf_wake(void);

// End MRF24J40 Interface

#endif



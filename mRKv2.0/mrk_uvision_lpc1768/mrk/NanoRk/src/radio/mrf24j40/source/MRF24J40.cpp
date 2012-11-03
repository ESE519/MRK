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

#include "MRF24J40.h"
#include "nrk_timer.h"


// ------ SPI interface ------

void mrf_spi_init(uint8_t dataSize, uint8_t clkMode, int frequency)
{
    LPC_SC->PCONP |= (1 << 21);         // Power on SPI

    LPC_SC->PCLKSEL1 |= (3 << 10);                      // SSP pclk = cclk/8 = 12 MHz
    LPC_SSP0->CPSR = (12000000/frequency) & 0xFE;       // SSP clk = pclk/24 = 500 kHz

    // Disable pull-up/down resistors on SSP0 pins
    LPC_PINCON->PINMODE0 &= ~(3U << 30);
    LPC_PINCON->PINMODE0 |=  (2U << 30);
    LPC_PINCON->PINMODE1 &= ~(0xF << 2);
    LPC_PINCON->PINMODE1 |=  (0xA << 2);

    // Enable SSP0 on p11/p12/p13
    LPC_PINCON->PINSEL0 &= ~(3U << 30);
    LPC_PINCON->PINSEL0 |=  (2U << 30);
    LPC_PINCON->PINSEL1 &= ~(0xF << 2);
    LPC_PINCON->PINSEL1 |=  (0xA << 2);

    // Set data transfer size
    LPC_SSP0->CR0 &= ~(0xF) & 0xFFFF;
    LPC_SSP0->CR0 |= 0xF & (dataSize - 1);

    // Configure clock polarity
    LPC_SSP0->CR0 &= ~(2 << 6) & 0xFFFF;
    LPC_SSP0->CR0 |= 0x3 & (clkMode);

    // Set to master mode
    LPC_SSP0->CR1 &= ~(1 << 2) & 0xF;

    // Enable device
    LPC_SSP0->CR1 |= (1 << 1);
}

uint16_t mrf_spi_write(uint16_t data)
{
    while(!(LPC_SSP0->SR & (1 << 1)));      // Wait for Tx ready
    LPC_SSP0->DR = (data) & 0xFFFF;         // Perform write
     while(!(LPC_SSP0->SR & (1 << 2)));      // Wait for Rx ready
    return (LPC_SSP0->DR & 0xFFFF);         // Read value sent by slave
}

// End SPI interface

// ------ GPIO interface ------


volatile uint8_t new_rx_data = 0, new_tx_data = 0;

void mrf_gpioint_init(void)
{
    // Configure external interrupts from p22 (P2.4)
    LPC_PINCON->PINMODE4 &= ~(3 << 8);      // Set P2.4 to use Pull-up resistor
    LPC_PINCON->PINSEL4 &= ~(3 << 8);       // Set P2.4 to use GPIO
    LPC_GPIO2->FIODIR &= ~(1 << 4);         // Set P2.4 as input
    LPC_GPIOINT->IO2IntEnR |= (1 << 4);     // Enable rising edge interrupts
}

void mrf_gpio_init(void)
{
    // Power on GPIO (should be on by default)
    LPC_SC->PCONP |= (1 << 15);

    // Disable pull-up/down resistors on p14 (P0.16) and p21 (P2.5)
    LPC_PINCON->PINMODE1 &= ~(3 << 0);
    LPC_PINCON->PINMODE1 |=  (2 << 0);
    LPC_PINCON->PINMODE4 &= ~(3 << 10);
    LPC_PINCON->PINMODE4 |=  (2 << 10);

    // Set p14/p21 to use GPIO
    LPC_PINCON->PINSEL1 &= ~(3 << 0);
    LPC_PINCON->PINSEL4 &= ~(3 << 10);

    // Set p14/p21 as output
    LPC_GPIO0->FIODIR |= (1 << 16);
    LPC_GPIO2->FIODIR |= (1 << 5);
    
}

void mrf_cs_set(uint8_t val)
{
    if(val)
        LPC_GPIO0->FIOSET |= (1 << 16);
    else
        LPC_GPIO0->FIOCLR |= (1 << 16);
}

void mrf_reset_set(uint8_t val)
{
    if(val)
        LPC_GPIO2->FIOSET |= (1 << 5);
    else
        LPC_GPIO2->FIOCLR |= (1 << 5);
}

// End GPIO interface


// ------ Address read/write interface

void mrf_write_short(uint8_t addr, uint8_t data)
{
    mrf_cs_set(0);
    nrk_spin_wait_us(1);
    mrf_spi_write(((addr << 1) & 0x7E) | 0x01);
    nrk_spin_wait_us(1);
    mrf_spi_write(data);
    nrk_spin_wait_us(1);
    mrf_cs_set(1);
    nrk_spin_wait_us(1);
}

uint8_t mrf_read_short(uint8_t addr)
{
    uint8_t result;

    mrf_cs_set(0);
    nrk_spin_wait_us(1);
    mrf_spi_write((addr << 1) & 0x7E);
    nrk_spin_wait_us(1);
    result = mrf_spi_write(0xFF);
    nrk_spin_wait_us(1);
    mrf_cs_set(1);
    nrk_spin_wait_us(1);

    return result;
}

void mrf_write_long(uint16_t addr, uint8_t data)
{
    mrf_cs_set(0);
    nrk_spin_wait_us(1);
    mrf_spi_write((addr >> 3) | 0x80);
    nrk_spin_wait_us(1);
    mrf_spi_write(((addr << 5) & 0xE0) | 0x10);
    nrk_spin_wait_us(1);
    mrf_spi_write(data);
    nrk_spin_wait_us(1);
    mrf_cs_set(1);
    nrk_spin_wait_us(1);
}

uint8_t mrf_read_long(uint16_t addr)
{
    uint8_t value;

    mrf_cs_set(0);
    nrk_spin_wait_us(1);
    mrf_spi_write((addr >> 3) | 0x80);
    nrk_spin_wait_us(1);
    mrf_spi_write((addr << 5) & 0xE0);
    nrk_spin_wait_us(1);
    value = mrf_spi_write(0xFF);
    nrk_spin_wait_us(1);
    mrf_cs_set(1);
    nrk_spin_wait_us(1);
		//printf("value=%d",value);
    return value;
}

// End addr r/w interface


// ------ MRF24J40 Interface ------

void mrf_init(void)
{
    mrf_gpio_init();                // Set up p14 and p21
    mrf_spi_init(8, 0, 500000);     // 8-bit transfers, CPOL=0/CPHA=0, sclk = 500kHz
    mrf_reset();                    // Hard and soft reset, configure for Tx/Rx
    
    // Only enable interrupts after the radio is configured
    mrf_gpioint_init();
}

void mrf_reset(void)
{
    mrf_cs_set(1);
    mrf_reset_set(0);
    nrk_spin_wait_us(100);
    mrf_reset_set(1);
    nrk_spin_wait_us(100);

    // Software RF reset
    mrf_write_short(RFCTL, 0x04);
    mrf_write_short(RFCTL, 0x00);

    // Flush Rx FIFO
    mrf_write_short(RXFLUSH, 0x01);

    // Enable and configure radio
    mrf_write_long(RFCON0, 0xE3);   // Set to channel 25 (2.475 GHz)
    mrf_write_long(RFCON1, 0x02);   // Set VCO param to 2
    mrf_write_long(RFCON2, 0x80);   // Enable PLL
    mrf_write_long(RFCON3, 0x00);   // Max Rx/Tx power
    mrf_write_long(RFCON6, 0x90);   // Enable Tx filter, faster sleep recovery
    mrf_write_long(RFCON7, 0x80);   // Set 100kHz internal clock as slpclk
    mrf_write_long(RFCON8, 0x10);   // Enable VCO
    
    mrf_write_long(SLPCON1, 0x20);  // Disable CLKOUT, set slpclk div to 1

    mrf_write_short(RXMCR, 0x01);   // Ignore address field

    mrf_write_short(BBREG2, 0x80);  // Set CCA mode to energy detect
    mrf_write_short(BBREG6, 0x40);  // Append RSSI to each packet
    mrf_write_short(CCAEDTH, 0x00); // Set RSSI threshold to minimum
    
    // Configure for ACK requests
    mrf_write_short(TXNCON, 0x04);  // Enable ACK request
    
    // Interrupt configuration
    mrf_write_short(INTCON, ~(0x09));  // Enable Rx and Tx(N) interrupts (bits active-low)
    mrf_write_long(SLPCON0, 0x02);  // Set interrupt polarity to rising edge
    
    // Sleep configuration
    mrf_write_short(WAKECON, 0x80); // Enable immediate wake-up
    
    // Software RF reset, with new settings
    mrf_write_short(RFCTL, 0x04);
    mrf_write_short(RFCTL, 0x00);
}

uint16_t myAddr, myPanid;

void mrf_set_addr(uint16_t addr)
{
    // Program address
    mrf_write_short(SADRL, addr & 0xFF);
    mrf_write_short(SADRH, addr >> 8);
    
    mrf_write_short(RXMCR, 0x00);   // Observe address field
    
    myAddr = addr;
}

void mrf_set_panid(uint16_t panid)
{
    // Program PAN id
    mrf_write_short(PANIDL, panid & 0xFF);
    mrf_write_short(PANIDH, panid >> 8);
    myPanid = panid;
    
    // Software RF reset, with new settings
    mrf_write_short(RFCTL, 0x04);
    mrf_write_short(RFCTL, 0x00);
}

/*
void mrf_debug(Serial &pc)
{
    pc.printf("TXMCR\t=0x%X\r\n", mrf_read_short(TXMCR));
    pc.printf("RXMCR\t=0x%X\r\n", mrf_read_short(RXMCR));
    pc.printf("RXFLUSH\t=0x%X\r\n", mrf_read_short(RXFLUSH));
    pc.printf("GPIO\t=0x%X\r\n", mrf_read_short(GPIO));
    pc.printf("TRISGPIO=0x%X\r\n", mrf_read_short(TRISGPIO));
    pc.printf("RFCTL\t=0x%X\r\n", mrf_read_short(RFCTL));
    pc.printf("BBREG2\t=0x%X\r\n", mrf_read_short(BBREG2));
    pc.printf("BBREG6\t=0x%X\r\n", mrf_read_short(BBREG6));

    pc.printf("Long address registers:\r\n");
    pc.printf("RFCTRL0\t=0x%X\r\n", mrf_read_long(RFCON0));
    pc.printf("RFCTRL2\t=0x%X\r\n", mrf_read_long(RFCON2));
    pc.printf("RFCTRL3\t=0x%X\r\n", mrf_read_long(RFCON3));
    pc.printf("RFCTRL6\t=0x%X\r\n", mrf_read_long(RFCON6));
    pc.printf("RFCTRL7\t=0x%X\r\n", mrf_read_long(RFCON7));
    pc.printf("RFCTRL8\t=0x%X\r\n", mrf_read_long(RFCON8));
    pc.printf("CLKINTCR=0x%X\r\n", mrf_read_long(SLPCON0));
    pc.printf("CLCCTRL\t=0x%X\r\n", mrf_read_long(SLPCON1));
}

uint8_t mrf_receive(uint8_t *buf, uint8_t buf_length)
{
    uint8_t length = new_rx_data;
    new_rx_data = 0;
    if(length) {
        uint8_t i;
        for(i = 0; i < buf_length && i < max_length; i++) {
            buf[i ] = rx_buf[i];
        }
    }
    return length;
}

void mrf_send(uint8_t *data, uint8_t length)
{
    uint8_t i;

    mrf_write_long(0x000, 2);
    mrf_write_long(0x000 + 1, length + 2);
    // Throw in a 7-byte bogus header as all 0's
    // Arbitrary data can be misinterpreted as 802.15.4 frame flags
    for(i = 0; i < 2; i++) {
        mrf_write_long(0x000 + 2 + i, 0);
    }
    for(i = 0; i < length; i++) {
        mrf_write_long(0x000 + 4 + i, data[i]);
    }
    mrf_write_short(TXNCON, 0x01);
}

uint8_t mrf_send_addr(uint8_t *data, uint8_t length, uint16_t destAddr, uint16_t destPanid)
{
    uint8_t i;
    
    mrf_write_short(TXNCON, 0x04);  // Enable ACK request
    
    mrf_write_long(0x000, 11);  // 11-byte header
    mrf_write_long(0x000 + 1, length + 11);
    
    mrf_write_long(0x000 + 2, 0x21);     // Data packet, ACK request, no security
    mrf_write_long(0x000 + 3, 0x88);     // src and dest are short addresses
    
    // Sequence number: ignore for now
    mrf_write_long(0x000 + 4, 57);
    
    // Fill out addressing fields
    mrf_write_long(0x000 + 5, destPanid & 0xFF);
    mrf_write_long(0x000 + 6, destPanid >> 8);
    mrf_write_long(0x000 + 7, destAddr & 0xFF);
    mrf_write_long(0x000 + 8, destAddr >> 8);
    mrf_write_long(0x000 + 9, myPanid & 0xFF);
    mrf_write_long(0x000 + 10, myPanid >> 8);
    mrf_write_long(0x000 + 11, myAddr & 0xFF);
    mrf_write_long(0x000 + 12, myAddr >> 8);
        
    // Message data
    for(i = 0; i < length; i++) {
        mrf_write_long(0x000 + 13 + i, data[i]);
    }
    
    // Send message with ACK request
    new_tx_data = 0;
    mrf_write_short(TXNCON, 0x05);  // Send message
        
    // Wait for interrupt
    //while(!(mrf_read_short(INTSTAT) & 0x01));
    while(!(new_tx_data));
    
    return mrf_read_short(TXSTAT) & 0x01;
}
*/

void mrf_test_mode(void)
{
    mrf_write_long(0x22F, 0x08 | 5);    // Put into single-tone test mode
    nrk_spin_wait_us(500);

    // Force RF FSM to Tx state
    mrf_write_short(RFCTL, 0x06);
    nrk_spin_wait_us(192);
    mrf_write_short(RFCTL, 0x02);
    nrk_spin_wait_us(192);
}    
    
void mrf_data_mode(void)
{
        
    mrf_write_long(0x22F, 0x08);        // Put back into default mode
    nrk_spin_wait_us(500);

    // Reset RF FSM to normal operation
    mrf_write_short(RFCTL, 0x04);
    nrk_spin_wait_us(192);
    mrf_write_short(RFCTL, 0x00);
    nrk_spin_wait_us(192);
}

uint8_t mrf_rssi(void)
{         
    uint8_t rssi;       
    mrf_write_short(0x3E, 0x80);
    
    // Wait for result to be ready
    while(!(mrf_read_short(0x3E) & 0x01));
    rssi = mrf_read_long(0x210);
    
    mrf_write_short(0x3E, 0x40);
    return rssi;
}

void mrf_sleep(void)
{
    mrf_write_short(SOFTRST, 0x04);      // Reset power management circuitry
    mrf_write_short(SLPACK, 0x80);      // Set into sleep mode
}

void mrf_wake(void)
{
    // Cycle REGWAKE bit 6 (remember to leave IMMWAKE set)
    mrf_write_short(WAKECON, 0xC0);           
    mrf_write_short(WAKECON, 0x80);
    nrk_spin_wait_us(2*1000);     // 2ms to stabilize main oscillator
}



/*
   Contributors:
   Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
   2015 Dennis Patella www.wasproject.it
*/

/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// **************************************************************************
//
// Description:          *** HAL for Arduino Due ***
//
// **************************************************************************

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL.h"
#include <Wire.h>

#define F_CPU       21000000        // should be factor of F_CPU_TRUE
#define F_CPU_TRUE  84000000        // actual CPU clock frequency

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

uint8_t MCUSR;

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

// disable interrupts
void cli(void)
{
	noInterrupts();
}

// enable interrupts
void sei(void)
{
	interrupts();
}

void _delay_ms (int delay_ms)
{
	
	uint32_t n = delay_ms * (F_CPU_TRUE / 3000000);
        asm volatile(
            "L2_%=_delayMicroseconds:"       "\n\t"
            "subs   %0, #1"                 "\n\t"
            "bge    L2_%=_delayMicroseconds" "\n"
            : "+r" (n) :  
        );

}

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// return free memory between end of heap (or end bss) and whatever is current
int freeMemory()
{
  int free_memory;
  int heap_end = (int)_sbrk(0);

  if(heap_end == 0)
    free_memory = ((int)&free_memory) - ((int)&_ebss);
  else
    free_memory = ((int)&free_memory) - heap_end;

  return free_memory;

}



// --------------------------------------------------------------------------
// eeprom
// --------------------------------------------------------------------------



static bool eeprom_initialised = false;
static uint8_t eeprom_device_address = 0x50;

static void eeprom_init(void)
{
	if (!eeprom_initialised)
	{
		Wire.begin();
		eeprom_initialised = true;
	}
}
bool check_eeprom() {
	byte err;
	eeprom_init();
	Wire.beginTransmission (eeprom_device_address);
    Wire.write((byte)(0 >> 8));   // MSB
	Wire.write((byte)(0 & 0xFF)); // LSB
    err = Wire.endTransmission ();
    if (err == 0)
		return true;
    else
		return false;
}
void eeprom_write_byte(unsigned char *pos, unsigned char value)
{
	unsigned eeprom_address = (unsigned) pos;

	eeprom_init();

	Wire.beginTransmission(eeprom_device_address);
	Wire.write((int)(eeprom_address >> 8));   // MSB
	Wire.write((int)(eeprom_address & 0xFF)); // LSB
    	Wire.write(value);
	Wire.endTransmission();
	
	// wait for write cycle to complete
	// this could be done more efficiently with "acknowledge polling"
	delay(5);
}



unsigned char eeprom_read_byte(unsigned char *pos)
{
	byte data = 0xFF;
	unsigned eeprom_address = (unsigned) pos;

	eeprom_init ();

	Wire.beginTransmission(eeprom_device_address);
	Wire.write((int)(eeprom_address >> 8));   // MSB
	Wire.write((int)(eeprom_address & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(eeprom_device_address, (byte)1);
	if (Wire.available())
		data = Wire.read();
	return data;
}
void eeprom_write_page(unsigned int eeaddress,  uint8_t* data, unsigned int  data_len) {
  unsigned char i=0, counter=0;
  unsigned int  address;
  unsigned int  page_space;
  unsigned int  page=0;
  unsigned int  num_writes;
  
  unsigned char first_write_size;
  unsigned char last_write_size;  
  unsigned char write_size;  
  page_space = int(((eeaddress/64) + 1)*64)-eeaddress;

  // Calculate first write size
  

     first_write_size=page_space; 
    
  // calculate size of last write  
  
     last_write_size = (data_len-first_write_size)%64;   
  
  // Calculate how many writes we need
  
     num_writes = 2;  
  
  
  
	i=0;   
  address=eeaddress;
  for(page=0;page<num_writes;page++) 
  {
     if(page==0) write_size=first_write_size;
     else if(page==(num_writes-1)) write_size=last_write_size;
     
  
     Wire.beginTransmission(eeprom_device_address);
     Wire.write((int)((address) >> 8));   // MSB
     Wire.write((int)((address) & 0xFF)); // LSB
     counter=0;
     do{ 
        Wire.write((byte) data[i]);
        i++;
        counter++;
     } while((data[i]) && (counter<write_size));  
     Wire.endTransmission();
     address+=write_size;   // Increment address for next write
     
     delay(6);  // needs 5ms for page write
  }
	
	
	
	
	
	
	
	
	
	
}
void writeEEPROM(unsigned int eeaddress,  char* data, unsigned int  data_len) 
{
	eeprom_init();
  // Uses Page Write for 24LC256
  // Allows for 64 byte page boundary
  // Splits string into max 16 byte writes
  unsigned char i=0, counter=0;
  unsigned int  address;
  unsigned int  page_space;
  unsigned int  page=0;
  unsigned int  num_writes;
  
  unsigned char first_write_size;
  unsigned char last_write_size;  
  unsigned char write_size;  
  
  
   
  // Calculate space available in first page
  page_space = int(((eeaddress/64) + 1)*64)-eeaddress;

  // Calculate first write size
  if (page_space>16){
     first_write_size=page_space-((page_space/16)*16);
     if (first_write_size==0) first_write_size=16;
  }   
  else 
     first_write_size=page_space; 
    
  // calculate size of last write  
  if (data_len>first_write_size) 
     last_write_size = (data_len-first_write_size)%16;   
  
  // Calculate how many writes we need
  if (data_len>first_write_size)
     num_writes = ((data_len-first_write_size)/16)+2;
  else
     num_writes = 1;  
     
  i=0;   
  address=eeaddress;
  for(page=0;page<num_writes;page++) 
  {
     if(page==0) write_size=first_write_size;
     else if(page==(num_writes-1)) write_size=last_write_size;
     else write_size=16;
  
     Wire.beginTransmission(eeprom_device_address);
     Wire.write((int)((address) >> 8));   // MSB
     Wire.write((int)((address) & 0xFF)); // LSB
     counter=0;
     do{ 
        Wire.write((byte) data[i]);
        i++;
        counter++;
     } while((data[i]) && (counter<write_size));  
     Wire.endTransmission();
     address+=write_size;   // Increment address for next write
     
     delay(6);  // needs 5ms for page write
  }
}



// --------------------------------------------------------------------------
// Timers
// --------------------------------------------------------------------------

typedef struct
{
    Tc          *pTimerRegs;
    uint16_t    channel;
    IRQn_Type   IRQ_Id;
  } tTimerConfig ;

#define  NUM_HARDWARE_TIMERS 9

static const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] =
  {
    { TC0, 0, TC0_IRQn},
    { TC0, 1, TC1_IRQn},
    { TC0, 2, TC2_IRQn},
    { TC1, 0, TC3_IRQn},
    { TC1, 1, TC4_IRQn},
    { TC1, 2, TC5_IRQn},
    { TC2, 0, TC6_IRQn},
    { TC2, 1, TC7_IRQn},
    { TC2, 2, TC8_IRQn},
  };


void HAL_timer_start (uint8_t timer_num, uint32_t frequency)
{
	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
	uint32_t channel = TimerConfig [timer_num].channel;

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
	TC_Configure (tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3);

	uint32_t rc = VARIANT_MCK/32/frequency;

	TC_SetRC(tc, channel, rc);

	TC_Start(tc, channel);

	// enable interrupt on RC compare
	tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;

	NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt (uint8_t timer_num)
{
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IER = TC_IER_CPCS;
}

void HAL_timer_disable_interrupt (uint8_t timer_num)
{
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = TC_IER_CPCS;
}

void HAL_timer_set_count (uint8_t timer_num, uint32_t count)
{
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	TC_SetRC (pConfig->pTimerRegs, pConfig->channel, count);
}

void HAL_timer_isr_prologue (uint8_t timer_num)
{
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	TC_GetStatus (pConfig->pTimerRegs, pConfig->channel);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
//! @brief
//! @param[in]
//! @param[out]
//! @return
// --------------------------------------------------------------------------

// File si5351bx_0_0.ino
// Copyright 2017, Jerry Gaffke, KE7ER, under the GNU General Public License 3.0

// An minimalist standalone set of Si5351 routines.
// VCOA is fixed at 875mhz, VCOB not used.
// The output msynth dividers are used to generate 3 independent clocks
// with 1hz resolution to any frequency between 4khz and 109mhz.

// Usage:  
// Call si5351bx_init() once at startup with no args;
// Call si5351bx_setfreq(clknum, freq) each time one of the 
// three output CLK pins is to be updated to a new frequency.
// A freq of 0 serves to shut down that output clock.

// The global variable si5351bx_vcoa starts out equal to the nominal VCOA
// frequency of 25mhz*35 = 875000000 Hz.  To correct for 25mhz crystal errors,
// the user can adjust this value.  The vco frequency will not change but
// the number used for the (a+b/c) output msynth calculations is affected.
// Example:  We call for a 5mhz signal, but it measures to be 5.001mhz.
// So the actual vcoa frequency is 875mhz*5.001/5.000 = 875175000 Hz,
// To correct for this error:     si5351bx_vcoa=875175000;

// Most users will never need to generate clocks below 500khz.
// But it is possible to do so by loading a value between 0 and 7 into 
// the global variable si5351bx_rdiv, be sure to return it to a value of 0
// before setting some other CLK output pin.  The affected clock will be
// divided down by a power of two defined by  2**si5351_rdiv
// A value of zero gives a divide factor of 1, a value of 7 divides by 128.
// This lightweight method is a reasonable compromise for a seldom used feature.


#include "Wire.h"

#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351BX_ADDR 0x60              // I2C address of Si5351   (typical)
#define SI5351BX_XTALPF 2               // 1:6pf  2:8pf  3:10pf

// If using 27mhz crystal, set XTAL=27000000, MSA=33.  Then vco=891mhz
#define SI5351BX_XTAL 25000000          // Crystal freq in Hz
#define SI5351BX_MSA  35                // VCOA is at 25mhz*35 = 875mhz

// User program may have reason to poke new values into these 3 RAM variables
uint32_t si5351bx_vcoa = (SI5351BX_XTAL*SI5351BX_MSA);  // 25mhzXtal calibrate
uint8_t  si5351bx_rdiv = 0;             // 0-7, CLK pin sees fout/(2**rdiv) 
uint8_t  si5351bx_drive[3] = {1,1,1};   // 0=2ma 1=4ma 2=6ma 3=8ma for CLK 0,1,2

uint8_t  si5351bx_clken = 0xFF;         // Private, all CLK output drivers off

void si5351bx_init() {                  // Call once at power-up, start PLLA
    uint8_t reg;  uint32_t msxp1;
    Wire.begin();  
    i2cWrite(149, 0);                   // SpreadSpectrum off
    i2cWrite(3, si5351bx_clken);        // Disable all CLK output drivers
    i2cWrite(183, SI5351BX_XTALPF<<6);  // Set 25mhz crystal load capacitance
    msxp1 = 128*SI5351BX_MSA - 512;     // and msxp2=0, msxp3=1, not fractional
    uint8_t  vals[8] = {0, 1, BB2(msxp1), BB1(msxp1), BB0(msxp1), 0, 0, 0};
    i2cWriten(26, vals, 8);             // Write to 8 PLLA msynth regs
    i2cWrite(177, 0x20);                // Reset PLLA  (0x80 resets PLLB)
    // for (reg=16; reg<=23; reg++) i2cWrite(reg, 0x80);    // Powerdown CLK's
    // i2cWrite(187, 0);                // No fannout of clkin, xtal, ms0, ms4
}

void si5351bx_setfreq(uint8_t clknum, uint32_t fout) {  // Set a CLK to fout Hz
    uint32_t  msa, msb, msc, msxp1, msxp2, msxp3p2top;
    if ((fout<500000) || (fout>109000000))  // If clock freq out of range
        si5351bx_clken |= 1<<clknum;        //  shut down the clock
    else {
        msa = si5351bx_vcoa / fout;     // Integer part of vco/fout
        msb = si5351bx_vcoa % fout;     // Fractional part of vco/fout 
        msc = fout;             // Divide by 2 till fits in reg
        while (msc & 0xfff00000) {msb=msb>>1; msc=msc>>1;}
        msxp1 =(128*msa + 128*msb/msc - 512) | (((uint32_t)si5351bx_rdiv)<<20);
        msxp2 = 128*msb - 128*msb/msc * msc;    // msxp3 == msc;        
        msxp3p2top = (((msc & 0x0F0000) <<4) | msxp2);      // 2 top nibbles
        uint8_t vals[8] = { BB1(msc), BB0(msc), BB2(msxp1), BB1(msxp1), 
            BB0(msxp1), BB2(msxp3p2top), BB1(msxp2), BB0(msxp2) };
        i2cWriten(42+(clknum*8), vals, 8);  // Write to 8 msynth regs
        i2cWrite(16+clknum, 0x0C|si5351bx_drive[clknum]);   // use local msynth
        si5351bx_clken &= ~(1<<clknum);     // Clear bit to enable clock
    }
    i2cWrite(3, si5351bx_clken);        // Enable/disable clock
}

void i2cWrite(uint8_t reg, uint8_t val){    // write reg via i2c
    Wire.beginTransmission(SI5351BX_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt){   // write array
    Wire.beginTransmission(SI5351BX_ADDR);
    Wire.write(reg);
    while (vcnt--) Wire.write(*vals++);
    Wire.endTransmission();
}



// *************** Testbench **********************************
// Assumes Raduino with Si5351 and tuning pot, no bitx40 required
// Monitor result using SW receiver tuned to 7.15 MHz

uint8_t   i2cRead(int regaddr){        // read one i2c reg
    uint8_t rval;
    Wire.beginTransmission(SI5351BX_ADDR);
    Wire.write(regaddr);
    Wire.endTransmission();
    Wire.requestFrom(SI5351BX_ADDR, 1);
    rval = Wire.read();
    return rval;
}

void setup() {
    Serial.begin(9600);
    si5351bx_init();
}

#include <stdio.h>
void loop() {
    uint32_t freq, freq_prev, aread, delta, regaddr, regval, n;
    char charbuf[10];

    // Seldom need sub 500khz on a Bitx40, so rdiv is not an arg to setfreq()
    // But we can hack in a non-zero rdiv like this
    si5351bx_rdiv=7;
    si5351bx_setfreq(0, 500000);         // CLK0 = 500khz/(2**7) = 3906.25hz
    si5351bx_rdiv=0;

    // An619 p6 says output msynth divides can be 6.0-1800.0, p2 says 8.0-900.0
    // Looks like the range is actually 8.0-1800.0  (also integers 4 and 6)
    // Arbitrary frequencies from integer output msynth divides would require
    // varying the vco using fractional pll msynth divides, not supported here.
    // So our max frequency is 875mhz/8.0 = 109mhz.
    si5351bx_setfreq(1, 109000000);      // CLK1 = 109 mhz

    // Set CLK2 to freq=7.150123mhz
    si5351bx_setfreq(2, 7150123);

    // Print a dump of all Si5351 registers to the serial monitor
    for (regaddr=0; regaddr<188; regaddr++) {
        regval = i2cRead(regaddr);      
        sprintf(charbuf, "%02x ", regval);
        Serial.print(charbuf);
        if (regaddr%4==3)  Serial.print(' ');
        if (regaddr%8==7)  Serial.print(' ');
        if (regaddr%16==15)  Serial.println();
    }
    Serial.println();

    // Tuning pot varies CLK2 freq in 1Hz steps over a range of 1024 Hz
    // Listen with a receiver tuned to 7.15mhz, varies smoothly without ticks.
    // Expected ticks due to the 8 I2C output msynth registers all being
    // loaded separately.  Perhaps will hear ticks when MSB's change.
    // Or perhaps SiLabs figured out a way to smoothly update.
    delta=0;  freq_prev=0;
    while (1) {
        for (n=0; n<1000; n++) {    // Smooth  out any noise from pot
            aread = analogRead(A7); // Read 0-1023 from tuning pot
            delta = delta - (delta/256 ) + aread;
        }   // delta is now a value between 0 and 1023*256
        freq = 7150000 + delta/256;   // 7.150000 to 7.151023mhz in Hz
        if (freq != freq_prev) {
            Serial.println(freq); 
            si5351bx_setfreq(2, freq);
            freq_prev = freq;
        }
    }
}

/**  Here's the result of the register dump:
11 f8 03 f8  00 ff 00 00   00 00 00 00  00 00 90 00   
0d 0d 0d 80  80 80 80 80   00 00 00 01  00 0f 80 00   
00 00 00 00  00 00 00 00   00 00 a1 20  73 69 00 70   
00 00 fe 6a  00 02 03 c6   cb 42 a3 45  00 3b 30 d0   
e7 10 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
00 00 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
00 00 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
00 00 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
00 00 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
00 00 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
00 00 00 00  00 00 00 00   00 00 00 00  00 00 00 00   
ff 00 00 00  00 30 10 80   60 60 b8 02  
**/

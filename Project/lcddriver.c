// ********************************************************************************************************************************
//   Graphics library by ladyada/adafruit with init code from Rossum
//   MIT license   (some modifications and improvements by Jim Lynch)
//
//
//   LPC1769 LPCXpresso LCD Driver for LadyAda's 2.8" 320x240 TFT LCD TouchScreen Module
//   -----------------------------------------------------------------------------------
//
//   tftlcd.cpp driver from AdaFruit was designed for a Arduino Atmel ATmega328 8-bit microprocessor
//   interfaced to the AdaFruit LCD Display (320x240 pixels, 262k colors) using a ILITEK 9325
//   LCD controller chip.
//
//   The original software driver was designed for a parallel interface using Arduino I/O port pins.
//
//   To connect this device to the NXP LPC1769 LPCXpresso evaluation board ($29.95),
//   we just need to connect the data and control lines to GPIO Port0 pins P0.0 - P0.7 and pins P0.21 - P0.25
//
//   First, let's look at the AdaFruit TFT LCD board:
//
//  					AdaFruit LCD Display
//  					--------------------
//
//  320x240 pixels     2.8" diagonal  8-bit parallel interface  16-bit color format (64k colors) ILITEK 9325 controller
//
//                List of AdaFruit TFT LCD Signals
//
// 1x20  2x10   Name        Description           Notes
// __________________________________________________________________________________________
//
//  1     1      GND         ground
//  2     2      3-5V        Vss supply           (5v or 3.3v)
//
//  3     3      CS          chip select          (low = chip selected)
//  4     4      CD          register select      (low = select index register, high = data)
//  5     5      WR          write strobe         (low = write data)
//  6     6      RD          read strobe          (low = read data)
//  7     7      RST         reset                (low = reset ILI9328 LCD controller)
//  8     8      Backlite    back light control   (high = turn on back light)
//
//  9     9      X+          touch screen
//  10    10     Y+          touch screen
//  11    11     X-          touch screen
//  12    12     Y-          touch screen
//
//  13	  13   	 D0          8-bit data bus (lsb)
//  14	  14     D1          8-bit data bus
//  15	  15     D2          8-bit data bus
//  16	  16     D3          8-bit data bus
//  17	  17     D4          8-bit data bus
//  18	  18     D5          8-bit data bus
//  19	  19     D6          8-bit data bus
//  20	  20     D7          8-bit data bus (msb)
//
//
//  Selecting the Microprocessor Interface
//  --------------------------------------
//
//  The ILI9325 Controller Pin Descriptions show that Input Interface Pins IM3..IM0 select the interface desired.
//
//  IM3   IM2   IM1   IM0
//  ----------------------------------------------------------
//   0     0     1     1    Select i80-system 8-bit interface
//
//   Notes: 	AdaFruit schematic only shows IM0=1 and IM3=0 connections.
//       		IM2=0 only used to select SPI-mode interface, assume that this is tied to ground but not shown
//       		IM1=1 only used to select i80-system 8/16 bit interface, assume that this is tied to Vcc but not shown
//
//       		bit TRI=0 and bit DFM=0 in Register R03h (Entry Mode) select "system 8-bit interface (2 transfer/pixel) 65536 colors
//       		TRI and DFM are set in LCD_Setup() function.
//
//
//   Writing a Single Pixel
//   ----------------------
//
//                              System 8-bit interface (2 transfers per pixel)
//   |-----|-----|-----|-----|-----|-----|-----|-----|  |-----|-----|-----|-----|-----|-----|-----|-----|
//   | R5  | R4  | R3  | R2  | R1  | G5  | G4  | G3  |  | G2  | G1  | G0  | B5  | B4  | B3  | B2  | B1  |
//   |-----|-----|-----|-----|-----|-----|-----|-----|  |-----|-----|-----|-----|-----|-----|-----|-----|
//     15    14    13    12    11    10     9     8        7     6     5     4     3     2     1     0
//                    First 8-bit transfer                             Second 8-bit transfer
//
//   Note:  5-bits RED    6-bits GREEN      5-bits BLUE
//
//   For example:    #define BLACK		0x0000                  Note: AdaFruit documentation for the 2.8" TFT
//                   #define BLUE		0x001F                        LCD Display lists 262144 colors.
//                   #define RED 		0xF800
//                   #define GREEN		0x07E0                        Since her pixel data transfer is only 16-bits,
//                   #define CYAN		0x07FF                        the number of possible colors is actually 65536.
//                   #define MAGENTA    0xF81F
//                   #define YELLOW     0xFFE0                        So what? That's still a lot of colors!
//                   #define WHITE		0xFFFF
//
//
//    Write Command Timing Example
//
//    RS  -----------|                        |---------------- register select = 0 (command)
//                   |                        |
//                   |________________________|
//
//    CS  ------------------|               |------------------ chip select
//                          |               |
//                          |_______________|
//
//    WR ---------------------|          |--------------------- write strobe (latches data on rising edge)
//                            |          |
//                            |__________|
//                                       :
//                                       :
//    D0..7 ---------------------|-------:---|----------------- write data (represents all eight data bus lines)
//                               |       V   |
//                               |___________|                  note: Data should be stable for 10nsec before WR rises
//                                                                    Data should be stable for 15nsec after WR rises
//                                      data
//                                     captured
//
//
//    Write Data Timing Example
//
//    RS  ----------------------------------------------------- register select = 1 (data)
//
//    CS  ------------------|               |------------------ chip select
//                          |               |
//                          |_______________|
//
//    WR ---------------------|          |--------------------- write strobe (latches data on rising edge)
//                            |          |
//                            |__________|
//                                       :
//                                       :
//    D0..7 ---------------------|-------:---|----------------- write data (represents all eight data bus lines)
//                               |       V   |
//                               |___________|                  note: Data should be stable for 10nsec before WR rises
//                                                                    Data should be stable for 15nsec after WR rises
//                                      data
//                                     captured
//                                      here
//
//
//    Read Data Timing Example
//
//    CD  ----------------------------------------------------- register select = 1 (data)
//
//    CS  ------------------|                   |------------------ chip select
//                          |                   |
//                          |___________________|
//
//    RD ---------------------|          |--------------------- read strobe (latches data on rising edge)
//                            |          |
//                            |__________|
//                                       :
//                                       :
//    D0..7 ------------------|----------:--|-------------------- read data (represents all eight data bus lines)
//                            |          V  |
//                            |_____________|                  note: Data should be stable for 10nsec before WR rises
//                                                                 Data should be stable for 15nsec after WR rises
//                                      data
//                                     captured
//                                      here
//
//
//
//  Assignment of LCD Control/Data Signals to LPCXpresso Board I/O Ports
//  ----------------------------------------------------------------------
//
//        LCD                                     LPCXpresso Board
//    Pin    Description                     J6 Pin   Description
//  ----------------------------------------------------------------------
//    1      Ground                            1      GND    ground
//    2      3.3 volts                         2      3V3    3.3 volts
//    3      CS chip select                    23     P2.8   Port2 bit8
//    4      RS register select  (RS)          24     P1.27  Port0 bit22
//    5      WR write strobe                   15     P1.28  Port0 bit22
//    6      RD read strobe                    16     P1.29  Port0 bit24
//    7      RST restart                       17     P0.25  Port0 bit25
//    8      Backlite on                       2      3V3    3.3 volts
//    9      X+  touch screen
//    10     Y+  touch screen
//    11     X-  touch screen
//    12     Y-  touch screen
//    13     D0  (lsb)                         15     P0.0   Port0 bit0
//    14     D1                                16     P0.1   Port0 bit1
//    15     D2                                17     P0.2   Port0 bit2
//    16     D3                                18     P0.3   Port0 bit3
//    17     D4                                19     P0.4   Port0 bit4
//    18     D5                                20     P0.5   Port0 bit5
//    19     D6                                21     P0.6   Port0 bit6
//    20     D7                                22     P0.7   Port0 bit7
//    13     D8                                15     P2.0   Port0 bit0
//    14     D9                                16     P2.1   Port0 bit1
//    15     D10                                17     P2.2   Port0 bit2
//    16     D11                                18     P2.3   Port0 bit3
//    17     D12                                19     P2.4   Port0 bit4
//    18     D13                                20     P2.5   Port0 bit5
//    19     D14                                21     P2.6   Port0 bit6
//    20     D15  (msb)                         22     P0.7   Port0 bit7

//
//
//	  GPIO Operations with NXP LPC1769 ARM-Cortex Microprocessors
//    -----------------------------------------------------------
//
//	  There are five GPIO ports (32-bits each): Port0, Port1, Port2, Port3, and Port4
//
//	  Control is via 4 configuration registers:
//
//			FIOxMASK =  mask register  (any bits set to 0, unmasked, can be changed)
//			FIOxDIR  =  direction register (0 = bit is a digital input,  1 = bit is a digital output)
//			FIOxSET  =  set bits     (1 = set this bit if it is also unmasked)
//			FIOxCLR  =  clear bits   (1 = clear this bit if it is also unmasked)
//
//
//
//   The direction (input / output) must be configured by the FIO0DIR 32-bit register  0x2009 C000
//   note:  0 = input,  1 = output
//
//            Fast GPIO Port0 Direction control register     FIO0DIR
//
//                                         RST   RD    WR    C/D   CS
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |     |     |     |     |     |     |P0.25|P0.24|P0.23|P0.22|P0.21|     |     |     |     |     |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      31    30    29    28    27    26    25    24    23    22    21    20    19    18    17    16
//
//      D15   D14   D13   D12  D11   D10   D9    D8    D7    D6    D5    D5    D3    D2    D1    D0
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |P0.15|  *  |  *  |  *  |     |     |     |P0.7 |P2.7 |P2.6 |P2.5 |P2.4 |P2.3 |P2.2 |P2.1 |P2.0 |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      15    14    13    12    11    10    9     8     7     6     5     4     3     2     1     0
//            Note: bits 12, 13, 14 not available for Port0
//
//
//   You can unmask only the above bits to be changed by the FIO0MASK 32-bit register  0x2009 C010
//   note:  0 = unmasked (allow bit to be changed),  1 = masked (bit does not participate in I/O operations)
//
//            Fast GPIO Port0 Mask control register     FIO0MASK
//
//                                         RST   RD    WR    C/D   CS
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |  1  |  1  |  1  |  1  |  1  |  1  |  0  |  0  |  0  |  0  |  0  |  1  |  1  |  1  |  1  |  1  |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      31    30    29    28    27    26    25    24    23    22    21    20    19    18    17    16
//
//                                                     D7    D6    D5    D5    D3    D2    D1    D0
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |  1  |  1  |  1  |  1  |  1  |  1  |  1  |  1  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      15    14    13    12    11    10    9     8     7     6     5     4     3     2     1     0
//      Note: FIO0MASK defaults to all zeros (all bits unmasked) at boot-up
//            Probably best to not count on this, rather explicitly specify which bits are unmasked
//
//
//
//   Any bit can be set by the FIO0SET 32-bit register  0x2009 C018  (bits must be unmasked via FIO0MASK register)
//   note:  0 = no change,  1 = bit set
//
//            Fast Port0 Output Set register register     FIO0SET
//
//                                         RST   RD    WR    C/D   CS
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |     |     |     |     |     |     |P0.25|P0.24|P0.23|P0.22|P0.21|     |     |     |     |     |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      31    30    29    28    27    26    25    24    23    22    21    20    19    18    17    16
//
//                                                     D7    D6    D5    D5    D3    D2    D1    D0
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |     |  *  |  *  |  *  |     |     |     |     |P0.7 |P0.6 |P0.5 |P0.4 |P0.3 |P0.2 |P0.1 |P0.0 |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      15    14    13    12    11    10    9     8     7     6     5     4     3     2     1     0
//            Note: bits 12, 13, 14 not available for Port0
//
//
//   Any bit can be cleared by the FIO0CLR 32-bit register  0x2009 C01C  (bits must be unmasked via FIO0MASK register)
//   note:  0 = no change,  1 = bit cleared
//
//            Fast Port0 Output Set register register     FIO0CLR
//
//                                         RST   RD    WR    C/D   CS
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |     |     |     |     |     |     |P0.25|P0.24|P0.23|P0.22|P0.21|     |     |     |     |     |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      31    30    29    28    27    26    25    24    23    22    21    20    19    18    17    16
//
//                                                     D7    D6    D5    D5    D3    D2    D1    D0
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//   |     |  *  |  *  |  *  |     |     |     |     |P0.7 |P0.6 |P0.5 |P0.4 |P0.3 |P0.2 |P0.1 |P0.0 |
//   |-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
//      15    14    13    12    11    10    9     8     7     6     5     4     3     2     1     0
//            Note: bits 12, 13, 14 not available for Port0
//
//
//
//   Simple Example  -  pulse the Write line (LCD_WR)
//   ================================================
//
//   from lcddriver.h, we have these useful #define macros:
//   	#define LCD_PORT0	0
//   	#define LCD_PORT1	1
//   	#define LCD_PORT2	2
//   	#define LCD_PORT3	3
//   	#define LCD_PORT4	4
//		#define LCD_MASK	1
//		#define LCD_UNMASK	0
//		#define LCD_INPUT	0
//		#define LCD_OUTPUT	1
//		#define LCD_CS		21
//		#define LCD_RS		22
//		#define LCD_WR		23
//		#define LCD_RD		24
//		#define LCD_RST		25
//
//	 from lpc_types.h, we have the following useful #define macros:
//		#define _BIT(n)	(1<<n)
//		#define _SBF(f,v) (v<<f)
//
//
//	  #include "lpc_types.h"
//	  #include "lcddriver.h"
//	  #include "lpc17xx_gpio.h"
//	  #include "lpc17xx_clkpwr.h"
//
//    uint32_t	LastStatePort0;
//
//    /* Enable GPIO Clock */
//    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE);
//
//    /* set mask for P0.23 (LCD_WR) via 32-bit Fast GPIO functions  */
//    FIO_SetMask(LCD_PORT0, _BIT(LCD_WR), UNMASK);
//
//    /* set I/O port direction via 32-bit FastGPIO functions  */
//    FIO_SetDir(LCD_PORT0, _BIT(LCD_WR), LCD_OUTPUT)
//
//    /* Output WR to high level via 32-bit FastGPIO functions  */
//    FIO_SetValue(LCD_PORT0, _BIT(LCD_WR));
//
//    /* Output WR to low level via 32-bit FastGPIO functions  */
//    FIO_ClearValue(LCD_PORT0, _BIT(LCD_WR));
//
//    /* for completeness, there's also a 32-bit FastGPIO function to read an I/O port  */
//    LastStatePort0 = FIO_ReadValue(LCD_PORT0);
//
//    Created on: August 9, 2013
//    Authors: Jim Lynch, Limor Fried, Rossum (adapted from AdaFruit Arduino driver)
//
// ********************************************************************************************************************************

// Include files (some for NXP LPC1769 components)
// -----------------------------------------------
#include "lcddriver.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_clkpwr.h"

 uint16_t _width, _height;

  uint8_t rotation; 
// ***********************************************************************************************
// *  goHome - sets the horizontal and vertical GRAM address to (0, 0) and then selects
// *           the Write Data to GRAM register
// *
// *	Inputs: none
// *
// *	Returns: nothing
// ***********************************************************************************************
void goHome(void)
{
	goTo(0, 0);
}


// ***********************************************************************************************
//    goTo - sets the horizontal and vertical GRAM address (x0,y0) and then selects
//           the Write Data to GRAM register
//
//  	Inputs: x       = starting x address (0 .. 239)
//  			y       = starting y address (0 .. 319)
//
//  	Returns: nothing
// ***********************************************************************************************
void goTo(int x, int y)
{
    // Set the GRAM address (x,y)
    writeRegister(TFTLCD_GRAM_HOR_AD, x); 	// GRAM Address Set (Horizontal Address)     (R20h)
    writeRegister(TFTLCD_GRAM_VER_AD, y); 	// GRAM Address Set (Vertical Address)       (R21h)

    // select the RW_GRAM register (R22h)
    writeCommand(TFTLCD_RW_GRAM);  			// Select the "Write Data to GRAM" register  (R22h)
}


// ****************************************************************************************
//    drawChar - draws an ASCII character at the specified (x,y) address, color, and size
//
//      Inputs:	  x       = starting x address (0 .. 239)  upper-left corner of font box
//      		  y       = starting y address (0 .. 319)  upper-left corner of font box
//  			  c       =   ASCII character to be displayed
//  	          fcolor  =   12-bit foreground color value		rrrrrggggggbbbbb
//  	          bcolor  =   12-bit background color value		rrrrrggggggbbbbb
//  			  size    =   font pitch (0=SMALL, 1=MEDIUM, 2=LARGE)
//
//      Returns:  nothing
//
//
//    Notes:  Assume that setWriteDir(); has already been called.
//
//            Here's an example to display "E" at address (20,20)
//
//  		  drawChar(20, 20, 'E', WHITE, BLACK, SMALL);
//
//  				 (20,20)        (27,20)
//  					|             |
//  					|             |
//  			      	V             V
//  				 ---------x-------->
//  			   :  _ # # # # # # #   0x7F
//  				 :  _ _ # # _ _ _ #   0x31
//  			   :  _ _ # # _ # _ _   0x34
//  			   y  _ _ # # # # _ _   0x3C
//  			   :  _ _ # # _ # _ _   0x34
//  			   :  _ _ # # _ _ _ #   0x31
//  			   :  _ # # # # # # #   0x7F
//  			   :  _ _ _ _ _ _ _ _   0x00
//                   V
//  					^             ^
//  					|             |
//  					|             |
//  				 (20,27)       (27,27)
//
//
//  	The most efficient way to display a character is to make use of the window address "wrap-around"
//      feature of the Ilitek ILI9325 LCD controller chip.
//
//  	Assume that we position the character at (20, 20). The starting pixel is thus top - left.
//  	With the window address set commands, you can specify an 8x8 window address box for the SMALL and
//      MEDIUM characters or a 16x24 window address box for the LARGE characters.
//
//  		writeRegister(LCD_HOR_START_AD, x0);
//  		writeRegister(LCD_HOR_END_AD, x1);
//  		writeRegister(LCD_VER_START_AD, y0 + 8);
//          writeRegister(LCD_VER_END_AD, y1 + 8);
//
//  	When the algorithm completes the pixel at col 27, the column address wraps back to 20
//  	At the same time, the row address increases by one (this is done by the controller)
//
//  	The following simple loop will suffice:
//
//      	writeRegister(TFTLCD_GRAM_HOR_AD, x)
//          writeRegister(TFTLCD_GRAM_VER_AD, y)
//          writeCommand(TFTLCD_RW_GRAM);
//
//  	    for (i = 0; i <= nBytes; i++) {
//          	// copy pixel byte from font table and then decrement row
//          	PixelRow = *pChar++;
//
//          	// loop on each pixel in the byte (left to right)
//          	Mask = 0x80;
//          	for (j = 0; j < nCols; j++) {
//          		// if pixel bit set, use foreground color; else use the background color
//          		writeData_unsafe(((PixelRow & Mask) == 0) ? bColor : fColor);
//          		Mask = Mask >> 1;
//  	    	}
//          }
//
//    Once the "window" is set up and the address registers are pointing to the first
//    pixel (x, y), the algorithm is simply 64 writeData_unsafe(color) operations.
//
//    	Author:  James P Lynch    May 2, 2011
//  ***************************************************************************************
void drawChar(uint16_t x, uint16_t y, char c, uint16_t fColor, uint16_t bColor, uint8_t s)
{
	extern const uint8_t	FONT8x8[97][8];				// external Font table - size = 0 SMALL
	extern const uint8_t	FONT8x16[97][8];			// external Font table - size = 1 MEDIUM
	extern const uint8_t	FONT16x24[97][16];		// external Font table - size = 2 LARGE

	uint8_t		*FontTable[] = {(uint8_t *)FONT8x8,		// pointer to SMALL font
								(uint8_t *)FONT8x16,	// pointer to MEDIUM font
								(uint8_t *)FONT16x24};	// pointer to LARGE font

	uint16_t				i,j;						// loop counters
	uint16_t				nCols;						// number of columns in a character
	uint16_t				nRows;						// number of rows in a character
	uint16_t				nBytes;						// total number of bytes in a character
	uint8_t					PixelRow;					// temporary storage of a pixel row
	uint8_t					Mask;						// mask used to inspect each pixel in a row
	uint8_t					*pFont;						// pointer to beginning of the font table
	uint8_t					*pChar;						// pointer to each row of the character

	// get pointer to the beginning of the selected font table
	pFont = (uint8_t *)FontTable[s];

	// get the nColumns, nRows and nBytes from 1st line of font table
	nCols = *pFont;
	nRows = *(pFont + 1);
	nBytes = *(pFont + 2);

	// get pointer to the first byte of the desired character
	pChar = pFont + (nBytes * (c - 0x1F));

	// specify the controller drawing box according to those limits
	writeRegister(TFTLCD_HOR_START_AD, x);					// R50h - Horizontal Address Start Position
	writeRegister(TFTLCD_HOR_END_AD, x + nCols - 1);		// R51h - Horizontal Address End Position
	writeRegister(TFTLCD_VER_START_AD, y);					// R52h - Vertical Address Start Position
	writeRegister(TFTLCD_VER_END_AD, y + nRows - 1);		// R53h - Vertical Address End Position

    // Set the GRAM address (x,y) and then select the RW_GRAM register (R22h)
	goTo(x, y);

    // select the chip
	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));

	// loop on each byte in the font character
	for (i = 0; i < nBytes; i++) {

		// copy pixel byte from font table and then increment pointer
		PixelRow = *pChar++;

		// loop on each pixel in the byte (left to right)
		Mask = 0x80;
		for (j = 0; j < 8; j++) {
			// if pixel bit set, use foreground color; else use the background color
			writeData_unsafe(((PixelRow & Mask) == 0) ? bColor : fColor);
			Mask = Mask >> 1;
		}
	}

	// restore the controller drawing box to full size
	writeRegister(TFTLCD_HOR_START_AD, 0);					// R50h - Horizontal Address Start Position
	writeRegister(TFTLCD_HOR_END_AD, TFTLCD_WIDTH);			// R51h - Horizontal Address End Position
	writeRegister(TFTLCD_VER_START_AD, 0);					// R52h - Vertical Address Start Position
	writeRegister(TFTLCD_VER_END_AD, TFTLCD_HEIGHT);		// R53h - Vertical Address End Position

	// de-select the chip
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));
}


// ***********************************************************************************************
//   drawString - Draws a null-terminated string c starting at(x0,y0) in the color specified
//
//		Inputs: x0    = row address (0 .. 131)
// 				y0    = column address (0 .. 131)
// 				c 	  = null-terminated string
// 				color = 16-bit color value rrrrrggggggbbbbb
//
// 		Returns: nothing
//
//   Note: if you run past the edges, there will be weird wrap-around effects
//         function will overrun and fail if the string is not null-terminated
// ***********************************************************************************************
void drawString(uint16_t x, uint16_t y, char *c, uint16_t fColor, uint16_t bColor, uint8_t s)
{
	extern const uint8_t	FONT8x8[97][8];				// external Font table - size = 0 SMALL
	extern const uint8_t	FONT8x16[97][8];			// external Font table - size = 1 MEDIUM
	extern const uint8_t	FONT16x24[97][16];			// external Font table - size = 2 LARGE

	uint8_t		*pFont;									// pointer to beginning of the font table
	uint16_t	CharWidth;								// 3 of pixels between characters
	uint8_t		*FontTable[] = {(uint8_t *)FONT8x8,		// pointer to SMALL font
								(uint8_t *)FONT8x16,	// pointer to MEDIUM font
								(uint8_t *)FONT16x24};	// pointer to LARGE font

	// get pointer to the beginning of the selected font table
	pFont = (uint8_t *)FontTable[s];

	// get the character spacing from 1st line of font table
	CharWidth = *(pFont + 3);

	// draw the string (Warning: will overrun if string is not null-terminated)
	while (*c != 0) {
		drawChar(x, y, *c++, fColor, bColor, s);
		x += CharWidth;
	}
}


// ***********************************************************************************************
//   drawCircle - Draws a circle outline in the specified color at center (x0,y0) with radius r
//
//		Inputs: x0    = row address (0 .. 131)
// 				y0    = column address (0 .. 131)
// 				r 	  = radius in pixels
// 				color = 16-bit color value rrrrrggggggbbbbb
//
// 		Returns: nothing
//
// Author: Jack Bresenham IBM, Winthrop University (Father of this algorithm, 1962)
//
// Note: taken verbatim Wikipedia article on Bresenham's line algorithm
//       http://www.wikipedia.org
// ***********************************************************************************************
void drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
	int f = 1 - r;
	int ddF_x = 0;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;

	drawPixel(x0, y0 + r, color);
	drawPixel(x0, y0 - r, color);
	drawPixel(x0 + r, y0, color);
	drawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x + 1;

		drawPixel(x0 + x, y0 + y, color);
		drawPixel(x0 - x, y0 + y, color);
		drawPixel(x0 + x, y0 - y, color);
		drawPixel(x0 - x, y0 - y, color);
		drawPixel(x0 + y, y0 + x, color);
		drawPixel(x0 - y, y0 + x, color);
		drawPixel(x0 + y, y0 - x, color);
		drawPixel(x0 - y, y0 - x, color);
	}
}


// ***********************************************************************************************
//   fillCircle - Fills a circle in the specified color at center (x0,y0) with radius r
//
//		Inputs: x0    = row address (0 .. 131)
// 				y0    = column address (0 .. 131)
// 				r 	  = radius in pixels
// 				color = 16-bit color value rrrrrggggggbbbbb
//
// 		Returns: nothing
// ***********************************************************************************************
void fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawVerticalLine(x0, y0-r, 2*r+1, color);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		drawVerticalLine(x0+x, y0-y, 2*y+1, color);
		drawVerticalLine(x0-x, y0-y, 2*y+1, color);
		drawVerticalLine(x0+y, y0-x, 2*x+1, color);
		drawVerticalLine(x0-y, y0-x, 2*x+1, color);
	  }
}


// ***************************************************************************************************
// *  fillScreen - fills entire screen with the specified color
// *
// *		Inputs: color = 16-bit fill color value  (rrrrrggggggbbbbb)
// *
// *	    Returns: nothing
// ********************************************************************************************
void fillScreen(uint16_t color)
{
	uint32_t	i = TFTLCD_WIDTH * TFTLCD_HEIGHT;

	// go to top-left corner (0,0)
	goHome();

	// select the chip
	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));

	// write to every pixel on the screen
	while (i--) {
		writeData_unsafe(color);
	}

	// de-select the chip
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));
}


// ***************************************************************************************************
//    drawRect - draws a NON-filled rectangle in the specified color from (x, y) to (x + w, y + h)
//
//  		Inputs: x     = horizontal address - top left corner  (0..239)
//  				y     = vertical address   - top left corner  (0..319)
//  				w     = width in pixels
//  				h     = height in pixels
//  				color = 16-bit fill color value  (rrrrrggggggbbbbb)
//
//  	    Returns: nothing
//
// ********************************************************************************************
void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	// smarter version
	  drawHorizontalLine(x, y, w, color);
	  drawHorizontalLine(x, y+h-1, w, color);
	  drawVerticalLine(x, y, h, color);
	  drawVerticalLine(x+w-1, y, h, color);
}


// ************************************************************************************************
//    fillRect - draws a filled rectangle in the specified color from (x0,y0) to (x0 + w, y0 + h)
//
//  		Inputs: x     = horizontal address - top left corner (0..239)
//  				y     = vertical address   - top left corner (0..319)
//  				w     = width in pixels
//  				h     = height in pixels
//  				color = 16-bit fill color value  (rrrrrggggggbbbbb)
//
//  	    Returns: nothing
//
//      Notes:
//
//      The best way to fill a rectangle is to take advantage of the "wrap-around" feature
//      built into the Ilitek ILI9325 controller. By defining a drawing box, the memory can
//      be simply filled by successive memory writes until all pixels have been illuminated.
//
//      Author: James P Lynch July 7, 2007
// ************************************************************************************************
void fillRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color)
{
	uint16_t	x1, y1;
	uint16_t	pixelCount;

	// locate opposite corner of rectangle
	x1 = x0 + w - 1;
	y1 = y0 + h - 1;

	// specify the controller drawing box to the size of the rectangle
    writeRegister(TFTLCD_HOR_START_AD, x0);		// R50h - Horizontal Address Start Position
    writeRegister(TFTLCD_HOR_END_AD, x1);		// R51h - Horizontal Address End Position
    writeRegister(TFTLCD_VER_START_AD, y0);		// R52h - Vertical Address Start Position
    writeRegister(TFTLCD_VER_END_AD, y1);		// R53h - Vertical Address End Position

    // set GRAM address to (x0, y0)
    goTo(x0, y0);

    // select the chip
    FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));

    // loop on total number of pixels
	for (pixelCount = 0; pixelCount < (w * h); pixelCount++) {
		// write the color value to the pixel
		writeData_unsafe(color);
	}

	// return the controller drawing box to full screen
    writeRegister(TFTLCD_HOR_START_AD, 0);		// R50h - Horizontal Address Start Position
    writeRegister(TFTLCD_HOR_END_AD, 239);		// R51h - Horizontal Address End Position
    writeRegister(TFTLCD_VER_START_AD, 0);		// R52h - Vertical Address Start Position
    writeRegister(TFTLCD_VER_END_AD, 319);		// R53h - Vertical Address End Position

	// de-select the chip
    FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));
}


// *************************************************************************************************
//   drawFastLine - draws a vertical or horizontal line in the specified color
//
//   Inputs:	x0    = starting x address (0 .. 239)
//   			y0    = starting y address (0 .. 319)
//          	l     = length in pixels
//   			color = 16-bit color value rrrrrggggggbbbbb
//              flag  = 0=horizontal, 1=vertical
//
//   Returns: nothing
// *************************************************************************************************
void drawFastLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color, uint8_t rotflag)
{
	// temporarily set rotation to VERTICAL
	if (rotflag == VERTICAL) {
		setRotation(2);
	}

    writeRegister(TFTLCD_GRAM_HOR_AD, x0); 				// GRAM Address Set (Horizontal Address) (R20h)
    writeRegister(TFTLCD_GRAM_VER_AD, y0);				// GRAM Address Set (Vertical Address) (R21h)
    writeCommand(TFTLCD_RW_GRAM);  					    	// Write Data to GRAM (R22h)

	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));					// select the chip
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RS));						// CD high (idle)
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));						// RD high (idle)
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));						// WR high (idle)

		/* DB[0.7] = P2.0...P2.7 */ 
	/* Set Output */
   LPC_GPIO2->FIODIR|=(1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);

	/* DB[8.15]= P0.15...P0.22 */
	
	/* Set Output */
   LPC_GPIO0->FIODIR|= (1<<15) | (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21) | (1<<22);

	while (length--) {
		writeData_unsafe(color);
	}

	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));					// select the chip

	if (rotflag == VERTICAL) {
		setRotation(3);
	}
}


// *************************************************************************************************
//   drawVerticalLine - draws a vertical line in the specified color
//
//   Inputs:	x0     = starting x address (0 .. 239)
//   			y0     = starting y address (0 .. 319)
//          	length = length in pixels
//   			color  = 16-bit color value rrrrrggggggbbbbb
//
//   Returns: nothing
// *************************************************************************************************
void drawVerticalLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color)
{
	// bail out if starting point is out-of-range
	if ((x0 >= TFTLCD_WIDTH) || (y0 >= TFTLCD_HEIGHT)) return;

	// draw the vertical line
	drawFastLine(x0, y0, length, color, 1);
}


// *************************************************************************************************
//   drawHorizontalLine - draws a horizontal line in the specified color
//
//   Inputs:	x0     = starting x address (0 .. 239)
//   			y0     = starting y address (0 .. 319)
//          	length = length in pixels
//   			color  = 16-bit color value rrrrrggggggbbbbb
//
//   Returns: nothing
// *************************************************************************************************
void drawHorizontalLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color)
{
	// bail out if starting point is out-of-range
	if ((x0 >= TFTLCD_WIDTH) || (y0 >= TFTLCD_HEIGHT)) return;

	// draw the vertical line
	drawFastLine(x0, y0, length, color, 0);
}


// *************************************************************************************************
//   drawLine - draws a line in the specified color from (x0,y0) to (x1,y1)
//
//   Inputs:	x0 = starting x address (0 .. 239)
//   			y0 = starting y address (0 .. 319)
//          	x1 = ending x address   (0 .. 239)
//   			y1 = ending y address   (0 .. 319)
//   			color = 16-bit color value rrrrrggggggbbbbb
//   				where:  rrrrr  = 11111 full red
//   										:
//   						         00000 red is off
//
//   						gggggg = 111111 full green
//   										:
//   								 000000 green is off
//
//   						bbbbb  = 11111 full blue
//   										:
//   								 00000 blue is off
//
//   Returns: nothing
//
//   Note: good write-up on this algorithm in Wikipedia (search for Bresenham's line algorithm)
//         see lcd.h for some sample color settings
//
//   Authors: Dr. Leonard McMillan, Associate Professor UNC
//   		  Jack Bresenham IBM, Winthrop University (Father of this algorithm, 1962)
//
//   Note: taken verbatim from Professor McMillan's presentation:
//         http://www.cs.unc.edu/~mcmillan/comp136/Lecture6/Lines.html
// *************************************************************************************************
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int dy = y1 - y0;
	int dx = x1 - x0;
	int stepx, stepy;

	if (dy < 0) {
		dy = -dy; stepy = -1;
	} else {
		stepy = 1;
	}

	if (dx < 0) {
		dx = -dx; stepx = -1;
	} else { stepx = 1;
	}

	dy <<= 1; 							// dy is now 2*dy
	dx <<= 1; 							// dx is now 2*dx

	drawPixel(x0, y0, color);

	if (dx > dy) {
		int fraction = dy - (dx >> 1); 	// same as 2*dy - dx

		while (x0 != x1) {
			if (fraction >= 0) {
				y0 += stepy;
				fraction -= dx; 		// same as fraction -= 2*dx
			}
		x0 += stepx;
		fraction += dy; 				// same as fraction -= 2*dy
		drawPixel(x0, y0, color);
		}
	} else {
		int fraction = dx - (dy >> 1);
		while (y0 != y1) {
			if (fraction >= 0) {
				x0 += stepx;
				fraction -= dy;
			}
		y0 += stepy;
		fraction += dx;
		drawPixel(x0, y0, color);
		}
	}
}


// ********************************************************************************
//   drawPixel -  draws a single pixel at the specified (x,y) address and color.
//
//   Parameters:  x:      horizontal address in pixels (0..239)
//                y:      vertical address in pixels (0..319)
//                color:  color value (16-bits)  RRRRRGGGGGGBBBBB
//
//   Returns:     nothing
// ********************************************************************************
void drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if ((x >= 240) || (y >= 320)) return;
  writeRegister(TFTLCD_GRAM_HOR_AD, x); 		// GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y); 		// GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);  				// Write color data to GRAM (R22h)
  FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));				// select the chip
  writeData_unsafe(color);						// write the color to GRAM
  FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));				// de-select the chip
}

/*******************************************************************************
* Function Name  : Lcd_IO_Configuration
* Description    : Configures LCD Control lines
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_IO_Configuration(void)
{
	/* Configure the LCD Control pins */
   

	/* DB[0.7] = P2.0...P2.7 */ 
	/* Set Output */
   LPC_GPIO2->FIODIR|=(1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);

	/* DB[8.15]= P0.15...P0.22 */
	
	/* Set Output */
   LPC_GPIO0->FIODIR|= (1<<15) | (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21) | (1<<22);
	
	/*RS = P1.27, WR = P1.28, RD = P1.29*/
	/* Set Output */
   LPC_GPIO1->FIODIR|=(1<<27) | (1<<28) | (1<<29);

	/*CSA = P2.8 */
	/* Output */
    LPC_GPIO2->FIODIR|=(1<<8);
}


// *************************************************************************************************************
//   lcdInitDisplay -  initializes the ILI9325 LCD controller
//
//  					1. turns on PortA and PortC clocks
//  					2. sets PortA bits 0..7 as digital outputs (data bus)
//  					3. sets PortB bits 0..5 as digital outputs (control lines)
//  					4. idles all control lines
//            5. resets the ILI9326 LCD controller
//  					6. sets up all required ILI9325 registers
//  					7. turns on the display
//
//   Parameters:  none
//
//   Returns:     nothing
// *************************************************************************************************************
void lcdInitDisplay(void)
{
	uint32_t	Count;			// loop counter

	static const uint16_t  regValues[51][2] =
		{
			{TFTLCD_START_OSC, 0x0001},				// R00h - start oscillator (need delay after)
			{TFTLCD_DELAYCMD, 50},					// delay 50 msec
			{TFTLCD_DRIV_OUT_CTRL, 0x0100},			// R01h - Driver Output Control
			{TFTLCD_DRIV_WAV_CTRL,0x0700},			// R02h - LCD Driving Control
			{TFTLCD_ENTRY_MOD,0x1030},				// R03h - Entry Mode
			{TFTLCD_RESIZE_CTRL,0x0000},			// R04h - Resize Control
			{TFTLCD_DISP_CTRL2,0x0202},				// R08h - Display Control 2
			{TFTLCD_DISP_CTRL3,0x0000},				// R09h - Display Control 3
			{TFTLCD_DISP_CTRL4,0x0000},				// R0Ah - Display Control 4
			{TFTLCD_RGB_DISP_IF_CTRL1,0x0000},		// R0Ch - RGB Display Interface Control 1
			{TFTLCD_FRM_MARKER_POS,0x0000},			// R0Dh - Frame Maker Position
			{TFTLCD_RGB_DISP_IF_CTRL2,0x0000},		// R0Fh - RGB Display Interface Control 2
			{TFTLCD_POW_CTRL1,0x0000},				// R10h - Power Control 1
			{TFTLCD_POW_CTRL2,0x0007},				// R11h - Power Control 2
			{TFTLCD_POW_CTRL3,0x0000},				// R12h - Power Control 3
			{TFTLCD_POW_CTRL4,0x0000},				// R13h - Power Control 4
			{TFTLCD_DELAYCMD, 200},					// delay 200 msec
			{TFTLCD_POW_CTRL1,0x1690},				// R10h - Power Control 1
			{TFTLCD_POW_CTRL2,0x0227},				// R11h - Power Control 2
			{TFTLCD_DELAYCMD, 50},					// delay 50 msec
			{TFTLCD_POW_CTRL3,0x001A},				// R12h - Power Control 3
			{TFTLCD_DELAYCMD, 50},					// delay 50 msec
			{TFTLCD_POW_CTRL4,0x1800},				// R13h - Power Control 4
			{TFTLCD_POW_CTRL7,0x002A},				// R29h - Power Control 7
			{TFTLCD_DELAYCMD, 50},					// delay 50 msec
			{TFTLCD_GAMMA_CTRL1,0x0000},			// R30h - Gamma Control 1
			{TFTLCD_GAMMA_CTRL2,0x0000},			// R31h - Gamma Control 2
			{TFTLCD_GAMMA_CTRL3,0x0000},			// R32h - Gamma Control 3
			{TFTLCD_GAMMA_CTRL4,0x206},				// R35h - Gamma Control 4
			{TFTLCD_GAMMA_CTRL5,0x808},				// R36h - Gamma Control 5
			{TFTLCD_GAMMA_CTRL6,0x0007},			// R37h - Gamma Control 6
			{TFTLCD_GAMMA_CTRL7,0x0201},			// R38h - Gamma Control 7
			{TFTLCD_GAMMA_CTRL8,0x0000},			// R39h - Gamma Control 8
			{TFTLCD_GAMMA_CTRL9,0x0000},			// R3Ch - Gamma Control 9
			{TFTLCD_GAMMA_CTRL10,0x0000},			// R3Dh - Gamma Control 10
			{TFTLCD_GRAM_HOR_AD,0x0000},			// R20h - Horizontal GRAM Address
			{TFTLCD_GRAM_VER_AD,0x0000},			// R21h - Vertical GRAM Address
			{TFTLCD_HOR_START_AD,0x0000},			// R50h - Horizontal Address Start Position
			{TFTLCD_HOR_END_AD,0x00EF},        		// R51h - Horizontal Address End Position
			{TFTLCD_VER_START_AD,0x0000},        	// R52h - Vertical Address Start Position
			{TFTLCD_VER_END_AD,0x13F},        		// R53h - Vertical Address End Position
			{TFTLCD_GATE_SCAN_CTRL1,0xA700},		// R60h - Driver Output Control 2
			{TFTLCD_GATE_SCAN_CTRL2,0x0003},		// R61h - Base Image Display Control
			{TFTLCD_GATE_SCAN_CTRL3,0x0000},   		// R6Ah - Vertical Scroll Control
			{TFTLCD_PANEL_IF_CTRL1,0x0010},			// R90h - Panel Interface Control 1
			{TFTLCD_PANEL_IF_CTRL2,0x0000},       	// R92h - Panel Interface Control 2
			{TFTLCD_PANEL_IF_CTRL3,0x0003},       	// R93h - Panel Interface Control 3
			{TFTLCD_PANEL_IF_CTRL4,0x1100},      	// R94h - Panel Interface Control 4
			{TFTLCD_PANEL_IF_CTRL5,0x0000},       	// R95h - Panel Interface Control 5
    		{TFTLCD_PANEL_IF_CTRL6,0x0000},       	// R96h - Panel Interface Control 6
    		{TFTLCD_DISP_CTRL1, 0x0133}      		// R07h - Display Control 1 - display on
		};

    // Enable GPIO Clock
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE);

  LCD_IO_Configuration();
		
	//  reset the LCD controller
	//reset();

	// set the RD, WR, CD, and CS control lines to the Idle state
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RS));

	// Set-up all ILI9325/ILI9328 registers per LadyAda's stored table regValues[51][2]
	// LadyAda's table-driven method is very space-efficient!
	for (Count = 0; Count < (sizeof(regValues) / 4); Count++)
		{
		if (regValues[Count][0] == TFTLCD_DELAYCMD) {
			lcdDelay(regValues[Count][1]);
		} else {
			writeRegister(regValues[Count][0], regValues[Count][1]);
		}
	}
}


// ****************************************************************************
//   reset -  resets the ILI9325 LCD controller
//
//   Parameters:  none
//
//   Returns:     nothing
// ****************************************************************************
void reset(void)
{
	//  reset the LCD controller
	//FIO_SetValue(LCD_PORT0, _BIT(LCD_RST));			// set RST high
	lcdDelay(1);								    // delay 1 millisecond
	//FIO_ClearValue(LCD_PORT0, _BIT(LCD_RST));		// set RST low
	lcdDelay(10);									// delay 10 milliseconds
	//FIO_SetValue(LCD_PORT0, _BIT(LCD_RST));			// set RST high
	lcdDelay(50);								    // delay 50 milliseconds

	// re-sync
	writeData(0);
	writeData(0);
	writeData(0);
	writeData(0);
}


// *********************************************************************************************
//   setRotation -  Sets the GRAM access writing direction
//
//   Parameters:  dir: access writing direction (0..3).
//
//   Returns:     Nothing
//
//   Note:  default direction is 3 where (0,0) is top left and
//          pixel writing is from left to right then down.
//
//
//    (239,319)|----------------------|(000,319)   (000,000)|----------------------|(000,000)
//             |E@<-----------------@ |                     | @   @              @E|
//           Y |         ::           |                     | ^   ^              ^ |
//           | |         ::           |                     | |   |    ..        | |
//           | | @<-----------------@ |                     | |   |    ..        | |
//           V |                      |                     | |   |              | |
//             | @<-----------------@B|                     |B@   @              @ |
//    (239,000)|----------------------|(000,000)   (000,000)|----------------------|(000,000)
//
//                      dir = 0                                       dir = 1
//
//
//                                                 ( x,  y )        x ---->
//             |----------------------|            (000,000)|----------------------|(000,319)
//             | @              @   @B|                     |B@----------------->@ |
//             | |              |   | |                   y |                      |
//             | |      ..      |   | |                   | | @----------------->@ |
//             | |      ..      |   | |                   | |          :           |
//             | V              V   V |                   v |          :           |
//             |E@              @   @ |                     | @----------------->@E|
//             |----------------------|            (239,000)|----------------------|(239,319)
//                    dir = 2                                   dir = 3 (DEFAULT)
//
// *********************************************************************************************
void setRotation(uint8_t dir)
{
	rotation = dir;
    switch (dir) {
    case 0:
      writeRegister(TFTLCD_ENTRY_MOD, 0x1000);
      _width = TFTLCD_WIDTH;
      _height = TFTLCD_HEIGHT;
      break;
    case 1:
      _width = TFTLCD_WIDTH;
      _height = TFTLCD_HEIGHT;
      writeRegister(TFTLCD_ENTRY_MOD, 0x1018);
      break;
    case 2:
      _width = TFTLCD_WIDTH;
      _height = TFTLCD_HEIGHT;
      writeRegister(TFTLCD_ENTRY_MOD, 0x1028);
      break;
    case 3:
      _width = TFTLCD_WIDTH;
      _height = TFTLCD_HEIGHT;
      writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
      break;
    default:
      break;
    }
}


// ****************************************************************************
//   setRotation -  returns the GRAM access writing direction
//
//   Parameters:  none
//
//   Returns:     access writing direction (0..3).
// ****************************************************************************
uint8_t getRotation(void)
{
  return rotation;
}


// **************************************************************************************
//   writeCommand -  select the ILI9325 index register prior to read/write operations.
//
//   Parameters:  c:   address of the selected register (00h..A5h)
//
//   Returns:     Nothing
// **************************************************************************************
void writeCommand(uint16_t c)
{
	
	uint32_t temp;
	temp = c;
	
	// set the RD and WR to the Idle state
	// clear CS (selects the chip) and clear CD (allows selection of index register)
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_RS));

	

	//LPC_GPIO2->FIOPIN =  temp & 0x000000ff;        /* Write D0..D7 */
	//LPC_GPIO0->FIOPIN =  (temp << 7) & 0x007F8000; /* Write D8..D15 */
	LPC_GPIO2->FIOPIN =  (LPC_GPIO2->FIOPIN & (~(0x000000FF))) | (temp & 0x00FF);      						  /* Write D0..D7 */
	LPC_GPIO0->FIOPIN =  (LPC_GPIO0->FIOPIN & (~(0x007F8000))) | ((temp << 7) & 0x007F8000);        /* Write D8..D15 */

	// strobe the WR write line  (data is latched on the rising-edge)
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));

	// all done, set CS (chip select) and CD (register select) high
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RS));
}


// ****************************************************************************
//   readData -  Reads from the LCD RAM.
//
//   Parameters:  none
//
//   Returns:     nothing
//
//   Note: This assumes that you have already called writeCommand(register)
//         to select the GRAM register.
// ****************************************************************************
uint16_t readData(void)
{
	  uint32_t low,high;

    // set the RD, WR, and RS to the Idle state
    // clear CS (selects the chip)
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_RS));

    
	LPC_GPIO2->FIODIR &= ~(0x000000FF);             /* P2.0...P2.7   Input DB[0..7] */
	LPC_GPIO0->FIODIR &= ~(0x007F8000); 						/* P0.15...P0.22 Input DB[8..15]*/

    // clear the RD read line to latch the upper 8-bits
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_RD));
  
	
	low  = LPC_GPIO2->FIOPIN & 0x000000ff;          /* Read D0..D7 */
	high = LPC_GPIO0->FIOPIN & 0x007f8000;          /* Read D8..D15 */
	low |= (high >> 7);

    // set the RD line
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));

  LPC_GPIO2->FIODIR |= 0x000000FF;                /* P2.0...P2.7   Output DB[0..7] */
	LPC_GPIO0->FIODIR |= 0x007F8000; 								/* P0.15...P0.22 Output DB[8..15]*/

    // all done, set CS (chip select) and RS (register select) high
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));

    return low;
}


// ****************************************************************************
//   writeData -  Writes to the LCD RAM.
//
//   Parameters:  d: the pixel color in RGB mode (5-6-5).
//
//   Returns:     Nothing
//
//   Note: This assumes that you have already called writeCommand(register)
//         to select the GRAM register.
// ****************************************************************************
void writeData(uint16_t d)
{
  uint32_t temp;
	temp = d;   
	// set the RD, WR, and RS to the Idle state
    // clear CS (selects the chip)
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_RS));

	LPC_GPIO2->FIODIR |= 0x000000FF;                /* P2.0...P2.7   Output DB[0..7] */
	LPC_GPIO0->FIODIR |= 0x007F8000; 								/* P0.15...P0.22 Output DB[8..15]*/
	

	//LPC_GPIO2->FIOPIN =  temp & 0x000000ff;        /* Write D0..D7 */
	//LPC_GPIO0->FIOPIN =  (temp << 7) & 0x007F8000; /* Write D8..D15 */
	LPC_GPIO2->FIOPIN =  (LPC_GPIO2->FIOPIN & (~(0x000000FF))) | (temp & 0x00FF);      						  /* Write D0..D7 */
	LPC_GPIO0->FIOPIN =  (LPC_GPIO0->FIOPIN & (~(0x007F8000))) | ((temp << 7) & 0x007F8000);        /* Write D8..D15 */

	// strobe the WR write line  (data is latched on the rising-edge)
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));

	// de-select the chip
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));

}


// *********************************************************************************
//   writeData_unsafe -  Writes to the LCD RAM.
//
//   Parameters:  d: the pixel color in RGB mode (5-6-5).
//
//   Returns:     Nothing
//
//   Note: This assumes that you have already called writeCommand(register)
//         to select the GRAM register (R22h).
//
//         This also assumes that:  LCD_RD = Bit_SET
//  	                            LCD_RS = Bit_SET
//  	                            LCD_CS = Bit_RESET
//
//  	   The data bus is always in "write" mode, except during readRegister()
//         where it is switched to "read" mode then restored to "write" mode
//         before exiting.
// *********************************************************************************
void writeData_unsafe(uint16_t d)
{
	uint32_t temp;
	temp = d;

	//LPC_GPIO2->FIOPIN =  temp & 0x000000ff;        /* Write D0..D7 */
	//LPC_GPIO0->FIOPIN =  (temp << 7) & 0x007F8000; /* Write D8..D15 */
	LPC_GPIO2->FIOPIN =  (LPC_GPIO2->FIOPIN & (~(0x000000FF))) | (temp & 0x00FF);      						  /* Write D0..D7 */
	LPC_GPIO0->FIOPIN =  (LPC_GPIO0->FIOPIN & (~(0x007F8000))) | ((temp << 7) & 0x007F8000);        /* Write D8..D15 */

	// strobe the WR write line  (data is latched on the rising-edge)
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));
}


// ****************************************************************************
//   readRegister -  Reads the selected LCD Register.
//
//   Parameters:  addr:   address of the selected register.
//
//   Returns:     Value:  16-bit register value
// ****************************************************************************
uint16_t readRegister(uint16_t addr)
{
	uint16_t	Data = 0;

	writeCommand(addr);
	Data = readData();
	return(Data);
}


// ****************************************************************************
//   writeRegister -  Writes the selected ILI9325/ILI9328 register.
//
//   Parameters:  addr:   address (index) of the selected register.
//                data:   16-bit data to be written.
//
//   Returns:     nothing
// ****************************************************************************
void writeRegister(uint16_t addr, uint16_t data)
{
	
	uint32_t temp1,temp2;
	temp1 = addr;temp2 = data;
	
	// set the RD and WR to the Idle state
	// clear CS (selects the chip) and clear RS (allows selection of index register)
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RD));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_ClearValue(LCD_PORT2, _BIT(LCD_CS));
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_RS));

	//LPC_GPIO2->FIOPIN =  temp1 & 0x000000ff;        /* Write D0..D7 */
	//LPC_GPIO0->FIOPIN =  (temp1 << 7) & 0x007F8000; /* Write D8..D15 */
  LPC_GPIO2->FIOPIN =  (LPC_GPIO2->FIOPIN & (~(0x000000FF))) | (temp1 & 0x00FF);      						  /* Write D0..D7 */
	LPC_GPIO0->FIOPIN =  (LPC_GPIO0->FIOPIN & (~(0x007F8000))) | ((temp1 << 7) & 0x007F8000);        /* Write D8..D15 */

	// strobe the WR write line  (data is latched on the rising-edge)
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));

	// index register selected, set RS (register select) high
	FIO_SetValue(LCD_PORT1, _BIT(LCD_RS));


	//LPC_GPIO2->FIOPIN =  temp2 & 0x000000ff;        /* Write D0..D7 */
	//LPC_GPIO0->FIOPIN =  (temp2 << 7) & 0x007F8000; /* Write D8..D15 */
	LPC_GPIO2->FIOPIN =  (LPC_GPIO2->FIOPIN & (~(0x000000FF))) | (temp2 & 0x00FF);      						  /* Write D0..D7 */
	LPC_GPIO0->FIOPIN =  (LPC_GPIO0->FIOPIN & (~(0x007F8000))) | ((temp2 << 7) & 0x007F8000);        /* Write D8..D15 */

	// strobe the WR write line  (data is latched on the rising-edge)
	FIO_ClearValue(LCD_PORT1, _BIT(LCD_WR));
	FIO_SetValue(LCD_PORT1, _BIT(LCD_WR));

	// de-select the chip
	FIO_SetValue(LCD_PORT2, _BIT(LCD_CS));
}



// ****************************************************************************
//   lcdDelay -  Inserts a delay time. (warning: this hog-ties the CPU)
//
//   Parameters:  nCount: specifies the delay time in msec.
//
//   Returns:     Nothing
// ****************************************************************************
void lcdDelay(uint32_t nCount)
{
  volatile uint32_t index = 0;
  for(index = (2400 * nCount); index != 0; index--)
  {
  }
}



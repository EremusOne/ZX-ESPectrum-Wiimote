///////////////////////////////////////////////////////////////////////////////
//
// ZX-ESPectrum - ZX Spectrum emulator for ESP32
//
// Copyright (c) 2020, 2021 David Crespo [dcrespo3d]
// https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote
//
// Based on previous work by Ramón Martinez, Jorge Fuertes and many others
// https://github.com/rampa069/ZX-ESPectrum
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//

#include "hardconfig.h"
#include <stdio.h>
#include <string.h>

#include "ESPectrum.h"
#include "Mem.h"
#include "Ports.h"

#include "PS2Kbd.h"
#include "CPU.h"
#include "Config.h"
#include "Tape.h"

#pragma GCC optimize ("O3")

///////////////////////////////////////////////////////////////////////////////

#include "Z80_JLS/z80.h"
static bool createCalled = false;
static bool interruptPending = false;

///////////////////////////////////////////////////////////////////////////////
//  AUDIO EMULATION DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

static void audiobuf_init();
static void audiobuf_update(uint32_t pre_tstates, uint32_t tstates);

///////////////////////////////////////////////////////////////////////////////

static uint32_t ts_start;
static uint32_t target_frame_micros;
static uint32_t target_frame_cycles;

static inline void setup_frame_timing(uint32_t _target_frame_cycles, uint32_t _target_frame_micros)
{
    target_frame_micros = _target_frame_micros;
    target_frame_cycles = _target_frame_cycles;
}

static inline void begin_frame_timing()
{
    ts_start = micros();
}

static inline void wait_to_frame_end()
{
    uint32_t ts_current = micros() - ts_start;
    uint32_t ts_target = target_frame_micros;
    if (ts_target > ts_current) {
        uint32_t us_to_wait = ts_target - ts_current;
        if (us_to_wait < target_frame_micros) {
            uint32_t m = micros();
            uint32_t e = (m + us_to_wait);
            if(m > e){ //overflow
                while(micros() > e){
                    NOP();
                }
            }
            while(micros() < e){
                NOP();
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

// machine tstates  f[MHz]   micros
//   48K:   69888 / 3.5    = 19968
//  128K:   70908 / 3.5469 = 19992

uint32_t CPU::statesPerFrame()
{
    if (Config::getArch() == "48K") return 69888;
    else                            return 70908;
}

uint32_t CPU::microsPerFrame()
{
    if (Config::getArch() == "48K") return 19968;
    else                            return 19992;
}

///////////////////////////////////////////////////////////////////////////////

uint32_t CPU::tstates = 0;
uint64_t CPU::global_tstates = 0;


#if (defined(LOG_DEBUG_TIMING) && defined(SHOW_FPS))
uint32_t CPU::framecnt = 0;
#endif

void CPU::setup()
{

    if (!createCalled)
    {
        Z80::create();
        createCalled = true;
    }

    ESPectrum::reset();
}

///////////////////////////////////////////////////////////////////////////////

void CPU::reset() {

    Z80::reset();
    global_tstates = 0;

    setup_frame_timing(statesPerFrame(), microsPerFrame());
}

///////////////////////////////////////////////////////////////////////////////

void CPU::loop()
{
    uint32_t statesInFrame = statesPerFrame();
    tstates = 0;

    begin_frame_timing();
    
    tstates = 0;

    audiobuf_init();
    ESPectrum::syncAudioTask();

	while (tstates < statesInFrame)
	{
    
        #ifdef TAPE_TRAPS
            switch (Z80::getRegPC()) {
            case 0x0556: // START LOAD        
                Tape::romLoading=true;
                if (Tape::tapeStatus!=TAPE_LOADING && Tape::tapeFileName!="none") Tape::TAP_Play();
                // Serial.printf("------\n");
                // Serial.printf("TapeStatus: %u\n", Tape::tapeStatus);
                break;
            case 0x04d0: // START SAVE (used for rerouting mic out to speaker in Ports.cpp)
                Tape::SaveStatus=TAPE_SAVING; 
                // Serial.printf("------\n");
                // Serial.printf("SaveStatus: %u\n", Tape::SaveStatus);
                break;
            case 0x053f: // END LOAD / SAVE
                Tape::romLoading=false;
                if (Tape::tapeStatus!=TAPE_STOPPED)
                    if (Z80::isCarryFlag()) Tape::tapeStatus=TAPE_PAUSED; else Tape::tapeStatus=TAPE_STOPPED;
                Tape::SaveStatus=SAVE_STOPPED;
                
                /*
                Serial.printf("TapeStatus: %u\n", Tape::tapeStatus);
                Serial.printf("SaveStatus: %u\n", Tape::SaveStatus);
                Serial.printf("Carry Flag: %u\n", Z80::isCarryFlag());            
                Serial.printf("------\n");
                */

                break;
            }
        #endif

            // frame Tstates before instruction
            uint32_t pre_tstates = tstates;
        
            Z80::execute();

            audiobuf_update(pre_tstates, tstates);

            // increase global Tstates
            global_tstates += (tstates - pre_tstates);

	}
    
    #if (defined(LOG_DEBUG_TIMING) && defined(SHOW_FPS))
    framecnt++;
    #endif

    // wait until all frame time has elapsed    
    wait_to_frame_end();

    interruptPending = true;

    // Flashing flag change
    if (halfsec) flashing ^= 0b10000000;
    sp_int_ctr++;
    halfsec = !(sp_int_ctr % 25);
}

///////////////////////////////////////////////////////////////////////////////
//  AUDIO EMULATION
///////////////////////////////////////////////////////////////////////////////

static uint8_t audiobufA[AUDIOBUF_SIZE];
static uint8_t audiobufB[AUDIOBUF_SIZE];
static uint8_t* audiobuf_ptr;

uint8_t* audiobuf_produce = audiobufA;
uint8_t* audiobuf_consume = audiobufB;

static void audiobuf_init()
{
    if (audiobuf_produce == audiobufA) {
        audiobuf_produce = audiobufB;
        audiobuf_consume = audiobufA;
    }
    else {
        audiobuf_produce = audiobufA;
        audiobuf_consume = audiobufB;
    }
}

static void audiobuf_update(uint32_t pre_tstates, uint32_t tstates)
{
    uint32_t beginSample = pre_tstates >> AUDIOBUF_ROTR;
    uint32_t endSample = tstates >> AUDIOBUF_ROTR;

    for (uint32_t i = beginSample; i < endSample && i < AUDIOBUF_SIZE; i++) {
        audiobuf_produce[i] = ESPectrum::beeperBit;
    }
}

///////////////////////////////////////////////////////////////////////////////
//
// Delay Contention: for emulating CPU slowing due to sharing bus with ULA
// NOTE: Only 48K spectrum contention implemented. This function must be called
// only when dealing with affected memory (use ADDRESS_IN_LOW_RAM macro)
//
// delay contention: emulates wait states introduced by the ULA (graphic chip)
// whenever there is a memory access to contended memory (shared between ULA and CPU).
// detailed info: https://worldofspectrum.org/faq/reference/48kreference.htm#ZXSpectrum
// from paragraph which starts with "The 50 Hz interrupt is synchronized with..."
// if you only read from https://worldofspectrum.org/faq/reference/48kreference.htm#Contention
// without reading the previous paragraphs about line timings, it may be confusing.
//
static unsigned char wait_states[8] = { 6, 5, 4, 3, 2, 1, 0, 0 }; // sequence of wait states
static unsigned char IRAM_ATTR delayContention(unsigned int currentTstates)
{

    // delay states one t-state BEFORE the first pixel to be drawn
    currentTstates++;

	// each line spans 224 t-states
	unsigned short int line = currentTstates / 224; // int line

    // only the 192 lines between 64 and 255 have graphic data, the rest is border
	if (line < 64 || line >= 256) return 0;

	// only the first 128 t-states of each line correspond to a graphic data transfer
	// the remaining 96 t-states correspond to border
	unsigned char halfpix = currentTstates % 224; // int halfpix

	if (halfpix >= 128) return 0;

    return(wait_states[halfpix & 0x07]);

}

///////////////////////////////////////////////////////////////////////////////
// Port Contention
///////////////////////////////////////////////////////////////////////////////
static void ALUContentEarly( uint16_t port )
{

    if ( ( port & 49152 ) == 16384 )
        ALU_video(delayContention(CPU::tstates)+1);
    else
        ALU_video(1);

}

void AluContentLate( uint16_t port )
{

  if( (port & 0x0001) == 0x00) {
        ALU_video(delayContention(CPU::tstates)+3);
  } else {
    if ( (port & 49152) == 16384 ) {
   		ALU_video(delayContention(CPU::tstates)+1);
	    ALU_video(delayContention(CPU::tstates)+1);
		ALU_video(delayContention(CPU::tstates)+1);
	} else {
	    ALU_video(3);
	}
 
  }

}
///////////////////////////////////////////////////////////////////////////////

/* Read opcode from RAM */
uint8_t IRAM_ATTR Z80Ops::fetchOpcode(uint16_t address) {
    // 3 clocks to fetch opcode from RAM and 1 execution clock
    if (ADDRESS_IN_LOW_RAM(address))
        ALU_video(delayContention(CPU::tstates) + 4);
    else
        ALU_video(4);

    return Mem::readbyte(address);
}

/* Read/Write byte from/to RAM */
uint8_t IRAM_ATTR Z80Ops::peek8(uint16_t address) {
    // 3 clocks for read byte from RAM
    if (ADDRESS_IN_LOW_RAM(address))
        ALU_video(delayContention(CPU::tstates) + 3);
    else
        ALU_video(3);

    return Mem::readbyte(address);
}

void IRAM_ATTR Z80Ops::poke8(uint16_t address, uint8_t value) {
    if (ADDRESS_IN_LOW_RAM(address)) {
       
        ALU_video(delayContention(CPU::tstates) + 3);
        
        // #ifndef NO_VIDEO
        // if (address < 0x5800) { // Bitmap
        //     lineChanged[address & 0x3fff] |= 0x01;
        // } else if (address < 0x5b00) { // Attr
        //     byte chgdata;
        //     uint16_t addrvid = address & 0x3fff;
        //     int rowbase = ((addrvid - 0x1800) >> 5) << 3;
        //     if (value & 0x80) chgdata=0x03; else chgdata=0x01; // Flashing bit on
        //     for (int i=0;i<8;i++){
        //         lineChanged[offBmp[rowbase + i] + (addrvid & 0x1f)] |= chgdata;
        //     }
        // }
        // #endif

    } else 
        #ifdef BORDER_EFFECTS
            ALU_video(3);
        #else
            //ALU_video(3); // I've changed this to CPU::tstates direct increment. All seems working OK. Investigate.
            CPU::tstates+=3;
        #endif

    Mem::writebyte(address, value);
}

/* Read/Write word from/to RAM */
uint16_t IRAM_ATTR Z80Ops::peek16(uint16_t address) {

    // Order matters, first read lsb, then read msb, don't "optimize"
    uint8_t lsb = Z80Ops::peek8(address);
    uint8_t msb = Z80Ops::peek8(address + 1);

    return (msb << 8) | lsb;
}

void IRAM_ATTR Z80Ops::poke16(uint16_t address, RegisterPair word) {
    // Order matters, first write lsb, then write msb, don't "optimize"
    Z80Ops::poke8(address, word.byte8.lo);
    Z80Ops::poke8(address + 1, word.byte8.hi);
}

/* In/Out byte from/to IO Bus */
uint8_t IRAM_ATTR Z80Ops::inPort(uint16_t port) {
    
    #ifdef BORDER_EFFECTS
        ALUContentEarly( port );   // Contended I/O
        AluContentLate( port );    // Contended I/O
    #else
        // 3 clocks for read byte from bus (4 according to https://worldofspectrum.org/faq/reference/48kreference.htm#IOContention)
        //ALU_video(4); // No contended I/O
        CPU::tstates+=4; // I've changed this to CPU::tstates direct increment. All seems working OK. Investigate.
    #endif

    uint8_t hiport = port >> 8;
    uint8_t loport = port & 0xFF;
    return Ports::input(loport, hiport);

}

void IRAM_ATTR Z80Ops::outPort(uint16_t port, uint8_t value) {
    
    #ifdef BORDER_EFFECTS
        ALUContentEarly( port );   // Contended I/O
    #else
        // 4 clocks for write byte to bus
        //ALU_video(4); // No contended I/O
        CPU::tstates+=4; // I've changed this to CPU::tstates direct increment. All seems working OK. Investigate.
    #endif

    uint8_t hiport = port >> 8;
    uint8_t loport = port & 0xFF;
    Ports::output(loport, hiport, value);

    #ifdef BORDER_EFFECTS
        AluContentLate( port );    // Contended I/O
    #endif
}

/* Put an address on bus lasting 'tstates' cycles */
void IRAM_ATTR Z80Ops::addressOnBus(uint16_t address, int32_t wstates){

    #ifdef BORDER_EFFECTS
        // Additional clocks to be added on some instructions
        if (ADDRESS_IN_LOW_RAM(address)) {
            for (int idx = 0; idx < wstates; idx++)
                ALU_video(delayContention(CPU::tstates) + 1);
        }
        else
            ALU_video(wstates);
    #else
        // Additional clocks to be added on some instructions
        if (ADDRESS_IN_LOW_RAM(address)) {
            for (int idx = 0; idx < wstates; idx++)
                CPU::tstates += delayContention(CPU::tstates) + 1;
                //ALU_video(delayContention(CPU::tstates) + 1); // I've changed this to CPU::tstates direct increment. All seems working OK. Investigate.
        }
        else
            //ALU_video(wstates); // I've changed this to CPU::tstates direct increment. All seems working OK. Investigate.
            CPU::tstates+=wstates;
    #endif
}

/* Clocks needed for processing INT and NMI */
void IRAM_ATTR Z80Ops::interruptHandlingTime(int32_t wstates) {

    #ifdef BORDER_EFFECTS
        ALU_video(wstates);
    #else
        //ALU_video(wstates); // I've changed this to CPU::tstates direct increment. All seems working OK. Investigate.
        CPU::tstates+=wstates;
    #endif

}

/* Callback to know when the INT signal is active */
bool IRAM_ATTR Z80Ops::isActiveINT(void) {
    if (!interruptPending) return false;
    interruptPending = false;
    return true;
}

///////////////////////////////////////////////////////////////////////////////
//  VIDEO EMULATION
///////////////////////////////////////////////////////////////////////////////
#define NUM_SPECTRUM_COLORS 16
static word spectrum_colors[NUM_SPECTRUM_COLORS] = {
    BLACK,     BLUE,     RED,     MAGENTA,     GREEN,     CYAN,     YELLOW,     WHITE,
    BRI_BLACK, BRI_BLUE, BRI_RED, BRI_MAGENTA, BRI_GREEN, BRI_CYAN, BRI_YELLOW, BRI_WHITE,
};

static word specfast_colors[128]; // Array for faster color calc in ALU_video

static unsigned int lastBorder[312]= { 0 };

#define TSTATES_PER_LINE 224

#define TS_PHASE_1_320x240 8943  // START OF VISIBLE ULA DRAW @ 320x240, SCANLINE 40
#define TS_PHASE_2_320x240 14319 // START OF LEFT BORDER OF TOP LEFT CORNER OF MAINSCREEN, SCANLINE 64
#define TS_PHASE_3_320x240 57327 // START OF BOTTOM BORDER, SCANLINE 256

#define TS_PHASE_1_320x240_BE 8945  // START OF VISIBLE ULA DRAW @ 320x240, SCANLINE 40 (ADDED 2 FOR BORDER EFFECTS TO FIT. NEED TO STUDY)
#define TS_PHASE_3_320x240_BE 57329 // START OF BOTTOM BORDER, SCANLINE 256 (ADDED 2 FOR BORDER EFFECTS TO FIT. NEED TO STUDY)

#define TS_PHASE_1_360x200 13415 // START OF VISIBLE ULA DRAW @ 360x200, SCANLINE 60
#define TS_PHASE_2_360x200 14311 // START OF LEFT BORDER OF TOP LEFT CORNER OF MAINSCREEN, SCANLINE 64
#define TS_PHASE_3_360x200 57319 // START OF BOTTOM BORDER, SCANLINE 256

void precalcColors() {
    for (int i = 0; i < NUM_SPECTRUM_COLORS; i++) {
        spectrum_colors[i] = (spectrum_colors[i] & ESPectrum::vga.RGBAXMask) | ESPectrum::vga.SBits;
    }

    // Calc array for faster color calcs in ALU_video
    for (int i = 0; i < (NUM_SPECTRUM_COLORS >> 1); i++) {
        // Normal
        specfast_colors[i] = spectrum_colors[i];
        specfast_colors[i << 3] = spectrum_colors[i];
        // Bright
        specfast_colors[i | 0x40] = spectrum_colors[i + (NUM_SPECTRUM_COLORS >> 1)];
        specfast_colors[(i << 3) | 0x40] = spectrum_colors[i + (NUM_SPECTRUM_COLORS >> 1)];
    }

}

uint16_t zxColor(uint8_t color, uint8_t bright) {
    if (bright) color += 8;
    return spectrum_colors[color];
}

// Precalc ULA_SWAP
#define ULA_SWAP(y) ((y & 0xC0) | ((y & 0x38) >> 3) | ((y & 0x07) << 3))
void precalcULASWAP() {
    for (int i = 0; i < SPEC_H; i++) {
        offBmp[i] = ULA_SWAP(i) << 5;
        offAtt[i] = ((i >> 3) << 5) + 0x1800;
    }
}

// Precalc border 32 bits values
static unsigned int border32[8];
void precalcborder32()
{
    for (int i = 0; i < 8; i++) {
        uint8_t border = zxColor(i,0);
        border32[i] = border | (border << 8) | (border << 16) | (border << 24);
    }
}

void ALU_video_init() {

    precalcColors();    // precalculate colors for current VGA mode

    precalcULASWAP();   // precalculate ULA SWAP values

    precalcborder32();  // Precalc border 32 bits values

    for (int i=0;i<312;i++) {
        lastBorder[i]=8; // 8 -> Force repaint of border
    }

    is169 = Config::aspect_16_9 ? 1 : 0;
    
    // for (int i=0;i<6144;i++) lineChanged[i]=1; // Mark screen changed array as dirty

}

void ALU_video_reset() {

    for (int i=0;i<312;i++) {
        lastBorder[i]=8; // 8 -> Force repaint of border
    }

    is169 = Config::aspect_16_9 ? 1 : 0;
    
    // for (int i=0;i<6144;i++) lineChanged[i]=1; // Mark screen changed array as dirty

}

///////////////////////////////////////////////////////////////////////////////
//  VIDEO DRAW FUNCTION
///////////////////////////////////////////////////////////////////////////////
static unsigned int bmpOffset;  // offset for bitmap in graphic memory
static unsigned int attOffset;  // offset for attrib in graphic memory
static unsigned int att, bmp;   // attribute and bitmap
static unsigned int palette[2]; //0 backcolor 1 Forecolor
static unsigned int a0,a1,a2,a3;

static uint8_t* grmem;
static uint32_t* lineptr32;

// DrawStatus values
#define TOPBORDER_BLANK 0
#define TOPBORDER 1
#define MAINSCREEN_BLANK 2
#define LEFTBORDER 3
#define LINEDRAW_SYNC 4
#define LINEDRAW 5
#define RIGHTBORDER 6
#define BOTTOMBORDER_BLANK 7
#define BOTTOMBORDER 8
#define BLANK 9
#define FRAMESKIP 10

#ifndef BORDER_EFFECTS
///////////////////////////////////////////////////////////////////////////////
// ALU_video -> Fast Border
///////////////////////////////////////////////////////////////////////////////
static unsigned char DrawStatus=BLANK;
static unsigned int tstateDraw; // Drawing start point (in Tstates)
static unsigned int linedraw_cnt;
static unsigned int mainscrline_cnt;
static unsigned int coldraw_cnt;
static unsigned int ALU_video_rest;
static unsigned int brd;

static void IRAM_ATTR ALU_video(unsigned int statestoadd) {

    CPU::tstates += statestoadd;
    
#ifndef NO_VIDEO

    // if (DrawStatus==FRAMESKIP) {
    //     if (tstates > tstateDraw) {
    //         DrawStatus = BLANK;
    //     }
    //     return;
    // }
    
    if (DrawStatus==TOPBORDER) {

        if (CPU::tstates > tstateDraw) {

            tstateDraw += TSTATES_PER_LINE;

            brd = ESPectrum::borderColor;
            if (lastBorder[linedraw_cnt] != brd) {

                lineptr32 = (uint32_t *)(ESPectrum::vga.backBuffer[linedraw_cnt]);
                if (is169) lineptr32++; // Border offset for 360 x 200

                for (int i=0; i < (is169 ? 44 : 40) ; i++) {
                    *lineptr32++ = border32[brd];
                    *lineptr32++ = border32[brd];            
                }

                lastBorder[linedraw_cnt]=brd;
            }

            linedraw_cnt++;

            if (linedraw_cnt >= (is169 ? 4 : 24)) {
                DrawStatus = LEFTBORDER;
                tstateDraw = is169 ? TS_PHASE_2_360x200 : TS_PHASE_2_320x240;
            }

        }

        return;

    }

    if (DrawStatus==LEFTBORDER) {
     
        if (CPU::tstates > tstateDraw) {

            lineptr32 = (uint32_t *)(ESPectrum::vga.backBuffer[linedraw_cnt]);
            if (is169) lineptr32++; // Border offset for 360 x 200

            brd = ESPectrum::borderColor;
            if (lastBorder[linedraw_cnt] != brd) {
                for (int i=0; i < (is169 ? 6 : 4); i++) {    
                    *lineptr32++ = border32[brd];
                    *lineptr32++ = border32[brd];
                }
            } else lineptr32+=(is169 ? 12 : 8);

            if (is169) tstateDraw += 24; else tstateDraw += 16;

            DrawStatus = LINEDRAW_SYNC;

        }
        
        return;

    }    

    if (DrawStatus == LINEDRAW_SYNC) {

        if (CPU::tstates > tstateDraw) {

            statestoadd = CPU::tstates - tstateDraw;

            bmpOffset = offBmp[mainscrline_cnt];
            attOffset = offAtt[mainscrline_cnt];

            grmem = Mem::videoLatch ? Mem::ram7 : Mem::ram5;

            coldraw_cnt = 0;
            ALU_video_rest = 0;
            DrawStatus = LINEDRAW;

        } else return;

    }

    if (DrawStatus == LINEDRAW) {
   
        statestoadd += ALU_video_rest;
        ALU_video_rest = statestoadd & 0x03; // Mod 4

        for (int i=0; i < (statestoadd >> 2); i++) {    

            // if (lineChanged[bmpOffset] !=0) {
            
                att = grmem[attOffset];  // get attribute byte

                // Faster color calc
                if (att & flashing) {
                    palette[0] = specfast_colors[att & 0x47];
                    palette[1] = specfast_colors[att & 0x78];
                } else {
                    palette[0] = specfast_colors[att & 0x78];
                    palette[1] = specfast_colors[att & 0x47];
                }

                bmp = grmem[bmpOffset];  // get bitmap byte

                a0 = palette[(bmp >> 7) & 0x01];
                a1 = palette[(bmp >> 6) & 0x01];
                a2 = palette[(bmp >> 5) & 0x01];
                a3 = palette[(bmp >> 4) & 0x01];
                *lineptr32++ = a2 | (a3<<8) | (a0<<16) | (a1<<24);

                a0 = palette[(bmp >> 3) & 0x01];
                a1 = palette[(bmp >> 2) & 0x01];
                a2 = palette[(bmp >> 1) & 0x01];
                a3 = palette[bmp & 0x01];
                *lineptr32++ = a2 | (a3<<8) | (a0<<16) | (a1<<24);

            //     lineChanged[bmpOffset] &= 0xfe;
            // } else {
            //     lineptr32 += 2;
            // }

            attOffset++;
            bmpOffset++;

            coldraw_cnt++;

            if (coldraw_cnt >= 32) {
                DrawStatus = RIGHTBORDER;
                return;
            }

        }

        return;

    }

    if (DrawStatus==RIGHTBORDER) {
     
        if (lastBorder[linedraw_cnt] != brd) {
            for (int i=0; i < (is169 ? 6 : 4); i++) {
                *lineptr32++ = border32[brd];
                *lineptr32++ = border32[brd];
            }
            lastBorder[linedraw_cnt]=brd;            
        }

        mainscrline_cnt++;
        linedraw_cnt++;

        if (mainscrline_cnt < 192) {
            DrawStatus = LEFTBORDER;
            if (is169) tstateDraw += 200; else tstateDraw += 208;
        } else {
            mainscrline_cnt = 0;
            DrawStatus = BOTTOMBORDER;
            tstateDraw = is169 ? TS_PHASE_3_360x200 : TS_PHASE_3_320x240;
        }

        return;

    }    

    if (DrawStatus==BOTTOMBORDER) {

        if (CPU::tstates > tstateDraw) {

            tstateDraw += TSTATES_PER_LINE;

            brd = ESPectrum::borderColor;
            if (lastBorder[linedraw_cnt] != brd) {

                lineptr32 = (uint32_t *)(ESPectrum::vga.backBuffer[linedraw_cnt]);
                if (is169) lineptr32++; // Border offset for 360 x 200

                for (int i=0; i < (is169 ? 44 : 40); i++) {    
                    *lineptr32++ = border32[brd];
                    *lineptr32++ = border32[brd];
                }

                lastBorder[linedraw_cnt]=brd;            
            }

            linedraw_cnt++;
            if (linedraw_cnt > (is169 ? 199 : 239)) {
                linedraw_cnt=0;
                DrawStatus = BLANK;
            }

        }

        return;

    }

    if (DrawStatus==BLANK)
        if (CPU::tstates < TSTATES_PER_LINE) {
//            if ((CPU::framecnt % 3) == 0)
                DrawStatus = TOPBORDER;
//            else
//                DrawStatus = FRAMESKIP;

            tstateDraw = is169 ? TS_PHASE_1_360x200 : TS_PHASE_1_320x240;
        }

#endif

}
#endif

#ifdef BORDER_EFFECTS
///////////////////////////////////////////////////////////////////////////////
// ALU_video -> Multicolour and Border effects support
// only for 4:3 (320x240)
// Works almost perfect with Bordertrix demo but it is slow
///////////////////////////////////////////////////////////////////////////////
static unsigned char DrawStatus=BLANK;
static unsigned int tstateDraw; // Drawing start point (in Tstates)
static unsigned int linedraw_cnt;
static unsigned int coldraw_cnt;
static unsigned int ALU_video_rest;
static unsigned int brd;

static void IRAM_ATTR ALU_video(unsigned int statestoadd) {

    CPU::tstates += statestoadd;
    
#ifndef NO_VIDEO

    if (DrawStatus==TOPBORDER_BLANK) {

        if (CPU::tstates > tstateDraw) {

            statestoadd = CPU::tstates - tstateDraw;
            tstateDraw += TSTATES_PER_LINE;

            lineptr32 = (uint32_t *)(ESPectrum::vga.backBuffer[linedraw_cnt]);
            coldraw_cnt = 0;

            DrawStatus = TOPBORDER;

        }

    }

    if (DrawStatus==TOPBORDER) {

        statestoadd += ALU_video_rest;
        ALU_video_rest = statestoadd & 0x03; // Mod 4

        brd = ESPectrum::borderColor;

        for (int i=0; i < (statestoadd >> 2); i++) {

            *lineptr32++ = border32[brd];
            *lineptr32++ = border32[brd];            

            coldraw_cnt++;

            if (coldraw_cnt > 39) {
                DrawStatus = TOPBORDER_BLANK;
                ALU_video_rest=0;
                linedraw_cnt++;
                if (linedraw_cnt > 23) {
                    DrawStatus = MAINSCREEN_BLANK;
                    tstateDraw= TS_PHASE_2_320x240;
                }
                return;
            }

        }

        return;

    }

    if (DrawStatus==MAINSCREEN_BLANK) {
     
        if (CPU::tstates > tstateDraw) {

            statestoadd = CPU::tstates - tstateDraw;
            tstateDraw += TSTATES_PER_LINE;
            lineptr32 = (uint32_t *)(ESPectrum::vga.backBuffer[linedraw_cnt]);
            coldraw_cnt = 0;
            DrawStatus = LEFTBORDER;

        }

    }    

    if (DrawStatus==LEFTBORDER) {
     
        statestoadd += ALU_video_rest;
        ALU_video_rest = statestoadd & 0x03; // Mod 4
        
        brd = ESPectrum::borderColor;

        for (int i=0; i < (statestoadd >> 2); i++) {    

            *lineptr32++ = border32[brd];
            *lineptr32++ = border32[brd];

            coldraw_cnt++;

            if (coldraw_cnt > 3) {  
                
                DrawStatus = LINEDRAW;
                
                bmpOffset = offBmp[linedraw_cnt - 24];
                attOffset = offAtt[linedraw_cnt - 24];

                grmem = Mem::videoLatch ? Mem::ram7 : Mem::ram5;

                // This is, maybe, more correct but it seems not needed
                // statestoadd -= ((i + 1) << 2);
                // ALU_video_rest += statestoadd;

                ALU_video_rest = 0;

                return;

            }

        }

        return;

    }    

    if (DrawStatus == LINEDRAW) {

        statestoadd += ALU_video_rest;
        ALU_video_rest = statestoadd & 0x03; // Mod 4

        for (int i=0; i < (statestoadd >> 2); i++) {    

            // if (lineChanged[bmpOffset] !=0) {
            
                att = grmem[attOffset];  // get attribute byte

                // Faster color calc
                if (att & flashing) {
                    palette[0] = specfast_colors[att & 0x47];
                    palette[1] = specfast_colors[att & 0x78];
                } else {
                    palette[0] = specfast_colors[att & 0x78];
                    palette[1] = specfast_colors[att & 0x47];
                }

                bmp = grmem[bmpOffset];  // get bitmap byte

                a0 = palette[(bmp >> 7) & 0x01];
                a1 = palette[(bmp >> 6) & 0x01];
                a2 = palette[(bmp >> 5) & 0x01];
                a3 = palette[(bmp >> 4) & 0x01];
                *lineptr32++ = a2 | (a3<<8) | (a0<<16) | (a1<<24);

                a0 = palette[(bmp >> 3) & 0x01];
                a1 = palette[(bmp >> 2) & 0x01];
                a2 = palette[(bmp >> 1) & 0x01];
                a3 = palette[bmp & 0x01];
                *lineptr32++ = a2 | (a3<<8) | (a0<<16) | (a1<<24);

            //     lineChanged[bmpOffset] &= 0xfe;
            // } else {
            //     lineptr32 += 2;
            // }

            attOffset++;
            bmpOffset++;

            coldraw_cnt++;

            if (coldraw_cnt > 35) {

                DrawStatus = RIGHTBORDER;

                // // This is, maybe, more correct but it seems not needed
                // statestoadd -= ((i+1) << 2);
                // ALU_video_rest += statestoadd;

                ALU_video_rest = 0;

                return;

            }

        }

        return;

    }

    if (DrawStatus==RIGHTBORDER) {
     
        statestoadd += ALU_video_rest;
        ALU_video_rest = statestoadd & 0x03; // Mod 4

        brd = ESPectrum::borderColor;

        for (int i=0; i < (statestoadd >> 2); i++) {

            *lineptr32++ = border32[brd];
            *lineptr32++ = border32[brd];

            coldraw_cnt++;

            if (coldraw_cnt > 39 ) { 

                DrawStatus = MAINSCREEN_BLANK;
                linedraw_cnt++;
                ALU_video_rest=0;

                if (linedraw_cnt > 215 ) {
                    DrawStatus = BOTTOMBORDER_BLANK;
                    tstateDraw = TS_PHASE_3_320x240_BE;
                }
        
                return;

            }

        }

        return;

    }    

    if (DrawStatus==BOTTOMBORDER_BLANK) {

            if (CPU::tstates > tstateDraw) {

                statestoadd = CPU::tstates - tstateDraw;
                tstateDraw += TSTATES_PER_LINE;

                lineptr32 = (uint32_t *)(ESPectrum::vga.backBuffer[linedraw_cnt]);
                coldraw_cnt = 0;

                DrawStatus = BOTTOMBORDER;

            }

    }

    if (DrawStatus==BOTTOMBORDER) {

        statestoadd += ALU_video_rest;
        ALU_video_rest = statestoadd & 0x03; // Mod 4

        brd = ESPectrum::borderColor;

        for (int i=0; i < (statestoadd >> 2); i++) {    

            *lineptr32++ = border32[brd];
            *lineptr32++ = border32[brd];

            coldraw_cnt++;

            if (coldraw_cnt > 39) {
                DrawStatus = BOTTOMBORDER_BLANK;
                ALU_video_rest=0;
                linedraw_cnt++;
                if (linedraw_cnt > 239) {
                    DrawStatus = BLANK;
                }
                return;
            }

        }

        return;

    }

    if (DrawStatus==BLANK) {
     
         if (CPU::tstates < TSTATES_PER_LINE) {
            linedraw_cnt=0;
            tstateDraw= TS_PHASE_1_320x240_BE;
            DrawStatus=TOPBORDER_BLANK;
        }
    
    }

#endif

}
#endif
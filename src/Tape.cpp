#include "hardpins.h"
#include "FileUtils.h"
#include "CPU.h"
#include "Tape.h"

String Tape::tapeFileName = "none";
byte Tape::tapeStatus = TAPE_STOPPED;
byte Tape::SaveStatus = SAVE_STOPPED;
uint8_t Tape::romLoading = false;

static uint8_t tapePhase;
static uint64_t tapeStart;
static uint32_t tapePulseCount;
static uint16_t tapeBitPulseLen;   
static uint8_t tapeBitPulseCount;     
static uint32_t tapebufByteCount;
static uint16_t tapeHdrPulses;
static uint32_t tapeBlockLen;
static size_t tapeFileSize;   
static uint8_t* tape;
static uint8_t tapeEarBit;
static uint8_t tapeBitMask;    

void Tape::Init()
{
    tape = NULL;
}

boolean Tape::TAP_Load()
{
    if (NULL != tape) {
        free(tape);
        tape = NULL;
    }

    Tape::tapeStatus=TAPE_STOPPED;

    File tapefile = FileUtils::safeOpenFileRead(Tape::tapeFileName);
    size_t filesize = tapefile.size();

    tape = (uint8_t*)ps_calloc(1, filesize);
    if (NULL == tape) {
        tapefile.close();
        return false;
    }

    Serial.printf("Allocated %lu bytes for .TAP\n", filesize);

    tapeFileSize = readBlockFile(tapefile, tape, filesize);
    tapefile.close();
    if (tapeFileSize != filesize) return false;

    return true;
}

void Tape::TAP_Play()
{
    switch (Tape::tapeStatus) {
    case TAPE_STOPPED:
        tapePhase=TAPE_PHASE_SYNC;
        tapePulseCount=0;
        tapeEarBit=LOW;
        tapeBitMask=0x80;
        tapeBitPulseCount=0;
        tapeBitPulseLen=TAPE_BIT0_PULSELEN;
        tapeHdrPulses=TAPE_HDR_LONG;
        tapeBlockLen=(tape[0] | (tape[1] <<8)) + 2;
        tapebufByteCount=2;
        tapeStart=CPU::global_tstates;
        Tape::tapeStatus=TAPE_LOADING;
        break;
    case TAPE_LOADING:
        Tape::tapeStatus=TAPE_PAUSED;
        break;
    case TAPE_PAUSED:
        tapeStart=CPU::global_tstates;        
        Tape::tapeStatus=TAPE_LOADING;
    }
}

uint8_t Tape::TAP_Read()
{
    uint64_t tapeCurrent = CPU::global_tstates - tapeStart;
    
    switch (tapePhase) {
    case TAPE_PHASE_SYNC:
        if (tapeCurrent > TAPE_SYNC_LEN) {
            tapeStart=CPU::global_tstates;
            tapeEarBit ^= 1;
            tapePulseCount++;
            if (tapePulseCount>tapeHdrPulses) {
                tapePulseCount=0;
                tapePhase=TAPE_PHASE_SYNC1;
            }
        }
        break;
    case TAPE_PHASE_SYNC1:
        if (tapeCurrent > TAPE_SYNC1_LEN) {
            tapeStart=CPU::global_tstates;
            tapeEarBit ^= 1;
            tapePhase=TAPE_PHASE_SYNC2;
        }
        break;
    case TAPE_PHASE_SYNC2:
        if (tapeCurrent > TAPE_SYNC2_LEN) {
            tapeStart=CPU::global_tstates;
            tapeEarBit ^= 1;
            if (tape[tapebufByteCount] & tapeBitMask) tapeBitPulseLen=TAPE_BIT1_PULSELEN; else tapeBitPulseLen=TAPE_BIT0_PULSELEN;            
            tapePhase=TAPE_PHASE_DATA;
        }
        break;
    case TAPE_PHASE_DATA:
        if (tapeCurrent > tapeBitPulseLen) {
            tapeStart=CPU::global_tstates;
            tapeEarBit ^= 1;
            tapeBitPulseCount++;
            if (tapeBitPulseCount==2) {
                tapeBitPulseCount=0;
                tapeBitMask = (tapeBitMask >>1 | tapeBitMask <<7);
                if (tapeBitMask==0x80) {
                    tapebufByteCount++;
                    if (tapebufByteCount == tapeBlockLen) {
                        tapePhase=TAPE_PHASE_PAUSE;
                        tapeEarBit=LOW;
                        break;
                    }
                }
                if (tape[tapebufByteCount] & tapeBitMask) tapeBitPulseLen=TAPE_BIT1_PULSELEN; else tapeBitPulseLen=TAPE_BIT0_PULSELEN;
            }
        }
        break;
    case TAPE_PHASE_PAUSE:
        if (tapebufByteCount < tapeFileSize) {
            if (tapeCurrent > TAPE_BLK_PAUSELEN) {
                tapeStart=CPU::global_tstates;
                tapePulseCount=0;
                tapePhase=TAPE_PHASE_SYNC;
                tapeBlockLen+=(tape[tapebufByteCount] | tape[tapebufByteCount + 1] <<8)+ 2;
                tapebufByteCount+=2;
                if (tape[tapebufByteCount]) tapeHdrPulses=TAPE_HDR_SHORT; else tapeHdrPulses=TAPE_HDR_LONG;
            }
        } else Tape::tapeStatus=TAPE_STOPPED;
        return LOW;
    } 
    
#ifdef SPEAKER_PRESENT
    // digitalWrite(SPEAKER_PIN, tapeEarBit); // Send tape load sound to speaker
    ESPectrum::beeperBit = tapeEarBit;
#endif
    
    return tapeEarBit;
}
/*
Copyright (C) 2020 Ismael García-Marlowe

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#include "APU.h"
#include "Bus.h"

void APU::reset() {

    _pulseWaveEngines[0].init(0);
    _pulseWaveEngines[1].init(1);

    setFrameInterruptFlag(false);

}

void APU::clock() {

    sequence_step seqStep = _frameCounterEngine.clock();

    if (seqStep != sequence_step::NONE) {

        if (_frameCounterEngine.mode == 0) {

            if (seqStep == sequence_step::STEP_1
                || seqStep == sequence_step::STEP_2
                || seqStep == sequence_step::STEP_3
                || seqStep == sequence_step::STEP_4P5) {

                _pulseWaveEngines[0].clockEnvelopeUnit();
                _pulseWaveEngines[1].clockEnvelopeUnit();
                _triangleWaveEngine.clockLinearCounterUnit();
                _noiseWaveEngine.clockEnvelopeUnit();

            }

            if (seqStep == sequence_step::CYCLE_0) {
                /*if (!_frameCounterReg.irqInhibit) {
                    setFrameInterruptFlag(true);
                }*/
            }
            else if (seqStep == sequence_step::STEP_2) {

                _pulseWaveEngines[0].clockLengthCounterUnit();
                _pulseWaveEngines[1].clockLengthCounterUnit();
                _pulseWaveEngines[0].clockSweepUnit();
                _pulseWaveEngines[1].clockSweepUnit();

                _triangleWaveEngine.clockLengthCounterUnit();

                _noiseWaveEngine.clockLengthCounterUnit();

            }
            else if (seqStep == sequence_step::STEP_4) {
                if (!_frameCounterReg.irqInhibit) {
                    setFrameInterruptFlag(true);
                }
            }
            else if (seqStep == sequence_step::STEP_4P5) {

                _pulseWaveEngines[0].clockLengthCounterUnit();
                _pulseWaveEngines[1].clockLengthCounterUnit();
                _pulseWaveEngines[0].clockSweepUnit();
                _pulseWaveEngines[1].clockSweepUnit();

                _triangleWaveEngine.clockLengthCounterUnit();

                _noiseWaveEngine.clockLengthCounterUnit();

                if (!_frameCounterReg.irqInhibit) {
                    setFrameInterruptFlag(true);
                }

            }

        }
        else {

            if (seqStep == sequence_step::STEP_1
                || seqStep == sequence_step::STEP_2
                || seqStep == sequence_step::STEP_3
                || seqStep == sequence_step::STEP_5) {

                _pulseWaveEngines[0].clockEnvelopeUnit();
                _pulseWaveEngines[1].clockEnvelopeUnit();
                _triangleWaveEngine.clockLinearCounterUnit();
                _noiseWaveEngine.clockEnvelopeUnit();

            }

            // In this mode, the frame interrupt flag is never set.
            if (seqStep == sequence_step::STEP_2
                || seqStep == sequence_step::STEP_5) {

                _pulseWaveEngines[0].clockLengthCounterUnit();
                _pulseWaveEngines[1].clockLengthCounterUnit();
                _pulseWaveEngines[0].clockSweepUnit();
                _pulseWaveEngines[1].clockSweepUnit();

                _triangleWaveEngine.clockLengthCounterUnit();

                _noiseWaveEngine.clockLengthCounterUnit();

            }

        }

    }

    if (_oddCycle) {
        _pulseWaveEngines[0].clock();
        _pulseWaveEngines[1].clock();
        _triangleWaveEngine.clock();
        _noiseWaveEngine.clock();
    }
    _oddCycle = !_oddCycle;

}

void APU::connectConsole(Bus* bus) {
    _nes = bus;
}

void APU::writePulseWave1Reg1(uint8_t data) {
    *((uint8_t*)&_pulseWave1Reg1) = data;
    setPulseWaveReg1Fields(0, _pulseWave1Reg1);
}

// A channel's second register configures the sweep unit
void APU::writePulseWave1Reg2(uint8_t data) {
    *((uint8_t*)&_pulseWave1Reg2) = data;
    setPulseWaveReg2Fields(0, _pulseWave1Reg2);
}

void APU::writePulseWave1Reg3(uint8_t data) {
    *((uint8_t*)&_pulseWave1Reg3) = data;
    setPulseWaveReg3Fields(0, _pulseWave1Reg3.timerLo);
}

void APU::writePulseWave1Reg4(uint8_t data) {
    *((uint8_t*)&_pulseWave1Reg4) = data;
    setPulseWaveReg4Fields(0, _pulseWave1Reg4);
}

void APU::writePulseWave2Reg1(uint8_t data) {
    *((uint8_t*)&_pulseWave2Reg1) = data;
    setPulseWaveReg1Fields(1, _pulseWave2Reg1);
}

void APU::writePulseWave2Reg2(uint8_t data) {
    *((uint8_t*)&_pulseWave2Reg2) = data;
    setPulseWaveReg2Fields(1, _pulseWave2Reg2);
}

void APU::writePulseWave2Reg3(uint8_t data) {
    *((uint8_t*)&_pulseWave2Reg3) = data;
    setPulseWaveReg3Fields(1, _pulseWave2Reg3.timerLo);
}

void APU::writePulseWave2Reg4(uint8_t data) {
    *((uint8_t*)&_pulseWave2Reg4) = data;
    setPulseWaveReg4Fields(1, _pulseWave2Reg4);
}

// 0x4000 & 0x4004
void APU::setPulseWaveReg1Fields(uint8_t id, pulse_wave_reg1_st reg) {
    // Sequencer
    _pulseWaveEngines[id].waveForm = waveForms[reg.duty];
    // Length counter
    _pulseWaveEngines[id].lengthCounterUnit.halt = reg.lengthCounterHalt == 1;
    // Envelope
    _pulseWaveEngines[id].envelopeUnit.volume = reg.volume; // Aka divider's period
    _pulseWaveEngines[id].envelopeUnit.constantVolumeFlag = reg.constantVolumeFlag == 1;
    // (Note that the bit position for the loop flag is also mapped to a flag in the Length Counter.)
    _pulseWaveEngines[id].envelopeUnit.loopFlag = reg.lengthCounterHalt == 1; // ?
}

// 0x4001 & 0x4005
void APU::setPulseWaveReg2Fields(uint8_t id, pulse_wave_reg2_st reg) {
    _pulseWaveEngines[id].sweepUnit.enabledFlag = reg.enabled == 1;
    _pulseWaveEngines[id].sweepUnit.negateFlag = reg.negate == 1;
    _pulseWaveEngines[id].sweepUnit.sweepPeriod = reg.period + 1; // The divider's period is set to p + 1.
    _pulseWaveEngines[id].sweepUnit.shiftCount = reg.shiftCount;

    // Seen in mesen, search for justification!!
    _pulseWaveEngines[id].sweepUnit.updateTargetPeriod();

    _pulseWaveEngines[id].sweepUnit.reloadFlag = true; // Side-effect
}

// 0x4002 & 0x4006
void APU::setPulseWaveReg3Fields(uint8_t id, uint8_t reg) {
    // Set LO part of pulse timer
    _pulseWaveEngines[id].configureTimerLo(reg);
}

// 0x4003 & 0x4007
void APU::setPulseWaveReg4Fields(uint8_t id, pulse_wave_reg4_st reg) {
    // Reload length counter
    _pulseWaveEngines[id].lengthCounterUnit.configureDivider(reg.lengthCounterLoad);
    // Set HI part of pulse timer
    _pulseWaveEngines[id].configureTimerHi(reg.timerHi);
    // When the fourth register is written to, the sequencer is restarted.
    _pulseWaveEngines[id].restartSequencer();

    _pulseWaveEngines[id].envelopeUnit.startFlag = true;
}

// 0x4008
void APU::writeTriangleWaveReg1(uint8_t data) {
    *((uint8_t*)&_triangleWaveReg1) = data;

    // Linear counter setup
    _triangleWaveEngine.linearCounterUnit.controlFlag = _triangleWaveReg1.lengthCounterHaltAndLinearCounterControl == 1;
    _triangleWaveEngine.linearCounterUnit.configuredCounter = _triangleWaveReg1.linearCounterLoad;
    //_triangleWaveEngine.linearCounterUnit.reload();

    _triangleWaveEngine.lengthCounterUnit.halt = _triangleWaveReg1.lengthCounterHaltAndLinearCounterControl;
}

void APU::writeTriangleWaveReg2(uint8_t data) {}

// 0x400A
void APU::writeTriangleWaveReg3(uint8_t data) {
    _triangleWaveReg3.timerLo = data;
    _triangleWaveEngine.configureTimerLo(_triangleWaveReg3.timerLo);
}

// 0x400B
void APU::writeTriangleWaveReg4(uint8_t data) {
    *((uint8_t*)&_triangleWaveReg4) = data;

    _triangleWaveEngine.configureTimerHi(_triangleWaveReg4.timerHi);

    _triangleWaveEngine.lengthCounterUnit.configureDivider(_triangleWaveReg4.lengthCounterLoad);

    // NESDEV: When register $400B is written to, the halt flag is set.
    // Called reload flag in other emulators.
    _triangleWaveEngine.linearCounterUnit.haltFlag = true;
}

void APU::writeNoiseReg1(uint8_t data) {
    *((uint8_t*)&_noiseReg1) = data;

    // Length counter
    _noiseWaveEngine.lengthCounterUnit.halt = _noiseReg1.envelopeLoopAndLengthCounterHalt == 1;
    // Envelope
    _noiseWaveEngine.envelopeUnit.volume = _noiseReg1.volume; // Aka divider's period
    _noiseWaveEngine.envelopeUnit.constantVolumeFlag = _noiseReg1.constantVolume == 1;
    // (Note that the bit position for the loop flag is also mapped to a flag in the Length Counter.)
    _noiseWaveEngine.envelopeUnit.loopFlag = _noiseReg1.envelopeLoopAndLengthCounterHalt == 1; // ?
}

void APU::writeNoiseReg2(uint8_t data) {
    // Nothing to do
}

void APU::writeNoiseReg3(uint8_t data) {
    *((uint8_t*)&_noiseReg3) = data;

    _noiseWaveEngine.configureTimer(_noiseReg3.noisePeriod);
    _noiseWaveEngine.modeFlag = _noiseReg3.mode == 1;
}

void APU::writeNoiseReg4(uint8_t data) {
    *((uint8_t*)&_noiseReg4) = data;

    _noiseWaveEngine.lengthCounterUnit.configureDivider(_noiseReg4.lengthCounterLoad);
    _noiseWaveEngine.envelopeUnit.startFlag = true;
}

void APU::writeStatusReg(uint8_t data) {
    *((uint8_t*)&_statusWrReg) = data;

    _pulseWaveEngines[0].lengthCounterUnit.setEnabled(_statusWrReg.enablePulseCh1 == 1);
    _pulseWaveEngines[1].lengthCounterUnit.setEnabled(_statusWrReg.enablePulseCh2 == 1);
    _triangleWaveEngine.lengthCounterUnit.setEnabled(_statusWrReg.enableTriangleChl == 1);
    _noiseWaveEngine.lengthCounterUnit.setEnabled(_statusWrReg.enableNoiseChl == 1);
}

void APU::writeFrameCounterReg(uint8_t data) {
    *((uint8_t*)&_frameCounterReg) = data;

    _frameCounterEngine.mode = _frameCounterReg.mode;

    // Interrupt inhibit flag. If set, the frame interrupt flag is cleared,
    // otherwise it is unaffected.
    if (_frameCounterReg.irqInhibit) {
        setFrameInterruptFlag(false);
    }
}

uint8_t APU::readStatusReg() {
    // Reading this register clears the frame interrupt flag.
    // If an interrupt flag was set at the same moment of the read, it will read back as 1 but it will not be cleared.
    _statusRdReg.frameInterrupt = (uint8_t)_frameInterruptFlag;
    setFrameInterruptFlag(false);

    _statusRdReg.pulseCh1LenCntActive = _pulseWaveEngines[0].lengthCounterUnit.divider != 0;
    _statusRdReg.pulseCh2LenCntActive = _pulseWaveEngines[1].lengthCounterUnit.divider != 0;
    _statusRdReg.triangleChLenCntActive = _triangleWaveEngine.lengthCounterUnit.divider != 0;
    _statusRdReg.noiseChLenCntActive = _noiseWaveEngine.lengthCounterUnit.divider != 0;

    return *((uint8_t*)&_statusRdReg);
}

sample_t APU::getOutput() {

    float square1 = 0;
    float square2 = 0;
    float triangle = 0;
    float noise = 0;

    square1 = _pulseWaveEngines[0].output && !_pulseWaveEngines[0].sweepUnit.isMuted() && _pulseWaveEngines[0].lengthCounterUnit.divider ?
        (float)_pulseWaveEngines[0].envelopeUnit.getVolume() : 0;

    square2 = _pulseWaveEngines[1].output && !_pulseWaveEngines[1].sweepUnit.isMuted() && _pulseWaveEngines[1].lengthCounterUnit.divider ?
        (float)_pulseWaveEngines[1].envelopeUnit.getVolume() : 0;

    triangle = (float)_triangleWaveEngine.output;

    noise = _noiseWaveEngine.lengthCounterUnit.divider > 0 && (_noiseWaveEngine.shiftRegister & 0x1) == 0x0 ?
        (float)_noiseWaveEngine.envelopeUnit.volume : 0;

    //square1 = 0;
    //square2 = 0;
    //triangle = 0;
    //noise = 0;

    float squareSum = square1 + square2;

    float squareOutput = 0;
    float tndOutput = 0;

    if (squareSum > 0) {
        squareOutput = (95.88f / ((8128.0f / squareSum) + 100.0f));
    }

    float div = triangle / 8227.0f + noise / 12241.0f;
    if (div > 0) {
        tndOutput = 159.79f / ((1.0f / div) + 100.0f);
    }

    return (sample_t)((squareOutput + tndOutput) * AMPLITUDE);

}

void APU::setFrameInterruptFlag(bool set) {
    _frameInterruptFlag = set;
    if (_frameInterruptFlag) {
        _nes->_cpu._irqLine.setIRQLow();
    }
    else {
        _nes->_cpu._irqLine.setIRQHigh();
    }
}

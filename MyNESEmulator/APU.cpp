#include "APU.h"
#include "Bus.h"

void APU::reset()
{

	_pulseWaveEngines[0].init(0);
	_pulseWaveEngines[1].init(1);

	_frameInterruptFlag = false;
}

void APU::clock()
{

	sequence_step seqStep = _frameCounterEngine.clock();

	if (seqStep != sequence_step::NONE) {

		if (_frameCounterEngine.mode == 0) {

			if (seqStep == sequence_step::STEP_1
				|| seqStep == sequence_step::STEP_2
				|| seqStep == sequence_step::STEP_3
				|| seqStep == sequence_step::STEP_4P5)
			{
				_pulseWaveEngines[0].envelopeUnit.clock();
				_pulseWaveEngines[1].envelopeUnit.clock();
				_triangleWaveEngine.linearCounterUnit.clock();
			}

			if (seqStep == sequence_step::CYCLE_0) {
				//_frameInterruptFlag = frameCounterReg.irq_inhibit == false;
			}
			else if (seqStep == sequence_step::STEP_2) {

				_pulseWaveEngines[0].lengthCounterUnit.clock();
				_pulseWaveEngines[1].lengthCounterUnit.clock();
				_pulseWaveEngines[0].sweepUnit.clock();
				_pulseWaveEngines[1].sweepUnit.clock();

				_triangleWaveEngine.lengthCounterUnit.clock();

			}
			else if (seqStep == sequence_step::STEP_4) {
				_frameInterruptFlag = frameCounterReg.irq_inhibit == false;
			}
			else if (seqStep == sequence_step::STEP_4P5) {

				_pulseWaveEngines[0].lengthCounterUnit.clock();
				_pulseWaveEngines[1].lengthCounterUnit.clock();
				_pulseWaveEngines[0].sweepUnit.clock();
				_pulseWaveEngines[1].sweepUnit.clock();

				_triangleWaveEngine.lengthCounterUnit.clock();

				_frameInterruptFlag = frameCounterReg.irq_inhibit == false;

			}

		}
		else {

			if (seqStep == sequence_step::STEP_1
				|| seqStep == sequence_step::STEP_2
				|| seqStep == sequence_step::STEP_3
				|| seqStep == sequence_step::STEP_5)
			{
				_pulseWaveEngines[0].envelopeUnit.clock();
				_pulseWaveEngines[1].envelopeUnit.clock();
				_triangleWaveEngine.linearCounterUnit.clock();
			}

			// In this mode, the frame interrupt flag is never set.
			if (seqStep == sequence_step::STEP_2
				|| seqStep == sequence_step::STEP_5) {

				_pulseWaveEngines[0].lengthCounterUnit.clock();
				_pulseWaveEngines[1].lengthCounterUnit.clock();
				_pulseWaveEngines[0].sweepUnit.clock();
				_pulseWaveEngines[1].sweepUnit.clock();

				_triangleWaveEngine.lengthCounterUnit.clock();

			}

		}

	}

	if (_oddCycle) {
		_pulseWaveEngines[0].clock();
		_pulseWaveEngines[1].clock();
		_triangleWaveEngine.clock();
	}
	_oddCycle = !_oddCycle;

}

void APU::connectConsole(Bus* bus)
{
	_nes = bus;
}

void APU::writePulseWave1Reg1(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg1) = data;
	setPulseWaveReg1Fields(0, pulseWave1Reg1);
}

// A channel's second register configures the sweep unit
void APU::writePulseWave1Reg2(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg2) = data;
	setPulseWaveReg2Fields(0, pulseWave1Reg2);
}

void APU::writePulseWave1Reg3(uint8_t data)
{
	pulseWave1Reg3 = data;
	setPulseWaveReg3Fields(0, pulseWave1Reg3);
}

void APU::writePulseWave1Reg4(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg4) = data;
	setPulseWaveReg4Fields(0, pulseWave1Reg4);
}

void APU::writePulseWave2Reg1(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg1) = data;
	setPulseWaveReg1Fields(1, pulseWave2Reg1);
}

void APU::writePulseWave2Reg2(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg2) = data;
	setPulseWaveReg2Fields(1, pulseWave2Reg2);
}

void APU::writePulseWave2Reg3(uint8_t data)
{
	pulseWave2Reg3 = data;
	setPulseWaveReg3Fields(1, pulseWave2Reg3);
}

void APU::writePulseWave2Reg4(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg4) = data;
	setPulseWaveReg4Fields(1, pulseWave2Reg4);
}

// 0x4000 & 0x4004
void APU::setPulseWaveReg1Fields(uint8_t id, pulse_wave_reg1_st reg)
{
	// Sequencer
	_pulseWaveEngines[id].waveForm = waveForms[reg.duty];
	_pulseWaveEngines[id].restartSequencer();
	// Length counter
	_pulseWaveEngines[id].lengthCounterUnit.halt = reg.lengthCounterHalt == 1;
	// Envelope
	_pulseWaveEngines[id].envelopeUnit.volume = reg.volume + 1; // Aka divider's period
	_pulseWaveEngines[id].envelopeUnit.constantVolumeFlag = reg.constantVolumeFlag == 1;
		// (Note that the bit position for the loop flag is also mapped to a flag in the Length Counter.)
	_pulseWaveEngines[id].envelopeUnit.loopFlag = reg.lengthCounterHalt == 1; // ?
}

// 0x4001 & 0x4005
void APU::setPulseWaveReg2Fields(uint8_t id, pulse_wave_reg2_st reg)
{
	_pulseWaveEngines[id].sweepUnit.enabled = reg.enabled == 1;
	_pulseWaveEngines[id].sweepUnit.negate = reg.negate == 1;
	_pulseWaveEngines[id].sweepUnit.sweepPeriod = reg.period + 1; // The divider's period is set to p + 1.
	_pulseWaveEngines[id].sweepUnit.shiftCount = reg.shiftCount;

	_pulseWaveEngines[id].sweepUnit.reloadFlag = true; // Side-effect
}

// 0x4002 & 0x4006
void APU::setPulseWaveReg3Fields(uint8_t id, uint8_t reg)
{
	_pulseWaveEngines[id].configuredTimer &= 0x700;
	_pulseWaveEngines[id].configuredTimer |= reg;
	_pulseWaveEngines[id].reloadTimer();
	_pulseWaveEngines[id].sweepUnit.updateTargetPeriod();
}

// 0x4003 & 0x4007
void APU::setPulseWaveReg4Fields(uint8_t id, pulse_wave_reg4_st reg)
{
	// Reload length counter
	_pulseWaveEngines[id].lengthCounterUnit.divider = lengthCounterLut[reg.lengthCounterLoad];
	// Reload timer
	_pulseWaveEngines[id].configuredTimer &= 0xFF;
	_pulseWaveEngines[id].configuredTimer |= reg.timer_hi << 8;
	_pulseWaveEngines[id].reloadTimer();
	_pulseWaveEngines[id].sweepUnit.updateTargetPeriod();

	// When the fourth register is written to, the sequencer is restarted.
	_pulseWaveEngines[id].restartSequencer();

	_pulseWaveEngines[id].envelopeUnit.startFlag = true;
}

void APU::writeTriangleWaveReg1(uint8_t data)
{
	*((uint8_t*)&triangleWaveReg1) = data;

	// Linear counter setup
	_triangleWaveEngine.linearCounterUnit.control = triangleWaveReg1.lengthCounterHaltAndLinearCounterControl == 1;
	_triangleWaveEngine.linearCounterUnit.configuredCounter = triangleWaveReg1.linearCounterLoad;
	_triangleWaveEngine.linearCounterUnit.reload();

	_triangleWaveEngine.lengthCounterUnit.halt = triangleWaveReg1.lengthCounterHaltAndLinearCounterControl;
}

void APU::writeTriangleWaveReg2(uint8_t data)
{
}

void APU::writeTriangleWaveReg3(uint8_t data)
{
	triangleWaveReg3.timerLo = data;

	_triangleWaveEngine.configuredTimer &= ~0xFF;
	_triangleWaveEngine.configuredTimer |= data;
	_triangleWaveEngine.reloadTimer();
}

void APU::writeTriangleWaveReg4(uint8_t data)
{
	*((uint8_t*)&triangleWaveReg4) = data;

	_triangleWaveEngine.configuredTimer &= 0xFF;
	_triangleWaveEngine.configuredTimer |= triangleWaveReg4.timerHi << 8;
	_triangleWaveEngine.reloadTimer();

	_triangleWaveEngine.lengthCounterUnit.divider = lengthCounterLut[triangleWaveReg4.lengthCounterLoad];

	// Side-effect: sets the linear counter reload flag
	_triangleWaveEngine.linearCounterUnit.reload();

}

void APU::writeStatusReg(uint8_t data)
{
	*((uint8_t*)&statusWrReg) = data;

	_pulseWaveEngines[0].lengthCounterUnit.enabled = statusWrReg.enablePulseCh1;
	_pulseWaveEngines[1].lengthCounterUnit.enabled = statusWrReg.enablePulseCh2;

	_triangleWaveEngine.lengthCounterUnit.enabled = statusWrReg.enableTriangleChl;
}

void APU::writeFrameCounterReg(uint8_t data)
{
	*((uint8_t*)&frameCounterReg) = data;

	_frameCounterEngine.mode = frameCounterReg.mode;

	// Interrupt inhibit flag. If set, the frame interrupt flag is cleared,
	// otherwise it is unaffected.
	if (!frameCounterReg.irq_inhibit) {
		_frameInterruptFlag = false;
	}

}

uint8_t APU::readStatusReg()
{
	// Reading this register clears the frame interrupt flag.
	// If an interrupt flag was set at the same moment of the read, it will read back as 1 but it will not be cleared.
	statusRdReg.frameInterrupt = (uint8_t)_frameInterruptFlag;
	_frameInterruptFlag = false;

	statusRdReg.pulseCh1LenCntActive = _pulseWaveEngines[0].lengthCounterUnit.divider != 0;
	statusRdReg.pulseCh2LenCntActive = _pulseWaveEngines[1].lengthCounterUnit.divider != 0;
	statusRdReg.triangleChLenCntActive = _triangleWaveEngine.lengthCounterUnit.divider != 0;

	return *((uint8_t*)&statusRdReg);
}

sample_t APU::getOutput()
{

	float ch1output = _pulseWaveEngines[0].output && !_pulseWaveEngines[0].sweepUnit.isMuted() && _pulseWaveEngines[0].lengthCounterUnit.divider ?
		(float)_pulseWaveEngines[0].envelopeUnit.getVolume() : 0;

	float ch2output = _pulseWaveEngines[1].output && !_pulseWaveEngines[1].sweepUnit.isMuted() && _pulseWaveEngines[1].lengthCounterUnit.divider ?
		(float)_pulseWaveEngines[1].envelopeUnit.getVolume() : 0;

	float sum = ch1output + ch2output;

	float pulseOutput = 0;
	float tndOutput = 0;

	if (sum > 0) {
		pulseOutput = (95.88 / ((8128.0 / sum) + 100.0));
	}

	if (_triangleWaveEngine.output != 0) {
		tndOutput = 159.79 / ((1.0 / ((float)_triangleWaveEngine.output / 8227.0)) + 100.0);
	}

	return (pulseOutput + tndOutput) * AMPLITUDE;


}
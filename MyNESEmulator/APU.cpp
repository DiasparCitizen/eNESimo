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

	if (seqStep == sequence_step::NONE) {

		if (_frameCounterEngine.mode == 0) {

			if (seqStep == sequence_step::CYCLE_0) {
				//_frameInterruptFlag = frameCounterReg.irq_inhibit == false;
			}
			else if (seqStep == sequence_step::STEP_2) {
				_pulseWaveEngines[0].lengthCounterUnit.clock();
				_pulseWaveEngines[1].lengthCounterUnit.clock();
				_pulseWaveEngines[0].sweepUnit.clock();
				_pulseWaveEngines[1].sweepUnit.clock();

			}
			else if (seqStep == sequence_step::STEP_4) {
				_frameInterruptFlag = frameCounterReg.irq_inhibit == false;
			}
			else if (seqStep == sequence_step::STEP_4P5) {
				_pulseWaveEngines[0].lengthCounterUnit.clock();
				_pulseWaveEngines[1].lengthCounterUnit.clock();
				_pulseWaveEngines[0].sweepUnit.clock();
				_pulseWaveEngines[1].sweepUnit.clock();
				_frameInterruptFlag = frameCounterReg.irq_inhibit == false;
			}

			if (seqStep == sequence_step::STEP_1
				|| seqStep == sequence_step::STEP_2
				|| seqStep == sequence_step::STEP_3
				|| seqStep == sequence_step::STEP_4P5)
			{
				_pulseWaveEngines[0].envelopeUnit.clock();
				_pulseWaveEngines[1].envelopeUnit.clock();
			}

		}
		else {
			// In this mode, the frame interrupt flag is never set.
			if (seqStep == sequence_step::STEP_2
				|| seqStep == sequence_step::STEP_5) {
				_pulseWaveEngines[0].lengthCounterUnit.clock();
				_pulseWaveEngines[1].lengthCounterUnit.clock();
				_pulseWaveEngines[0].sweepUnit.clock();
				_pulseWaveEngines[1].sweepUnit.clock();

			}

			if (seqStep == sequence_step::STEP_1
				|| seqStep == sequence_step::STEP_2
				|| seqStep == sequence_step::STEP_3
				|| seqStep == sequence_step::STEP_5)
			{
				_pulseWaveEngines[0].envelopeUnit.clock();
				_pulseWaveEngines[1].envelopeUnit.clock();
			}
		}

	}

	_pulseWaveEngines[0].clock();
	_pulseWaveEngines[1].clock();

}

void APU::connectConsole(Bus* bus)
{
	_nes = bus;
}

void APU::writePulseWave1Reg1(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg1) = data;

	_pulseWaveEngines[0].configuredWaveForm = waveForms[pulseWave1Reg1.duty];
	_pulseWaveEngines[0].restartSequencer();
	// Length counter
	_pulseWaveEngines[0].lengthCounterUnit.halt = pulseWave1Reg1.lengthCounterHalt == 1;
	// Envelope
	_pulseWaveEngines[0].envelopeUnit.volume = pulseWave1Reg1.volume + 1; // Aka divider's period
	_pulseWaveEngines[0].envelopeUnit.constantVolumeFlag = pulseWave1Reg1.constantVolumeFlag == 1;
	// (Note that the bit position for the loop flag is also mapped to a flag in the Length Counter.)
	_pulseWaveEngines[0].envelopeUnit.loopFlag = pulseWave1Reg1.lengthCounterHalt == 1;
}

// A channel's second register configures the sweep unit
void APU::writePulseWave1Reg2(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg2) = data;
	_pulseWaveEngines[0].sweepUnit.enabled = pulseWave1Reg2.enabled == 1;
	_pulseWaveEngines[0].sweepUnit.negate = pulseWave1Reg2.negate == 1;
	_pulseWaveEngines[0].sweepUnit.period = pulseWave1Reg2.period + 1; // The divider's period is set to p + 1.
	_pulseWaveEngines[0].sweepUnit.shiftCount = pulseWave1Reg2.shiftCount;
	_pulseWaveEngines[0].sweepUnit.reloadFlag = true; // Side-effect
}

void APU::writePulseWave1Reg3(uint8_t data)
{
	pulseWave1Reg3 = data;

	_pulseWaveEngines[0].configuredTimer &= ~0xFF;
	_pulseWaveEngines[0].configuredTimer |= data;
	//_pulseWaveEngines[0].reloadTimer();
}

void APU::writePulseWave1Reg4(uint8_t data)
{
	*((uint8_t*)&pulseWave1Reg4) = data;

	_pulseWaveEngines[0].lengthCounterUnit.divider = lengthCounterLut[pulseWave1Reg4.lengthCounterLoad];

	_pulseWaveEngines[0].configuredTimer &= 0xFF;
	_pulseWaveEngines[0].configuredTimer |= pulseWave1Reg4.timer_hi << 8;
	_pulseWaveEngines[0].reloadTimer();

	// When the fourth register is written to, the sequencer is restarted.
	_pulseWaveEngines[0].restartSequencer();

}

void APU::writePulseWave2Reg1(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg1) = data;
}

void APU::writePulseWave2Reg2(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg2) = data;
}

void APU::writePulseWave2Reg3(uint8_t data)
{
	pulseWave2Reg3 = data;
}

void APU::writePulseWave2Reg4(uint8_t data)
{
	*((uint8_t*)&pulseWave2Reg4) = data;
}

void APU::writeStatusReg(uint8_t data)
{
	*((uint8_t*)&statusWrReg) = data;

	_pulseWaveEngines[0].lengthCounterUnit.enabled = statusWrReg.enablePulseCh1;
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

	statusRdReg.pulseCh1LenCntActive = _pulseWaveEngines[0].lengthCounterUnit.divider > 0;
	statusRdReg.pulseCh2LenCntActive = _pulseWaveEngines[1].lengthCounterUnit.divider > 0;

	return *((uint8_t*)&statusRdReg);
}
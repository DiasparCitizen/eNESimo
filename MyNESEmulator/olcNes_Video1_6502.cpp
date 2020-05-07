/*
	olc6502 - An emulation of the 6502/2A03 processor
	"Thanks Dad for believing computers were gonna be a big deal..." - javidx9
	License (OLC-3)
	~~~~~~~~~~~~~~~
	Copyright 2018-2019 OneLoneCoder.com
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:
	1. Redistributions or derivations of source code must retain the above
	copyright notice, this list of conditions and the following disclaimer.
	2. Redistributions or derivative works in binary form must reproduce
	the above copyright notice. This list of conditions and the following
	disclaimer must be reproduced in the documentation and/or other
	materials provided with the distribution.
	3. Neither the name of the copyright holder nor the names of its
	contributors may be used to endorse or promote products derived
	from this software without specific prior written permission.
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
	HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	Background
	~~~~~~~~~~
	I love this microprocessor. It was at the heart of two of my favourite
	machines, the BBC Micro, and the Nintendo Entertainment System, as well
	as countless others in that era. I learnt to program on the Model B, and
	I learnt to love games on the NES, so in many ways, this processor is
	why I am the way I am today.
	In February 2019, I decided to undertake a selfish personal project and
	build a NES emulator. Ive always wanted to, and as such I've avoided
	looking at source code for such things. This made making this a real
	personal challenge. I know its been done countless times, and very likely
	in far more clever and accurate ways than mine, but I'm proud of this.
	Datasheet: http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf
	Files: olc6502.h, olc6502.cpp
	Relevant Video: https://youtu.be/8XmxKPJDGU0
	Links
	~~~~~
	YouTube:	https://www.youtube.com/javidx9
				https://www.youtube.com/javidx9extra
	Discord:	https://discord.gg/WhwHUMV
	Twitter:	https://www.twitter.com/javidx9
	Twitch:		https://www.twitch.tv/javidx9
	GitHub:		https://www.github.com/onelonecoder
	Patreon:	https://www.patreon.com/javidx9
	Homepage:	https://www.onelonecoder.com
	Author
	~~~~~~
	David Barr, aka javidx9, ©OneLoneCoder 2019
*/

#include <iostream>
#include <sstream>

#include "Bus.h"
#include "mostech6502.h"
#include "NesCoreApi.h"

//#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#define OLC_PGE_APPLICATION
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include "olcPixelGameEngine.h"

class Demo_olc6502 : public olc::PixelGameEngine
{

	std::shared_ptr<Cartridge> cart;
	Bus nes;
	std::map<uint16_t, std::string> mapAsm;
	bool bEmulationRun = false;
	float fResidualTime = 0.0f;
	uint8_t nSelectedPalette = 0x00;
	pixel_st pixelOutput;
	olc::Pixel  palScreen[0x40];
	olc::Sprite sprScreen = olc::Sprite(256, 240);

public:
	Demo_olc6502() {

		sAppName = "olc6502 Demonstration";

		// Copied
		palScreen[0x00] = olc::Pixel(84, 84, 84);
		palScreen[0x01] = olc::Pixel(0, 30, 116);
		palScreen[0x02] = olc::Pixel(8, 16, 144);
		palScreen[0x03] = olc::Pixel(48, 0, 136);
		palScreen[0x04] = olc::Pixel(68, 0, 100);
		palScreen[0x05] = olc::Pixel(92, 0, 48);
		palScreen[0x06] = olc::Pixel(84, 4, 0);
		palScreen[0x07] = olc::Pixel(60, 24, 0);
		palScreen[0x08] = olc::Pixel(32, 42, 0);
		palScreen[0x09] = olc::Pixel(8, 58, 0);
		palScreen[0x0A] = olc::Pixel(0, 64, 0);
		palScreen[0x0B] = olc::Pixel(0, 60, 0);
		palScreen[0x0C] = olc::Pixel(0, 50, 60);
		palScreen[0x0D] = olc::Pixel(0, 0, 0);
		palScreen[0x0E] = olc::Pixel(0, 0, 0);
		palScreen[0x0F] = olc::Pixel(0, 0, 0);

		palScreen[0x10] = olc::Pixel(152, 150, 152);
		palScreen[0x11] = olc::Pixel(8, 76, 196);
		palScreen[0x12] = olc::Pixel(48, 50, 236);
		palScreen[0x13] = olc::Pixel(92, 30, 228);
		palScreen[0x14] = olc::Pixel(136, 20, 176);
		palScreen[0x15] = olc::Pixel(160, 20, 100);
		palScreen[0x16] = olc::Pixel(152, 34, 32);
		palScreen[0x17] = olc::Pixel(120, 60, 0);
		palScreen[0x18] = olc::Pixel(84, 90, 0);
		palScreen[0x19] = olc::Pixel(40, 114, 0);
		palScreen[0x1A] = olc::Pixel(8, 124, 0);
		palScreen[0x1B] = olc::Pixel(0, 118, 40);
		palScreen[0x1C] = olc::Pixel(0, 102, 120);
		palScreen[0x1D] = olc::Pixel(0, 0, 0);
		palScreen[0x1E] = olc::Pixel(0, 0, 0);
		palScreen[0x1F] = olc::Pixel(0, 0, 0);

		palScreen[0x20] = olc::Pixel(236, 238, 236);
		palScreen[0x21] = olc::Pixel(76, 154, 236);
		palScreen[0x22] = olc::Pixel(120, 124, 236);
		palScreen[0x23] = olc::Pixel(176, 98, 236);
		palScreen[0x24] = olc::Pixel(228, 84, 236);
		palScreen[0x25] = olc::Pixel(236, 88, 180);
		palScreen[0x26] = olc::Pixel(236, 106, 100);
		palScreen[0x27] = olc::Pixel(212, 136, 32);
		palScreen[0x28] = olc::Pixel(160, 170, 0);
		palScreen[0x29] = olc::Pixel(116, 196, 0);
		palScreen[0x2A] = olc::Pixel(76, 208, 32);
		palScreen[0x2B] = olc::Pixel(56, 204, 108);
		palScreen[0x2C] = olc::Pixel(56, 180, 204);
		palScreen[0x2D] = olc::Pixel(60, 60, 60);
		palScreen[0x2E] = olc::Pixel(0, 0, 0);
		palScreen[0x2F] = olc::Pixel(0, 0, 0);

		palScreen[0x30] = olc::Pixel(236, 238, 236);
		palScreen[0x31] = olc::Pixel(168, 204, 236);
		palScreen[0x32] = olc::Pixel(188, 188, 236);
		palScreen[0x33] = olc::Pixel(212, 178, 236);
		palScreen[0x34] = olc::Pixel(236, 174, 236);
		palScreen[0x35] = olc::Pixel(236, 174, 212);
		palScreen[0x36] = olc::Pixel(236, 180, 176);
		palScreen[0x37] = olc::Pixel(228, 196, 144);
		palScreen[0x38] = olc::Pixel(204, 210, 120);
		palScreen[0x39] = olc::Pixel(180, 222, 120);
		palScreen[0x3A] = olc::Pixel(168, 226, 144);
		palScreen[0x3B] = olc::Pixel(152, 226, 180);
		palScreen[0x3C] = olc::Pixel(160, 214, 228);
		palScreen[0x3D] = olc::Pixel(160, 162, 160);
		palScreen[0x3E] = olc::Pixel(0, 0, 0);
		palScreen[0x3F] = olc::Pixel(0, 0, 0);

	}

	std::string hex(uint32_t n, uint8_t d)
	{
		std::string s(d, '0');
		for (int i = d - 1; i >= 0; i--, n >>= 4)
			s[i] = "0123456789ABCDEF"[n & 0xF];
		return s;
	};

	bool OnUserCreate() {
		// Load cartridge
		cart = std::make_shared<Cartridge>(CARTRIDGE_NAME);
		nes.insertCartridge(cart);
		nes.setPixelMode(&pixelOutput);
		nes.resetNES();
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) {

		Clear(olc::DARK_BLUE);

		// Sneaky peek of controller input in next video! ;P
		uint16_t controllerId = 0;
		nes._controllers[controllerId].setConnected(true);

		nes._controllers[controllerId].setB(GetKey(olc::Key::B).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setSelect(GetKey(olc::Key::P).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setA(GetKey(olc::Key::A).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setStart(GetKey(olc::Key::S).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setUP(GetKey(olc::Key::UP).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setDOWN(GetKey(olc::Key::DOWN).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setLEFT(GetKey(olc::Key::LEFT).bHeld ? 0x1 : 0x0);
		nes._controllers[controllerId].setRIGHT(GetKey(olc::Key::RIGHT).bHeld ? 0x1 : 0x0);

		if (GetKey(olc::Key::SPACE).bPressed) bEmulationRun = !bEmulationRun;
		if (GetKey(olc::Key::R).bPressed) nes.resetNES();
		if (GetKey(olc::Key::P).bPressed) (++nSelectedPalette) &= 0x07;

		//nes.clockNES();
		if (bEmulationRun)
		{
			if (fResidualTime > 0.0f) {
				fResidualTime -= fElapsedTime;
			}
				
			else
			{
				fResidualTime += (1.0f / 60.0f) - fElapsedTime;
				do {

					nes.clockNES();
					sprScreen.SetPixel(pixelOutput.scanlineCycle - 1, pixelOutput.scanline, palScreen[pixelOutput.paletteColorCode]);
				
				} while (!nes._ppu._frameComplete);
				nes._ppu._frameComplete = false;
			}
		}
		else
		{
			// Emulate code step-by-step
			if (GetKey(olc::Key::C).bPressed)
			{
				// Clock enough times to execute a whole CPU instruction
				do { nes.clockNES(); } while (!nes._cpu.isInstructionComplete());
				// CPU clock runs slower than system clock, so it may be
				// complete for additional system clock cycles. Drain
				// those out
				do { nes.clockNES(); } while (nes._cpu.isInstructionComplete());
			}

			// Emulate one whole frame
			if (GetKey(olc::Key::F).bPressed)
			{
				// Clock enough times to draw a single frame
				do { nes.clockNES(); } while (!nes._ppu._frameComplete);
				// Use residual clock cycles to complete current instruction
				do { nes.clockNES(); } while (!nes._cpu.isInstructionComplete());
				// Reset frame completion flag
				nes._ppu._frameComplete = false;
				//nes.ppu.printPPURamRange(0x2000, 0x2000 + 1024);
			}
		}


		// Draw rendered output ========================================================
		DrawSprite(0, 0, &sprScreen, 2);
		return true;

	}

};




#ifdef CPU_DEBUG_MODE
int main_()
#else
int main()
#endif
{
	Demo_olc6502 demo;
	demo.Construct(640, 480, 2, 2);
	demo.Start();
	return 0;
}
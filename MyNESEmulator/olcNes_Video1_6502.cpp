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

//#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"



class Demo_olc6502 : public olc::PixelGameEngine
{
public:
	Demo_olc6502() { sAppName = "olc6502 Demonstration"; }

	std::shared_ptr<Cartridge> cart;
	Bus nes;
	std::map<uint16_t, std::string> mapAsm;
	bool bEmulationRun = false;
	float fResidualTime = 0.0f;
	uint8_t nSelectedPalette = 0x00;

	std::string hex(uint32_t n, uint8_t d)
	{
		std::string s(d, '0');
		for (int i = d - 1; i >= 0; i--, n >>= 4)
			s[i] = "0123456789ABCDEF"[n & 0xF];
		return s;
	};

	void DrawRam(int x, int y, uint16_t nAddr, int nRows, int nColumns)
	{
		int nRamX = x, nRamY = y;
		for (int row = 0; row < nRows; row++)
		{
			std::string sOffset = "$" + hex(nAddr, 4) + ":";
			for (int col = 0; col < nColumns; col++)
			{
				sOffset += " " + hex(nes.cpuRead(nAddr, true), 2);
				nAddr += 1;
			}
			DrawString(nRamX, nRamY, sOffset);
			nRamY += 10;
		}
	}

	void DrawCpu(int x, int y)
	{
		std::string status = "STATUS: ";
		DrawString(x, y, "STATUS:", olc::WHITE);
		DrawString(x + 64, y, "N", nes.cpu.reg_status.raw & 0x80 ? olc::GREEN : olc::RED);
		DrawString(x + 80, y, "V", nes.cpu.reg_status.raw & 0x40 ? olc::GREEN : olc::RED);
		DrawString(x + 96, y, "-", nes.cpu.reg_status.raw & 0x20 ? olc::GREEN : olc::RED);
		DrawString(x + 112, y, "B", nes.cpu.reg_status.raw & 0x10 ? olc::GREEN : olc::RED);
		DrawString(x + 128, y, "D", nes.cpu.reg_status.raw & 0x8 ? olc::GREEN : olc::RED);
		DrawString(x + 144, y, "I", nes.cpu.reg_status.raw & 0x4 ? olc::GREEN : olc::RED);
		DrawString(x + 160, y, "Z", nes.cpu.reg_status.raw & 0x2 ? olc::GREEN : olc::RED);
		DrawString(x + 178, y, "C", nes.cpu.reg_status.raw & 0x1 ? olc::GREEN : olc::RED);
		DrawString(x, y + 10, "PC: $" + hex(nes.cpu.pc, 4));
		DrawString(x, y + 20, "A: $" + hex(nes.cpu.reg_acc, 2) + "  [" + std::to_string(nes.cpu.reg_acc) + "]");
		DrawString(x, y + 30, "X: $" + hex(nes.cpu.reg_x, 2) + "  [" + std::to_string(nes.cpu.reg_x) + "]");
		DrawString(x, y + 40, "Y: $" + hex(nes.cpu.reg_y, 2) + "  [" + std::to_string(nes.cpu.reg_y) + "]");
		DrawString(x, y + 50, "Stack P: $" + hex(nes.cpu.stack_ptr, 4));
	}

	void DrawCode(int x, int y, int nLines)
	{
		auto it_a = mapAsm.find(nes.cpu.pc);
		int nLineY = (nLines >> 1) * 10 + y;
		if (it_a != mapAsm.end())
		{
			DrawString(x, nLineY, (*it_a).second, olc::CYAN);
			while (nLineY < (nLines * 10) + y)
			{
				nLineY += 10;
				if (++it_a != mapAsm.end())
				{
					DrawString(x, nLineY, (*it_a).second);
				}
			}
		}

		it_a = mapAsm.find(nes.cpu.pc);
		nLineY = (nLines >> 1) * 10 + y;
		if (it_a != mapAsm.end())
		{
			while (nLineY > y)
			{
				nLineY -= 10;
				if (--it_a != mapAsm.end())
				{
					DrawString(x, nLineY, (*it_a).second);
				}
			}
		}
	}

	bool OnUserCreate() {

		// Load cartridge
		cart = std::make_shared<Cartridge>(CARTRIDGE_NAME);

		// Insert cart
		nes.insertCartridge(cart);

		// Extract disassembly
		//mapAsm = nes.nesCPU.disassemble(0x0000, 0xFFFF);

		nes.resetNES();
	    //nes.printRamRange(RESET_ADDR, RESET_ADDR + 2);
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) {

		Clear(olc::DARK_BLUE);

		// Sneaky peek of controller input in next video! ;P
		nes.controller[0] = 0x00;
		nes.controller[0] |= GetKey(olc::Key::X).bHeld ? 0x80 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::Z).bHeld ? 0x40 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::A).bHeld ? 0x20 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::S).bHeld ? 0x10 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::UP).bHeld ? 0x08 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::DOWN).bHeld ? 0x04 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::LEFT).bHeld ? 0x02 : 0x00;
		nes.controller[0] |= GetKey(olc::Key::RIGHT).bHeld ? 0x01 : 0x00;

		if (GetKey(olc::Key::SPACE).bPressed) bEmulationRun = !bEmulationRun;
		if (GetKey(olc::Key::R).bPressed) nes.resetNES();
		if (GetKey(olc::Key::P).bPressed) (++nSelectedPalette) &= 0x07;

		if (bEmulationRun)
		{
			if (fResidualTime > 0.0f)
				fResidualTime -= fElapsedTime;
			else
			{
				fResidualTime += (1.0f / 60.0f) - fElapsedTime;
				do { nes.clockNES(); } while (!nes.ppu.frame_complete);
				nes.ppu.frame_complete = false;
			}
		}
		else
		{
			// Emulate code step-by-step
			if (GetKey(olc::Key::C).bPressed)
			{
				// Clock enough times to execute a whole CPU instruction
				do { nes.clockNES(); } while (!nes.cpu.isInstructionComplete());
				// CPU clock runs slower than system clock, so it may be
				// complete for additional system clock cycles. Drain
				// those out
				do { nes.clockNES(); } while (nes.cpu.isInstructionComplete());
			}

			// Emulate one whole frame
			if (GetKey(olc::Key::F).bPressed)
			{
				// Clock enough times to draw a single frame
				do { nes.clockNES(); } while (!nes.ppu.frame_complete);
				// Use residual clock cycles to complete current instruction
				do { nes.clockNES(); } while (!nes.cpu.isInstructionComplete());
				// Reset frame completion flag
				nes.ppu.frame_complete = false;
				//nes.ppu.printPPURamRange(0x2000, 0x2000 + 1024);
			}
		}




		//DrawCpu(516, 2);
		//DrawCode(516, 72, 26);

		// Draw Palettes & Pattern Tables ==============================================
		/*const int nSwatchSize = 6;
		for (int p = 0; p < 8; p++) // For each palette
			for (int s = 0; s < 4; s++) // For each index
				FillRect(516 + p * (nSwatchSize * 5) + s * nSwatchSize, 340,
					nSwatchSize, nSwatchSize, nes.ppu.GetColourFromPaletteRam(p, s));

		// Draw selection reticule around selected palette
		DrawRect(516 + nSelectedPalette * (nSwatchSize * 5) - 1, 339, (nSwatchSize * 4), nSwatchSize, olc::WHITE);
		// Generate Pattern Tables
		DrawSprite(516, 348, &nes.ppu.GetPatternTable(0, nSelectedPalette));
		DrawSprite(648, 348, &nes.ppu.GetPatternTable(1, nSelectedPalette));*/

		// Draw rendered output ========================================================
		DrawSprite(0, 0, &nes.ppu.GetScreen(), 2);
		return true;

	}

	bool OnUserCreate_()
	{
		std::cout << "hello, start!\n";
		// Load Program (assembled at https://www.masswerk.at/6502/assembler.html)
		/*
			*=$8000
			LDX #10
			STX $0000
			LDX #3
			STX $0001
			LDY $0000
			LDA #0
			CLC
			loop
			ADC $0001
			DEY
			BNE loop
			STA $0002
			NOP
			NOP
			NOP
		*/

		// Convert hex string into bytes for RAM
		std::stringstream ss;
		ss << "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA";
		uint16_t nOffset = 0x8000;
		while (!ss.eof())
		{
			std::string b;
			ss >> b;
			nes.cpuRam[nOffset++] = (uint8_t)std::stoul(b, nullptr, 16);
		}
		nes.printRamRange(0x8000, 0x8050);
		// Set Reset Vector
		nes.cpuRam[RESET_ADDR] = 0x00;
		nes.cpuRam[RESET_ADDR + 1] = 0x80;
		nes.printRamRange(RESET_ADDR, RESET_ADDR+1);
		// Dont forget to set IRQ and NMI vectors if you want to play with those

		// Extract dissassembly
		mapAsm = nes.cpu.disassemble(0x0000, 0xFFFF);

		// Reset
		nes.cpu.reset();
		return true;
	}

	bool OnUserUpdate_(float fElapsedTime)
	{
		Clear(olc::DARK_BLUE);


		if (GetKey(olc::Key::SPACE).bPressed)
		{
			do
			{
				nes.cpu.advanceClock();
			} while (!nes.cpu.isInstructionComplete());
			nes.cpu.printCpuState();
		}

		if (GetKey(olc::Key::R).bPressed)
			nes.cpu.reset();

		if (GetKey(olc::Key::I).bPressed)
			nes.cpu.irq();

		if (GetKey(olc::Key::N).bPressed)
			nes.cpu.nmi();
		
		// Draw Ram Page 0x00		
		DrawRam(2, 2, 0x0000, 16, 16);
		DrawRam(2, 182, 0x8000, 16, 16);
		DrawCpu(448, 2);
		DrawCode(448, 72, 26);


		DrawString(10, 370, "SPACE = Step Instruction    R = RESET    I = IRQ    N = NMI");

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
	demo.Construct(680, 480, 2, 2);
	demo.Start();
	return 0;
}
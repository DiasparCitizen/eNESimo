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
#pragma once

//#define GAME_TERMINAL_LOG
//#define GAME_FILE_LOG

//#define CARTRIDGE_NAME "castlevania_usa.nes"
//#define CARTRIDGE_NAME "mega_man_usa.nes"
//#define CARTRIDGE_NAME "mario.nes"
//#define CARTRIDGE_NAME "metroid.nes"
//#define CARTRIDGE_NAME "megaman2.nes"
#define CARTRIDGE_NAME "zelda1.nes"
//#define CARTRIDGE_NAME "iceclimbers.nes"
//#define CARTRIDGE_NAME "antad.nes"

// TESTS

#ifndef CARTRIDGE_NAME

//#define CARTRIDGE_NAME "full_palette.nes"

//#define CARTRIDGE_NAME "branch_timing_tests/1.Branch_Basics.nes" // PASS
//#define CARTRIDGE_NAME "branch_timing_tests/2.Backward_Branch.nes" // PASS
//#define CARTRIDGE_NAME "branch_timing_tests/3.Forward_Branch.nes" // PASS
#define CARTRIDGE_NAME "nestest.nes" // PASS
//#define CARTRIDGE_NAME "cpu_dummy_writes_oam.nes"
//#define CARTRIDGE_NAME "cpu_dummy_reads.nes"

//#define CARTRIDGE_NAME "zero_hit/01-basics.nes" // pass
//#define CARTRIDGE_NAME "zero_hit/02-alignment.nes" // pass
//#define CARTRIDGE_NAME "zero_hit/03-corners.nes" // pass
//#define CARTRIDGE_NAME "zero_hit/04-flip.nes" // pass
//#define CARTRIDGE_NAME "zero_hit/05-left_clip.nes" // fail, "should miss entirely on left-edge clipping" --> pass
//#define CARTRIDGE_NAME "zero_hit/06-right_edge.nes" // pass
//#define CARTRIDGE_NAME "zero_hit/07-screen_bottom.nes" // fail --> pass
//#define CARTRIDGE_NAME "zero_hit/08-double_height.nes" // fail, "lower tile sprite should miss bottom of bg tile" --> pass
//#define CARTRIDGE_NAME "zero_hit/09-timing.nes" // fail, "flag set too soon for upper flag corner"
//#define CARTRIDGE_NAME "zero_hit/10-timing_order.nes" // fail --> pass

//#define CARTRIDGE_NAME "spr_ovfl/01-basics.nes" // pass
//#define CARTRIDGE_NAME "spr_ovfl/02-details.nes" // pass
//#define CARTRIDGE_NAME "spr_ovfl/03-timing.nes" // fail
//#define CARTRIDGE_NAME "spr_ovfl/04-obscure.nes" // fail
//#define CARTRIDGE_NAME "spr_ovfl/05-emulator.nes" // pass

#define CARTRIDGE_NAME "test_apu_2/test_10.nes"
// fail: 3, 4, 7, 8, 9, 10


//#define CARTRIDGE_NAME "blargg_apu_2005.07.30/01.len_ctr.nes"
//#define CARTRIDGE_NAME "test_apu_env/test_apu_env.nes"

#endif
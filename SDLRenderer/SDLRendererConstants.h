#pragma once

//#define GAME_TERMINAL_LOG
//#define GAME_FILE_LOG

#define CARTRIDGE_NAME "mario.nes"
//#define CARTRIDGE_NAME "iceclimbers.nes"
//#define CARTRIDGE_NAME "antad.nes"

// TESTS

#ifndef CARTRIDGE_NAME

//#define CARTRIDGE_NAME "full_palette.nes"

//#define CARTRIDGE_NAME "branch_timing_tests/1.Branch_Basics.nes" // PASS
//#define CARTRIDGE_NAME "branch_timing_tests/2.Backward_Branch.nes" // PASS
//#define CARTRIDGE_NAME "branch_timing_tests/3.Forward_Branch.nes" // PASS
//#define CARTRIDGE_NAME "nestest.nes" // PASS
#define CARTRIDGE_NAME "cpu_dummy_writes_oam.nes"
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

#endif
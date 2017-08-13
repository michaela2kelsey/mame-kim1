// license:GPL-2.0+
// copyright-holders:Juergen Buchmueller
/*************************************************************************

    includes/kim1.h

*************************************************************************/

#pragma once

#ifndef __KIM1__
#define __KIM1__

#include "softlist.h"
#include "cpu/m6502/m6502.h"
#include "machine/mos6530.h"
#include "imagedev/cassette.h"
#include "formats/kim1_cas.h"

//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

class kim1_state : public driver_device
{
public:
	kim1_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_videoram(*this, "videoram"),		
		m_riot2(*this, "miot_u2"),
		m_cass(*this, "cassette"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette"),
		m_row0(*this, "ROW0"),
		m_row1(*this, "ROW1"),
		m_row2(*this, "ROW2"),
		m_special(*this, "SPECIAL")
		 { }

	// devices
	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_videoram;	
	required_device<mos6530_device> m_riot2;
	required_device<cassette_image_device> m_cass;
	DECLARE_READ8_MEMBER(kim1_u2_read_a);
	DECLARE_WRITE8_MEMBER(kim1_u2_write_a);
	DECLARE_READ8_MEMBER(kim1_u2_read_b);
	DECLARE_WRITE8_MEMBER(kim1_u2_write_b);
	uint8_t m_u2_port_b;
	uint8_t m_311_output;
	uint32_t m_cassette_high_count;
	uint8_t m_led_time[6];
	
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
	
	uint8_t m_flipscreen;
	uint64_t m_madsel_lastcycles;
	DECLARE_WRITE8_MEMBER(missile_w);
	DECLARE_READ8_MEMBER(missile_r);

	// device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;
	
	uint32_t screen_update_kim1(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	//inline int scanline_to_v(int scanline);
	//inline int v_to_scanline(int v);
	//inline void schedule_next_irq(int curv);
	inline bool get_madsel();
	//inline offs_t get_bit3_addr(offs_t pixaddr);
	void write_vram(address_space &space, offs_t address, uint8_t data);
	uint8_t read_vram(address_space &space, offs_t address);

	DECLARE_INPUT_CHANGED_MEMBER(trigger_reset);
	DECLARE_INPUT_CHANGED_MEMBER(trigger_nmi);
	TIMER_DEVICE_CALLBACK_MEMBER(kim1_cassette_input);
	TIMER_DEVICE_CALLBACK_MEMBER(kim1_update_leds);

protected:
	required_ioport m_row0;
	required_ioport m_row1;
	required_ioport m_row2;
	required_ioport m_special;
};

#endif /* KIM1_H */

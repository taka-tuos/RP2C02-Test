#include <Arduino.h>
#include <Wire.h>
#include <Si5351.h>

#include "slide.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

Si5351 si5351;

#define GPIO_ADR_BASE 20
#define GPIO_DAT_BASE 6
#define GPIO_nCS 19
#define GPIO_R_W 28
#define GPIO_nRST 17
#define GPIO_nINT 16

void write_ppu(word addr, byte data);
void write_vram(word addr, byte data);

byte chr_ram[2048];
byte ntb_ram[64*30];



bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

bool vsync_flag = false;

void vsync_handler() {
	vsync_flag = true;
}

void setup()
{
	Serial.begin(9600);

	si5351.init(SI5351_CRYSTAL_LOAD_10PF, 0, 0);
	si5351.set_freq(2147727272ULL, SI5351_CLK0);

	pinMode(GPIO_nCS, OUTPUT);
	gpio_set_drive_strength(GPIO_nCS, GPIO_DRIVE_STRENGTH_12MA);
	digitalWrite(GPIO_nCS, HIGH);

	pinMode(GPIO_R_W, OUTPUT);
	gpio_set_drive_strength(GPIO_R_W, GPIO_DRIVE_STRENGTH_12MA);
	digitalWrite(GPIO_R_W, LOW);

	pinMode(GPIO_nRST, INPUT);
	gpio_set_drive_strength(GPIO_nRST, GPIO_DRIVE_STRENGTH_12MA);

	pinMode(GPIO_nINT, INPUT_PULLUP);
	gpio_set_drive_strength(GPIO_nINT, GPIO_DRIVE_STRENGTH_12MA);

	attachInterrupt(GPIO_nINT, vsync_handler, FALLING);

	pinMode(GPIO_nINT, INPUT_PULLUP);
	gpio_set_drive_strength(GPIO_nINT, GPIO_DRIVE_STRENGTH_12MA);

	for(int i = 0; i < 8; i++) {
		pinMode(GPIO_DAT_BASE + i, OUTPUT);
		gpio_set_drive_strength(GPIO_DAT_BASE + i, GPIO_DRIVE_STRENGTH_12MA);
	}

	for(int i = 0; i < 3; i++) {
		pinMode(GPIO_ADR_BASE + i, OUTPUT);
		gpio_set_drive_strength(GPIO_ADR_BASE + i, GPIO_DRIVE_STRENGTH_12MA);
	}

	delay(100);

	write_ppu(0x2000, 0x80);
	write_ppu(0x2001, 0x00);

	write_ppu(0x2003, 0x00); 
	for(int i = 0; i < 256; i++) write_ppu(0x2004, 0x00);

	byte palette[16] = {
		0x0f, 0x30, 0x0f, 0x30,
		0x0f, 0x30, 0x0f, 0x30,
		0x0f, 0x30, 0x0f, 0x30,
		0x0f, 0x30, 0x0f, 0x30,
	};

	write_ppu(0x2006, 0x3f); 
	write_ppu(0x2006, 0x00);
	for(int i = 0; i < 16; i++) write_ppu(0x2007, palette[i]);
}

int vcnt = 0;

int vtick = 0;

int pg_ptr = 0;

uint8_t *prg_list[] = {
	prg_0_rom,
	prg_1_rom,
	prg_2_rom,
	prg_3_rom,
	prg_4_rom,
	prg_5_rom,
	prg_6_rom,
	prg_7_rom,
};

uint8_t *chr_list[] = {
	chr_0_rom,
	chr_1_rom,
	chr_2_rom,
	chr_3_rom,
	chr_4_rom,
	chr_5_rom,
	chr_6_rom,
	chr_7_rom,
};

void loop()
{
	write_ppu(0x2000, 0x80);
	write_ppu(0x2001, 0x00);
	write_ppu(0x2006, 0x00); 
	write_ppu(0x2006, 0x00);

	memcpy(chr_ram, chr_list[pg_ptr], 2048);
	memcpy(ntb_ram, prg_list[pg_ptr], 1920);

	pg_ptr++;
	if(pg_ptr >= 8) pg_ptr = 0;

	for(int i = 0; i < 0x3000; i++) {
		if(i < 0x2000) write_ppu(0x2007, chr_ram[(((i & 0xfff0) >> 1) | (i & 7)) & (2048 - 1)] ^ 0xff);
		else write_ppu(0x2007, ntb_ram[(i - 0x2000) % (64*30)]);
	}

	//delay(100);

	write_ppu(0x2000, 0x80);
	write_ppu(0x2005, 0x00);
	write_ppu(0x2005, 0x00);
	write_ppu(0x2001, 0x1e);

	for(int i = 0; i < 60 * 2; i++) {
		while(!vsync_flag) {
			asm volatile(
				" nop\n nop\n nop\n nop\n"  // 4 cycle
				" nop\n nop\n nop\n"  		// 3 cycle
			); // 7 * 8 = 56ns
		}
		vsync_flag = false;
	}

	//while(!get_bootsel_button()) delay(10);
	//while(get_bootsel_button()) delay(10);
}

void write_ppu(word addr, byte data)
{
	for(int i = 0; i < 8; i++) {
		gpio_put(GPIO_DAT_BASE + i, (data & (1 << i)) != 0 ? HIGH : LOW);
	}

	for(int i = 0; i < 3; i++) {
		gpio_put(GPIO_ADR_BASE + i, (addr & (1 << i)) != 0 ? HIGH : LOW);
	}

	gpio_put(GPIO_nCS, LOW);

	asm volatile(
		" nop\n nop\n nop\n nop\n"  // 4 cycle
		" nop\n nop\n nop\n"  		// 3 cycle
	); // 7 * 8 = 56ns

	//delayMicroseconds(1);

	gpio_put(GPIO_nCS, HIGH);
}

void write_vram(word addr, byte data) {
	write_ppu(0x2006, addr >> 8); 
	write_ppu(0x2006, addr & 0xff);
	write_ppu(0x2007, data);
}
#include <Arduino.h>
#include <Wire.h>
#include <Si5351.h>

Si5351 si5351;

#define GPIO_ADR_BASE 20
#define GPIO_DAT_BASE 6
#define GPIO_nCS 19
#define GPIO_R_W 28

void write_ppu(word addr, byte data);
void write_vram(word addr, byte data);

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

  pinMode(16, INPUT_PULLUP);
  gpio_set_drive_strength(16, GPIO_DRIVE_STRENGTH_12MA);

  pinMode(17, INPUT);
  gpio_set_drive_strength(17, GPIO_DRIVE_STRENGTH_12MA);

  for(int i = 0; i < 8; i++) {
    pinMode(GPIO_DAT_BASE + i, OUTPUT);
    gpio_set_drive_strength(GPIO_DAT_BASE + i, GPIO_DRIVE_STRENGTH_12MA);
  }

  for(int i = 0; i < 3; i++) {
    pinMode(GPIO_ADR_BASE + i, OUTPUT);
    gpio_set_drive_strength(GPIO_ADR_BASE + i, GPIO_DRIVE_STRENGTH_12MA);
  }

  /*write_ppu(0x2000, 0);
  write_ppu(0x2001, 0);

  delay(100);

  for(int i = 0; i < 0x4000; i++) {
    write_ppu(0x2006, i >> 8);
    write_ppu(0x2006, i & 255);

    write_ppu(0x2007, rand());

    //Serial.println(i, HEX);

    //delay(20);
  }*/

  delay(100);

  write_ppu(0x2000, 0x80);
  write_ppu(0x2001, 0x1e);

  //while (digitalRead(16) == HIGH);

  write_ppu(0x2003, 0x00); 
  for(int i = 0; i < 256; i++) write_ppu(0x2004, 0x00);

  byte palette[16] = {
    0x30, 0x0f, 0x30, 0x30,
    0x30, 0x0f, 0x30, 0x30,
    0x30, 0x0f, 0x30, 0x30,
    0x30, 0x0f, 0x30, 0x30,
  };

  write_ppu(0x2006, 0x3f); 
  write_ppu(0x2006, 0x00);
  for(int i = 0; i < 16; i++) write_ppu(0x2007, palette[i]);

  //while (digitalRead(16) == HIGH);

  write_ppu(0x2006, 0x20); 
  write_ppu(0x2006, 0x00);
  for(int i = 0; i < 0xc00; i++) write_ppu(0x2007, 0);

  write_ppu(0x2006, 0x00); 
  write_ppu(0x2006, 0x00);
  for(int i = 0; i < 0x100; i++) write_ppu(0x2007, 0x55);
}

int vcnt = 0;

int vtick = 0;

void loop()
{
  while (digitalRead(16) == HIGH);
  //write_vram(0x2000, rand());
  write_ppu(0x2006, 0x20); 
  write_ppu(0x2006, 0);
  for(int i = 0; i < 0x10; i++) {
    write_ppu(0x2007, rand());
    //Serial.println(i, HEX);
  }

  write_ppu(0x2000, 0x80);
  write_ppu(0x2005, 0x00);
  write_ppu(0x2005, 0x00);

  /*while (digitalRead(16) == HIGH);
  vcnt++;
  if(millis() - vtick >= 1000) {
    Serial.println(vcnt);
    vcnt = 0;
    vtick = millis();
  }*/
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
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);

  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);

  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);

  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);
  gpio_put(GPIO_nCS, LOW);

  //delayMicroseconds(1);

  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);

  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);

  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);

  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
  gpio_put(GPIO_nCS, HIGH);
}

void write_vram(word addr, byte data) {
  write_ppu(0x2006, addr >> 8); 
  write_ppu(0x2006, addr & 0xff);
  write_ppu(0x2007, data);
}
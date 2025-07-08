#include "timers.h"
#include "pin_macros.h"
#include "peripheral_nums.h"

#define PIN_TCC0_DEBUG 20 
#define PRT_TCC0_DEBUG 0 

void setup_tcc_0(void){
  // use TCC0 for very fast pwm out, 
  MCLK->APBBMASK.bit.TCC0_ = 1; 

  // setup a generic clock for TCC0, 
  GCLK->PCHCTRL[TCC0_GCLK_ID].bit.GEN = GCLK_PCHCTRL_GEN_GCLK0_Val; // use GCLK0 (@ 48MHz?)
  GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN = 1; // enable ?
  while(!GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN);

  // configure an output pin to check 
  PIN_SETUP_PMUXEN(PRT_TCC0_DEBUG, PIN_TCC0_DEBUG);
  PORT->Group[PRT_TCC0_DEBUG].PMUX[PIN_TCC0_DEBUG >> 1].reg |= 
    (PIN_TCC0_DEBUG % 2 ? PORT_PMUX_PMUXO(PERIPHERAL_G) : PORT_PMUX_PMUXE(PERIPHERAL_G)); 

  // setup TCC0: reset and wait to clear 
  TCC0->CTRLA.bit.SWRST = 1;
  while(TCC0->SYNCBUSY.bit.SWRST);

  // set prescaler and reload on gclk, this is baseline main_freq (120MHz default)
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_GCLK; 

  // we want... match frequency mode
  // in MFRQ, top = CC0 
  // and toggle will only come out on TCC0[0] (PA20, Periperal G) 
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_MFRQ_Val;
  while(TCC0->SYNCBUSY.bit.WAVE);

  // we'll use the output from this to increase count on the next 
  // nyet, dmac trig 
  // TCC0->EVCTRL.reg = TCC_EVCTRL_OVFEO;

  // setup ... half duty 
  write_freq(1 << 8);

  // now en-able, 
  TCC0->CTRLA.bit.ENABLE = 1; 
  while(TCC0->SYNCBUSY.bit.ENABLE);
}

void setup_dac(void){
  // ... hello arduino 
  pinMode(DAC0, OUTPUT);
  analogWriteResolution(12); 
  analogWrite(DAC0, 0);
}

// our dmac buffer ... 12 bits, so 
#define DMAC_WAVE_SIZE 4096 
uint16_t dmac_wave[DMAC_WAVE_SIZE];

// we allocate 32 each, *just in case* CH2 > is ever enabled, 
// it won't point at garbage and hard fault 
static DmacDescriptor descriptors[32] __attribute__((aligned(16)));
static DmacDescriptor wb_descriptors[32] __attribute__((aligned(16)));

void setup_dmac(void){
  // write vals 2 wave
  for(uint16_t i = 0; i < DMAC_WAVE_SIZE; i ++){
    dmac_wave[i] = i;
  }
  // setup dmac ... 
  // disable to config 
  DMAC->CTRL.bit.DMAENABLE = 0; 
  // descriptor addr, writeback addr 
  DMAC->BASEADDR.reg = (uint32_t)&descriptors;
  DMAC->WRBADDR.reg = (uint32_t)&wb_descriptors; 
  // turn on all levels 
  DMAC->CTRL.bit.LVLEN0 = 1;
  DMAC->CTRL.bit.LVLEN1 = 1;
  DMAC->CTRL.bit.LVLEN2 = 1;
  DMAC->CTRL.bit.LVLEN3 = 1;
  // re-enable 
  DMAC->CTRL.bit.DMAENABLE = 1; 
  // configure channel 0 TCC0 -> DAC 
  DMAC->Channel[0].CHCTRLA.bit.BURSTLEN = 0x0;  // single beat burst
  DMAC->Channel[0].CHCTRLA.bit.TRIGACT = 0x2;   // trigger per burst 
  DMAC->Channel[0].CHCTRLA.bit.TRIGSRC = TCC0_DMAC_ID_OVF; // trigger from TCC0 Overflow 
  DMAC->Channel[0].CHPRILVL.bit.PRILVL = 0x0;     // channel has 0 priority (lowest!)
  // set descriptor values... we need two to wrap, 
  for(uint8_t i = 0; i < 2; i ++){
    descriptors[i].BTCTRL.bit.VALID = 0x1;            // descriptor is valid 
    descriptors[i].BTCTRL.bit.EVOSEL = 0x0;           // no events 
    descriptors[i].BTCTRL.bit.BLOCKACT = 0x0;         // no end of block action: disables apres 
    descriptors[i].BTCTRL.bit.BEATSIZE = 0x1;         // beatsize = 1 = HWORD = 2 bytes
    descriptors[i].BTCTRL.bit.SRCINC = 0x1;           // inc. source addr 
    descriptors[i].BTCTRL.bit.DSTINC = 0x0;           // don't increment destination 
    descriptors[i].BTCTRL.bit.STEPSEL = 0x1;          // step applies to src 
    descriptors[i].BTCTRL.bit.STEPSIZE = 0x0;         // step size is "X1" 
    // number of 'beats' (events) 
    descriptors[i].BTCNT.reg = DMAC_WAVE_SIZE;        // how many itemz 
    // pipe from...to 
    descriptors[i].SRCADDR.reg = (uint32_t)(dmac_wave + DMAC_WAVE_SIZE);   // src starts here (pre decriment?)
    descriptors[i].DSTADDR.reg = (uint32_t)&(DAC->DATA[0].reg) ;            // hwords to dac 
  }
  // end of descriptor -> start of next, 
  descriptors[0].DESCADDR.reg = (uint32_t)&(descriptors[1]);
  descriptors[1].DESCADDR.reg = (uint32_t)&(descriptors[0]);

  // and turn the channel on, forever ? 
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1; 
}

void setup_timers(void){
  setup_tcc_0();
  setup_dac();
  setup_dmac();
}

void write_freq(uint16_t per){
  TCC0->CCBUF[0].reg = per; 
}
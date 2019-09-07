/******************************************************************************
* SoftSerial.cpp
* Multi-instance Software Serial Library for STM32Duino
* Using Timers and Interrupts enabling use of any GPIO pins for RX/TX
*
* Copyright 2015 Ron Curry, InSyte Technologies
*

Features:
- Fully interrupt driven - no delay routines.
- Any GPIO pin may be used for tx or rx (hence the AP libray name suffix)
- Circular buffers on both send and receive.
- Works up to 115,200 baud.
- Member functions compatible with Hardware Serial and Arduino NewSoftSerial
Libs
- Extensions for non-blocking read and transmit control
- Supports up to 4 ports (one timer used per port) without modification.
- Easily modified for more ports or different timers on chips with more timers.
- Can do full duplex under certain circumstances at lower baud rates.
- Can send/receive simultaneously with other ports/instantiatious at some baud
  rates and certain circumstances.

Notes:

Performance
- More than two ports use has not been extensively tested. High ISR latencies in
the low-level Maple timer ISR code, C++ ISR wranglings, and the STM32 sharing
interrupt architecture (not in that order) restrict simultaneous port use and
speeds. Two ports sending simultaniously at 115,200. As well, two ports
simultansiously receiving at 57,600 simultaneously have been successfully
tested. Various other situations have been tested. Results are dependent on
various factors - you'll need to experiment.

Reliability
- Because of the way STM32 shares interrupts and the way STM32Arduino low level
ISRs processes them latencies can be very high for interrupts on a given timer.
I won't go into all the details of why but some interrupts can be essentially
locked out. Causing extremely delayed interrupt servicing. Some of this could be
alleviated with more sophisticated low-level ISRs that make sure that all shared
interrupt sources get serviced on an equal basis but that's not been done yet.
This impacts the ability to do full duplex on a single port even at medium bit
rates. I've done some experimentation with two ports/instantiations with RX on
one port and TX on the other with good results for full-duplex. In any case, to
be sure that the serial data streams are not corrupted at higher data rates it's
best to write the application software such that rx and tx are not
sending/receiving simultaneously in each port and that other ports are not
operating simultaneously as well. This is, effectively, how the existing
NewSoftSerial library on Arduino operates so not a new concept. Again,
experiment and find what works in your application.

Improvements
No doubt the code can be improved upon. If you use the code PLEASE give back by
providing soure code for improvements or modifications you've made!
- Specific improvements that come to mind and I'd like to explore are:
  o Replacing the STM32/Maple timer interrupt handlers with something more
streamlined and lower latency and overhead. o A better way to implement the high
level C++ ISR's to reduce latency/overhead o Minor improvements that can save
cycles in the C++ ISR's such as using bit-banding o Possibly a way to coordinate
RX/TX to increase full-duplex capability.

License
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*****************************************************************************/

#include <Arduino.h>
#include <HardwareTimer.h>
#ifdef BMAP
#include <ext_interrupts.h>
#include <libmaple/timer.h>
#undef __always_inline
#define __always_inline inline __attribute__((always_inline))
#endif
#include "SoftSerial.h"
char s_dbg[300];
static void dbg(const String &p) {
  String fmt = String(s_dbg) + "[OUT][" + String(millis()) + "] " + p + "\n";
  if (fmt.length() < 300)
    strcpy(s_dbg, fmt.c_str());
}
void p_dbg(Stream *S) {
  S->print("DBG:" + String(s_dbg));
  s_dbg[0] = '\0';
}

#define T_PAUSE timerSerial.pause
#define T_DINT timerSerial.detachInterrupt
#define T_AINT timerSerial.attachInterrupt
#define T_SMODE timerSerial.setMode
#define T_SETOF timerSerial.setOverflow
#define T_SPSCALE timerSerial.setPrescaleFactor
#define T_GPSCALE timerSerial.getPrescaleFactor
#define T_TIMFREQ(a) F_CPU
#define T_RESUM timerSerial.resume

#ifdef BMAP
#define TIMS(a) (a)

#if 1
#define TIM_GET_COMPARE(__HANDLE__, __CHANNEL__)                               \
  (*(__IO uint32_t *)(&(((__HANDLE__)->regs).gen->CCR1) + ((__CHANNEL__))))
#define TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__)                  \
  (*(__IO uint32_t *)(&(((__HANDLE__)->regs).gen->CCR1) + ((__CHANNEL__))) = (__COMPARE__))
#define T_GCOMP(a) TIM_GET_COMPARE(timerSerialDEV, (a)-1)
#define T_SCOMP(a, b) TIM_SET_COMPARE(timerSerialDEV, (a)-1, (b))
#define T_GCNT(a) ((int16_t)(timerSerialDEV->regs).gen->CNT)
#define T_SCNT(a) (timerSerialDEV->regs).gen->CNT = (a)
#else
#define T_GCOMP timerSerial.getCompare
#define T_SCOMP timerSerial.setCompare
#define T_GCNT timerSerial.getCount
#define T_SCNT timerSerial.setCount
#endif
#define T_RFR timerSerial.refresh

#define _gpiowrite(a, b, c) (a)->regs->BSRR = (1U << (b)) << (16 * !(c))
#define _gpioread(a, b) ((a)->regs->IDR & (1U << (b)))
#define _readrx(a) _gpioread(rxport, rxbit)
#define _writepin(a, b)                                                        \
  (PIN_MAP[(a)].gpio_device)->regs->BSRR = (1U << PIN_MAP[(a)].gpio_bit)       \
                                           << (16 * !(b))
#define _writetx(c) txport->regs->BSRR = (1U << txbit) << (16 * !(c))

#elif defined(BHAL)

TIM_TypeDef *TIMS(uint8_t num) {
  switch (num) {
  case 1:
  default:
    return TIM1;
    break;
#ifdef TIM2
  case 2:
    return TIM2;
    break;
#endif
#ifdef TIM3
  case 3:
    return TIM3;
    break;
#endif
#ifdef TIM4
  case 4:
    return TIM4;
    break;
#endif
  }
}

#if 1
#define TIM_GET_COMPARE(__HANDLE__, __CHANNEL__)                               \
  (*(__IO uint32_t *)(&((__HANDLE__)->CCR1) + ((__CHANNEL__))))
#define TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__)                  \
  (*(__IO uint32_t *)(&((__HANDLE__)->CCR1) + ((__CHANNEL__))) = (__COMPARE__))
#define T_GCOMP(a) TIM_GET_COMPARE(timerSerialDEV, (a)-1)
#define T_SCOMP(a, b) TIM_SET_COMPARE(timerSerialDEV, (a)-1, (b))
#define T_GCNT(a) ((int16_t)timerSerialDEV->CNT)
#define T_SCNT(a) timerSerialDEV->CNT = (a)
#else
#define T_GCOMP timerSerial.getCaptureCompare
#define T_SCOMP timerSerial.setCaptureCompare
#define T_GCNT timerSerial.getCount
#define T_SCNT timerSerial.setCount
#endif

#define T_RFR(a) SET_BIT(timerSerialDEV->EGR, 0U)

#define _gpiowrite(a, b, c) (a)->BSRR = (1U << (b)) << (16 * !(c))
#define _gpioread(a, b) ((a)->IDR & (1U << (b)))
#define _readrx(a) _gpioread(rxport, rxbit)
#define _writepin(a, b)                                                        \
  _gpiowrite(get_GPIO_Port(STM_PORT(digitalPin[((a))])),                       \
             STM_PIN(digitalPin[((a))]), ((b)))
#define _writetx(c) _gpiowrite(txport, txbit, ((c)))
#elif defined(BGEN)
extern HardwareTimer *interruptTimers[18];
HardwareTimer *TIMS(uint8_t num) {
  switch (num) {
  case 1:
  default:
    return &Timer1;
    break;
#ifdef TIM2
  case 2:
    return &Timer2;
    break;
#endif
#ifdef TIM3
  case 3:
    return &Timer3;
    break;
#endif
#ifdef TIM4
  case 4:
    return &Timer4;
    break;
#endif
  }
  // return *interruptTimers[num];
}

TIM_TypeDef *TIMDEVS(uint8_t num) {
  switch (num) {
  case 1:
  default:
    return TIM1;
    break;
#ifdef TIM2
  case 2:
    return TIM2;
    break;
#endif
#ifdef TIM3
  case 3:
    return TIM3;
    break;
#endif
#ifdef TIM4
  case 4:
    return TIM4;
    break;
#endif
  }
}

#define T_GCOMP timerSerial.getCompare
#define T_SCOMP timerSerial.setCompare
#define T_GCNT timerSerial.getCount
#define T_SCNT timerSerial.setCount
#define T_RFR(a)

#define _gpiowrite(a, b, c) (a)->BSRR = (1U << (b)) << (16 * !(c))
#define _gpioread(a, b) ((a)->IDR & (1U << (b)))
#define _readrx(a) _gpioread(rxport, rxbit)
#define _writepin(a, b)                                                        \
  _gpiowrite(variant_pin_list[((a))].port,                                     \
             get_pin_id(variant_pin_list[((a))].pin_mask), ((b)))
#define _writetx(c) _gpiowrite(txport, txbit, ((c)))
#else
#error Core not supported
#endif

#define BIT_CHECK(a, b) ((a) & (1 << (b)))
/******************************************************************************
 * Timer Definitions
 * Change these if you wish to use different timer channels
 ******************************************************************************/

#define TIMER_MAX_COUNT 0xffff
#define _TX_CHANNEL 3
#define _RX_CHANNEL 4

/******************************************************************************
 * ISR Related to Statics
 * Don't modify anything here unless you know why and what
 ******************************************************************************/
SoftSerial *SoftSerial::interruptObject1;
SoftSerial *SoftSerial::interruptObject2;
SoftSerial *SoftSerial::interruptObject3;
SoftSerial *SoftSerial::interruptObject4;

voidFuncPtr SoftSerial::handleRXEdgeInterruptP[4] = {
    handleRXEdgeInterrupt1, handleRXEdgeInterrupt2, handleRXEdgeInterrupt3,
    handleRXEdgeInterrupt4};

htFuncPtr SoftSerial::handleRXBitInterruptP[4] = {
    handleRXBitInterrupt1, handleRXBitInterrupt2, handleRXBitInterrupt3,
    handleRXBitInterrupt4};

htFuncPtr SoftSerial::handleTXBitInterruptP[4] = {
    handleTXBitInterrupt1, handleTXBitInterrupt2, handleTXBitInterrupt3,
    handleTXBitInterrupt4};

#if defined(BHAL) || defined(BGEN)
#define TIMER_CH1 1 // TIM_CHANNEL_1
#define TIMER_CH2 2 // TIM_CHANNEL_2
#define TIMER_CH3 3 // TIM_CHANNEL_3
#define TIMER_CH4 4 // TIM_CHANNEL_4

#define TIMER_DIER_CC1IE_BIT TIM_DIER_CC1IE
#define TIMER_DIER_CC2IE_BIT TIM_DIER_CC2IE
#define TIMER_DIER_CC3IE_BIT TIM_DIER_CC3IE
#define TIMER_DIER_CC4IE_BIT TIM_DIER_CC4IE

#define TIMER_SR_CC1IF_BIT TIM_SR_CC1IF
#define TIMER_SR_CC2IF_BIT TIM_SR_CC2IF
#define TIMER_SR_CC3IF_BIT TIM_SR_CC3IF
#define TIMER_SR_CC4IF_BIT TIM_SR_CC4IF
#endif

#ifdef BMAP
static __always_inline uint32_t __RBIT(uint32_t value) {
  uint32_t result;
  asm volatile("rbit %0, %1" : "=r"(result) : "r"(value));
  return result;
}

#define POSITION_VAL(a) __builtin_clz(__RBIT((a)))
#endif

#if 0
static
  __always_inline
  uint8_t
get_pin_id (uint16_t pin)
{
  uint8_t
    id = 0;

  while (pin != 0x0001)
    {
      pin = pin >> 1;
      id++;
    }

  return id;
}
#else
#define get_pin_id POSITION_VAL
#endif

void SoftSerial::print_counters(Stream *S) {
  S->println("[" + String(millis()) + "] rxbit=" + String(rxbitc) +
             ", rxedge=" + String(rxedgec) + ", txbit=" + String(txbitc));
}

__always_inline void SoftSerial::init_timer_values(uint8_t TX_CHANNEL,
                                                   uint8_t RX_CHANNEL) {
  // reset counters
  rxbitc = rxedgec = txbitc = 0;
  s_dbg[0] = '\0';

  // Assign pointer to the hardware registers
#ifdef BHAL
  timerSerialDEV = TIMS(rxtxTimer);
#elif defined(BGEN)
  timerSerialDEV = TIMDEVS(rxtxTimer);
#else
  timerSerialDEV = timerSerial.c_dev();
#endif

  _rx_channel = RX_CHANNEL;
  _tx_channel = TX_CHANNEL;
  if (_rx_channel == _tx_channel) {
    _rx_channel = _RX_CHANNEL;
    _tx_channel = _TX_CHANNEL;
  }
  switch (_rx_channel) {
  case 1:
    RX_TIMER_CHANNEL = TIMER_CH1;
    RX_TIMER_MASK = TIMER_DIER_CC1IE_BIT;
    RX_TIMER_PENDING = TIMER_SR_CC1IF_BIT;
    break;
  case 2:
    RX_TIMER_CHANNEL = TIMER_CH2;
    RX_TIMER_MASK = TIMER_DIER_CC2IE_BIT;
    RX_TIMER_PENDING = TIMER_SR_CC2IF_BIT;
    break;
  case 3:
    RX_TIMER_CHANNEL = TIMER_CH3;
    RX_TIMER_MASK = TIMER_DIER_CC3IE_BIT;
    RX_TIMER_PENDING = TIMER_SR_CC3IF_BIT;
    break;
  case 4:
  default:
    RX_TIMER_CHANNEL = TIMER_CH4;
    RX_TIMER_MASK = TIMER_DIER_CC4IE_BIT;
    RX_TIMER_PENDING = TIMER_SR_CC4IF_BIT;
    break;
  }
  switch (_tx_channel) {
  case 1:
    TX_TIMER_CHANNEL = TIMER_CH1;
    TX_TIMER_MASK = TIMER_DIER_CC1IE_BIT;
    TX_TIMER_PENDING = TIMER_SR_CC1IF_BIT;
    break;
  case 2:
    TX_TIMER_CHANNEL = TIMER_CH2;
    TX_TIMER_MASK = TIMER_DIER_CC2IE_BIT;
    TX_TIMER_PENDING = TIMER_SR_CC2IF_BIT;
    break;
  default:
  case 3:
    TX_TIMER_CHANNEL = TIMER_CH3;
    TX_TIMER_MASK = TIMER_DIER_CC3IE_BIT;
    TX_TIMER_PENDING = TIMER_SR_CC3IF_BIT;
    break;
  case 4:
    TX_TIMER_CHANNEL = TIMER_CH4;
    TX_TIMER_MASK = TIMER_DIER_CC4IE_BIT;
    TX_TIMER_PENDING = TIMER_SR_CC4IF_BIT;
    break;
  }

  // Translate transmit pin number to external interrupt number
#if defined(BMAP)
  txport = PIN_MAP[transmitPin].gpio_device;
  txbit = PIN_MAP[transmitPin].gpio_bit;
  rxport = PIN_MAP[receivePin].gpio_device;
  rxbit = PIN_MAP[receivePin].gpio_bit;
  gpioBit = (exti_num)(txbit);
#elif defined(BHAL)
  txport = get_GPIO_Port(STM_PORT(digitalPin[transmitPin]));
  txbit = STM_PIN(digitalPin[transmitPin]);
  rxport = get_GPIO_Port(STM_PORT(digitalPin[receivePin]));
  rxbit = STM_PIN(digitalPin[receivePin]);
  gpioBit = get_pin_id(STM_GPIO_PIN(digitalPinToPinName(transmitPin)));
#else
  txport = variant_pin_list[transmitPin].port;
  txbit = get_pin_id(variant_pin_list[transmitPin].pin_mask);
  rxport = variant_pin_list[receivePin].port;
  rxbit = get_pin_id(variant_pin_list[receivePin].pin_mask);
  gpioBit = get_pin_id(txbit);
#endif
}

/******************************************************************************
 * Convenience functions to disable/enable tx and rx interrupts
 ******************************************************************************/
// Mask transmit interrupt
__always_inline void SoftSerial::noTXInterrupts() {
  // dbg("noTXint");
#ifdef BMAP
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 0;
#else
  CLEAR_BIT(timerSerialDEV->DIER, TX_TIMER_MASK);
#endif
}

// Enable transmit interrupt
// Note: Purposely does not clear pending interrupt
__always_inline void SoftSerial::txInterrupts() {
  // dbg("TXint");
#ifdef BMAP
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 1;
#else
  SET_BIT(timerSerialDEV->DIER, TX_TIMER_MASK);
#endif
}

// Test if transmit interrupt is enabled
__always_inline uint16_t SoftSerial::isTXInterruptEnabled() {
#ifdef BMAP
  uint16_t val = (*bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK));
#else
  uint16_t val =
      (READ_BIT(timerSerialDEV->DIER, TX_TIMER_MASK) == TX_TIMER_MASK);
#endif
  return (val);
}

// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
__always_inline void SoftSerial::txInterruptsClr() {
#ifdef BMAP
  *bb_perip(&(timerSerialDEV->regs).gen->SR, TX_TIMER_PENDING) = 0; // Clear
  // int
  // pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 1;
#else
  CLEAR_BIT(timerSerialDEV->SR, TX_TIMER_PENDING);
  SET_BIT(timerSerialDEV->DIER, TX_TIMER_MASK);
#endif
}

// Mask receive start bit interrupt
__always_inline void SoftSerial::noRXStartInterrupts() {
#ifdef BMAP
  bb_peri_set_bit(&EXTI_BASE->FTSR, gpioBit, 0);
#else
  CLEAR_BIT(EXTI->FTSR, 1U << gpioBit);
#endif
}

// Enable receive start bit interrupt
// Note: Purposely does not clear pending interrupt
__always_inline void SoftSerial::rxStartInterrupts() {
#ifdef BMAP
  bb_peri_set_bit(&EXTI_BASE->FTSR, gpioBit, 1);
#else
  SET_BIT(EXTI->FTSR, 1U << gpioBit);
#endif
}

// Mask receive interrupt
__always_inline void SoftSerial::noRXInterrupts() {
#ifdef BMAP
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 0;
#else
  CLEAR_BIT(timerSerialDEV->DIER, RX_TIMER_MASK);
#endif
}

// Enable receive interrupt
// Note: Purposely does not clear pending interrupt
__always_inline void SoftSerial::rxInterrupts() {
#ifdef BMAP
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 1;
#else
  SET_BIT(timerSerialDEV->DIER, RX_TIMER_MASK);
#endif
}

// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
__always_inline void SoftSerial::rxInterruptsClr() {
#ifdef BMAP
  *bb_perip(&(timerSerialDEV->regs).gen->SR, RX_TIMER_PENDING) = 0; // Clear
  // int
  // pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 1;
#else
  CLEAR_BIT(timerSerialDEV->SR, RX_TIMER_PENDING);
  SET_BIT(timerSerialDEV->DIER, RX_TIMER_MASK);
#endif
}

/******************************************************************************
 * Specialized functions to set interrupt priorities and assign object ointers
 * These are needed due to the gyrations required to support per instance ISRs
 ******************************************************************************/
// Set Interrupt Priority for EXTInt line
void setEXTIntPriority(uint8_t pin, uint8_t priority) {
#if defined(BHAL) || defined(BGEN)
  static const IRQn_Type irqnb[16] = {
      EXTI0_IRQn,     // GPIO_PIN_0
      EXTI1_IRQn,     // GPIO_PIN_1
      EXTI2_IRQn,     // GPIO_PIN_2
      EXTI3_IRQn,     // GPIO_PIN_3
      EXTI4_IRQn,     // GPIO_PIN_4
      EXTI9_5_IRQn,   // GPIO_PIN_5
      EXTI9_5_IRQn,   // GPIO_PIN_6
      EXTI9_5_IRQn,   // GPIO_PIN_7
      EXTI9_5_IRQn,   // GPIO_PIN_8
      EXTI9_5_IRQn,   // GPIO_PIN_9
      EXTI15_10_IRQn, // GPIO_PIN_10
      EXTI15_10_IRQn, // GPIO_PIN_11
      EXTI15_10_IRQn, // GPIO_PIN_12
      EXTI15_10_IRQn, // GPIO_PIN_13
      EXTI15_10_IRQn, // GPIO_PIN_14
      EXTI15_10_IRQn, // GPIO_PIN_15
  };
#ifdef BHAL
  uint16_t ppin = get_pin_id(STM_GPIO_PIN(digitalPinToPinName(pin)));
#else
  uint16_t ppin = get_pin_id(variant_pin_list[pin].pin_mask);
#endif
  NVIC_SetPriority(irqnb[ppin], priority);
#elif defined(BMAP)
  switch ((exti_num)(PIN_MAP[pin].gpio_bit)) {
  case EXTI0:
    nvic_irq_set_priority(NVIC_EXTI0, priority);
    break;
  case EXTI1:
    nvic_irq_set_priority(NVIC_EXTI1, priority);
    break;
  case EXTI2:
    nvic_irq_set_priority(NVIC_EXTI2, priority);
    break;
  case EXTI3:
    nvic_irq_set_priority(NVIC_EXTI3, priority);
    break;
  case EXTI4:
    nvic_irq_set_priority(NVIC_EXTI4, priority);
    break;
  case EXTI5:
  case EXTI6:
  case EXTI7:
  case EXTI8:
  case EXTI9:
    nvic_irq_set_priority(NVIC_EXTI_9_5, priority);
    break;
  case EXTI10:
  case EXTI11:
  case EXTI12:
  case EXTI13:
  case EXTI14:
  case EXTI15:
    nvic_irq_set_priority(NVIC_EXTI_15_10, priority);
    break;
  }
#else
// BGEN
#endif
}

// Set Interrupt Priority for Timer Interrupts
// mapple core only, HAL requires hack
void setTimerIntPriority(uint8_t timerNumber, uint8_t priority) {
#ifdef BMAP
  switch (timerNumber) {
  case 1:
    nvic_irq_set_priority(NVIC_TIMER1_UP, priority);
    nvic_irq_set_priority(NVIC_TIMER1_CC, priority);
    break;
  case 2:
    nvic_irq_set_priority(NVIC_TIMER2, priority);
    break;
  case 3:
    nvic_irq_set_priority(NVIC_TIMER3, priority);
    break;
  case 4:
    nvic_irq_set_priority(NVIC_TIMER4, priority);
    break;
  }
#endif
}

// Set the correct interruptObject for this instance
void SoftSerial::setInterruptObject(uint8_t timerNumber) {

  switch (timerNumber) {
  case 1:
    interruptObject1 = this;
    break;
  case 2:
    interruptObject2 = this;
    break;
  case 3:
    interruptObject3 = this;
    break;
  case 4:
    interruptObject4 = this;
    break;
  }
}

/******************************************************************************
 * Constructor / Destructor
 ******************************************************************************/
// Constructor
SoftSerial::SoftSerial(int receivePinT = 15, int transmitPinT = 16,
                       uint8_t rxtxTimerT = 1, uint8_t tx_channel = _TX_CHANNEL,
                       uint8_t rx_channel = _RX_CHANNEL)
    : receivePin(receivePinT), transmitPin(transmitPinT),
#ifndef BGEN
      timerSerial(TIMS(rxtxTimerT)),
#else
      timerSerialP(TIMS(rxtxTimerT)),
#endif
      rxtxTimer(rxtxTimerT) {
  init_timer_values(tx_channel, rx_channel);

  // Setup ISR pointer for this instance and timer (one timer per
  // instance)
  // This is a workaround for c++
  setInterruptObject(rxtxTimer);
}

SoftSerial::SoftSerial(int receivePinT = 15, int transmitPinT = 16,
                       uint8_t rxtxTimerT = 1)
    : receivePin(receivePinT), transmitPin(transmitPinT),
#ifndef BGEN
      timerSerial(TIMS(rxtxTimerT)),
#else
      timerSerialP(TIMS(rxtxTimerT)),
#endif
      rxtxTimer(rxtxTimerT) {
  init_timer_values(_TX_CHANNEL, _RX_CHANNEL);

  // Setup ISR pointer for this instance and timer (one timer per
  // instance)
  // This is a workaround for c++
  setInterruptObject(rxtxTimer);
}

// Destructor
SoftSerial::~SoftSerial() { end(); }

#ifdef BHAL
#define HTIM HardwareTimer *ht
#define _HTIM ht
//#define _SCOMP ht->setCaptureCompare
//#define _GCOMP ht->getCaptureCompare
#define _SCOMP T_SCOMP
#define _GCOMP T_GCOMP
#else
#define HTIM void
#define _HTIM
#define _SCOMP T_SCOMP
#define _GCOMP T_GCOMP
#endif

/******************************************************************************
 * TX and RX Interrupt Service Routines
 ******************************************************************************/
// Transmits next bit. Called by timer ch1 compare interrupt
void SoftSerial::txNextBit(HTIM) {
  ++txbitc;
  // State 0 through 7 - receive bits
  if (txBitCount <= 7) {
    if (BIT_CHECK(transmitBuffer[transmitBufferRead], txBitCount) > 0)
      _writetx(HIGH);
    else
      _writetx(LOW);

    _SCOMP(TX_TIMER_CHANNEL, ((uint16_t)_GCOMP(TX_TIMER_CHANNEL)) + bitPeriod);

    interrupts();
    // Bump the bit/state counter to state 8
    ++txBitCount;

#if DEBUG_DELAY
    _writepin(DEBUG_PIN1, 1);
    _writepin(DEBUG_PIN1, 0);
#endif

    // State 8 - Send the stop bit and reset state to state -1
    // Shutdown timer interrupt if buffer empty
  } else if (txBitCount == 8) {
    // Send the stop bit
    _writetx(HIGH);

    interrupts();

    transmitBufferRead =
        (transmitBufferRead == SS_MAX_TX_BUFF) ? 0 : transmitBufferRead + 1;

    if ((transmitBufferRead != transmitBufferWrite) && activeTX) {

      _SCOMP(TX_TIMER_CHANNEL,
             ((uint16_t)_GCOMP(TX_TIMER_CHANNEL)) + bitPeriod);
      txBitCount = 10;

    } else {

      ++txBitCount;
      // Buffer empty so shutdown delay/timer until "write" puts
      // data in
      noTXInterrupts();
    }

    // Send start bit for new byte
  } else if (txBitCount == 10) {
    _writetx(LOW);
    interrupts();

    _SCOMP(TX_TIMER_CHANNEL, ((uint16_t)_GCOMP(TX_TIMER_CHANNEL)) + bitPeriod);

    txBitCount = 0;
  }
}

// Start Bit Receive ISR
__always_inline void SoftSerial::onRXPinChange(void) {
  ++rxedgec;
  // Test if this is really the start bit and not a spurious edge
  if ((rxBitCount == 9) && activeRX) {

    // Receive Timer/delay interrupt should be off now - unmask it and
    // center the sampling time
    T_SCOMP(RX_TIMER_CHANNEL, (uint16_t)T_GCNT() + startBitPeriod);

    rxInterruptsClr();

    // Mask pinchange interrupt to reduce needless interrupt
    // overhead while receiving this byte
    noRXStartInterrupts();
    interrupts();

    // Set state/bit to first bit
    rxBitCount = 0;

  } else
    interrupts();
}

// Receive next bit. Called by timer ch2 interrupt
__always_inline void SoftSerial::rxNextBit(HTIM) {
  ++rxbitc;
  //  if (!activeRX) return;
  if (rxBitCount < 8) {

    _SCOMP(RX_TIMER_CHANNEL, ((uint16_t)_GCOMP(RX_TIMER_CHANNEL)) + bitPeriod);

    receiveBuffer[receiveBufferWrite] >>= 1;
    if (_readrx())
      receiveBuffer[receiveBufferWrite] |= 0x80;

#if DEBUG_DELAY
    _writepin(DEBUG_PIN, 1);
    _writepin(DEBUG_PIN, 0);
#endif

    interrupts();

    ++rxBitCount;

    // State 8 - Save incoming byte and update buffer
  } else if (rxBitCount == 8) {

    // Finish out stop bit while we...
    // Calculate location in buffer for next incoming byte
    // Test if buffer full
    // If the buffer isn't full update the tail pointer to point to
    // next location
    // Else if it is now full set the buffer overflow flag
    // FYI - With this logic we effectively only have an
    // (SS_MAX_RX_BUFF - 1) buffer size

    interrupts();
    uint8_t next = (receiveBufferWrite + 1) % SS_MAX_RX_BUFF;
    if (next != receiveBufferRead)
      receiveBufferWrite = next;
    else {
      bufferOverflow = true;

#if DEBUG_DELAY
      overFlowTail = receiveBufferWrite;
      overFlowHead = receiveBufferRead;

      _writepin(DEBUG_PIN1, 1);
      _writepin(DEBUG_PIN1, 0);
#endif
    }

    // Re-enable start bit detection
    rxStartInterrupts();

    // Shutdown nextbit timer interrupt until next start bit detected
    noRXInterrupts();

    // Set for state 9 to receive next byte
    rxBitCount = 9;

  } else {
    interrupts();
  }
}

/******************************************************************************
 * Begin - Instance setup
 ******************************************************************************/
void SoftSerial::begin(uint32_t tBaud) {

  _writetx(HIGH);
  pinMode(receivePin, INPUT_PULLUP);
  pinMode(transmitPin, OUTPUT);

#if DEBUG_DELAY
  pinMode(DEBUG_PIN, OUTPUT);
  _writepin(DEBUG_PIN, 0);
  pinMode(DEBUG_PIN1, OUTPUT);
  _writepin(DEBUG_PIN1, 0);
#endif

  // Initialize the timer
  noInterrupts();

  T_PAUSE();

#define DIV 24

  bitPeriod = (uint16_t)(((uint32_t)(T_TIMFREQ()) / DIV) / tBaud);
  startBitPeriod = bitPeriod + (bitPeriod / 2) - (320 / DIV);
  T_SPSCALE(DIV);
  T_SETOF(TIMER_MAX_COUNT);

  // Set transmit bit timer
  // Compare value set later
  T_SMODE(TX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);

  // State tx machine start state, attach bit interrupt, and mask it
  // until a byte is sent
  transmitBufferRead = transmitBufferWrite = 0;
  txBitCount = 9;
  noTXInterrupts();
  T_AINT(TX_TIMER_CHANNEL, handleTXBitInterruptP[rxtxTimer - 1]);

  // Set receive bit timer
  // Compare value set later
  T_SMODE(RX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);

  // Set rx State machine start state, attach the bit interrupt and mask
  // it until start bit is received
  receiveBufferRead = receiveBufferWrite = 0;
  rxBitCount = 9;
  noRXInterrupts();
  T_AINT(RX_TIMER_CHANNEL, handleRXBitInterruptP[rxtxTimer - 1]);

  // Make the timer we are using a high priority interrupt
#ifdef BMAP
  setTimerIntPriority(rxtxTimer, 0);
#endif

  // Load the timer values and start it
  T_RFR();
  T_RESUM();
#ifndef BMAP
  // timer IRQ priority hack
  static const IRQn_Type irqtim[4] = {TIM1_UP_IRQn, TIM2_IRQn, TIM3_IRQn,
                                      TIM4_IRQn};
  NVIC_DisableIRQ(irqtim[rxtxTimer - 1]);
  if (rxtxTimer == 1)
    NVIC_DisableIRQ(TIM1_CC_IRQn);
  NVIC_SetPriority(irqtim[rxtxTimer - 1], 0);
  if (rxtxTimer == 1)
    NVIC_SetPriority(TIM1_CC_IRQn, 0);
  NVIC_EnableIRQ(irqtim[rxtxTimer - 1]);
  if (rxtxTimer == 1)
    NVIC_EnableIRQ(TIM1_CC_IRQn);
  CLEAR_BIT(timerSerialDEV->DIER, TIM_DIER_UIE);
#endif

  // Set start bit interrupt and priority and leave it enabled to rx
  // first byte
  attachInterrupt(receivePin, handleRXEdgeInterruptP[rxtxTimer - 1], FALLING);

#ifdef BHAL
  set_GPIO_Port_Clock(STM_PORT(digitalPin[receivePin]));
#elif defined(BGEN)
  stm32GpioClockEnable(rxport);
#endif
  setEXTIntPriority(receivePin, 0);

  bufferOverflow = false;
  receiveBufferRead = receiveBufferWrite = 0;
  transmitBufferRead = transmitBufferWrite = 0;

  noTXInterrupts();
  noRXInterrupts();
  interrupts();

  listen();
  talk();
}

/******************************************************************************
 * RX Related Public Methods
 ******************************************************************************/
// Sets current instance listening. Transmit is always enabled
// If his instance was already activeRX does nothing and returns false
bool SoftSerial::listen() {

  // If receive not activeRX then re-init and set activeRX
  if (!activeRX) {

    // Reset receieve buffer and mark activeRX
    bufferOverflow = false;
    receiveBufferRead = receiveBufferWrite = 0;
    activeRX = true;

    // Turn the receive start bit detection on
    rxStartInterrupts();

    return true;
  }
  return false;
}

// Stop Listening - Shuts down only RX - Use end() to stop both rx and tx
// Returns true if was listening when called
// This instance will stop all RX interrupts after current in-process
// byte is finished receiving (if any).
// If no in-process receive byte it stops immediately
bool SoftSerial::stopListening() {

  if (activeRX) {

    noRXStartInterrupts();
    activeRX = false;
    return true;

  } else
    return false;
}

// Completely shutsdown this instance
// Not an RX related method but needs to be after stopListening
void SoftSerial::end() {

  stopListening();
  T_PAUSE();
  detachInterrupt(receivePin);
  T_DINT(RX_TIMER_CHANNEL);
  T_DINT(TX_TIMER_CHANNEL);
  T_SMODE(TX_TIMER_CHANNEL, TIMER_DISABLED);
  T_SMODE(RX_TIMER_CHANNEL, TIMER_DISABLED);
  _writetx(HIGH);
}

// Returns number of bytes in the RX buffer
int SoftSerial::available() {
  int i;

  if (!activeRX)
    return 0;

  // noRXInterrupts();
  i = (receiveBufferWrite + SS_MAX_RX_BUFF - receiveBufferRead) %
      SS_MAX_RX_BUFF;
  // rxInterrupts();

  return i;
}

// Non-blocking read.
// Returns -1 if this instance isn't listening or the buffer is empty
int SoftSerial::readnb() {

  if (!activeRX)
    return -1;

  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  uint8_t inData = receiveBuffer[receiveBufferRead];

  // noRXInterrupts();
  receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;
  // rxInterrupts();

  return inData;
}

// Blocking read to be compatible with HardwareSerial
// Blocks until byte is available in buffer
// Returns -1 if instance is not activeRX
int SoftSerial::read() {

  if (!activeRX)
    return -1;

  // Wait if buffer is empty
  while (receiveBufferRead == receiveBufferWrite)
    ;

  uint8_t inData = receiveBuffer[receiveBufferRead];

  // noRXInterrupts();
  receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;
  // rxInterrupts();

  return inData;
}

// Flush the receive buffer
void SoftSerial::flush() {

  // noRXInterrupts();
  receiveBufferRead = receiveBufferWrite = 0;
  // rxInterrupts();
}

// Return the next item in the receive buffer but leave in buffer
int SoftSerial::peek() {

  if (!activeRX)
    return -1;

  // If buffer is empty return false
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  // Otherwise read the byte at head of buffer but don't delete
  return receiveBuffer[receiveBufferRead];
}

/******************************************************************************
 * TX Related Public Method(s)
 ******************************************************************************/
// Sets current instance enabled for sending
// If his instance was already activeRX does nothing and returns false
bool SoftSerial::talk() {

  // If transmit not active then re-init and set activeTX
  if (!activeTX) {

    // Reset transmit buffer and mark active
    transmitBufferRead = transmitBufferWrite = 0;
    activeTX = true;

    // Turn transmit interrupts on
    // txInterrupts();

    return true;
  }
  return false;
}

// Stop Sending - Shuts down only TX - Use end() to stop both rx and tx
// or "stopListening" for rx
// Returns true if sending already enabled when called
// This instance will stop sending at end of current byte immediately
bool SoftSerial::stopTalking() {

  if (activeTX) {

    while (txBitCount < 8)
      ;
    activeTX = false;
    noTXInterrupts();
    return true;

  } else
    return false;
}

// Virtual write
// Saves tx byte in buffer and restarts transmit delay timer
// 1 bit time latency prior to transmit start if buffer was empty
size_t SoftSerial::write(uint8_t b) {
  if (txBitCount == 9) {
    transmitBufferRead = transmitBufferWrite = 0;
    txBitCount = 10;
  }
  // Check if transmit timer interrupt enabled and if not unmask it
  // transmit timer interrupt will get masked by transmit ISR when
  // buffer becomes empty
  if ((!isTXInterruptEnabled())) {

    // Save new data in buffer
    transmitBuffer[transmitBufferWrite] = b;
    transmitBufferWrite =
        (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;

    // Set state to 10 (send start bit) and re-enable transmit
    // interrupt
    txBitCount = 10;

    T_SCOMP(TX_TIMER_CHANNEL, (int16_t)T_GCNT() + 1);
    txInterruptsClr();

  } else {

    // Blocks if buffer full
    bool i;
    do {
      i = (((transmitBufferWrite + 1) % SS_MAX_TX_BUFF) == transmitBufferRead);
    } while (i);

    // Save new data in buffer and bump the write pointer
    transmitBuffer[transmitBufferWrite] = b;
    transmitBufferWrite =
        (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;
  }

  return 1;
}

void SoftSerial::dbg(const String &p) {
  String fmt = String(s_dbg) + "[" + String(rxtxTimer) + "][" +
               String(millis()) + "] " + p + "\n";
  if (fmt.length() < 300)
    strcpy(s_dbg, fmt.c_str());
}

/******************************************************************************
 *
 * Intermediate Level Interrupt Service Routines
 * One ISR for each interrupt times 4 to support 4 instantiations of the class
 * on up to 4 different timers.
 *
 * This is to work around the fact that static data and
 * static member functions become part of the base class and are common to all
 * instantiations and ISRs must be static in order to derive a std C type
 *pointer to them which is required by the NVIC and hardware timer interrupt
 *code. If there is a better way to do this I'd welcome to learn about it.
 *
 * These are at the bottom of the file just to get them out of the way.
 ******************************************************************************/

__always_inline void SoftSerial::handleRXBitInterrupt1(HTIM) {
  noInterrupts();
  interruptObject1->rxNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXEdgeInterrupt1() {
  if (interruptObject1)
    noInterrupts();
  interruptObject1->onRXPinChange();
  interrupts();
}

__always_inline void SoftSerial::handleTXBitInterrupt1(HTIM) {
  noInterrupts();
  interruptObject1->txNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXBitInterrupt2(HTIM) {
  noInterrupts();
  interruptObject2->rxNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXEdgeInterrupt2() {
  noInterrupts();
  interruptObject2->onRXPinChange();
  interrupts();
}

__always_inline void SoftSerial::handleTXBitInterrupt2(HTIM) {
  noInterrupts();
  interruptObject2->txNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXBitInterrupt3(HTIM) {
  noInterrupts();
  interruptObject3->rxNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXEdgeInterrupt3() {
  noInterrupts();
  interruptObject3->onRXPinChange();
  interrupts();
}

__always_inline void SoftSerial::handleTXBitInterrupt3(HTIM) {
  noInterrupts();
  interruptObject3->txNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXBitInterrupt4(HTIM) {
  noInterrupts();
  interruptObject4->rxNextBit(_HTIM);
  interrupts();
}

__always_inline void SoftSerial::handleRXEdgeInterrupt4() {
  noInterrupts();
  interruptObject4->onRXPinChange();
  interrupts();
}

__always_inline void SoftSerial::handleTXBitInterrupt4(HTIM) {
  noInterrupts();
  interruptObject4->txNextBit(_HTIM);
  interrupts();
}

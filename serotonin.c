//
// Serotonin
//
// The Wookie plexus has three types of nodes: brain, ganglia, and neuron.
// To control hairs, the singleton brain talks to numerous ganglia, and each of
// them talk to numerous neurons, which directly control hairs. To gather
// instrumentation data, neurons respond to requests from ganglia, and ganglia
// respond to requests from the brain.
//
// To the neurons, ganglia are masters. To the ganglia, brain is the master.
// The opposite of a master is a slave. Slaves only talk to their master in
// response to a request.
//
// Talkers wait at least 16 us between messages. Listeners expect at least
// 8 us between messages. Messages consist of a four-bit request type and a
// fixed length payload, corresponding to the request type.
//
// Pins are generally divided into two directions, axons downward and dendrites
// upward.  For instance, ganglia have two dendrite pins and eleven axon pins.
//
// Request types:
//
//   0001 BBBB BBBB BB
//     ganglia to neuron hair (B) write, 12 bits
//
//   0010 IIII IIII
//     ganglia to neuron instrument (I) request, 16 bits
//
//   0011 VVVV VVVV VVVV VVVV VVVV VVVV VVVV VVVV
//     long ganglia to neuron instrument (V) response, 36 bits
//
//   0100 BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB ...
//        BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB BBBB
//     brain to ganglia (B) write, 104 bits
//
//   0101 LLLL IIII IIII
//     brain to ganglia instrument (I) request, 16 bits
//     if LLLL is 1111, read ganglia, otherwise read neuron (L)
//
//   0110 VVVV VVVV VVVV VVVV VVVV VVVV VVVV VVVV
//     long ganglia to brain instrument (V) response, 36 bits
//
//   0000 0000 0000 0000 0000 0000 0000 0000 0000
//     instrument response from peer, ignored, 36 bits
//
//   0111
//   1XXX
//     reserved for future use, currently illegal, 0 bits
//
// Instruments:
//
//   code         details
//   0000 0000    do not respond, request is for peer only
//
//   0001 0000    protocol version number
//   0001 0001    uptime in milliseconds
//   0001 0010    clock speed in hertz
//   0001 0100    fuse bits, 00EEHHLL
//   0001 1000    avr-libc version
//   0001 1001    avr-libc date
//
//   0010 0000    messages received
//   0010 0001    messages sent
//   0010 0010    sync errors
//   0010 0011    unexpected message type errors
//   0010 0100    truncated message errors
//   0010 0101    unknown instrument errors
//   0010 0110    response read timeout errors, ganglia only
//   0010 0111    latent read errors
//   0010 1000    hair values, neuron only, lower 10 bits
//   0010 1001    triac power voltage
//   0010 1010    chip power voltage
//   0010 1011    temperature (K)
//
//   0011 BBBB    messaging errors by type (B)
//   0100 BBBB    messages received by type (B)
//   0101 BBBB    messages sent by type (B)
//   0110 BBBB    current draw per hair (B)
//
// Notes:
//
// Since chips spend a large fraction of time operating without interrupts, the
// Arduino timers (micros and millis) are roughly useless.

// include <avr/boot.h>
// include <avr/version.h>
// include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>

// 
#if defined ATTINYX61
#define __AVR_ATtinyX61__
#endif

#if defined __AVR_ATmega328P__
#define roleGanglia
#elif defined __AVR_ATtiny24__
#define roleNeuron
#elif defined __AVR_ATtinyX61__
#define roleNeuron
#else
#error "Could not determine role."
#endif

#define protocolVersion 1

#define typeBits 4
#define neuronDataBits 10
#define neuronInstrumentBits 8
#define instrumentBits 32
#define gangliaDataBits 100
#define gangliaInstrumentBits 12
#define maxPins 24
#define messageTypes (1 << typeBits)

#define longestMessage 100
#define readTimeout 8
#define messageTimeout 256

#define numGanglia 4

#define peerResponse 0x00
#define gangliaNeuronData 0x01
#define gangliaNeuronInstrument 0x02
#define neuronGangliaInstrument 0x03
#define brainGangliaData 0x04
#define brainGangliaInstrument 0x05
#define gangliaBrainInstrument 0x06

// lowMemory is intended to squeeze into 128 bytes of RAM.
#if defined __AVR_ATmega328P__
#define lowMemory false
#define debug true
#define pinChangeVector PCINT1_vect
#elif defined __AVR_ATtiny24__
#define lowMemory true
#define debug false
#define pinChangeVector PCINT1_vect
#elif defined __AVR_ATtinyX61__
#define lowMemory false
#define debug true
#define pinChangeVector PCINT_vect
#endif

#if lowMemory
#define stat byte
#else
#define stat unsigned long
byte messageLengths[16] = {
  0, 10,  8, 32, 100, 12, 32,  0,
  0,  0,  0,  0,   0,  0,  0,  0,
};
#endif

#if defined roleBrain
#define bufferSize 256
#define axonXOffset 0
#define axonYOffset 128
#define axonReadOffset 0
#elif defined roleGanglia
#define bufferSize 36
#define axonXOffset 0
#define axonYOffset 18
#define axonReadOffset 0
#define dendriteOffset 0
#elif defined roleNeuron
#define bufferSize 36
#define dendriteOffset 0
#endif

#if !lowMemory
// TODO: initialize these dynamically instead.
stat messagesReceived[16] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};

stat messagesSent[16] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};

stat messageErrors[16] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
};
#endif

stat syncErrors = 0;
stat unexpectedTypeErrors = 0;
stat truncationErrors = 0;
stat unknownInstrumentErrors = 0;
stat responseTimeouts = 0;
stat latentReadErrors = 0;

#define x4(s) s; s; s; s

#define NoOp() __asm__ __volatile__("nop")

#define NoOp3() NoOp(); NoOp(); NoOp()
#define NoOp4() x4(NoOp())
#define NoOp6() NoOp3(); NoOp3()
#define NoOp8() NoOp4(); NoOp4()
#define NoOp16() x4(NoOp4())
#define NoOp256() for (byte _r = 0; _r < 16; _r++) NoOp16()
#define NoOp1024() x4(NoOp256())

#define bankOutRegister(bank) PORT ## bank
#define bankInRegister(bank) PIN ## bank
#define bankDirectionRegister(bank) DDR ## bank

#define outRegisterPrescan(x) bankOutRegister(x)
#define inRegisterPrescan(x) bankInRegister(x)
#define directionRegisterPrescan(x) bankDirectionRegister(x)

#define portConstant(side, pin) side ## pin ## Port

#define outRegister(side, pin) outRegisterPrescan(portConstant(side, pin))
#define inRegister(side, pin) inRegisterPrescan(portConstant(side, pin))
#define directionRegister(side, pin) \
  directionRegisterPrescan(portConstant(side, pin))

#define portBit(side, pin) side ## pin ## Bit

#define portMask(side, pin) bit(portBit(side, pin))
#define portXMask(side, pin) \
  ((outRegister(side, pin) == PORTX) ? portMask(side, pin) : 0)
#define portYMask(side, pin) \
  ((outRegister(side, pin) == PORTY) ? portMask(side, pin) : 0)

#define AXM(pin) portXMask(axon, pin)
#define AYM(pin) portYMask(axon, pin)
#define HXM(pin) portXMask(hair, pin)
#define HYM(pin) portYMask(hair, pin)

#if defined __AVR_ATtiny24__
#define PORTX PORTA
#define PORTY PORTB
#define PINX PINA
#define PINY PINB
#elif defined __AVR_ATtinyX61__
#define PORTX PORTA
#define PORTY PORTB
#define PINX PINA
#define PINY PINB
// TODO: this is only needed for sendResponse, and not sendMessage.  In
// particular, it might be only crucial for writeDendriteMessageType.  Perhaps
// split into multiple functions.
#define tickDelay NoOp3
#elif defined __AVR_ATmega328P__
#define PORTX PORTB
#define PORTY PORTD
#define PINX PINB
#define PINY PIND
#define tickDelay NoOp8

#endif

#if defined roleBrain  // ========= BRAIN =========

#define numAxonPins 4
#define hasDendrite false
#define hasAxons true

// No dendrite pins

// Axon control pin
#define axonControlPort D
#define axonControlBit 2

// Axon data pins
#define axon0Port D
#define axon0Bit 3
#define axon1Port D
#define axon1Bit 4
#define axon2Port D
#define axon2Bit 5
#define axon3Port D
#define axon3Bit 6

volatile byte* axonRegisters[4] = {
  &outRegister(axon, 0),
  &outRegister(axon, 1),
  &outRegister(axon, 2),
  &outRegister(axon, 3),
};

byte axonMasks[4] = {
  portMask(axon, 0),
  portMask(axon, 1),
  portMask(axon, 2),
  portMask(axon, 3),
};

#elif defined roleGanglia  // ========= GANGLIA =========

#define numAxonPins 4
#define hasDendrite true
#define hasAxons true

#if defined __AVR_ATtiny24__

// Dendrite pins
#define dendriteControlPort A
#define dendriteControlBit 3
#define dendriteDataPort A
#define dendriteDataBit 4

// Axon control pin
#define axonControlPort B
#define axonControlBit 0

// Axon data pins
#define axon0Port B
#define axon0Bit 1
#define axon1Port A
#define axon1Bit 1
#define axon2Port A
#define axon2Bit 2
#define axon3Port B
#define axon3Bit 4

#elif defined __AVR_ATmega328P__

// Dendrite pins
#define dendriteControlPort C
#define dendriteControlBit 3
#define dendriteDataPort C
#define dendriteDataBit 4

// Axon control pin
#define axonControlPort D
#define axonControlBit 2

// Axon data pins
#define axon0Port D
#define axon0Bit 3
#define axon1Port D
#define axon1Bit 4
#define axon2Port D
#define axon2Bit 5
#define axon3Port D
#define axon3Bit 6

#endif  // chip

volatile byte* axonRegisters[4] = {
  &outRegister(axon, 0),
  &outRegister(axon, 1),
  &outRegister(axon, 2),
  &outRegister(axon, 3),
};

byte axonMasks[4] = {
  portMask(axon, 0),
  portMask(axon, 1),
  portMask(axon, 2),
  portMask(axon, 3),
};

#elif defined roleNeuron  // ========= NEURON =========

#define hasDendrite true
#define hasAxons false

#if defined __AVR_ATtiny24__

// Dendrite pins
#define dendriteControlPort B
#define dendriteControlBit 0
#define dendriteDataPort B
#define dendriteDataBit 1

// No axon pins

// Hairs
#define hair0Port A
#define hair0Bit 1
#define hair1Port A
#define hair1Bit 2
#define hair2Port A
#define hair2Bit 3
#define hair3Port B
#define hair3Bit 4

#elif defined __AVR_ATtinyX61__ 

// Dendrite pins
#define dendriteControlPort B
#define dendriteControlBit 4
#define dendriteDataPort B
#define dendriteDataBit 5

// No axon pins

// Hairs
#define hair0Port A
#define hair0Bit 0
#define hair1Port A
#define hair1Bit 1
#define hair2Port A
#define hair2Bit 3
#define hair3Port A
#define hair3Bit 4

#elif defined __AVR_ATmega328P__

// Dendrite pins
#define dendriteControlPort D
#define dendriteControlBit 2
#define dendriteDataPort D
#define dendriteDataBit 3

// No axon pins

// Hairs
#define hair0Port D
#define hair0Bit 4
#define hair1Port D
#define hair1Bit 5
#define hair2Port D
#define hair2Bit 6
#define hair3Port D
#define hair3Bit 7

#endif  // chip type

// Other hairs faked for now
#define hair4Port hair0Port
#define hair4Bit hair0Bit
#define hair5Port hair0Port
#define hair5Bit hair0Bit
#define hair6Port hair0Port
#define hair6Bit hair0Bit
#define hair7Port hair0Port
#define hair7Bit hair0Bit
#define hair8Port hair0Port
#define hair8Bit hair0Bit
#define hair9Port hair0Port
#define hair9Bit hair0Bit

volatile byte* hairRegisters[neuronDataBits] = {
  &outRegister(hair, 0),
  &outRegister(hair, 1),
  &outRegister(hair, 2),
  &outRegister(hair, 3),
  &outRegister(hair, 4),
  &outRegister(hair, 5),
  &outRegister(hair, 6),
  &outRegister(hair, 7),
  &outRegister(hair, 8),
  &outRegister(hair, 9),
};

byte hairMasks[neuronDataBits] = {
  portMask(hair, 0),
  portMask(hair, 1),
  portMask(hair, 2),
  portMask(hair, 3),
  portMask(hair, 4),
  portMask(hair, 5),
  portMask(hair, 6),
  portMask(hair, 7),
  portMask(hair, 8),
  portMask(hair, 9),
};

#define hairXMask (HXM(0) | HXM(1) | HXM(2) | HXM(3) | HXM(4) | \
                   HXM(5) | HXM(6) | HXM(7) | HXM(8) | HXM(9))
#define hairYMask (HYM(0) | HYM(1) | HYM(2) | HYM(3) | HYM(4) | \
                   HYM(5) | HYM(6) | HYM(7) | HYM(8) | HYM(9))

word hairValues = 0;

#endif  // role

byte dataBuffer[bufferSize];

#if hasAxons
#define axonWriteBufferSize (axonYOffset - axonXOffset)
#define axonReadBufferSize (bufferSize - axonReadOffset)
#define axonXBuffer (dataBuffer + axonXOffset)
#define axonYBuffer (dataBuffer + axonYOffset)
#define axonReadBuffer (dataBuffer + axonReadOffset)
#endif

#if hasDendrite
#define dendriteBufferSize (bufferSize - dendriteOffset)
#define dendriteBuffer (dataBuffer + dendriteOffset)
#endif

#if hasAxons
// Axon combined pins
#define axonXMask (AXM(0) | AXM(1) | AXM(2) | AXM(3))
#define axonYMask (AYM(0) | AYM(1) | AYM(2) | AYM(3))
#endif

#define setOutput(side, pin) \
  directionRegister(side, pin) |= portMask(side, pin)
#define setInput(side, pin) \
  directionRegister(side, pin) &= ~portMask(side, pin)
  
#define setPin(side, pin) bitSet(outRegister(side, pin), portBit(side, pin))
#define clearPin(side, pin) bitClear(outRegister(side, pin), portBit(side, pin))
#define snapPin(side, pin) bitRead(inRegister(side, pin), portBit(side, pin))
#define snapPort(side, pin) inRegister(side, pin)

#define assignPin(side, pin, value) \
  if (value) \
    setPin(side, pin); \
  else \
    clearPin(side, pin) \

#define clearControl(s) clearPin(s, Control)
#define setControl(s) setPin(s, Control)
#define snapControl(s) snapPin(s, Control)

#define checkControlInline(s, g) if (snapControl(s)) goto g
#define checkControlInline4(s, g) x4(checkControlInline(s, g))
#define checkControlInline16(s, g) x4(checkControlInline4(s, g))

#if lowMemory
#define checkControlInline64(s, g) \
  for (byte _b = 0; _b < 4; _b++) { \
    checkControlInline16(s, g); \
  }
#define checkControlInline256(s, g) \
  for (byte _c = 0; _c < 16; _c++) { \
    checkControlInline16(s, g); \
  }
#else
#define checkControlInline64(s, g) x4(checkControlInline16(s, g))
#define checkControlInline256(s, g) x4(checkControlInline64(s, g))
#endif

#define checkControlInline8192(s, g) \
  for (byte _d = 0; _d < 128; _d++) { \
    checkControlInline64(s, g); \
  }

#define expectReadableInline(s, g) checkControlInline16(s, g)
#define expectResponseInline(s, g) checkControlInline8192(s, g)

#define awaitReadable(s) while (!snapControl(s)) {}
#define awaitUnreadable(s) while (snapControl(s)) {}

#define tick(s) \
  setControl(s); \
  tickDelay(); \
  clearControl(s); \
  tickDelay() \

#define setupOutput(side, pin) \
  clearPin(side, pin); \
  setOutput(side, pin) \

#define setupInput(side, pin) \
  clearPin(side, pin); \
  setInput(side, pin) \

void handleNeuronData(byte* buffer);
void handleNeuronInstrument(byte* buffer);
void handleGangliaData(byte* buffer);
void handleGangliaInstrument(byte* buffer);

volatile byte* inputRegister(volatile byte* output) {
  if (output == &PORTX) {
    return &PINX;
  } else {
    return &PINY;
  }
}

ISR(pinChangeVector) {
  sleep_disable();
}

void blinkLed() {
#if defined __AVR_ATtiny24__
  PORTA |= 1;
  DDRA |= 1;
  NoOp4();
  PORTA &= ~1;
  DDRA &= ~1;
#elif defined __AVR_ATmega328P__
  PORTC |= 1;
  DDRC |= 1;
  NoOp8();
  PORTC &= ~1;
  DDRC &= ~1;
#endif
}

byte messageLength(byte type) {
#if lowMemory
  switch (type) {
    case gangliaNeuronData:
      return neuronDataBits;
    case gangliaNeuronInstrument:
      return neuronInstrumentBits;
    case neuronGangliaInstrument:
    case gangliaBrainInstrument:
      return instrumentBits;
    case brainGangliaData:
      return gangliaDataBits;
    case brainGangliaInstrument:
      return gangliaInstrumentBits;
    default:
      return 0;
  }
#else
  return messageLengths[type];
#endif
}

void incrementMessagesReceived(byte type) {
#if !lowMemory
  messagesReceived[type]++;
#endif
}

unsigned long getMessagesReceived(byte type) {
#if lowMemory
  return 0;
#else
  return messagesReceived[type];
#endif
}

unsigned long getMessagesSent(byte type) {
#if lowMemory
  return 0;
#else
  return messagesSent[type];
#endif
}

unsigned long getMessageErrors(byte type) {
#if lowMemory
  return 0;
#else
  return messageErrors[type];
#endif
}

unsigned long totalMessages(stat* array) {
  unsigned long output = 0;
#if !lowMemory
  for (byte i = 0; i < messageTypes; i++) {
    output += array[i];
  }
#endif
  return output;
}

unsigned long fuseBits() {
  unsigned long output = 0;
#if defined __AVR_ATmega328P__
  output = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
  output <<= 8;
  output |= boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
  output <<= 8;
  output |= boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
#endif
  return output;
}

byte readType(byte* buffer) {
  byte output = 0;
  for (byte i = 0; i < typeBits; i++) {
    output = (output << 1) | (buffer[i] ? 1 : 0);
  }
  return output;
}

unsigned long readLong(byte* buffer) {
  unsigned long output = 0;
  for (byte i = 0; i < instrumentBits; i++) {
    output = (output << 1) | (buffer[i] ? 1 : 0);
  }
  return output;
}

#if hasDendrite

byte readMessage() {
  byte i = 0;
  noInterrupts();

 begin:
  expectReadableInline(dendrite, begin);

 sleep:
#if defined __AVR_ATmega328P__
  awaitReadable(dendrite);
#else
  sleep_enable();
  // TODO(cody): fix hard coding.
  GIMSK |= (1 << PCIE1);
  PCMSK1 |= 1;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  blinkLed();
  interrupts();
  sleep_cpu();
  noInterrupts();
  blinkLed();
  GIMSK &= ~(1 << PCIE1);
  PCMSK1 &= ~1;
  sleep_disable();
  if (!snapControl(dendrite))
    goto sleep;
#endif

 next:
  dendriteBuffer[i++] = snapPort(dendrite, Data);
  awaitUnreadable(dendrite);
  expectReadableInline(dendrite, next);

  interrupts();
  for (byte j = 0; j < i; j++) {
    dendriteBuffer[j] &= portMask(dendrite, Data);
  }
  blinkLed();
  return i;
}

byte handleMessage() {  
  byte length = readMessage();
  if (length < typeBits) {
    // TODO: maybe get rid of this conditional for efficiency.
    syncErrors++;
    return length;
  }

  byte type = readType(dendriteBuffer);
  incrementMessagesReceived(type);
  byte* message = dendriteBuffer + typeBits;
  if (length - typeBits != messageLength(type)) {
    syncErrors++;
    return length;
  }

#if defined roleNeuron  
  if (type == gangliaNeuronData) {
    handleNeuronData(message);
  } else if (type == gangliaNeuronInstrument) {
    handleNeuronInstrument(message);
  } else if (type != peerResponse) {
    unexpectedTypeErrors += 1;
  }
#endif

#if defined roleGanglia
  if (type == brainGangliaData) {
    handleGangliaData(message);
  } else if (type == brainGangliaInstrument) {
    handleGangliaInstrument(message);
  } else if (type != peerResponse) {
    unexpectedTypeErrors += 1;
  }
#endif

  return length;
}

bool writeDendriteMessageType(byte type) {
  assignPin(dendrite, Data, type & 0x08);
  tick(dendrite);
  assignPin(dendrite, Data, type & 0x04);
  tick(dendrite);
  assignPin(dendrite, Data, type & 0x02);
  tick(dendrite);
  assignPin(dendrite, Data, type & 0x01);
  tick(dendrite);
  return true;
}

void sendInstrumentResponse(unsigned long response) {
  noInterrupts();
  setupOutput(dendrite, Control);
  setupOutput(dendrite, Data);
  writeDendriteMessageType(neuronGangliaInstrument);
  for (byte i = 0; i < instrumentBits; i++) {
    assignPin(dendrite, Data, response & 0x80000000UL);
    response <<= 1;
    tick(dendrite);
  }
  setupInput(dendrite, Data);
  setupInput(dendrite, Control);
  interrupts();
}

void handleLocalInstrument(byte instrument) {
  unsigned long response = 0;
  switch (instrument) {
    case 0x00:
      return;
    case 0x10:
      response = protocolVersion;
      break;
    case 0x11:
      response = millis();
      break;
    case 0x12:
      response = F_CPU;
      break;
    case 0x14:
      response = fuseBits();
      break;
    case 0x80:
      response = __AVR_LIBC_VERSION__;
      break;
    case 0x81:
      response = __AVR_LIBC_DATE_;
      break;
    case 0x20:
#if lowMemory
      response = 0;
#else
      response = totalMessages(messagesReceived);
#endif
      break;
    case 0x21:
#if lowMemory
      response = 0;
#else
      response = totalMessages(messagesSent);
#endif
      break;
    case 0x22:
      response = syncErrors;
      break;
    case 0x23:
      response = unexpectedTypeErrors;
      break;
    case 0x24:
      response = truncationErrors;
      break;
    case 0x25:
      response = unknownInstrumentErrors;
      break;
    case 0x26:
      response = responseTimeouts;
      break;
    case 0x27:
      response = latentReadErrors;
      break;
#if defined roleNeuron
    case 0x28:
      response = hairValues;
      break;
#endif
    default:
      switch (instrument & 0xf0) {
        case 0x30:
          response = getMessageErrors(instrument & 0x0f);
          break;
        case 0x40:
          response = getMessagesReceived(instrument & 0x0f);
          break;
        case 0x50:
          response = getMessagesSent(instrument & 0x0f);
          break;
        default:
          unknownInstrumentErrors += 1;
      }
  }
  sendInstrumentResponse(response);
}

#endif  // hasDendrite

#if defined roleNeuron

#define handleBit(n) \
  if (buffer[n]) { \
    hairValues |= bit(n); \
    xMask |= portXMask(hair, n); \
    yMask |= portYMask(hair, n); \
  }

void handleNeuronData(byte* buffer) {
  hairValues = 0;
  byte xMask = 0;
  byte yMask = 0;
  handleBit(0);
  handleBit(1);
  handleBit(2);
  handleBit(3);
  // handleBit(4);
  // handleBit(5);
  // handleBit(6);
  // handleBit(7);
  // handleBit(8);
  // handleBit(9);
  PORTX = (PORTX & ~hairXMask) | xMask;
  PORTY = (PORTY & ~hairYMask) | yMask;
}

void handleNeuronDataOld(byte* buffer) {
  hairValues = 0;
  for (byte i = 0; i < neuronDataBits; i++) {
    byte hair = neuronDataBits - i - 1;
    if (buffer[i]) {
      bitSet(hairValues, hair);
      *hairRegisters[hair] |= hairMasks[hair];
    } else {
      bitClear(hairValues, hair);
      *hairRegisters[hair] &= ~hairMasks[hair];
    }
  }
}

void handleNeuronInstrument(byte* buffer) {
  byte instrument = 0;
  for (byte i = 0; i < neuronInstrumentBits; i++) {
    instrument = (instrument << 1) | (buffer[i] ? 1 : 0);
  }
  handleLocalInstrument(instrument);
}

#elif defined roleGanglia

void handleGangliaData(byte* buffer) {
  // XXX TODO XXX
}

void handleGangliaInstrument(byte* buffer) {
  // XXX TODO XXX
}

#endif

#if hasDendrite

void setupDendrite() {
  setupInput(dendrite, Control);
  setupInput(dendrite, Data);
}

// As a slave who just started, wait long enough to ensure we don't start
// reading in the middle of a message.
void readSync() {
  bool done = false;
  while (!done) {
    done = true;
    for (int i = 0; i < readTimeout; i++) {
      if (snapControl(dendrite)) {
        done = false;
        break;
      }
    }
  }
}

#endif  // hasDendrite

#if hasAxons

// As a master who just started, wait long enough to ensure we don't collide
// with a response from a slave from an earlier request.
void writeSync() {
  bool done = false;
  while (!done) {
    done = true;
    for (int i = 0; i < messageTimeout; i++) {
      if (snapControl(axon)) {
        done = false;
        break;
      }
    }
  }
}

inline void setAxons() {
  PORTX |= axonXMask;
  PORTY |= axonYMask;
}

inline void clearAxons() {
  PORTX &= ~axonXMask;
  PORTY &= ~axonYMask;
}

void axonOutput() {
  setupOutput(axon, Control);
  setupOutput(axon, 0);
  setupOutput(axon, 1);
  setupOutput(axon, 2);
  setupOutput(axon, 3);
}

void axonInput() {
  setupInput(axon, Control);
  setupInput(axon, 0);
  setupInput(axon, 1);
  setupInput(axon, 2);
  setupInput(axon, 3);
}

void prepareOffset(byte offset, byte value, byte xmask, byte ymask) {
  if (value) {
    axonXBuffer[offset] |= xmask;
    axonYBuffer[offset] |= ymask;
  } else {
    axonXBuffer[offset] &= ~xmask;
    axonYBuffer[offset] &= ~ymask;
  }
}

void prepareBuffers(byte length, byte* defaultX, byte* defaultY) {
  byte controlX = portXMask(axon, Control);
  byte controlY = portYMask(axon, Control);
  *defaultX = PORTX & ~axonXMask & ~controlX;
  *defaultY = PORTY & ~axonYMask & ~controlY;
  for (byte i = 0; i < length; i++) {
    axonXBuffer[i] = *defaultX | controlX;
    axonYBuffer[i] = *defaultY | controlY;
  }
}

void prepareType(byte type) {
  for (byte i = 0; i < typeBits; i++) {
    bool value = type & bit(typeBits - i - 1);
    prepareOffset(i, value, axonXMask, axonYMask);
  }
}

void prepareNeuronInstrument(byte axon, byte instrument) {
  byte xMask = 0;
  byte yMask = 0;
  if (axonRegisters[axon] == &PORTX) {
    xMask = axonMasks[axon];
  } else if (axonRegisters[axon] == &PORTY) {
    yMask = axonMasks[axon];
  }
  byte n = neuronInstrumentBits;
  for (byte i = 0; i < n; i++) {
    bool value = instrument & bit(n - i - 1);
    prepareOffset(typeBits + i, value, xMask, yMask);
  }
}

void prepareBroadcast(byte* data, byte length) {
  for (byte i = 0; i < length; i++) {
    prepareOffset(typeBits + i, data[i], axonXMask, axonYMask);
  }
}

void prepareUnicast(byte axon, byte* data, byte length) {
  byte xMask = 0;
  byte yMask = 0;
  if (axonRegisters[axon] == &PORTX) {
    xMask = axonMasks[axon];
  } else if (axonRegisters[axon] == &PORTY) {
    yMask = axonMasks[axon];
  }
  for (byte i = 0; i < length; i++) {
    prepareOffset(typeBits + i, data[i], xMask, yMask);
  }
}

// Currently optimized for ATtiny, requires roughly 2.3 us per bit.
// Instructions have been placed carefully to avoid requiring any sleeps.
// ATMega might require a different approach.
void sendMessage(byte length, byte defaultX, byte defaultY) {
  noInterrupts();
  axonOutput();
  byte i = 0;
  register byte xval = axonXBuffer[i];
  register byte yval = axonYBuffer[i];
  PORTX = xval;
  PORTY = yval;
  x4(x4(tickDelay()));
  x4(tickDelay());
  x4(tickDelay());
  i++;
  xval = axonXBuffer[i];
  PORTX = defaultX;
  PORTY = defaultY;
  tickDelay();
  yval = axonYBuffer[i];
 begin:
  PORTX = xval;
  PORTY = yval;
  tickDelay();
  i++;
  xval = axonXBuffer[i];
  PORTX = defaultX;
  PORTY = defaultY;
  tickDelay();
  yval = axonYBuffer[i];
  if (i != length) {
    goto begin;
  }
  axonInput();
  interrupts();
}

byte readResponse(byte axon) {
  byte i = 0;
  volatile byte* reg = inputRegister(axonRegisters[axon]);
  noInterrupts();
  expectResponseInline(axon, read);
  goto done;
 read:
  axonReadBuffer[i++] = *reg;
  awaitUnreadable(axon);
  if (i <= axonReadBufferSize) {
    expectReadableInline(axon, read);
  }
 done:
  interrupts();
  for (byte j = 0; j < i; j++) {
    axonReadBuffer[j] &= axonMasks[axon];
  }
  return i;
}

void writeDataMessages(word value) {
  byte type = gangliaNeuronData;
  byte length = messageLength(type);
  byte values[10];
  byte n = sizeof(values);
  for (byte i = 0; i < n; i++) {
    values[i] = (value & bit(n - i - 1) ? 1 : 0);
  }
  byte defaultX, defaultY;
  prepareBuffers(typeBits + length, &defaultX, &defaultY);
  prepareType(type);
  prepareBroadcast(values, length);
  sendMessage(typeBits + length, defaultX, defaultY);
}

unsigned long retrieveInstrument(byte axon, byte instrument) {
  byte type = gangliaNeuronInstrument;
  byte length = messageLength(type);
  byte defaultX, defaultY;
  prepareBuffers(typeBits + length, &defaultX, &defaultY);
  prepareType(type);
  prepareNeuronInstrument(axon, instrument);
  sendMessage(typeBits + length, defaultX, defaultY);
  length = readResponse(axon);
  if (length != instrumentBits + typeBits ||
      readType(axonReadBuffer) != neuronGangliaInstrument) {
    syncErrors++;
    return 0;
  } else {
    return readLong(typeBits + axonReadBuffer); 
  }
}

#endif  // hasAxons

#if defined roleGanglia

void gangliaTestLoop() {
  writeSync();
  while (true) {
    retrieveInstrument(0, 0x28);
    writeDataMessages(0x155);
    NoOp256();
  }
}

#if debug

void gangliaDebugLoop() {
  writeSync();
  while (true) {
    word hairs = 0;
    unsigned long errors = 0;
    for (long i = 0; i < 1000L; i++) {
      writeDataMessages(hairs);
      x4(NoOp1024());
      unsigned long value = retrieveInstrument(0, 0x28);
      if (value != hairs) {
        // Serial.print("Bogus value: ");
        // Serial.print(value, BIN);
        // Serial.print(" (expected ");
        // Serial.print(hairs & 0x3ff, BIN);
        // Serial.println(")");
        errors++;
      }
      hairs = (hairs + 521) & 0x3ff;
      delay(1);
    }
    Serial.print("Errors: ");
    Serial.println(errors);
    Serial.flush();
  }
}

#endif  // debug

#endif // defined roleGanglia

#if defined roleNeuron

void neuronLoop() {
  setupDendrite();
  readSync();
  while (true) {
    handleMessage();
  }
}

#if debug

void neuronDebugLoop() {
  unsigned long wrongValue = 0;
  setupDendrite();
  readSync();
  Serial.println("Read synced");
  Serial.flush();
  word lastHair = 0;
  unsigned long inferredResponseErrors = 0;
  while (true) {    
    for (long i = 0; i < 10000L; i++) {
      handleMessage();
      if (hairValues != 0x02aa) {
        wrongValue += 1;
      }
      if (hairValues < lastHair) {
        inferredResponseErrors += hairValues;
        inferredResponseErrors += (256 - lastHair);
      } else {
        inferredResponseErrors += (hairValues - lastHair);
      }
      lastHair = hairValues;
    }
    Serial.println("=== Finished 100k reads === ");
    // Serial.print("Wrong values: ");
    // Serial.println(wrongValue);
    Serial.print("Last value: ");
    Serial.println(hairValues, DEC);
    Serial.print("Sync errors: ");
    Serial.println(syncErrors);
    Serial.print("Inferred response errors: ");
    Serial.println(inferredResponseErrors);
    Serial.print("Unknown instrument errors: ");
    Serial.println(unknownInstrumentErrors);
    Serial.println("Received message types:");
    for (int i = 0; i < 16; i++) {
      if (!messagesReceived[i])
        continue;
      Serial.print("  ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(messagesReceived[i]);
      messagesReceived[i] = 0;
    }
    Serial.println("");
    Serial.flush();
    wrongValue = 0;
    syncErrors = 0;
  }
}

#endif // debug

#endif // defined roleNeuron

void loop() {
#if defined roleNeuron
#if debug
  neuronDebugLoop();
#else
  neuronLoop();
#endif
#elif defined roleGanglia
#if debug
  gangliaDebugLoop();
#else
  gangliaTestLoop();
#endif
#endif
}

void setup() {
  memset(dataBuffer, 0, bufferSize);
#if debug
  Serial.begin(9600);
#else
  power_all_disable();
#endif
}

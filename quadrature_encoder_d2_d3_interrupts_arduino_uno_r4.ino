// encoder input pins
const int axleEncoderAPin = 2; // P104, interrupt
const int axleEncoderBPin = 3; // P105, interrupt
const int axleEncoderZPin = 7; // P112

byte oldPins {0}; // only used inside ISR
volatile int32_t position{0}; // written inside isr, used outside
#define ticksPerRotation 4096
// Encoder logic from https://github.com/PaulStoffregen/Encoder
// Port reading from https://forum.arduino.cc/t/avr-compatibility-port-and-pin-registers/1150693/9
// Checked physical pinout first to combine pins in reading register
// and writing register if possible. This is different even between R4 Wifi and Minima
// https://docs.arduino.cc/resources/pinouts/ABX00087-full-pinout.pdf
// D2 P104, D3 P105, D7 P112 all on the same register
// on the Uno R4 Wifi:
// port=0x40040026
// digitalPinToBitMask(axleEncoderAPin)==10000
// digitalPinToBitMask(axleEncoderBPin)==100000
// digitalPinToBitMask(axleEncoderZPin)==1000000000000
void quadratureEncoderIsr() {
  const int8_t dirlut[] = {
        //                           _______         _______       
        //               Pin1 ______|       |_______|       |______ Pin1
        // negative <---         _______         _______         __      --> positive
        //               Pin2 __|       |_______|       |_______|   Pin2
        //	new		new		old		old
        //	pin2	pin1	pin2	pin1	Result
        //	----	----	----	----	------
    0,	//	0		0		0		0		no movement
    1,	//	0		0		0		1		+1
    -1,	//	0		0		1		0		-1
    2,	//	0		0		1		1		+2  (assume pin1 edges only)
    -1,	//	0		1		0		0		-1
    0,	//	0		1		0		1		no movement
    -2,	//	0		1		1		0		-2  (assume pin1 edges only)
    1,	//	0		1		1		1		+1
    1,	//	1		0		0		0		+1
    -2,	//	1		0		0		1		-2  (assume pin1 edges only)
    0,	//	1		0		1		0		no movement
    -1,	//	1		0		1		1		-1
    2,	//	1		1		0		0		+2  (assume pin1 edges only)
    -1,	//	1		1		0		1		-1
    1,	//	1		1		1		0		+1
    0	//	1		1		1		1		no movement
  };

  // can't remember where the volatile came from, may be needed because on separate GPIO bus. pointer-to-register is const
  volatile uint32_t* const port = (volatile uint32_t*)(portInputRegister(digitalPinToPort(axleEncoderAPin)));
  const uint32_t readpins = *port;
  // now swizzle bits to form index number for dirlut.
  byte lowerReadPins = readpins; // D2 and D3 in lower byte
  lowerReadPins >>= 2;
  lowerReadPins &= 0b00001100; // mask in D2 and D3, now these bits same as in PORTD on old Uno
  const byte temp = oldPins >> 2;
  oldPins = lowerReadPins;
  lowerReadPins |= temp;
  int8_t delta = dirlut[lowerReadPins]; // signed!

  // This encoder happens to have a Z next to A and B pulses. one narrow high pulse at a fixed angle.
  // Some flanks of A and B will be within the Z==HIGH pulse so it can be caught here in the isr.
  if(readpins & digitalPinToBitMask(axleEncoderZPin)) { // digitalReadFast-ish
    position = 0;
  }
  // else reset every full rotation. counter with 2^x pulses is handy,
  // otherwise use modulus% but that is quite expensive.
  // position &= (ticksPerRotation-1); // modulus rotation, wrap around if no zero pulse
  position += delta;
}

// Ideally the the lookup tables should be tucked away in a library.
// Making do with what is available...
// https://forum.arduino.cc/t/digitalwritefast-with-uno-r4/1145206
R_PORT0_Type *port_table[] = { R_PORT0, R_PORT1, R_PORT2, R_PORT3, R_PORT4, R_PORT5, R_PORT6, R_PORT7 };
static const uint16_t mask_table[] = { 1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7,
                                       1 << 8, 1 << 9, 1 << 10, 1 << 11, 1 << 12, 1 << 13, 1 << 14, 1 << 15 };
static inline void digitalWriteFast(pin_size_t pin, byte val) {
  uint16_t hardware_port_pin = g_pin_cfg[pin].pin;
  //uint16_t mask = 1 << (hardware_port_pin & 0xf);
  uint16_t mask = mask_table[hardware_port_pin & 0xf];
  R_PORT0_Type *port = port_table[hardware_port_pin >> 8];
  if (val != 0){
    port->POSR = mask;
  }
  else {
    port->PORR = mask;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(axleEncoderAPin, INPUT_PULLUP); // pullup important for rotary encoder
  pinMode(axleEncoderBPin, INPUT_PULLUP);
  pinMode(axleEncoderZPin, INPUT_PULLUP); // optional if only A and B are available
  attachInterrupt(digitalPinToInterrupt(axleEncoderAPin), quadratureEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(axleEncoderBPin), quadratureEncoderIsr, CHANGE);
  digitalWriteFast(LED_BUILTIN, HIGH);
  delay(100);
}

// this is the event loop, all the functions indepedently check position
bool update = false;
void loop() {
  int32_t pos{position}; // grab once, atomic read (was only possible with byte in AVR)
  // do something at a particular position.
  if(0 <= pos && pos < ticksPerRotation/2) {
    digitalWriteFast(LED_BUILTIN, HIGH);
  } else {
    digitalWriteFast(LED_BUILTIN, LOW);
  }
  // Can be made more efficient by maintaining a local variable for the port state and
  // checking that first to see if anything actually needs to be written out.
  // A very fast pattern when triggering at a certain angle is to prepare a LookUpTable
  // with all the pin states for each counter value and just index into that with the counter. 


}

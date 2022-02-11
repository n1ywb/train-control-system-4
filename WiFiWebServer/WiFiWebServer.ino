
#include <WiFi101OTA.h>
#include <OTAStorage.h>
#include <InternalStorage.h>
#include <SDStorage.h>

/*
  WiFi Web Server

  A simple web server that shows the value of the analog input pins.
  using a WiFi shield.
  M
  This example is written for a network using WPA encryption. For
  WEP or WPA, change the WiFi.begin() call accordingly.

  Circuit:
   WiFi shield attached
   Analog inputs attached to pins A0 through A5 (optional)

  created 13 July 2010
  by dlf (Metodo2 srl)
  modified 31 May 2012
  by Tom Igoe

*/

#include <SPI.h>
#include <WiFi101.h>
#include <ArduinoJson.h>
#include <ArduinoHttpServer.h>
// #include <samd21g18a.h>
#include "wiring_private.h" // pinPeripheral() function
#include "wiring_analog.h" // pinPeripheral() function

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;


static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

static __inline__ void syncAC() __attribute__((always_inline, unused));
static void syncAC() {
  while (AC->STATUSB.bit.SYNCBUSY);
}


#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

#include "types.h"

#undef min
#undef max

#include <array>

const uint32_t PARAMBUF_LEN = 32 * 2;
std::array<params_t, PARAMBUF_LEN> parambuf;


#define PIN_PWM (10)
#define PIN_DIR (5)
#define PIN_BRAKE (6)
#define PIN_TACH (11)
#define PIN_PERIOD (21)
#define PIN_AC (17)

#define DIR_FWD (0)
#define DIR_REV (1)

#define BRAKE_ON (1)
#define BRAKE_OFF (0)

#define TCC (TCC0)
#define TCC_CHAN_PWM (2)
#define TCC_DIV_FAC (5)
#define TCC_MAX ((0xFFFF >> TCC_DIV_FAC) - 1)
//#define TCC_ON (TCC_MAX + 1)
//#define TCC_ON (1)
//#define TCC_ON (1)


params_t params;


void dump_params(JsonObject& root) {
  root["Kp"] = params.Kp;
  root["Ki"] = params.Ki;
  root["Kd"] = params.Kd;
  root["Ko"] = params.Ko;
  root["throttle_slew_rate"] = params.throttle_slew_rate;
  root["mode"] = (int)params.mode;
  root["speed_cmd"] = params.speed_cmd;
  root["throttle_cmd"] = params.throttle_cmd;
  root["speed"] = params.speed;
  root["previous_speed"] = params.previous_speed;
  root["throttle"] = params.throttle;
  root["previous_throttle"] = params.previous_throttle;
  root["vac"] = params.vac;
  root["vcc"] = params.vcc;
  root["amps"] = params.amps;
  root["tc3_ovf_count"] = params.tc3_ovf_count;
  root["current_tach"] = params.current_tach;
  root["previous_tach"] = params.previous_tach;
  root["pin20"] = digitalRead(20);
}


void load_params(JsonObject& root) {
  params.Kp = root["Kp"];
  params.Ki = root["Ki"];
  params.Kd = root["Kd"];
  params.Ko = root["Ko"];
  params.throttle_slew_rate = root["throttle_slew_rate"];
  params.mode = (tcs_mode_t)((int)root["mode"]);
  params.speed_cmd = root["speed_cmd"];
  params.throttle_cmd = root["throttle_cmd"];
}


WiFiServer server(80);


void setup_tc3() {
  // Control loop ISR timer
  // read 32khz from clock generator 1
  // 2^15 / 2^8 / 4 == 32hz

  // Set TC3 clock source to 32khz xtal and enable
  Serial.println("disable tc3 irq");
  NVIC_DisableIRQ(TC3_IRQn);
  NVIC_ClearPendingIRQ(TC3_IRQn);

  Serial.println("disable tc3");
  REG_TC3_CTRLA = (uint16_t)(0);
  while (REG_TC3_STATUS & TC_STATUS_SYNCBUSY) {}

  Serial.println("disable tc3 clock");
  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_TCC2_TC3);
  while (REG_GCLK_CLKCTRL & GCLK_CLKCTRL_CLKEN) {}

  //  Serial.println("set clock gen 8 divider");
  //  REG_GCLK_GENDIV = (uint32_t)(GCLK_GENDIV_ID(8) | GCLK_GENDIV_DIV(32));

  Serial.println("enable clock gen 8, clock to 32khz xtal");
  REG_GCLK_GENCTRL = (uint32_t)(GCLK_GENCTRL_ID(8) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN);

  Serial.println("enable tc3 clock");
  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_TCC2_TC3 | GCLK_CLKCTRL_GEN(8)  | GCLK_CLKCTRL_CLKEN);

  Serial.println("reset tc3");
  REG_TC3_CTRLA = (uint16_t)(TC_CTRLA_SWRST);
  while (REG_TC3_STATUS & TC_STATUS_SYNCBUSY) {
    delay(1000);
    Serial.println(REG_TC3_CTRLA & TC_CTRLA_SWRST);
  }

  Serial.println("enable tc3");
  // 8 bit counter, / 4 prescale, enable
  REG_TC3_CTRLA = (uint16_t)(TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_ENABLE);

  // for debugging, clock manually
  pinMode(0, OUTPUT);

  Serial.println("enable tc3 interrupt");
  //  interrupt can be individually enabled by writing a one
  // to the corresponding bit in the Interrupt Enable Set register (INTENSET),
  REG_TC3_INTENSET = (uint8_t)(TC_INTENSET_OVF);

  // Configure interrupt request
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);
}


void TC3_Handler(void) {
  static uint32_t syncPending = 0;

  // Do I need to enforce atomic access to params?
  ++params.tc3_ovf_count;
  digitalWrite(13, params.tc3_ovf_count & (1 << 2));

  //  // DEBUG
//    REG_TC3_INTFLAG = (uint8_t)(TC_INTFLAG_OVF);
//    return;

  // DEBUG toggle loopback clock pin
  // is this causing the crash? maybe don't do this in ISR?
  ///  digitalWrite(0, !digitalRead(0));

  // Read tach
  if (!TC4->COUNT32.STATUS.bit.SYNCBUSY) {
    if (syncPending) {
      // this read is slow when the tc4 clock is slow; it's fast when the tc4
      // clock is disabled.
      // or sometimes it blocks when clock is stopped?
      // ok it takes 3 (or 6)? TC clock cycles (tach pulses in my case) to
      // sync; so reads are ALWAYS 3-6 pulses behind
      // and the register cannot be synchronized (so cannot be read) if the
      // clock is stopped (of course it's not changing  in that case)
      // this is potentially a concern at low RPMs; need to analyze
      // at low speeds we could measure pulse period instead of frequency
      // or we could disable the counter, switch it to a fast clock, read the
      // count, then switch back to the tach and re-enable
      // that could cause missed pulses at high RPMs; noanywayt necessarily a
      // deal breaker; need to analyze impact
      // what if we CLEAR the count then switch back?
      // options to improve low speed resolution
      // 1. use TCC PER to measure period instead of frequency; might be good
      // if we can get enough dynamic range for high speed operation; TC would
      // be clocked at CPU speed so no problems w read latency
      // 2. stop counter, switch clock, read, reset, switch clock, start,
      // continue ISR; error proportion should be constant WRT speed and
      // completely negligible

      //    Serial.println("read");
      params.previous_tach = params.current_tach;
      params.current_tach = REG_TC4_COUNT32_COUNT;
      //    Serial.println("read complete");
      params.speed = ((params.current_tach - params.previous_tach) << 10) / syncPending;
      syncPending = 0;
    }
    else {
      ++syncPending;
      // REG_TC4_READREQ = (uint16_t)(TC_READREQ_RREQ | TC_READREQ_RCONT | TC_READREQ_ADDR(TC_COUNT32_COUNT_OFFSET));
      //    Serial.println("request read");
      REG_TC4_READREQ = (uint16_t)(TC_READREQ_RREQ | TC_READREQ_ADDR(TC_COUNT32_COUNT_OFFSET));
      //    Serial.println("request read complete");
    }
  }
  else {
    ++syncPending;
    if (syncPending > 32)
      params.speed = 0;
  }

  ///  PinDescription pinDesc = g_APinDescription[pin];
  //  /static uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
  //  /static uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);

  //  control logic goes here
  if (params.mode == THROTTLE) {
    params.speed_cmd = 0;
    params.throttle = params.throttle_cmd;
  }
  else if (params.mode == SPEED) {
    // The big problem is that the speed response is non-linear WRT throttle
    // in particular the throttle must be raised to some minimum value before
    // speed responds at all.
    // The exact value is dependent on a number of variables including track
    // voltage, train length, and lubrication and maintenance schedules for
    // both track and rolling stock.
    // so we need to do something different when starting from a dead stop
    // maybe just ramp throttle at fixed rate until speed > 0
    // TODO speed scale might need to be compressed; max speed seems REALLY fast
    static float integral = 0;
    params.throttle_cmd = 0;
    if (params.speed_cmd) {
      // TODO support negative speed_cmd
      float err = params.speed_cmd - params.speed;
      float deriv = params.previous_error - err;
      params.previous_error = err;
      integral = MAX(MIN(integral + (err / 16.0), INT16_MAX), INT16_MIN);
      params.throttle = MAX(MIN(
                              (float)params.throttle
                              + (
                                (err * params.Kp)
                                + (integral * params.Ki)
                                + (deriv * params.Kd)
                              ) * params.Ko
                              , INT16_MAX), 5000);
      // FIXME reverse cruise
      //      , INT16_MAX), INT16_MIN);
    }
    else {
      params.throttle = 0;
      integral = 0;
    }
  }
  else {
    Serial.println("invalid mode");
  }

  uint16_t new_th = (((uint16_t)abs(params.throttle)) << 1) >> TCC_DIV_FAC;


  if (params.throttle >= 0 && params.previous_throttle < 0 || params.throttle < 0 && params.previous_throttle >= 0) {
    TCC0->CTRLBSET.bit.LUPD = 1;
    syncTCC(TCC0);
    
    TCC0->CCB[TCC_CHAN_PWM].reg = (uint32_t) 0;
    syncTCC(TCC0);
    
    TCC0->CTRLBCLR.bit.LUPD = 1;
    syncTCC(TCC0);
    
//    delay(1);
    digitalWrite(PIN_DIR, params.throttle >= 0 ? DIR_FWD : DIR_REV);
//    delay(1);
  }

  TCC0->CTRLBSET.bit.LUPD = 1;
  syncTCC(TCC0);

  TCC0->CCB[TCC_CHAN_PWM].reg = (uint32_t) new_th;
  syncTCC(TCC0);

  TCC0->CTRLBCLR.bit.LUPD = 1;
  syncTCC(TCC0);  

  memcpy(&(parambuf[params.tc3_ovf_count % PARAMBUF_LEN]), &params, sizeof(params_t));

  REG_TC3_INTFLAG = (uint8_t)(TC_INTFLAG_OVF);
}


void setup_tc4() {
  Serial.println("power up clocks");
  PM->APBCMASK.reg |= PM_APBCMASK_AC | PM_APBCMASK_EVSYS | PM_APBCMASK_TC4 | PM_APBCMASK_TC5;

  Serial.println("set clocks to CPU clock");
  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_EVSYS_0 | GCLK_CLKCTRL_GEN(0) | GCLK_CLKCTRL_CLKEN);
  while (GCLK->STATUS.bit.SYNCBUSY);
  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_AC_DIG | GCLK_CLKCTRL_GEN(0) | GCLK_CLKCTRL_CLKEN);
  while (GCLK->STATUS.bit.SYNCBUSY);
  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_AC_ANA | GCLK_CLKCTRL_GEN(0) | GCLK_CLKCTRL_CLKEN);
  while (GCLK->STATUS.bit.SYNCBUSY);
  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_TC4_TC5 | GCLK_CLKCTRL_GEN(0) | GCLK_CLKCTRL_CLKEN);
  while (GCLK->STATUS.bit.SYNCBUSY);

  //  must write channel/user n+1
  EVSYS->USER.reg = (uint16_t)(EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU) | EVSYS_USER_CHANNEL(1 + 0));

  EVSYS->CHANNEL.reg = (uint32_t)(EVSYS_CHANNEL_PATH_ASYNCHRONOUS | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_AC_COMP_0) | EVSYS_CHANNEL_CHANNEL(0));

  //  EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU)
  //  EVSYS_USER_CHANNEL(1 + 0)
  //#define EVSYS_ID_GEN_AC_COMP_0      68
  //#define EVSYS_ID_GEN_AC_COMP_1      69
  //#define EVSYS_ID_GEN_AC_WIN_0       70

//  0x44 AC COMP0 Comparator 0
//  0x45 AC COMP1 Comparator 1
//  0x46 AC WIN0 Window 0
  
  Serial.println("setup AC clock");

  AC->SCALER[0].bit.VALUE = 32;
  syncAC();

  AC->COMPCTRL[0].bit.FLEN = AC_COMPCTRL_FLEN_MAJ5_Val;
  syncAC();

  AC->COMPCTRL[0].bit.HYST = 1;
  syncAC();
  
  AC->COMPCTRL[0].bit.MUXPOS = AC_COMPCTRL_MUXNEG_PIN0_Val;
  syncAC();

  AC->COMPCTRL[0].bit.MUXNEG = AC_COMPCTRL_MUXNEG_VSCALE_Val;
  syncAC();

  AC->COMPCTRL[0].bit.SPEED = AC_COMPCTRL_SPEED_HIGH_Val;
  syncAC();

  AC->EVCTRL.bit.COMPEO0 = 1;
  syncAC();

  AC->COMPCTRL[0].bit.ENABLE = 1;
  syncAC();

  AC->CTRLA.bit.ENABLE = 1;
  syncAC();

//  pinMode(PIN_AC, INPUT_PULLUP); // this doesn't seem to have any effect when the pin is connected to gclk_io
//  digitalWrite(PIN_AC, 1);
//  digitalRead(PIN_AC);
  pinPeripheral(PIN_AC, PIO_ANALOG);
  
  Serial.println("setup tc4");
  // Tachometer pulse counter; 3/rev
  //  signal path:
  //  port -> gclock io -> gclock controller -> ahb-apb bridge A -> hi speed bus matrix -> ahb-apb bridge C -> TC4/5

  // gclock detail:
  // gclk io -> controller -> generator -> divider/masker -> multiplexer -> gate -> gclk_peripheral -> peripheral

  // doing period capture might work better but since samd21g doesn't have PIO events we
  // would have to route gclock to a timer who's sole job would be generating an event on every pulse.

  //  Using the TC’s I/O lines requires the I/O pins to be configured. Refer to “PORT” on page 379 for details.
  //  REG_PORTA
  // configure arduino pin 20 uc pin 31 multiplex for gclkio[6]
  // pin 20 drives clock 6
//  pinMode(PIN_TACH, INPUT_PULLUP); // this doesn't seem to have any effect when the pin is connected to gclk_io
//  digitalRead(PIN_TACH);
//  pinPeripheral(PIN_TACH, PIO_AC_CLK);

  // hacky way to pull up PIN_TACH

  pinMode(PIN_PERIOD, INPUT_PULLUP); // this doesn't seem to have any effect when the pin is connected to gclk_io
  digitalWrite(PIN_PERIOD, 1);
  digitalRead(PIN_PERIOD);

  Serial.println("disable tc4/5 irqs");
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);

//  Serial.println("set gen2 src to cpu");
//  REG_GCLK_GENCTRL = (uint32_t)(GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN);
//  while (GCLK->STATUS.bit.SYNCBUSY);


  Serial.println("disable tc4");
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (REG_TC4_STATUS & TC_STATUS_SYNCBUSY);

  Serial.println("disable tc5");
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  Serial.println("reset tc5");
  REG_TC5_CTRLA = (uint16_t)(TC_CTRLA_SWRST);
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY) {
//    delay(1000);
//    Serial.println(REG_TC5_CTRLA & TC_CTRLA_SWRST);
  }

  Serial.println("reset tc4");
  REG_TC4_CTRLA = (uint16_t)(TC_CTRLA_SWRST);
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
//    delay(1000);
//    Serial.println(REG_TC4_CTRLA & TC_CTRLA_SWRST);
  }

  Serial.println("tc4 count on event");
  TC4->COUNT32.EVCTRL.reg = TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_COUNT;
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

  // The mode (8, 16 or 32 bits) of the TC must be selected in the TC Mode bit group in the Control A register
  // (CTRLA.MODE). The default mode is 16 bits
  Serial.println("enable tc4");
  REG_TC4_CTRLA = (uint16_t)(TC_CTRLA_MODE_COUNT32 | TC_CTRLA_ENABLE);
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

  // is this slow?
  //  REG_TC4_READREQ = (uint16_t)(TC_READREQ_RCONT | TC_READREQ_ADDR(TC_COUNT32_COUNT_OFFSET));
  //  REG_TC4_READREQ = (uint16_t)(TC_READREQ_RREQ | TC_READREQ_RCONT | TC_READREQ_ADDR(TC_COUNT32_COUNT_OFFSET));
  //  REG_TC4_READREQ = (uint16_t)(TC_READREQ_RREQ | TC_READREQ_ADDR(TC_COUNT32_COUNT_OFFSET));
  //  while (STATUSTC4->COUNT16.STATUS.bit.SYNCBUSY);

//  // disable tc clock to gen
//  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_TC4_TC5);
//  while (GCLK->.bit.SYNCBUSY);
//
//  // disable gen2
//  REG_GCLK_GENCTRL = (uint32_t)(GCLK_GENCTRL_ID(2));
//  while (GCLK->STATUS.bit.SYNCBUSY);
//
//  // set gen2 src to clkio input and enable
//  REG_GCLK_GENCTRL = (uint32_t)(GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_GCLKIN | GCLK_GENCTRL_GENEN);
//  while (GCLK->STATUS.bit.SYNCBUSY);
//
//  // set tc4 clock to gen2 and enable
//  REG_GCLK_CLKCTRL = (uint16_t)(GCLK_CLKCTRL_ID_TC4_TC5 | GCLK_CLKCTRL_GEN(2) | GCLK_CLKCTRL_CLKEN);
//  while (GCLK->STATUS.bit.SYNCBUSY);
}


void setup() {
  WiFi.setPins(8, 7, 4, 2);

  //Initialize serial and wait for port to open:
  Serial.begin(9600);

#if 1
  delay(2000);
#else
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  // dump pin to TC map
#if 0
  Serial.print("pin");
  Serial.print("\t");
  Serial.print("TC");
  Serial.print("\t");
  Serial.print("Chan");
  Serial.println("");

  for (int pin = 0; pin <= 24; ++pin) {
    PinDescription pinDesc = g_APinDescription[pin];
    Serial.print(pin);
    Serial.print("\t");
    Serial.print(GetTCNumber(pinDesc.ulPWMChannel));
    Serial.print("\t");
    Serial.print(GetTCChannelNumber(pinDesc.ulPWMChannel));
    Serial.println("");
  }
#endif

  memset((void*)(&parambuf[0]), 0, PARAMBUF_LEN * sizeof(params_t));

//  Serial.println("starting OTA");
//  WiFiOTA.begin("tcs", "ilovetrains", InternalStorage);

  // SETUP PINS

  // hold brake line low (brake off)
  Serial.println("brake");
  digitalWrite(PIN_BRAKE, BRAKE_OFF);

  // set direction
  Serial.println("dir");
  digitalWrite(PIN_DIR, DIR_FWD);

  // pwm pin use TCC0.
  // setup tcc0
  Serial.println("pwm");

// don't do this; set up TCC0 and PWM_PIN manually
//  analogWrite(PIN_PWM, 0);

  // setup TCC0
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1));
  while (GCLK->STATUS.bit.SYNCBUSY == 1);

  Serial.println("enable");
  TCC0->CTRLA.bit.ENABLE = 0;
  syncTCC(TCC0);

  //  TCC0->DRVCTRL.reg |= (uint32_t) ((1 << TCC_CHAN_H1) | (1 << TCC_CHAN_H2)) << TCC_DRVCTRL_INVEN_Pos ;
  //  TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN_Msk ;
//  syncTCC(TCC0);

  Serial.println("cfg tcc0");
  TCC0->CTRLA.reg &= ~TCC_CTRLA_PRESCALER_Msk;
  syncTCC(TCC0);

//  Serial.println("cfg tcc0 2");
//  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(2);
//  syncTCC(TCC0);

  // Set TCx as normal PWM
  TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
  syncTCC(TCC0);

  Serial.println("cfg tcc0 3");
  TCC0->PER.reg = (uint32_t)(TCC_MAX); // do I have to left shift this out of the dith bits?
  syncTCC(TCC0);

  Serial.println("cfg tcc0 4");
  TCC0->CTRLA.bit.ENABLE = 1;
  syncTCC(TCC0);

  // setup PIN_PWM
  pinPeripheral(PIN_PWM, PIO_TIMER_ALT);

  setup_tc4();
  setup_tc3();

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  //  WiFi.maxLowPowerMode();

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(5000);
  }
  server.begin();
  // you're connected now, so print out the status:
  // this causes response time to jump from ~0.6s to 2-3s and doesn't save a lot of power.
  //  WiFi.lowPowerMode();
  printWiFiStatus();
}

void sendreply(Client& client) {
  const size_t bufferSize = JSON_OBJECT_SIZE(18) + 300;
  StaticJsonBuffer<bufferSize> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  dump_params(root);

  while (client.read() >= 0);
  String httpErrorReply("HTTP/1.1 200\r\n");
  httpErrorReply += "Connection: close\r\n";
  //              httpErrorReply += "Content-Length: ";
  //              httpErrorReply += data.length(); httpErrorReply += "\r\n";
  httpErrorReply += "Content-Type: application/json\r\n";
  httpErrorReply += "Access-Control-Allow-Origin: *\r\n";
  httpErrorReply += "Allow: GET,POST,OPTIONS\r\n";
  httpErrorReply += "\r\n";
  //              httpErrorReply += data;
  client.print(httpErrorReply);
  root.printTo(client);
}

void sendreply_bin(Client& client, void* data, size_t data_len) {
  Serial.print("content-length: "); Serial.println(data_len);
  while (client.read() >= 0);
  client.println("HTTP/1.1 200");
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(data_len);
  client.println("Content-Type: application/octet-stream");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Allow: GET,POST,OPTIONS");
  client.println("");
  const size_t WRITE_MAX = 1024;
  size_t remaining = data_len;
  uint8_t* p = (uint8_t*)data;
  while (remaining) {
    size_t pktsize = remaining >= WRITE_MAX ? WRITE_MAX : remaining;
    size_t bytes = client.write(p, pktsize);
    if (bytes != pktsize) {
      Serial.println("error sending");
      break;
    }
    remaining -= bytes;
    p += bytes;
    Serial.print("bytes sent: "); Serial.println(bytes);
  }
}

void loop() {
  // single threaded http server
  // single threaded is OK for http since requests are small and connections transient
  // to handle long lived connections (e.g. web sockets or chunked mode or
  // something) we will need some form of concurrency.
  // Switching WiFi PM modes rapidly seems to cause a crash after a short time
  //   WiFi.maxLowPowerMode();
  //  Serial.println("listen for incoming clients");
  //  WiFiOTA.poll();
  WiFiClient client = server.available();
  if (client) {
    //    WiFi.noLowPowerMode();
    Serial.println("new client");
    while (client.connected()) {
      Serial.println("client connected");
      if (client.available()) {
        Serial.println("client available");
        ArduinoHttpServer::StreamHttpRequest<1023> httpRequest(client);
        // Parse the request.
        if (httpRequest.readRequest())
        {
          // Use the information you like they way you like.

          // Retrieve HTTP resource / URL requested

          // Retrieve 2nd part of HTTP resource.
          // E.g.: "on" from "/api/sensors/on"
          // Serial.println( httpRequest.getResource()[2] );

          // Retrieve HTTP method.
          // E.g.: GET / PUT / HEAD / DELETE / POST
          ArduinoHttpServer::MethodEnum method( ArduinoHttpServer::MethodInvalid );
          method = httpRequest.getMethod();

          Serial.println( httpRequest.getResource().toString() );

          if ( method == ArduinoHttpServer::MethodGet )
          {
            Serial.println("GET");
            if (httpRequest.getResource()[0].equals("buffer")) {
              params_t buf[PARAMBUF_LEN];
              size_t buflen = 0;

              int t = httpRequest.getResource()[1].toInt();
              // enter critical; disable interrupts
              for (int i = 1; i <= PARAMBUF_LEN; ++i) {
                params_t* p = &parambuf[(params.tc3_ovf_count + i) % PARAMBUF_LEN];
                if (p->tc3_ovf_count > t) {
                  memcpy(&buf[buflen], p, sizeof(params_t));
                  ++buflen;
                }
              }
              // exit critical; enable interrupts
              // send buf; in binary
              int contentlen = sizeof(params_t) * buflen;
              sendreply_bin(client, buf, contentlen);
              client.flush();
            }
            else {
              sendreply(client);
            }
            //              client.stop();
            break;
          }
          else if ( method == ArduinoHttpServer::MethodPost )
          {
            Serial.println("POST");
            const size_t bufferSize = JSON_OBJECT_SIZE(18) + 300;
            StaticJsonBuffer<bufferSize> jsonBuffer;
            Serial.println(httpRequest.getBody());

            JsonObject& root2 = jsonBuffer.parseObject(httpRequest.getBody());
            if (root2.success()) {
              load_params(root2);
              sendreply(client);
            }
            else {
              Serial.println("json decoding error");
            }
            //              client.stop();
            break;
          }
          else if ( method == ArduinoHttpServer::MethodOptions ) {
            ArduinoHttpServer::StreamHttpReply httpReply(client, "application/json", "Allow: GET,POST,OPTIONS\r\nAccess-Control-Allow-Headers: content-type\r\n");
            httpReply.send("");
          }
        }
        else
        {
          // HTTP parsing failed. Client did not provide correct HTTP data or
          // client requested an unsupported feature.
          ArduinoHttpServer::StreamHttpErrorReply httpReply(client, httpRequest.getContentType());
          httpReply.send(httpRequest.getErrorDescrition());
          //           client.stop();
          break;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

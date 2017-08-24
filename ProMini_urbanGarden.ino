
// Libs required to communicate with the I2C ADC, which gets the analog values from the soil moisture ensor
#include <Wire.h>
#include <Adafruit_ADS1015.h>


//Libs for LoRaWAN and the connection of the RFM95 with SPI 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

Adafruit_ADS1015 ads;     /* Use this for the 12-bit ADC version */

/****************************************************************************
 ********************* USER SETTINGS ****************************************
 ****************************************************************************/

//PIN connected to the magnetic valve via MOSFET. It is helpful for debugging to use the internal LED pin...if possible
#define VALVE_PIN 9

// define values from ADC that represent complete dry (sensor in air) or wet (sensor in water) values. 
#define DRY_LEVEL 860
#define WET_LEVEL 430

//minimum time in seconds between two waterings
// TO DO: Valve becomes hot with 15 minute. Seems a longer cooldown is required
const unsigned long WATER_INTERVAL = 15 * 1000UL;

// we will water (i.e. open the valve) this time (in seconds) and checking again
const unsigned long WATER_DURATION = 60 * 1000UL;

// we expect at least this moisture increase percent points after a complete watering cycled. otherwise something went wrong
#define MIN_STEP_UP 15

// stops program if it fails the check for MIN_STEP_UP
#define step_required false

//provide a minimum soil moisture level in percent. If below, it will be watered
#define MOISTURE_LOW 50

//******************

/* UPLINK SPREADING FACTOR
 *  Payload of 2 Bytes, note that fixed SF of 12 is not allowed 
 *  SpreadingFactor         SF7     SF8     SF9     SF10    SF11    SF12        spreading factor; higher means more range and better reception, but also more airtime     
 *  Tpacket                 46.336  92.672  164.864 329.728 659.456 1155.072    ms total airtime to send a full packet inc. overhead     
 *  TTN Fair Access Policy  647     323     181     90      45      25          average messages/day for maximum of 30 seconds airtime on The Things Network   
 *                          27.0    13.5    7.6     3.8     1.9     1.1         average messages/hour when sending all day  
 *  
 *  see https://docs.google.com/spreadsheets/d/1QvcKsGeTTPpr9icj4XkKXq4r2zTc2j0gsHLrnplzM3I/edit#gid=0 
 *  or
 *  https://www.thethingsnetwork.org/forum/t/spreadsheet-for-lora-airtime-calculation/1190/12
 */
#define SPREADING_FACTOR DR_SF7

// LoRaWAN NwkSKey, network session key 
// devaddr LSB-first and the keys MSB-first

// This is the default Semtech key, which is used by the early prototype TTN network
// in MSB order
static const PROGMEM u1_t NWKSKEY[16] = { xxxxx };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// in MSB order.
static const u1_t PROGMEM APPSKEY[16] = { xxxxx };

// LoRaWAN end-device address (DevAddr)
// in LSB order
static const u4_t DEVADDR = 0xXXXXX ; // <-- Change this address for every node!

/*************************************************************************
**************************************************************************
**************************************************************************/

// Tracks the time since the last watering event
unsigned long LastWateringMillis=0;

// When the last report was written to serial
unsigned long LastSerialReportMillis=0;

//minimum time in seconds between two serial reports. 0 turns it off
const unsigned long SERIAL_INTERVAL = 1 * 1000UL;

// When the last report was sent to TTN
unsigned long LastLoraReportMillis=0;

//minimum time in MINUTES between two submissions over LoRa (a.k.a. TX_INTERVAL)
const unsigned long LORA_INTERVAL = 10 * 60 * 1000UL;

// Track the time since the valve is open or 0 if not open
unsigned long valveopensince=0;

// just in case there is something wrong
boolean watering_error = false;



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// byte array that holds the payload for sending to TTN
static byte mydata[2];

// if we leave it out, it seems that the RFM95 module is not clearing itself after the first submission or at least the status variable
// it reports OP_TXRXPEND error if you try to send the 2nd package.
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {4, 5, LMIC_UNUSED_PIN},
};


/* ****************************************************************************** 
 *  *************************** function headers - see below main loop **********
 *  *****************************************************************************
 */
// gardening functions
byte get_moisture();
void open_valve();
void close_valve();
boolean watering_success(byte previous_moisture, byte moisture);
void system_status(byte cur_moisture, unsigned long valveopen, unsigned long lastwater, unsigned long LastLora);

//Lora related stuff
void onEvent (ev_t ev);
void do_send(osjob_t* j);
void lmic_init();
void loraTX_status(byte tx_moisture, unsigned long tx_LastWateringMillis, unsigned long tx_LastLoraReportMillis);


/* ======================================================================
Function: setup
Purpose : initialize all the stuff we need
Input   : -
Output  : -
Comments: the setup function runs once when you press reset or power the board
====================================================================== */
// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize VALVE_PIN as an output.
  pinMode(VALVE_PIN, OUTPUT);

  // init I2C
  // Arduino pro Mini standard is 4,5 for SDA,SCL so we don't need to add it as arguments
  Wire.begin();

  // init serial console
  Serial.begin(115200);
  //while (!Serial);             // wait for serial monitor for debugging
  Serial.println();

  //init the RFM95
  lmic_init();

  // it should be default, but let's be sure the valve is closed
  digitalWrite(VALVE_PIN, LOW);   
}


/* ======================================================================
Function: main loop
Purpose : -
Input   : -
Output  : -
Comments: running over and over again until the zombie apocalypse
====================================================================== */
void loop() {

  /* *******************************************************************************
   *  1. Check if moisture is too low
   *  1.1 if moisture is too low, check if time since last watering > WATERING_INTERVAL
   *  
   *  2. Check if Valve is open and time since it is open > WATER_DURATION
   *  2.1. Check if soil moisture increased by at least MIN_STEP_UP
   *  
   *  3. Report data to serial
   *  
   *  4. Report to LoRa / TTN
   *  *******************************************************************************
   */

  // current and moisture before watering
  byte old_moisture, moisture;

  //value that inidcates the moisture from 0-dry .. 100-wet
  moisture = get_moisture();

  // ********************** 1. Check if moisture is too low **********************************************************
  if (moisture < MOISTURE_LOW) {
    
      //  ********************** 1.1 if moisture is too low, check if time since last watering > WATERING_INTERVAL

    if( (millis() - LastWateringMillis ) >= WATER_INTERVAL && (valveopensince == 0) ) {
 
      //Serial.println("Open valve");
      open_valve();
      
      // remember time when valve was opened
      valveopensince = millis();

      //save moisture value to check of success afterwards
      old_moisture = moisture;
    }
  } 

  //************* 2. Check if Valve is open and time since it is open > WATER_DURATION*******************************
  if ( (valveopensince !=0) && ( (millis() - valveopensince ) >= (WATER_DURATION) ) ){
    LastWateringMillis =  millis();
    
    //Serial.println("close valve");
    valveopensince=0;
    close_valve();  

    // **********************Check if soil moisture increased by at least MIN_STEP_UP
    if (watering_success(old_moisture, moisture) == true){
      Serial.println("Watering success");
    } else{
      Serial.println("Watering error");
      if (step_required){
        Serial.println("Stopping program");
        while (1>0){ 
         delay(1000);
       }
      }
    }
  }
  
   // ********************** 3. Report data to serial *********************************************************
   if( (millis() - LastSerialReportMillis ) >= SERIAL_INTERVAL && SERIAL_INTERVAL !=0 ) {
 
      //report data to serial
      system_status(moisture, valveopensince, LastWateringMillis, LastLoraReportMillis );
      
      // remember time when last report was written to serial
      LastSerialReportMillis = millis();

    }

   // ********************** 4. Report data to Lora / TTN *********************************************************
   if( (millis() - LastLoraReportMillis ) >= LORA_INTERVAL && LORA_INTERVAL !=0 ) {
 
      //air data to TTM
      loraTX_status(moisture, LastWateringMillis, LastLoraReportMillis, watering_error );
      
      // remember time when last report was sent
      LastLoraReportMillis = millis();
   }  
  delay(1000);                       // wait for some seconds 

  //
  //https://github.com/matthijskooijman/arduino-lmic/blob/master/src/lmic/oslmic.c#L102
  os_runloop_once();
}


/* ======================================================================
Function: get_moisture
Purpose : reading soil moisture sensor over I2C ADC and convert it to a 0...100 value
Input   : get the input from the ADC
Output  : returns a value between 0 (total dry) and 100 (pure water)
Comments: -
====================================================================== */
byte get_moisture(){

  //read value from the i2c ADC
  int16_t adc0;
  adc0 = ads.readADC_SingleEnded(0);

  // convert it to a 0...100 value
  float float_moisture = 100 - (adc0-WET_LEVEL)/((DRY_LEVEL- WET_LEVEL)/100) ;
 
  //set it back to 0..100 if outside
  if (float_moisture > 100) {
    float_moisture = 100;
  }
  if (float_moisture < 0) {
    float_moisture = 0;
  }
  
  //we only need 0...100, precise enough
  byte relative_moisture = byte(float_moisture);
  return relative_moisture;
}


/* ======================================================================
Function: open_valve
Purpose : opening the magnetic valve for a given period of time and closing it
Input   : -
Output  : none
Comments: -
====================================================================== */
void open_valve(){
  // Serial.println("Opening valve");
  digitalWrite(VALVE_PIN, HIGH);   // turn the pin on (HIGH is the voltage level to open MOSFET)
}


/* ======================================================================
Function: closing_valve
Purpose : closing the valve
Input   : none
Output  : none
Comments: -
====================================================================== */
void close_valve(){
 // Serial.println("Closing valve");
  digitalWrite(VALVE_PIN, LOW);    // turn the pin off by making the voltage LOW to close the MOSFET  
}


/* ======================================================================
Function: watering success
Purpose : simple check if watering increased moisture by predefined value
Input   : moisture before and after watering
Output  : true if moisture increased by at least MIN_STEP_UP value
Comments: -
====================================================================== */
boolean watering_success(byte prev_moisture, byte cur_moisture){
  if ( (cur_moisture-prev_moisture-MIN_STEP_UP) > 0 ){
    watering_error = false;
    return true;
  } else{
    watering_error = true;
    return false;
  }
}


/* ======================================================================
Function: systen_status
Purpose : print some formatted system status data to serial
Input   : current moisture, if/since when the valve is open, when did the last watering end
Output  : none
Comments: -
====================================================================== */
void system_status(byte cur_moisture, unsigned long valveopen, unsigned long lastwater, unsigned long LastLora){
 char buffer[235];
 unsigned long temp  = (millis()-lastwater)/1000;
 unsigned long temp2 = (millis()-valveopen)/1000;
 unsigned long tempLora = (millis()-LastLora)/1000;

 if (valveopen == 0){
   sprintf(buffer, "Current Moisture : %3d%% (Target : %3d%%) || Last Watering : %5lds (Min delay : %4lds) || Last TTN TX : %5lds (Min delay : %4lds) || Valve closed",                    cur_moisture, MOISTURE_LOW, temp, WATER_INTERVAL/1000, tempLora, LORA_INTERVAL/1000 ); 
 } else {
   sprintf(buffer, "Current Moisture : %3d%% (Target : %3d%%) || Last Watering : %5lds (Min delay : %4lds) || Last TTN TX : %5lds (Min delay : %4lds) || Valve open since %5lds (Max %4ds)", cur_moisture, MOISTURE_LOW, temp, WATER_INTERVAL/1000, tempLora, LORA_INTERVAL/1000, temp2, WATER_DURATION/1000 );
 }
 Serial.println(buffer);
}

/* ======================================================================
Function: onEvent
Purpose : event handler for LoRa events
Input   : events
Output  : 
Comments: taken from the example but changed the scheduling. do_send is now called from main loop
====================================================================== */
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //  os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

/* ======================================================================
Function: loraTX_status
Purpose : defining the payload for submission to TTN
Input   : current moisture, millis of last wattering, millis of last submission to TTN, watering_error state
Output  : -
Comments: calls the do_send with the byta array mydata. 
====================================================================== */

//TODO: Watering error

void loraTX_status(byte tx_moisture, unsigned long tx_LastWateringMillis, unsigned long tx_LastLoraReportMillis, boolean tx_watering_error){
  mydata[0] = tx_moisture;
  if ( tx_LastLoraReportMillis < tx_LastWateringMillis || valveopensince !=0){
    mydata[1] = 1;
  } else{
    mydata[1] = 0;
  }
  Serial.print("ARGS : ");
  Serial.print(tx_moisture);
  Serial.print(tx_LastWateringMillis);
  Serial.print(" ");
  Serial.println(tx_LastLoraReportMillis);
  
  Serial.print("LORA : ");
  Serial.print(mydata[0]);
  Serial.println(mydata[1]);
  do_send(&sendjob);
}

/* ======================================================================
Function: do_send
Purpose : sendting the payload to TTN
Input   : it send the data that given to LMIC_setTxData2 as 2nd argument
Output  : 
Comments: taken from the example
====================================================================== */
void do_send(osjob_t* j){
//void do_send(){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

/* ======================================================================
Function: lmic_init 
Purpose : Initialize the RFM95
Input   : 
Output  : 
Comments: Taken out from the setup part of the example
====================================================================== */
void lmic_init(){
  // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

     #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif
    

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(SPREADING_FACTOR,14);
}

  



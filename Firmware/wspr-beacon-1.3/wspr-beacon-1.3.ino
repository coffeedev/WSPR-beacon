#include <JTEncode.h>
#include <si5351.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <TinyGPS.h>

//20251222
//Band Hopping attempt for 15 & 10 meters
#define FIRMWARE_VERSION 1.3

//ERROR CODES
// TX RED ERROR
// GPS ERROR            5 TIMES
// SI535I ERROR         3 TIMES
// DATETIME SYNC ERROR 10 TIMES

//******************************************************************
//                      WSPR configuration
//******************************************************************
// WSPR protocol configuration
#define WSPR_TONE_SPACING          146
#define WSPR_DELAY                 683
#define WSPR_MESSAGE_BUFFER_SIZE   255

// WSPR center frequency in Hz
// #define WSPR_DEFAULT_FREQ       137500ULL    // 0.1375 MHz - 2200m
// #define WSPR_DEFAULT_FREQ       475700ULL    // 0.4757 MHz - 600m
// #define WSPR_DEFAULT_FREQ       1838100ULL   // 1.8381 MHz - 160m
// #define WSPR_DEFAULT_FREQ       3570100ULL   // 3.5701 MHz - 80m
// #define WSPR_DEFAULT_FREQ       5288700ULL   // 5.2887 MHz - 60m
// #define WSPR_DEFAULT_FREQ       7040100ULL   // 7.0401 MHz - 40m
// #define WSPR_DEFAULT_FREQ       10140200ULL  // 10.1402 MHz - 30m
// #define WSPR_DEFAULT_FREQ       14097100ULL  // 14.0971 MHz - 20m
// #define WSPR_DEFAULT_FREQ       18106100ULL  // 18.1061 MHz - 17m
// #define WSPR_DEFAULT_FREQ       21096100ULL  // 21.0961 MHz - 15m
// #define WSPR_DEFAULT_FREQ       24926100ULL  // 24.9261 MHz - 12m
// #define WSPR_DEFAULT_FREQ       28126100ULL  // 28.1261 MHz - 10m
// #define WSPR_DEFAULT_FREQ       50294500ULL  // 50.2945 MHz - 6m
// #define WSPR_DEFAULT_FREQ       70092500ULL  // 70.0925 MHz - 4m
// #define WSPR_DEFAULT_FREQ       144490500ULL // 144.4905 MHz - 2m

#define WSPR_DEFAULT_FREQ_15       21096100ULL  // 21.0961 MHz - 15m
#define WSPR_DEFAULT_FREQ_28       28126100ULL  // 28.1261 MHz - 10m

// WSPR message parameters
#define WSPR_CALL                 "VU3GWN"
#define WSPR_DBM                   23

/*
 * 
20 dBm  0.1000 W
21 dBm  0.1259 W
22 dBm  0.1585 W
23 dBm  0.1995 W
24 dBm  0.2512 W
25 dBm  0.3165 W
26 dBm  0.3981 W
27 dBm  0.5012 W
28 dBm  0.6310 W
29 dBm  0.7943 W
30 dBm  1.0000 W
 * 
 */

//******************************************************************
//                      Hardware defines
//******************************************************************
#define TX_LED_PIN                 8
#define GPS_STATUS_LED_PIN         9
#define POWER_ON_LED_PIN           10

#define SI5351_CAL_FACTOR          2000
#define SI5351_I2C_ADDRESS         0x60

#define GPS_RX_PIN                 4
#define GPS_TX_PIN                 3
#define GPS_BAUDRATE               9600
#define GPS_SERIAL_READ_DURATION   1200

#define GPS_INIT_MAX_TIME          5000
#define GPS_INIT_DELAY             500
#define GPS_SYNC_ATTEMPTS          20
#define GPS_SYNC_DELAY             10000

//******************************************************************
//                      TX Delay defines
//******************************************************************

#define TX_DELAYLOOPS 1
#define TX_DELAY 2000

//******************************************************************
//                      Global variables
//******************************************************************
uint8_t tx_buffer[WSPR_MESSAGE_BUFFER_SIZE];
Si5351 si5351(SI5351_I2C_ADDRESS);
uint64_t transmissionFrequency;
int whichBand = 15 ;

void(* resetHardware) (void) = 0;

//******************************************************************
//                      Function prototypes
//******************************************************************
void encodeWSPRMessage(const TinyGPSPlus& gpsDataObj);
void errorLEDIndicationAndReboot();
void initializeGPSSerialConnection(SoftwareSerial& gpsSerial);
void initializeLEDs();
void initializeSI5351();
void setQTHLocator(const TinyGPSPlus& gpsDataObj, char qthLocator[]);
void setTransmissionFrequency();
void synchronizeDateTime(TinyGPSPlus& gpsDataObj);
void transmitWSPRMessage();
bool trySyncGPSData(SoftwareSerial& gpsSerial, TinyGPSPlus& gpsDataObj);
void PowerLEDBlink(int);

//******************************************************************
//                      Function definitions
//******************************************************************
void encodeWSPRMessage(const TinyGPSPlus& gpsDataObj)
{
    memset(tx_buffer, 0, WSPR_MESSAGE_BUFFER_SIZE);
    
    char qthLocator[5];
    setQTHLocator(gpsDataObj, qthLocator);

    JTEncode jtencode;
    jtencode.wspr_encode(WSPR_CALL, qthLocator, WSPR_DBM, tx_buffer);
}

void PowerLEDBlink(int nTimes)
{
    return ;
    
    digitalWrite(POWER_ON_LED_PIN, LOW);
    delay(500);   
    
    for (uint8_t i{0}; i < nTimes; ++i) {
        digitalWrite(POWER_ON_LED_PIN, HIGH);
        delay(500);
        digitalWrite(POWER_ON_LED_PIN, LOW);
        delay(500);
    }

    digitalWrite(POWER_ON_LED_PIN, HIGH);
    delay(500);
}

void errorLEDIndicationAndReboot(int nTimes = 3)
{
    // Turn off all the green LEDs (GPS, ON)
    digitalWrite(POWER_ON_LED_PIN, LOW);
    digitalWrite(GPS_STATUS_LED_PIN, LOW);

    // Flash the red LED (TX) ten times
    for (uint8_t i{0}; i < nTimes; ++i) {
        digitalWrite(TX_LED_PIN, HIGH);
        delay(500);
        digitalWrite(TX_LED_PIN, LOW);
        delay(500);
    }

    resetHardware();
}

void initializeGPSSerialConnection(SoftwareSerial& gpsSerial)
{   
    gpsSerial.begin(GPS_BAUDRATE);
    
    const uint32_t startTime{millis()};
    while (gpsSerial.available() == false && millis() <= startTime + GPS_INIT_MAX_TIME)
        delay(GPS_INIT_DELAY);

    if (gpsSerial.available() == false)
        errorLEDIndicationAndReboot(5);
}

void initializeLEDs()
{  
    pinMode(TX_LED_PIN, OUTPUT);
    digitalWrite(TX_LED_PIN, LOW);

    pinMode(POWER_ON_LED_PIN, OUTPUT);
    digitalWrite(POWER_ON_LED_PIN, HIGH);
    
    pinMode(GPS_STATUS_LED_PIN, OUTPUT);
    digitalWrite(GPS_STATUS_LED_PIN, LOW);
}

void initializeSI5351()
{
    if (si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, SI5351_CAL_FACTOR))
        // Set CLK0 as TX OUT
        si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    else
        errorLEDIndicationAndReboot(3);
}

void setQTHLocator(const TinyGPSPlus& gpsDataObj, char qthLocator[]) {
    const float latitude{gpsDataObj.location.lat() + 90.0};
    const float longitude{gpsDataObj.location.lng() + 180.0};

    qthLocator[0] = 'A' + (longitude / 20);
    qthLocator[1] = 'A' + (latitude / 10);

    qthLocator[2] = '0' + (uint8_t)(longitude / 2) % 10;
    qthLocator[3] = '0' + (uint8_t)(latitude) % 10;

    qthLocator[4] = '\0';
}

void synchronizeDateTime(TinyGPSPlus& gpsDataObj)
{
    SoftwareSerial gpsSerial{GPS_RX_PIN, GPS_TX_PIN};

    initializeGPSSerialConnection(gpsSerial);
    
    uint8_t syncAttemps{1};
    bool dataSynchronized{trySyncGPSData(gpsSerial, gpsDataObj)};
    while (dataSynchronized == false && syncAttemps < GPS_SYNC_ATTEMPTS)
    {
        delay(GPS_SYNC_DELAY);
        dataSynchronized = trySyncGPSData(gpsSerial, gpsDataObj);
        ++syncAttemps;
    }

    if (dataSynchronized == false)
        errorLEDIndicationAndReboot(10);
}

void transmitWSPRMessage()
{
    digitalWrite(TX_LED_PIN, HIGH);

    si5351.output_enable(SI5351_CLK0, 1);

    for(uint8_t i{0}; i < WSPR_SYMBOL_COUNT; ++i)
    {
        si5351.set_freq(transmissionFrequency + (tx_buffer[i] * WSPR_TONE_SPACING), SI5351_CLK0);
        delay(WSPR_DELAY);
    }

    si5351.output_enable(SI5351_CLK0, 0);

    digitalWrite(TX_LED_PIN, LOW);
}

bool trySyncGPSData(SoftwareSerial& gpsSerial, TinyGPSPlus& gpsDataObj)
{ 
    const uint32_t startTime{millis()};
    while (millis() - startTime < GPS_SERIAL_READ_DURATION) 
    {
        while (gpsSerial.available())
            gpsDataObj.encode(gpsSerial.read());
    }

    if (gpsDataObj.time.isValid() && gpsDataObj.date.isValid() && gpsDataObj.location.isValid()){
        setTime(gpsDataObj.time.hour(), gpsDataObj.time.minute(), gpsDataObj.time.second(), 
                gpsDataObj.date.day(), gpsDataObj.date.month(), gpsDataObj.date.year());

        digitalWrite(GPS_STATUS_LED_PIN, HIGH);
        return true;
    }
    
    digitalWrite(GPS_STATUS_LED_PIN, LOW);
    return false; 
}

void setTransmissionFrequency()
{
    // WSPR message transmission at each transmitWSPRMessage() function call is performed on 
    // a randomly selected frequency within the range of +/- 100 Hz from the center frequency
    transmissionFrequency = (WSPR_DEFAULT_FREQ_15 + random(-100, 101)) * 100ULL;
}

//******************************************************************
//                      Main firmware code
//******************************************************************
void setup()
{
    initializeLEDs();
    initializeSI5351();

    TinyGPSPlus gpsDataObj;
    synchronizeDateTime(gpsDataObj);

    // RAM-intensive operation, generate WSPR message only once, at device startup
    encodeWSPRMessage(gpsDataObj);
    
    // Initialize the random number generator with a seed based on the current time
    randomSeed(millis());
    // Setting the random transmission frequency within the range of +/- 100 Hz from 
    // the center operating frequency
    setTransmissionFrequency();
}

void loop()
{
    static int lastTxMinute = -1;

    // Only act at start of even minute
    if (second() == 0 && (minute() % 2 == 0))
    {
        // Prevent multiple triggers in same minute
        if (minute() == lastTxMinute) return;
        lastTxMinute = minute();

        // Select band
        if (whichBand == 15)
        {
            transmissionFrequency = (WSPR_DEFAULT_FREQ_15 + random(-100, 101)) * 100ULL;
            whichBand = 28;  // switch next time
        }
        else
        {
            transmissionFrequency = (WSPR_DEFAULT_FREQ_28 + random(-100, 101)) * 100ULL;
            whichBand = 15;  // switch next time
        }

        transmitWSPRMessage();

        // Sync AFTER transmission (safe timing)
        TinyGPSPlus gpsDataObj;
        synchronizeDateTime(gpsDataObj);
    }
}

void loop_old_v1()
{
  
    // Transmission of a WSPR message every even minute (00:00, 00:02, 00:04, ...)
    if(whichBand == 15 && second() == 0 && minute() % 2 == 0)
    {

        // Set a new, random transmission frequency
        // setTransmissionFrequency();
        // WSPR message transmission at each transmitWSPRMessage() function call is performed on 
        // a randomly selected frequency within the range of +/- 100 Hz from the center frequency
        transmissionFrequency = (WSPR_DEFAULT_FREQ_15 + random(-100, 101)) * 100ULL;
        
        transmitWSPRMessage();
        
        // Time synchronization based on current GPS data for a new transmission cycle
        TinyGPSPlus gpsDataObj;
        synchronizeDateTime(gpsDataObj);
                
        //next Band to be hopped to
        whichBand = 28 ;
    }
    
    // Transmission of a WSPR message every even minute (00:00, 00:02, 00:04, ...)
    
    if(whichBand == 28 && second() == 0 && minute() % 2 == 0)
    {
        // Set a new, random transmission frequency
        // setTransmissionFrequency();
        // WSPR message transmission at each transmitWSPRMessage() function call is performed on 
        // a randomly selected frequency within the range of +/- 100 Hz from the center frequency
        transmissionFrequency = (WSPR_DEFAULT_FREQ_28 + random(-100, 101)) * 100ULL;
        
        transmitWSPRMessage();
        
        // Time synchronization based on current GPS data for a new transmission cycle
        TinyGPSPlus gpsDataObj;
        synchronizeDateTime(gpsDataObj);
        
        //next Band to be hopped to
        whichBand = 15 ;
    }

}


/*
 * 
 * 
void loop_UNUSED()
{
    static uint8_t slot = 0; // 0–29

    if (slot < 15)
        transmitOnBand(WSPR_DEFAULT_FREQ_28);
    else
        transmitOnBand(WSPR_DEFAULT_FREQ_15);

    slot = (slot + 1) % 30;

    delay(TX_DELAY) ;
}


void transmitOnBand(uint64_t txFrequency)
{
    // Transmission of a WSPR message every even minute (00:00, 00:02, 00:04, ...)
    if(second() == 0 && minute() % 2 == 0)
    {
        transmitWSPRMessage();
        
        // Time synchronization based on current GPS data for a new transmission cycle
        TinyGPSPlus gpsDataObj;
        synchronizeDateTime(gpsDataObj);
        
        // Set a new, random transmission frequency
        // setTransmissionFrequency();
        // WSPR message transmission at each transmitWSPRMessage() function call is performed on 
        // a randomly selected frequency within the range of +/- 100 Hz from the center frequency
        transmissionFrequency = (txFrequency + random(-100, 101)) * 100ULL;
    }
}
*/

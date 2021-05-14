/*
  Blink_sleep
  combination of Blink_sleep demo and eddystone url with sensors
  4/28/21 - adding med vib sensor and led
  4/29/21 - adding led interupt code
  5/3/21 - working dual sensors and led
  5/14/21 - adding longer broadcast time, 
            shorter intervals, 
            code for battery voltage,
            
*/
#include <bluefruit.h>
#include <math.h>
// Sleepydog
#include <Adafruit_SleepyDog.h>

// Sensor data:
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>

// Battery reading code:
#include <Arduino.h>

#if defined ARDUINO_NRF52840_CIRCUITPLAY
#define PIN_VBAT A6 // this is just a mock read, we'll use the light sensor, so we can run the test
#endif

uint32_t vbat_pin = PIN_VBAT; // A7 for feather nRF52832, A6 for nRF52840

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

#ifdef NRF52840_XXAA
#define VBAT_DIVIDER (0.5F)      // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F) // Compensation factor for the VBAT divider
#else
#define VBAT_DIVIDER (0.71275837F) // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F) // Compensation factor for the VBAT divider
#endif

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
// END battery reading code

Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity

float temperature, pressure, altitude;
float accel_x, accel_y, accel_z, accel_mag;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;

#define WAKE_PIN PIN_A0
#define WAKE_LED_PIN PIN_A1
const int led = 12;
int ledFlag = 0;

// Battery reading code:
float readVBAT(void)
{
    float raw;

    // Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);

    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    // Set the ADC back to the default settings
    analogReference(AR_DEFAULT);
    analogReadResolution(10);

    // Convert the raw value to compensated mv, taking the resistor-
    // divider into account (providing the actual LIPO voltage)
    // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
    return raw * REAL_VBAT_MV_PER_LSB;
}

uint8_t mvToPercent(float mvolts)
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30;
    }

    mvolts -= 3600;
    return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}
// END battery reading code

#define SLEEPING_DELAY 10000 // sleep after x seconds of blinking/broadcasting
void gotoSleep(unsigned long time)
{
    // shutdown when time reaches SLEEPING_DELAY ms
    if ((time > SLEEPING_DELAY))
    {
        // to reduce power consumption when sleeping, turn off all your LEDs (and other power hungry devices)
        digitalWrite(LED_BUILTIN, LOW);

        // turn off BRIGHT_LED
        digitalWrite(led, LOW);

        // setup your wake-up pins.
        pinMode(WAKE_PIN, INPUT_PULLDOWN_SENSE); // A0 - high vib sensor - this pin (WAKE_LOW_PIN) is pulled up and wakes up the feather when externally connected to ground.
        //pinMode(WAKE_LED_PIN, INPUT_PULLDOWN_SENSE);   // A1 - med vib sensor - this pin (WAKE_HIGH_PIN) is pulled down and wakes up the feather when externally connected to 3.3v.

        // power down nrf52.
        sd_power_system_off(); // this function puts the whole nRF52 to deep sleep (no Bluetooth).  If no sense pins are setup (or other hardware interrupts), the nrf52 will not wake up.
    }
}

void broadcastEddystone(void)
{

    bmp280.begin();       // temp barometric pressure
    lsm6ds33.begin_I2C(); // accel gyroscope
    sht30.begin();        // humidity

    // get sensor values
    temperature = bmp280.readTemperature();
    pressure = bmp280.readPressure();
    altitude = bmp280.readAltitude(1013.25);

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    accel_x = accel.acceleration.x;
    accel_y = accel.acceleration.y;
    accel_z = accel.acceleration.z;
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;

    accel_mag = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

    humidity = sht30.readHumidity();

    // Battery read code:
    float vbat_mv = readVBAT();

    // Convert from raw mv to percentage (based on LIPO chemistry)
    uint8_t vbat_per = mvToPercent(vbat_mv);

    // Display the results

    Serial.print("LIPO = ");
    Serial.print(vbat_mv);
    Serial.print(" mV (");
    Serial.print(vbat_per);
    Serial.println("%)");
    // END battery read code

    /*****************************************/
    Serial.println("\nFeather Sense Sensor Demo");
    Serial.println("---------------------------------------------");
    Serial.print("Temperature: ");
    Serial.print(temperature); // 25.55
    Serial.println(" C");
    Serial.print("Barometric pressure: ");
    Serial.println(pressure); // 101487.64
    Serial.print("Altitude: ");
    Serial.print(altitude); // -13.53
    Serial.println(" m");
    Serial.print("Acceleration: ");
    Serial.print(accel_x);
    Serial.print(" ");
    Serial.print(accel_y);
    Serial.print(" ");
    Serial.print(accel_z);
    Serial.print(" || ");
    Serial.print(accel_mag); // -0.22 0.03 9.87 || 9.87 m/s^2
    Serial.println(" m/s^2");
    Serial.print("Gyro: ");
    Serial.print(gyro_x);
    Serial.print(" ");
    Serial.print(gyro_y);
    Serial.print(" ");
    Serial.print(gyro_z);
    Serial.println(" dps");
    Serial.print("Humidity: ");
    Serial.print(humidity); // 43.02
    Serial.println(" %");
    /*****************************************/

    // configure URL
    char URL[23];
    char buffer[10];

    URL[0] = 'h';
    URL[1] = 't';
    URL[2] = 't';
    URL[3] = 'p';
    URL[4] = 's';
    URL[5] = ':';
    URL[6] = '/';
    URL[7] = '/';
    URL[8] = String(temperature)[0];
    URL[9] = String(temperature)[1];
    URL[10] = String(temperature)[3];
    URL[11] = String(temperature)[4];
    URL[12] = String(humidity)[0];
    URL[13] = String(humidity)[1];
    URL[14] = String(humidity)[3];
    URL[15] = String(humidity)[4];
    URL[16] = String(accel_mag)[0];
    URL[17] = String(accel_mag)[2];
    URL[18] = String(accel_mag)[3];

    // URL[19] = ' ';
    if (accel_x < -6)
    {
        URL[19] = 'a';
    }
    else if (accel_x > 6)
    {
        URL[19] = 'b';
    }
    else if (accel_y < -6)
    {
        URL[19] = 'c';
    }
    else if (accel_y > 6)
    {
        URL[19] = 'd';
    }
    else if (accel_z < -6)
    {
        URL[19] = 'e';
    }
    else if (accel_z > 6)
    {
        URL[19] = 'f';
    }
    else
    {
        URL[19] = 'g';
    }
    URL[20] = String(vbat_per)[0];
    URL[21] = String(vbat_per)[1];
    URL[22] = '1';
    URL[23] = '1';
    URL[24] = '1';

    // configure eddystone
    //   URL += String(temperature, 4);
    EddyStoneUrl eddyUrl(-40, URL);
    // broadcast
    Bluefruit.Advertising.setBeacon(eddyUrl);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(480, 480); // in unit of 0.625 ms originally 160(100ms)
    Bluefruit.Advertising.setFastTimeout(0);     // number of seconds in fast mode
    Bluefruit.Advertising.start(1);              // 0 = Don't stop advertising after n seconds
    delay(1000);
    Bluefruit.Advertising.clearData();
}

void setup()
{
    // led testing
    pinMode(WAKE_LED_PIN, INPUT);
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);

    Bluefruit.begin(); // Sleep functions need the softdevice to be active.

    // initialize the sensors
    //  lis3mdl.begin_I2C(); // magnetometer
    // bmp280.begin();       // temp barometric pressure
    // lsm6ds33.begin_I2C(); // accel gyroscope
    // sht30.begin();        // humidity

    // bluefruit init
    Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
    Bluefruit.setName("BESTAID2");

    // battery setup
    readVBAT();
}

void loop()
{
    // led testing
    if (digitalRead(WAKE_LED_PIN) == HIGH)
    {
        if (ledFlag == 0)
        {
            ledFlag = 1;
            digitalWrite(led, HIGH);
        }
        else
        {
            ledFlag = 0;
            digitalWrite(led, LOW);
        }
    }

    // broadcast
    broadcastEddystone();

    gotoSleep(millis()); // call millis() and pass it to the sleep function.  On wake-up, millis will start at 0 again.
}
#include <Adafruit_BMP3XX.h>
class Barometer
{
private:
    unsigned long tStart = micros();
    unsigned long dtBaro = 0;
    bool baroOn = false;
    Adafruit_BMP3XX bme;
    uint16_t BMP390_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
    float estimatedZ = 0;
    float oldZ = 0;
public:
    Barometer();
    void init();
    float getEstimatedZ();
    float getVelocityZ();
    bool updateBarometer();
};
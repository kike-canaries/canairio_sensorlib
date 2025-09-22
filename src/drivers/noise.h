#ifndef NOISE_H
#define NOISE_H

#include <Arduino.h>
#include "driver/adc.h"

class Noise {
public:
    Noise(adc1_channel_t channel = ADC1_CHANNEL_4, adc_atten_t atten = ADC_ATTEN_DB_12);
    void noiseInit();
    void noiseRead();

private:
    adc1_channel_t _channel;
    adc_atten_t _atten;

    const int DutyCycle = 120000;
    const int Sleep4NoNoise = 300000;
    int LowNoiseLevel = 36;
    int noiseDiffSleep = 0;

    int loops;
    unsigned int cycles;
    float icycles;
    float noise_avg;
    float noise_avg_pre;
    unsigned int noise_peak;
    unsigned int noise_min;
    unsigned int noise;
    unsigned long noise_sum;

    unsigned long tmp_ini;
    int loops_legal;
    float noise_avg_legal;
    float noise_avg_legal_max;
    unsigned int noise_avg_legal_period;
    unsigned long noise_sum_legal;
    long LegalStart;
    long CountStart;

    float SensorId;
};

#endif

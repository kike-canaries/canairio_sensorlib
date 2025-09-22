#include "noise.h"

Noise::Noise(adc1_channel_t channel, adc_atten_t atten)
    : _channel(channel), _atten(atten), SensorId(48.05), noise_avg_legal_period(5000)
{}

void Noise::noiseInit() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("starting");

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(_channel, _atten);

    tmp_ini = millis();
    CountStart = millis();
    noise_avg = LowNoiseLevel;
    noise_avg_legal = LowNoiseLevel;
    noise_avg_pre = LowNoiseLevel;
    noise_avg_legal_max = LowNoiseLevel;
    noise_peak = 0;
    noise_min = 1000;
    noise_sum = 0;
    noise_sum_legal = 0;
    loops = 0;
    loops_legal = 0;
    cycles = 50;
    icycles = 1;
    LegalStart = millis();
}

void Noise::noiseRead() {
    int noise = adc1_get_raw(_channel);

    if (noise > 4095) {
        Serial.print("outlier removed: ");
        Serial.println(noise);
        return;
    }

    if (noise < LowNoiseLevel) {
        LowNoiseLevel = noise;
    }

    if (millis() - tmp_ini > 1000) {
        noise_sum += noise;
        loops++;

        Serial.print("Noise: ");
        Serial.print(noise);
        Serial.print(" loop: ");
        Serial.print(loops);
        Serial.print(" cycles: ");
        Serial.println(cycles);
        Serial.print(" loops_legal: ");
        Serial.println(loops_legal);

        tmp_ini = millis();
    }

    loops_legal++;
    noise_sum_legal += noise;
    if (millis() - LegalStart > noise_avg_legal_period) {
        Serial.print(" Legal time: ");
        Serial.println(millis() - LegalStart);
        LegalStart = millis();

        noise_avg_legal = int(noise_sum_legal / loops_legal);
        if (noise_avg_legal > noise_avg_legal_max) {
            noise_avg_legal_max = noise_avg_legal;
            Serial.print("  Noise legal current maximum: ");
            Serial.println(noise_avg_legal_max);
        }

        Serial.print("   (Legal) noise_avg_legal: ");
        Serial.print(noise_avg_legal);
        Serial.print(" noise_avg_legal_max: ");
        Serial.print(noise_avg_legal_max);
        Serial.print(" samples: ");
        Serial.println(loops_legal);

        loops_legal = 0;
        noise_sum_legal = 0;
    }

    if (noise > noise_peak) {
        noise_peak = noise;
        Serial.print("Noise peak: ");
        Serial.println(noise_peak);
    }

    if (noise < noise_min && loops > 5) {
        noise_min = noise;
        Serial.print("Noise min: ");
        Serial.println(noise_min);
    }

    if (millis() - CountStart > DutyCycle) {
        Serial.print(" DutyCycle time: ");
        Serial.println(millis() - CountStart);
        CountStart = millis();

        noise_avg = int(noise_sum / loops);
        Serial.print("  Noise average: ");
        Serial.println(noise_avg);
        Serial.print("  Noise sum: ");
        Serial.println(noise_sum);
        Serial.print("  Noise min: ");
        Serial.println(noise_min);
        Serial.print("  Samples: ");
        Serial.println(loops);

        if (cycles > 99) {
            icycles = -1;
        } else if (cycles < 1) {
            LowNoiseLevel = noise_min;
            icycles = +1;
        }

        cycles += icycles;

        if (noise_avg < LowNoiseLevel + noiseDiffSleep && noise_avg_pre < LowNoiseLevel + noiseDiffSleep) {
            cycles -= icycles;
        }

        noise_peak = 0;
        noise_min = 1000;
        noise_sum = 0;
        loops = 0;
        noise_avg_legal_max = 0;
        noise_avg_pre = noise_avg;

        LegalStart = millis();
    }
}

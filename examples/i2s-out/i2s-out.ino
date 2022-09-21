#include "AudioTools.h"
#include "stm32-i2s.h"

SineWaveGenerator<int16_t> sineWave(32000);   // subclass of SoundGenerator with max amplitude of 32000
int sample_rate = 8000;
int channels = 1;

void readToTransmit(uint8_t *buffer, uint16_t byteCount) {
	uint16_t samples = byteCount / 2;
	int16_t *buffer_16 = (int16_t*) buffer;
	for (uint j = 0; j < samples; j+=2) {
		int16_t sample = sineWave.readSample();
		buffer_16[j] = sample;
		buffer_16[j+1] = sample;
	}
}

void setup() {
	Serial.begin(115200);
	sineWave.begin(channels, sample_rate, N_B4);
	i2s_default_samplerate = I2S_AUDIOFREQ_8K;
	startI2STransmit(&hi2s3, readToTransmit, I2S_BUFFER_SIZE);
}

void loop() {

}

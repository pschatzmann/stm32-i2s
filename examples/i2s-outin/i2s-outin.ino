#include "AudioTools.h"
#include "stm32-i2s.h"

SineWaveGenerator<int16_t> sineWave(32000);   // subclass of SoundGenerator with max amplitude of 32000
int sample_rate = 44100;
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

void writeToReceive(uint8_t *buffer, uint16_t byteCount) {
	out.write(buffer, byteCount);
}

void setup() {
	Serial.begin(115200);
	sineWave.begin(channels, sample_rate, N_B4);
	startI2STransmitReceive(&hi2s3, readToTransmit, writeToReceive, I2S_BUFFER_SIZE);
}

void loop() {

}
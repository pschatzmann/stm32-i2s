#include "AudioTools.h"
#include "stm32-i2s.h"

SineWaveGenerator<int16_t> sineWave(32000);   // subclass of SoundGenerator with max amplitude of 32000
I2SSettingsSTM32 i2s_settings;
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
	i2s_settings.sample_rate =I2S_AUDIOFREQ_44K;
	if (!startI2STransmitReceive(&i2s_settings, readToTransmit, writeToReceive, I2S_BUFFER_SIZE)){
		Serial.println("I2S Error");
		return;
	}

}

void loop() {

}
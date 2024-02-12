#include "AudioTools.h"
#include "stm32-i2s.h"

I2SSettingsSTM32 i2s_settings;
int sample_rate = 44100;
int channels = 1;
uint8_t buffer[I2S_BUFFER_SIZE]; // byte buffer

// from buffer to dac
void readToTransmit(uint8_t *data, uint16_t byteCount) {
	assert(byteCount==I2S_BUFFER_SIZE);
	memmove(data, buffer, byteCount);
}

// from mic to buffer
void writeToReceive(uint8_t *data, uint16_t byteCount) {
	assert(byteCount==I2S_BUFFER_SIZE);
	memmove(buffer, data, byteCount);
}

void setup() {
	Serial.begin(115200);
	i2s_settings.sample_rate =I2S_AUDIOFREQ_44K;
	if (!I2S.startI2STransmitReceive(&i2s_settings, readToTransmit, writeToReceive, I2S_BUFFER_SIZE)){
		Serial.println("I2S Error");
		return;
	}
}

void loop() {

}
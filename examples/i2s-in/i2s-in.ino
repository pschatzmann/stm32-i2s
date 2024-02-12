#include "AudioTools.h"
#include "stm32-i2s.h"

CsvStream<int16_t> out(Serial, 2); // ASCII output stream 
I2SSettingsSTM32 i2s_settings;

void writeFromReceive(uint8_t *buffer, uint16_t byteCount){
	out.write(buffer, byteCount);
}

void setup() {
	Serial.begin(115200);
	i2s_settings.sample_rate = I2S_AUDIOFREQ_8K;
	if (!I2s.startI2SReceive(&i2s_settings, writeFromReceive, I2S_BUFFER_SIZE){
		Serial.println("I2S Error");
	}
}

void loop() {

}

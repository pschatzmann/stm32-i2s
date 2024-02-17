#include "AudioTools.h"
#include "stm32-i2s.h"

using namespace stm32_i2s;

CsvStream<int16_t> out(Serial, 2); // ASCII output stream 
I2SSettingsSTM32 i2s_settings;

void writeFromReceive(uint8_t *buffer, uint16_t byteCount, void*){
	out.write(buffer, byteCount);
}

void setup() {
	Serial.begin(115200);
	i2s_settings.sample_rate = I2S_AUDIOFREQ_8K;
	if (!I2s.beginReadDMA(i2s_settings, writeFromReceive){
		Serial.println("I2S Error");
	}
}

void loop() {

}

#include "AudioTools.h"
#include "stm32-i2s.h"

CsvStream<int16_t> out(Serial, 2); // ASCII output stream 

void writeFromReceive(uint8_t *buffer, uint16_t byteCount){
	out.write(buffer, byteCount);
}

void setup() {
	Serial.begin(115200);
	startI2SReceive(&hi2s3, writeFromReceive, I2S_BUFFER_SIZE);
}

void loop() {

}

# Arduino STM32F411 I2S Library

I wanted to use I2S in Arduino with my __STM32F411 Black Pill__ processor. 

My first trials failed miserably using the DMA versions of the API, so I decideed to generate a working solution using the STM Cube IDE and then convert this to Arduino.

- The API is using Callbacks to transfer the data.
- The DMA is used to transfer the data
- I2S Protocol can be defined with __i2s_default_standard__ variable (default is I2S_STANDARD_PHILIPS)
- Mode can be selected with __i2s_default_mode__ variable (default is I2S_MODE_MASTER_TX)
- Full Duplex is supported with __i2s_default_fullduplexmode__ variable (default is I2S_FULLDUPLEXMODE_ENABLE)
- Sampling rate can be selected with __is2_default_samplerate__ variable(default value is I2S_AUDIOFREQ_44K) 
- Only 16bit data is supported

## Pins

PINs  |	FUNCTIONs 
------|------------	
PA4	  | I2S3_WS	
PB10  |	I2S3_MCK	
PB3	  | I2S3_CK	
PB4	  | I2S3_ext_SD	
PB5	  | I2S3_SD	


## Sending Data

```
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

void setup() {
	sineWave.begin(channels, sample_rate, N_B4);
	startI2STransmit(&hi2s3, readToTransmit);
}

void loop() {

}
```


## Receiving Data

```
#include "AudioTools.h"
#include "stm32-i2s.h"

CsvStream<int16_t> out(Serial, 2); // ASCII output stream 

void writeFromReceive(uint8_t *buffer, uint16_t byteCount){
	out.write(buffer, byteCount);
}

void setup() {
	startI2SReceive(&hi2s3, writeFromReceive, I2S_BUFFER_SIZE);
}

void loop() {

}

```

### Installation in Arduino

You can download the library as zip and call include Library -> zip library. Or you can git clone this project into the Arduino libraries folder e.g. with

```
cd  ~/Documents/Arduino/libraries
git clone pschatzmann/https://github.com/pschatzmann/stm32f411-i2s.git
```

I recommend to use git because you can easily update to the latest version just by executing the ```git pull``` command in the project folder.

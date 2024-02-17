# Arduino STM32F411 I2S Library

I wanted to use __I2S__ in Arduino with my __STM32F411 Black Pill__ processor together with my [Arduino Audio Tools](https://github.com/pschatzmann/arduino-audio-tools)! 

![stm32f411](https://pschatzmann.github.io/stm32f411-i2s/stm32f411.jpeg)

Unfortunately [STMDuino](https://github.com/stm32duino) does not provide this functionality.

My first trials failed miserably using the DMA versions of the HAL API, so I decided to generate a working solution using the __STM Cube IDE__ and then convert this to Arduino library, that provides the following functionality:

- The DMA is used to transfer the data
- The API is using __Callbacks__ to transfer the data.
- The following settings are supported:
	- I2S Protocol can be defined with __i2s_default_standard__ variable (default is I2S_STANDARD_PHILIPS)
	- Mode can be selected with __i2s_default_mode__ variable (default is I2S_MODE_MASTER_TX)
	- Full Duplex is supported with __i2s_default_fullduplexmode__ variable (default is I2S_FULLDUPLEXMODE_ENABLE)
	- Sampling rate can be selected with __is2_default_samplerate__ variable(default value is I2S_AUDIOFREQ_44K) 
- Only __16bit__ data is supported
- I also incuded the __codec drivers__ that are part of some stm32 evaluation boards. 

## Pins for I2S3

FUNCTIONs  | BlackP | Disco
-----------|--------|------
MCK	       | PB10   | PC7
BCK	       | PB3    | PC10
WS (LRC)   | PA4	| PA4
SD	       | PB5    | PC3
ext_SD	   | PB4    | PC12

## Supported Sample Rates

- I2S_AUDIOFREQ_192K
- I2S_AUDIOFREQ_96K
- I2S_AUDIOFREQ_48K
- I2S_AUDIOFREQ_44K
- I2S_AUDIOFREQ_32K
- I2S_AUDIOFREQ_22K
- I2S_AUDIOFREQ_16K
- I2S_AUDIOFREQ_11K
- I2S_AUDIOFREQ_8K


## API

Below I demonstrate the basic API provided by this library. However, I recommend that you use the I2SStream class from the [Arduino Audio Tools](https://github.com/pschatzmann/arduino-audio-tools) library which uses this functionality.

### Sending Data

```
#include "AudioTools.h"
#include "stm32-i2s.h"

using namespace stm32_i2s;

SineWaveGenerator<int16_t> sineWave(32000);   // subclass of SoundGenerator with max amplitude of 32000
I2SSettingsSTM32 i2s_settings;
int sample_rate = 8000;
int channels = 1;

void readToTransmit(uint8_t *buffer, uint16_t byteCount, void*) {
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
	i2s_settings.sample_rate = I2S_AUDIOFREQ_8K;
	if (!I2S.beginWriteDMA(i2s_settings, readToTransmit)){
		Serial.println("I2S Error");
	}
}

void loop() {}

```


### Receiving Data

```
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

void loop() {}

```

## Documentation

Here is the link to the [actual documentation](https://pschatzmann.github.io/stm32f411-i2s/html/classstm32__i2s_1_1_stm32_i2s_class.html).

You might also find further information in [my Blogs](https://www.pschatzmann.ch/tags/stm32)


## Installation in Arduino

You can download the library as zip and call include Library -> zip library. Or you can git clone this project into the Arduino libraries folder e.g. with

```
cd  ~/Documents/Arduino/libraries
git clone https://github.com/pschatzmann/stm32f411-i2s.git
```

I recommend to use git because you can easily update to the latest version just by executing the ```git pull``` command in the project folder.


## Copyright

__Copyright © 2022 Phil Schatzmann__

[GNU General Public License](License.txt)


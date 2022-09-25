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

PINs  |	FUNCTIONs 
------|------------	
PB10  |	MCK	
PB3	  | BCK	
PA4	  | WS (LRC)	
PB5	  | SD	
PB4	  | ext_SD	

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

SineWaveGenerator<int16_t> sineWave(32000);   // subclass of SoundGenerator with max amplitude of 32000
I2SSettingsSTM32 i2s_settings;
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
	sineWave.begin(channels, sample_rate, N_B4);
	i2s_settings.sample_rate = I2S_AUDIOFREQ_8K;
	startI2STransmit(&i2s_settings, readToTransmit);
}

void loop() {

}
```


### Receiving Data

```
#include "AudioTools.h"
#include "stm32-i2s.h"

CsvStream<int16_t> out(Serial, 2); // ASCII output stream 
I2SSettingsSTM32 i2s_settings;

void writeFromReceive(uint8_t *buffer, uint16_t byteCount){
	out.write(buffer, byteCount);
}

void setup() {
	i2s_settings.sample_rate = I2S_AUDIOFREQ_8K;
	startI2SReceive(&i2s_settings, writeFromReceive, I2S_BUFFER_SIZE);
}

void loop() {

}

```

## Documentation

Here is the link to the [actual documentation](https://pschatzmann.github.io/stm32f411-i2s/html/modules.html).

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


__Copyright © 2015 STMicroelectronics__
  
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.
	3. Neither the name of STMicroelectronics nor the names of its contributors
	may be used to endorse or promote products derived from this software
	without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
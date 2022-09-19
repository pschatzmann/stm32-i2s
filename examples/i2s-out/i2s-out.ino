#include "AudioTools.h"
#include "stm32-i2s.h"

uint32_t dma_buffer[I2S_BUFFER_SIZE];
//float osc_phi = 0;
//float osc_phi_inc = 440.0f / 44100.0f;  // generating 440HZ
extern I2S_HandleTypeDef hi2s3;
SineWaveGenerator<int16_t> sineWave(32000);   // subclass of SoundGenerator with max amplitude of 32000
int sample_rate = 44100;
int channels = 1;

void fillBuffer(uint32_t *buffer, uint16_t len) {
	for (uint j = 0; j < len; j++) {
		int16_t y = sineWave.readSample();
		buffer[j] = ((uint32_t) (uint16_t) y) << 0 | ((uint32_t) (uint16_t) y) << 16;
	}
}

void startAudioBuffers(I2S_HandleTypeDef *hi2s) {
	// clear buffer
	memset(dma_buffer, 0, sizeof(dma_buffer));
	// start circular dma
	if (HAL_I2S_Transmit_DMA(hi2s, (uint16_t*) dma_buffer, I2S_BUFFER_SIZE << 1)!=HAL_OK){
		LOGE("HAL_I2S_Transmit_DMA");
	}
}

extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	// second half finished, filling it up again while first  half is playing
	fillBuffer(&(dma_buffer[I2S_BUFFER_SIZE >> 1]), I2S_BUFFER_SIZE >> 1);
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	// first half finished, filling it up again while second half is playing
	fillBuffer(&(dma_buffer[0]), I2S_BUFFER_SIZE >> 1);
}

void setup() {
	sineWave.begin(channels, sample_rate, N_B4);
	i2s_begin();
	startAudioBuffers (&hi2s3);
}

void loop() {

}

#pragma once
#include "codecs/cs43l22/cs43l22.h"

/// @brief Supported output devices
enum CS43L22OutputDevice {
 CS43L22_SPEAKER        =1,
 CS43L22_HEADPHONE      =2,
 CS43L22_BOTH           =3,
 CS43L22_AUTO           =4
};

/// @brief Supported Sampling rates
enum CS43L22Frequency {
 CS43L22_FREQUENCY_192K   =       ((uint32_t)192000),
 CS43L22_FREQUENCY_96K    =       ((uint32_t)96000),
 CS43L22_FREQUENCY_48K    =       ((uint32_t)48000),
 CS43L22_FREQUENCY_44K    =       ((uint32_t)44100),
 CS43L22_FREQUENCY_32K    =       ((uint32_t)32000),
 CS43L22_FREQUENCY_22K    =       ((uint32_t)22050),
 CS43L22_FREQUENCY_16K    =       ((uint32_t)16000),
 CS43L22_FREQUENCY_11K    =       ((uint32_t)11025),
 CS43L22_FREQUENCY_8K     =       ((uint32_t)8000)  
};

/// @brief  Supported Beep Configuration Settings
enum CS43L22BeepConfiguration {BeepOff, BeepSingle, BeepMultiple, BeepContinuous};


/**
 * @brief CS43L22 Audio Codec which is managed via I2C using the Wire library
 * @addtogroup CS43L22
 */
class CS43L22 {
public:
    // The default I2C address is 0x94, The reset pin for STM32F411 Disco is  PD4
    CS43L22(uint16_t DeviceAddr=CS43L22_I2C_ADDRESS ){
        this->deviceAddr = DeviceAddr;
    }
    bool begin(CS43L22OutputDevice OutputDevice, float Volume=0.8, CS43L22Frequency AudioFreq=CS43L22_FREQUENCY_44K){
        return 0==cs43l22_Init(deviceAddr, (uint32_t) OutputDevice,  toIntVol(Volume),  AudioFreq);
    }
    void end(void){
        cs43l22_DeInit();
        CODEC_AUDIO_POWER(false);
    }
    uint32_t readID(){
         return cs43l22_ReadID(deviceAddr);
    }
    bool play(uint16_t *pBuffer, uint16_t Size){
        return 0 == cs43l22_Play(deviceAddr, pBuffer, Size);
    }
    bool pause(){
        return 0 == cs43l22_Pause(deviceAddr);
    }
    bool resume(){
        return 0 == cs43l22_Resume(deviceAddr);
    }
    bool stop(bool active){
        return 0 == cs43l22_Stop(deviceAddr, (uint32_t)active);
    }
    // volume is in range between 0.0 and 1.0
    bool setVolume(float Volume){
        return 0 == cs43l22_SetVolume( deviceAddr,  toIntVol(Volume));
    }
    bool setFrequency(uint32_t AudioFreq){
        return 0 == cs43l22_SetFrequency(deviceAddr, AudioFreq);
    }
    bool setMute(bool active){
        return 0 == cs43l22_SetMute(deviceAddr,  active);
    }
    bool setOutputDevice(CS43L22OutputDevice device){
        return 0 == cs43l22_SetOutputMode(deviceAddr, device);
    }
    bool reset(){
        return 0 == cs43l22_Reset(deviceAddr);
    }

    CS43L22BeepConfiguration getBeepConfiguration(){
        uint8_t value = AUDIO_IO_Read(deviceAddr, 0x1E) >> 6;
        return (CS43L22BeepConfiguration)value;
    }

    void setBeepConfiguration(CS43L22BeepConfiguration val){
        uint8_t value = AUDIO_IO_Read(deviceAddr, 0x1E);
        uint8_t toKeep = value;
        BITMASK_CLEAR(toKeep, 0xC0); 
        uint8_t mix_disable = ((uint8_t)1) << 5;
        uint8_t newValue = val<<6 | toKeep | mix_disable ;
        if (CODEC_IO_Write(deviceAddr, 0x1E, newValue)!=0){
            STM32_LOG("setBeepConfiguration failed");
        }
    }

    // freq 0 to 15, onTime 0 to 15, offTime 0 to 7
    void beep(uint8_t freq, uint8_t onTime=3, uint8_t offTime=1){
        if (onTime>0xF){
            onTime = 0xF;
        }
        if (offTime>7){
            offTime = 7;
        }
        if (freq>0xF){
            freq = 0xF;
        }
        uint8_t newValue = freq<<4 | offTime;
        AUDIO_IO_Write(deviceAddr,0x1C, newValue);
        AUDIO_IO_Write(deviceAddr,0x1D, offTime<<6);
    }


#ifdef ARDUINO
    /// @brief Dumps all register values
    void printRegisters(Stream &out=Serial) {
        AUDIO_IO_Init();     
        for (int j=0;j< 0x34;j++){
            out.print("0x");
            out.print(j, HEX);
            out.print(" ");
            out.print("0x");
            out.println(AUDIO_IO_Read(deviceAddr, j),HEX);
        }
    }
#endif

protected:
    uint16_t deviceAddr;

    // convert to range from 0.0 - 1.0 ->  0 - 100.
    uint32_t toIntVol(float vol){
        if (vol>1.0) return 100;
        if (vol<0.0) return 0;
        return vol * 100;
    }


};


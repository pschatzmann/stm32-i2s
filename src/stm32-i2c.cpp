/**
 * Implements the AUDIO_IO I2C API calls using the Wire library
 * @author Phil Schatzmann
*/
#ifdef ARDUINO

#include "Wire.h"
#include "stm32-common.h"

extern "C" {

/* AUDIO IO functions */
void AUDIO_IO_Init(void){
#ifdef STM32
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    if (I2C_CLOCK_FREQ>0) Wire.setClock(I2C_CLOCK_FREQ);
#endif
    //Wire.setTimeout(10000);
    Wire.begin();
}

void AUDIO_IO_DeInit(void) {
    Wire.end();
}

void AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value){
    Wire.beginTransmission(Addr);
    Wire.write((uint8_t)Reg);
    int rc0 = Wire.endTransmission(false);
    if (rc0!=0){
        STM32_LOG("Wire.endTransmission 1 failed: %d", rc0);
    }
    Wire.beginTransmission(Addr);
    Wire.write((uint8_t)Value);
    int rc1 = Wire.endTransmission();
    if (rc1!=0){
        STM32_LOG("Wire.endTransmission failed: %d", rc1);
    }
}

int AUDIO_IO_Read(uint8_t Addr, uint8_t Reg){
    Wire.beginTransmission(Addr);
    Wire.write((uint8_t)Reg);
    int rc = Wire.endTransmission(false);
    if (rc!=0){
        STM32_LOG("Wire.endTransmission (read) failed: %d", rc);
        return -1;
    }
    Wire.requestFrom((int)Addr, 1);
    int byte = Wire.read();
    if (byte<0){
        STM32_LOG("Wire.read failed");
    }
    return byte;
}

}

#endif
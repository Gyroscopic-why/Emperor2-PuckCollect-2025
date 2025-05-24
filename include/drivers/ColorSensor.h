#pragma once

#include <Arduino.h>
#include "MultiWire.h"

#define TCS34725_ADDRESS 0x29
#define REQUEST_TCS34725_ID 0x12 // 0x44 0x4D

#define TCS34725_SET_GAIN 0x0F
#define TCS34725_SET_INTEGRATIONTIME 0x01

#define TCS34725_ENABLE 0x00

#define TCS34725_ENABLE_AEN 0x02
#define TCS34725_ENABLE_PON 0x01

#define TCS34725_COMAND 0x80

#define TCS34725_GAIN_1X 0x00
#define TCS34725_GAIN_4X 0x01
#define TCS34725_GAIN_16X 0x02
#define TCS34725_GAIN_60X 0x03

#define TCS34725_INTEGRATIONTIME_2_4MS 0xFF
#define TCS34725_INTEGRATIONTIME_24MS 0xF6
#define TCS34725_INTEGRATIONTIME_50MS 0xEB
#define TCS34725_INTEGRATIONTIME_60MS 0xE7
#define TCS34725_INTEGRATIONTIME_101MS 0xD6
#define TCS34725_INTEGRATIONTIME_120MS 0xCE
#define TCS34725_INTEGRATIONTIME_154MS 0xC0
#define TCS34725_INTEGRATIONTIME_180MS 0xB5
#define TCS34725_INTEGRATIONTIME_199MS 0xAD
#define TCS34725_INTEGRATIONTIME_240MS 0x9C
#define TCS34725_INTEGRATIONTIME_300MS 0x83
#define TCS34725_INTEGRATIONTIME_360MS 0x6A
#define TCS34725_INTEGRATIONTIME_401MS 0x59
#define TCS34725_INTEGRATIONTIME_420MS 0x51
#define TCS34725_INTEGRATIONTIME_480MS 0x38
#define TCS34725_INTEGRATIONTIME_499MS 0x30
#define TCS34725_INTEGRATIONTIME_540MS 0x1F
#define TCS34725_INTEGRATIONTIME_600MS 0x06
#define TCS34725_INTEGRATIONTIME_614MS 0x00

#define TCS34725_GET_C 0x14
#define TCS34725_GET_R 0x16
#define TCS34725_GET_G 0x18
#define TCS34725_GET_B 0x1A

struct Color
{
    int16_t r, g, b, c;

    Color(int16_t r, int16_t g, int16_t b, int16_t c = 0)
    {
        this->r = r;
        this->g = g;
        this->b = b;
    }

    Color()
    {
        r = 0;
        g = 0;
        b = 0;
        c = 0;
    }
};

class TCS34725ColorSensor
{
private:
    IWire *_wire;

    uint8_t _gain, _integrationTime;

public:
    TCS34725ColorSensor(IWire *wire, uint8_t gain = TCS34725_GAIN_1X, uint8_t integrationTime = TCS34725_INTEGRATIONTIME_2_4MS)
    {
        _wire = wire;
        _gain = gain;
        _integrationTime = integrationTime;
    }

    bool isConnected()
    {
        _wire->write8(TCS34725_ADDRESS, TCS34725_COMAND | REQUEST_TCS34725_ID);

        uint8_t data = _wire->read();

        if ((data != 0x4D) && (data != 0x44) && (data != 0x10))
            return false;

        return true;
    }

    void setGain(uint8_t gain)
    {
        if (gain != _gain)
        {
            _wire->write2x8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_SET_GAIN, gain);

            _gain = gain;
        }
    }

    void setIntegrationTime(uint8_t integrationTime)
    {
        if (integrationTime != _integrationTime)
        {
            _wire->write2x8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_SET_INTEGRATIONTIME, integrationTime);

            _integrationTime = integrationTime;
        }
    }

    void begin()
    {
        _wire->beginTransmission(TCS34725_ADDRESS);

        _wire->write(TCS34725_COMAND | TCS34725_SET_GAIN);
        _wire->write(_gain);

        _wire->write(TCS34725_COMAND | TCS34725_SET_INTEGRATIONTIME);
        _wire->write(_integrationTime);

        _wire->write(TCS34725_COMAND | TCS34725_ENABLE);
        _wire->write(TCS34725_ENABLE_PON);

        _wire->endTransmission();

        delay(3);

        _wire->write2x8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    }

    Color readRawRGB(){
        Color resultColor;
        uint8_t buffer[2];

        _wire->write8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_GET_R);
        _wire->requestFrom(TCS34725_ADDRESS, 2);
        _wire->readBytes(buffer, 2);

        resultColor.r = (uint16_t)(buffer[0]) | ((uint16_t)(buffer[1]) << 8);
        
        _wire->write8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_GET_G);
        _wire->requestFrom(TCS34725_ADDRESS, 2);
        _wire->readBytes(buffer, 2);

        resultColor.g = (uint16_t)(buffer[0]) | ((uint16_t)(buffer[1]) << 8);
        
        _wire->write8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_GET_B);
        _wire->requestFrom(TCS34725_ADDRESS, 2);
        _wire->readBytes(buffer, 2);

        resultColor.b = (uint16_t)(buffer[0]) | ((uint16_t)(buffer[1]) << 8);

        _wire->write8(TCS34725_ADDRESS, TCS34725_COMAND | TCS34725_GET_C);
        _wire->requestFrom(TCS34725_ADDRESS, 2);
        _wire->readBytes(buffer, 2);

        resultColor.c = (uint16_t)(buffer[0]) | ((uint16_t)(buffer[1]) << 8);

        return resultColor;
    }

    Color readRGB(){
        Color raw = readRawRGB();

        uint16_t c = raw.c;

        if(c == 0)
            return Color();

        return Color((float)(raw.r) / c * 255, (float)(raw.g) / c * 255, (float)(raw.b) / c * 255);
    }
};
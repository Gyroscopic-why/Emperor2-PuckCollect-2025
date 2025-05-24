#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareWire.h>

#define DEFAULT_HARD_WIRE_TIMEOUT 4
#define DEFAULT_SOFT_WIRE_TIMEOUT 10

class IWire
{
public:
    virtual void begin();
    virtual void setClock(uint32_t);
    virtual void beginTransmission(uint8_t) const = 0;
    virtual void endTransmission() const = 0;
    virtual void write(uint8_t) const = 0;
    virtual uint8_t read() const = 0;
    virtual void requestFrom(uint8_t, uint8_t) const = 0;
    virtual void readBytes(uint8_t *, size_t) const = 0;
    virtual uint16_t available() const = 0;

    void write8(uint8_t addr, uint8_t data) const
    {
        beginTransmission(addr);
        write(data);
        endTransmission();
    }

    void write2x8(uint8_t addr, uint8_t data1, uint8_t data2) const
    {
        beginTransmission(addr);
        write(data1);
        write(data2);
        endTransmission();
    }
};

class HardwareWire : public IWire
{
public:
    void begin()
    {
        Wire.setTimeout(DEFAULT_HARD_WIRE_TIMEOUT);
        Wire.begin();
    }

    void setTimeout(uint32_t timeout){
        Wire.setTimeout(timeout);
    }

    void setClock(uint32_t clock)
    {
        Wire.setClock(clock);
    }

    void beginTransmission(uint8_t addr) const
    {
        Wire.beginTransmission(addr);
    }

    void endTransmission() const
    {
        Wire.endTransmission();
    }

    void write(uint8_t data) const
    {
        Wire.write(data);
    }

    uint8_t read() const
    {
        return Wire.read();
    }

    void requestFrom(uint8_t addr, uint8_t count) const
    {
        Wire.requestFrom(addr, count);
    }

    void readBytes(uint8_t *buffer, size_t length) const
    {
        Wire.readBytes(buffer, length);
    }

    uint16_t available() const
    {
        return Wire.available();
    }
};

class SoftWire : public IWire
{
    SoftwareWire *_softWire;

public:
    SoftWire(uint8_t sda, uint8_t scl)
    {
        _softWire = new SoftwareWire(sda, scl);
    }

    void setTimeout(uint32_t timeout){
        _softWire->setTimeout(timeout);
    }

    void begin()
    {
        _softWire->setTimeout(DEFAULT_SOFT_WIRE_TIMEOUT);
        _softWire->begin();
    }

    void setClock(uint32_t clock)
    {
        _softWire->setClock(clock);
    }

    void beginTransmission(uint8_t addr) const
    {
        _softWire->beginTransmission(addr);
    }

    void endTransmission() const
    {
        _softWire->endTransmission();
    }

    void write(uint8_t data) const
    {
        _softWire->write(data);
    }

    uint8_t read() const
    {
        return _softWire->read();
    }

    void requestFrom(uint8_t addr, uint8_t count) const
    {
        _softWire->requestFrom(addr, count);
    }

    void readBytes(uint8_t *buffer, size_t length) const
    {
        _softWire->readBytes(buffer, length);
    }

    uint16_t available() const
    {
        return _softWire->available();
    }
};
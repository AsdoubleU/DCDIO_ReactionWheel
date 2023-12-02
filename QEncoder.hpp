/* Qencoder Version 1.0 */
#pragma once  //instead of safe include guard
#include <mbed.h>
#include <iostream>
extern DigitalOut led1;
class QEncoder {
    public:
    QEncoder(PinName pinA, PinName pinB)
      : _pinA(pinA), _pinB(pinB),_bi(pinA,pinB){}
    void init()    {  // callbacks for all edges
        _pinA.rise(callback(this, &QEncoder::decode)); 
        _pinA.fall(callback(this, &QEncoder::decode));
        _pinB.rise(callback(this, &QEncoder::decode));
        _pinB.fall(callback(this, &QEncoder::decode));
        _previousState = _bi;
        _count = 0;
        _errorCount=0;
    }
    int32_t getCount()    {
        return _count;
    }
    void setCount(int32_t EncoderVal){
        _pinA.disable_irq (); _pinB.disable_irq ();
        _count = EncoderVal;
        _pinA.enable_irq ();_pinB.enable_irq ();
    }
    int32_t getErrorCount()    {
        return _errorCount;
    }     
private:
    InterruptIn _pinA;
    InterruptIn _pinB;
    BusIn _bi;
    uint8_t _previousState;
    volatile int32_t _count;
    volatile uint32_t _errorCount;
    void decode()    {
        uint8_t newState = (_pinA.read() << 1) | _pinB.read();
        switch((_previousState << 2) | newState) {
        case 0b0001: // 0x01 PREV|CURR
        case 0b0111: // 0x07
        case 0b1110: // 0x0E
        case 0b1000: // 0x08
            _count --; break;
        case 0b0010: // 0x02
        case 0b1011: // 0x0B
        case 0b1101: // 0x0D
        case 0b0100: // 0x04
            _count ++; break;
        default:
            _errorCount++; break;
        } 
        _previousState = newState; 
        led1=!led1;
        if(_count == 8200) {_count = 0;}
        else if(_count == -8200) {_count = 0;}
    } 
};
#pragma once
#include "Arduino.h"
    
class Solo  {
protected:
    enum Command  {
        SetAddress             = 0x01,
        SetCommandMode         = 0x02,
        SetCurrentLimit        = 0x03,
        SetTorqueReference     = 0x04,
        SetSpeedReference      = 0x05,
        SetPowerReference      = 0x06,
        MotorIdentification    = 0x07,
        StopSystem             = 0x08,
        SetPWMFrequency        = 0x09,
        SetSpeedControllerKp   = 0x0A,
        SetSpeedControllerKi   = 0x0B,
        SetMotorDirection      = 0x0C,
        SetResistance          = 0x0D,
        SetInductance          = 0x0E,
        SetNumberOfPoles       = 0x0F,
        SetEncoderLines        = 0x10,
        SetSpeedControlMode    = 0x11,
        ResetAllAddresses      = 0x12,
        ResetToFactory         = 0x14,
        SetMotorType           = 0x15,
        SetControlMode         = 0x16,
        SetCurrentControllerKp = 0x17,
        SetCurrentControllerKi = 0x18,
        EnableDisableMotor     = 0x19,
        GetAddress             = 0x81,
        GetVoltageA            = 0x82,
        GetVoltageB            = 0x83,
        GetCurrentA            = 0x84,
        GetCurrentB            = 0x85,
        GetBusVoltage          = 0x86,
        GetDCCurrent           = 0x87,
        GetDCVoltage           = 0x88,
        GetSpeedControllerKp   = 0x89,
        GetSpeedControllerKi   = 0x8A,
        GetPWMFrequency        = 0x8B,
        GetCurrentLimit        = 0x8C,
        GetQuadratureCurrent   = 0x8D,
        GetDirectCurrent       = 0x8E,
        GetNumberOfPoles       = 0x8F,
        GetEncoderLine         = 0x90,
        GetCurrentControllerKp = 0x91,
        GetCurrentControllerKi = 0x92,
        GetTemperature         = 0x93,
        GetResistance          = 0x94,
        GetInductance          = 0x95,
        GetSpeed               = 0x96,
        GetMotorType           = 0x97,
        // TODO: ox98 empty?
        GetSpeedControlMode    = 0x99,
        GetCommandMode         = 0x9A,
        GetControlMode         = 0x9B
    };
    
public:
    // I am so sorry to break convention here, but some idiots used preprocessor defines for ANALOG and DIGITAL.
    enum CommandMode       { Analog=0    , Digital=1 };
    enum MotorDirection    { Clockwise=0 , Counterclockwise=1 };
    enum SpeedControlMode  { Sensorless=0, Encoders=1 };
    enum MotorType         { DC=0, PMSM=1, ACIM=2, PMSM_ULTRAFAST=3 };   // NOTE: A brushless DC motor (BLDC) is actually a PMSM!
    enum ControlMode       { Speed=0, Torque=1, Position=2 };

private:
    Stream *stream;                              // e.g. Serial or Serial1, etc., used to communicate via UART
    byte addr;                                   // address (0..254)
    uint32_t read_result;
    
public:
    Solo(Stream *_stream, byte _addr=0);
    ~Solo();

protected:
    uint32_t floatToData(float f);
    float dataToFloat(uint32_t d);
    void write(Command command, uint32_t data);
    bool read(Command &command, uint32_t &data);    // reads into specified buffer
    //bool read();                                 // reads into object's buffer
    bool execute(Command command, uint32_t data);   // executes a command and returns true on success, false on failure

public:
    // ---------- write commands ----------
    bool setAddress(byte _addr);                 // careful! Will only respond to new address immediately!
    bool setCommandMode(CommandMode mode);
    bool setCurrentLimit(float limit);           // TODO: use custom float class?
    bool setTorqueReference(float reference);
    bool setSpeedReference(uint32_t reference);
    bool setPowerReference(float reference);
    // TODO: voidSetMotorIdentification
    bool stopSystem();                           // careful! power recycle necessary to restart!
    bool setPWMFrequency(byte frequency);
    bool setSpeedControllerKp(float Kp);
    bool setSpeedControllerKi(float Ki);
    bool setMotorDirection(MotorDirection direction);
    bool setResistance(float R);
    bool setInductance(float L);
    bool setNumberOfPoles(byte n);
    bool setEncoderLines(uint32_t n);
    bool setSpeedControlMode(SpeedControlMode mode);
    bool resetToFactory();                       // careful! resets all settings, including address!
    bool setMotorType(MotorType type);
    bool setControlMode(ControlMode mode);
    bool setCurrentControllerKp(float Kp);
    bool setCurrentControllerKi(float Ki);
    //TODO: bool enableMonitorMode();            // TODO: Is this really useful in Arduino? Or will the received data spam the input buffer and cause more trouble than use?
    //TODO: bool disableMonitorMode();
    
    // ---------- read commands ----------
    bool getDeviceAddress(byte &address);
    bool getVoltageA(float &voltage);
    bool getVoltageB(float &voltage);
    bool getCurrentA(float &current);
    bool getCurrentB(float &current);
    bool getBusVoltage(float &voltage);
    bool getDCCurrent(float current);
    bool getDCVoltage(float voltage);
    bool getSpeedControllerKp(float &Kp);
    bool getSpeedControllerKi(float &Ki);
    bool getPWMFrequency(uint32_t frequency);
    bool getCurrentLimit(float &limit);
    bool getQuadratureCurrent(float &current);   // Iq
    bool getDirectCurrent(float &current);       // Id
    bool getNumberOfPoles(byte &n);
    bool getEncoderLine(uint32_t &n);
    bool getCurrentControllerKp(float &Kp);
    bool getCurrentControllerKi(float &Ki);
    bool getTemperature(float &temperature);
    bool getResistance(float &R);
    bool getInductance(float &L);
    bool getSpeed(uint32_t &reference);
    bool getMotorType(MotorType &type);
    bool getSpeedControlMode(SpeedControlMode &mode);
    bool getCommandMode(CommandMode &mode);
    bool getControlMode(ControlMode &mode);
    
    // ---------- static commands ----------
    static void resetAllAddresses();              // careful! resets all attached modules' addresses to default (0)
};

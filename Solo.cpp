#include <Solo.h>

// -------------------- constructor & destructor --------------------
Solo::Solo(Stream *_stream, byte _addr)  :  stream(_stream), addr(_addr), read_result(0)  {
    // TODO: init stream, set correct Baud rate
    if(dynamic_cast<HardwareSerial>(stream)!=nullptr || dynamic_cast<SoftwareSerial>(stream)!=nullptr)
        ; // TODO: stream is a UART interface
    else if(dynamic_cast<HardwareSPI>(stream)!=nullptr || dynamic_cast<SoftwareSPI>(stream)!=nullptr)
        ;  // TODO: stream is an SPI interface
    // TODO: further interface types?
    else
        ; // TODO: stream is an unknown interface type
}
Solo::~Solo()  {
    // TODO: This should never be destructed, should it?
    // However, what should the behavior be then?
}

// -------------------- utility --------------------
float Solo::dataToFloat(uint32_t data)  {  // TODO: test
    if(data <= 0x7FFE0000ul)
        return float(data) / 131072.0;
    else
        return float(0xFFFFFFFFul - data + 0x1ul) / 131072.0 * -1.0;
}
uint32_t Solo::floatToData(float f)  {  // TODO: test
    if(f>=0.0)
        return static_cast<uint32_t>(f*131072.0);
    else
        return 0xFFFFFFFFul - static_cast<uint32_t>(abs(f*131072.0));
}
void Solo::write(Command command, uint32_t data)  {// TODO: implement; TODO: test
    // TODO: implement
    // TODO: if(command==ResetAllAddresses) use addr 0xFF
}
bool Solo::read(Command &command, uint32_t &data)  {// TODO: implement; TODO: test
    // TODO: implement
    // TODO: check whether device address matches
        // TODO: behavior on address change?
}
bool Solo::execute(Command command, uint32_t data)  {  // TODO: test
    write(command, data);
    Command return_command;
    uint32_t return_data;
    if( ! read(return_command, return_data) )
        return false;
    if(command!=return_command)
        return false;
    if(return_data == 0xEEEEEEEE)
        return false;
    read_result = return_data;
    return true;
}

// -------------------- write commands --------------------
bool Solo::setAddress(byte _addr)  {// TODO: test
    if(_addr<0 || _addr>254)
        return false;
    if( ! execute(SetAddress, _addr) )
        return false;
    addr = _addr;  // we know the new address, so no reason to kill the connection
    return true;
}
bool Solo::setCommandMode(CommandMode mode) {// TODO: test
    return execute(SetCommandMode, mode);
}
bool Solo::setCurrentLimit(float limit)  {  // TODO: test
    if(limit<0.2 || limit>32)
        return false;
    return execute(SetCurrentLimit, floatToData(limit));
}
bool Solo::setTorqueReference(float reference)  {  // TODO: test
    if(reference<0.2 || reference>32)
        return false;
    return execute(SetTorqueReference, floatToData(reference));
}
bool Solo::setSpeedReference(uint32_t reference)  {  // TODO: test
    if(reference<0 || reference>30000)
        return false;
    return execute(SetSpeedReference, floatToData(reference));
}
bool Solo::setPowerReference(float reference)  {  // TODO: test
    if(reference<0 || reference>100)
        return false;
    return execute(SetPowerReference, floatToData(reference));
}
// TODO: voidSetMotorIdentification
bool Solo::stopSystem()  {  // TODO: test
    return execute(StopSystem,0x0);
}
bool Solo::setPWMFrequency(byte frequency)  {  // TODO: test
    if(frequency<8 || frequency>80)
        return false;
    return execute(SetPWMFrequency, frequency);
}
bool Solo::setSpeedControllerKp(float Kp)  {  // TODO: test
    if(Kp<0 || Kp>300)
        return false;
    return execute(SetSpeedControllerKp, floatToData(Kp));
}
bool Solo::setSpeedControllerKi(float Ki)  {  // TODO: test
    if(Ki<0 || Ki>300)
        return false;
    return execute(SetSpeedControllerKi, floatToData(Ki));
}
bool Solo::setMotorDirection(MotorDirection direction)  {  // TODO: test
    return execute(SetMotorDirection, direction);
}
bool Solo::setResistance(float R)  {  // TODO: test
    if(R<0.001 || R>50)
        return false;
    return execute(SetResistance, floatToData(R));
}
bool Solo::setInductance(float L)  {  // TODO: test
    if(L<0.00001 || L>50)
        return false;
    return execute(SetInductance, floatToData(L));
}
bool Solo::setNumberOfPoles(byte n)  {  // TODO: test
    if(n<1 || n>80)
        return false;
    return execute(SetNumberOfPoles, n);
}
bool Solo::setEncoderLines(uint32_t n)  {  // TODO: test
    if(n<0 || n>40000)
        return false;
    return execute(SetEncoderLines, n);
}
bool Solo::setSpeedControlMode(SpeedControlMode mode)  {  // TODO: test
    return execute(SetSpeedControlMode, mode);
}
bool Solo::resetToFactory()  {  // TODO: test    // careful! resets all settings, including address!
    return execute(ResetToFactory,0x01);
}
bool Solo::setMotorType(MotorType type)  {  // TODO: test
    return execute(SetMotorType, type);
}
bool Solo::setControlMode(ControlMode mode)  {  // TODO: test
    return execute(SetControlMode, mode);
}
bool Solo::setCurrentControllerKp(float Kp)  {  // TODO: test
    if(Kp<0 || Kp>16000)
        return false;
    return execute(SetCurrentControllerKp, floatToData(Kp));
}
bool Solo::setCurrentControllerKi(float Ki)  {  // TODO: test
    if(Ki<0 || Ki>16000)
        return false;
    return execute(SetCurrentControllerKi, floatToData(Ki));
}
//TODO: bool Solo::enableMonitorMode()  {  // TODO: implement; TODO: test

//TODO: bool Solo::disableMonitorMode()  {  // TODO: implement; TODO: test

// -------------------- read commands --------------------
bool Solo::getDeviceAddress(byte &address)  {  // TODO: test
    if( ! execute(GetAddress, 0x0) )
        return false;
    address = read_result;
}
bool Solo::getVoltageA(float &voltage)  {  // TODO: test
    if( ! execute(GetVoltageA, 0x0) )
        return false;
    voltage = dataToFloat(read_result);
}
bool Solo::getVoltageB(float &voltage)  {  // TODO: test
    if( ! execute(GetVoltageB, 0x0) )
        return false;
    voltage = dataToFloat(read_result);
}
bool Solo::getCurrentA(float &current)  {  // TODO: test
    if( ! execute(GetCurrentA, 0x0) )
        return false;
    current = dataToFloat(read_result);
}
bool Solo::getCurrentB(float &current)  {  // TODO: test
    if( ! execute(GetCurrentB, 0x0) )
        return false;
    current = dataToFloat(read_result);
}
bool Solo::getBusVoltage(float &voltage)  {  // TODO: test
    if( ! execute(GetBusVoltage, 0x0) )
        return false;
    voltage = dataToFloat(read_result);
}
bool Solo::getDCCurrent(float current)  {  // TODO: test
    if( ! execute(GetDCCurrent, 0x0) )
        return false;
    current = dataToFloat(read_result);
}
bool Solo::getDCVoltage(float voltage)  {  // TODO: test
    if( ! execute(GetDCVoltage, 0x0) )
        return false;
    voltage = dataToFloat(read_result);
}
bool Solo::getSpeedControllerKp(float &Kp)  {  // TODO: test
    if( ! execute(GetSpeedControllerKp, 0x0) )
        return false;
    Kp = dataToFloat(read_result);
}
bool Solo::getSpeedControllerKi(float &Ki)  {  // TODO: test
    if( ! execute(GetSpeedControllerKi, 0x0) )
        return false;
    Ki = dataToFloat(read_result);
}
bool Solo::getPWMFrequency(uint32_t frequency)  {  // TODO: test
    if( ! execute(GetPWMFrequency, 0x0) )
        return false;
    frequency = read_result;
}
bool Solo::getCurrentLimit(float &limit)  {  // TODO: test
    if( ! execute(GetCurrentLimit, 0x0) )
        return false;
    limit = dataToFloat(read_result);
}
bool Solo::getQuadratureCurrent(float &current)  {  // TODO: test  
    if( ! execute(GetQuadratureCurrent, 0x0) )
        return false;
    current = dataToFloat(read_result);
} // Iq
bool Solo::getDirectCurrent(float &current)  {  // TODO: test    
    if( ! execute(GetDirectCurrent, 0x0) )
        return false;
    current = dataToFloat(read_result);
}   // Id
bool Solo::getNumberOfPoles(byte &n)  {  // TODO: test
    if( ! execute(GetNumberOfPoles, 0x0) )
        return false;
    n = read_result;
}
bool Solo::getEncoderLine(uint32_t &n)  {  // TODO: test
    if( ! execute(GetEncoderLine, 0x0) )
        return false;
    n = read_result;
}
bool Solo::getCurrentControllerKp(float &Kp)  {  // TODO: test
    if( ! execute(GetCurrentControllerKp, 0x0) )
        return false;
    Kp = dataToFloat(read_result);
}
bool Solo::getCurrentControllerKi(float &Ki)  {  // TODO: test
    if( ! execute(GetCurrentControllerKi, 0x0) )
        return false;
    Ki = dataToFloat(read_result);
}
bool Solo::getTemperature(float &temperature)  {  // TODO: test
    if( ! execute(GetTemperature, 0x0) )
        return false;
    temperature = dataToFloat(read_result);
}
bool Solo::getResistance(float &R)  {  // TODO: test
    if( ! execute(GetResistance, 0x0) )
        return false;
    R = dataToFloat(read_result);
}
bool Solo::getInductance(float &L)  {  // TODO: test
    if( ! execute(GetInductance, 0x0) )
        return false;
    L = dataToFloat(read_result);
}
bool Solo::getSpeed(uint32_t &reference)  {  // TODO: test
    if( ! execute(GetSpeed, 0x0) )
        return false;
    reference = read_result;
}
bool Solo::getMotorType(MotorType &type)  {  // TODO: test
    if( ! execute(GetMotorType, 0x0) )
        return false;
    type = static_cast<MotorType>(read_result);
}
bool Solo::getSpeedControlMode(SpeedControlMode &mode)  {  // TODO: test
    if( ! execute(GetSpeedControlMode, 0x0) )
        return false;
    mode = static_cast<SpeedControlMode>(read_result);
}
bool Solo::getCommandMode(CommandMode &mode)  {  // TODO: test
    if( ! execute(GetCommandMode, 0x0) )
        return false;
    mode = static_cast<CommandMode>(read_result);
}
bool Solo::getControlMode(ControlMode &mode)  {  // TODO: test
    if( ! execute(GetControlMode, 0x0) )
        return false;
    mode = static_cast<ControlMode>(read_result);
}
    
// -------------------- static commands --------------------
void Solo::resetAllAddresses()  {  // TODO: implement; TODO: test
    //return execute(ResetAllAddresses, 0x00); address=0xFF
}

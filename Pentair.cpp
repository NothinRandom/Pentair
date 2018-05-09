#include "Pentair.h"
#include "Streaming.h"

Pentair::Pentair()
{
	_address = 0;
	velocity = 0;
	_mode = false;
}

Pentair::Pentair(uint8_t address)
{
	if(address < 1)
		address = 1;
	else if (address > 16)
		address = 16;
	_address = address - 1;
	if(_setpoint < 10) //1PSI
		_setpoint = 50; //5 psi
	velocity = 0;		
}

//address, //address of the pump 
//type, //boolean for choosing type
//status //boolean for enabling
Pentair::Pentair(uint8_t address, boolean type, boolean status)
{
	if(address < 1)
		address = 1;
	else if (address > 16)
		address = 16;
	_address = address - 1;
	_pType = type;
	_enabled = status;
	if(_setpoint < 10) //1 psi
		_setpoint = 50; //5 psi
	setType(_pType);	
	velocity = 0;	
}

void Pentair::setChannel(uint8_t channel)
{
	_channel = channel;
}

byte Pentair::getChannel()
{
	if(_channel - 14 > 128) //overflow
		return 0;
	return _channel - 14;
}

void Pentair::setSampling(int channel)
{
	_channel = channel;
}

int Pentair::getSampling()
{
	return _sampling;
}

//default automatic
void Pentair::setStatus(byte enable)
{
	_enabled = enable;
}

byte Pentair::getStatus()
{
	return _enabled;
}

//default automatic
void Pentair::setMode(boolean type)
{
	_mode = type;
}

boolean Pentair::getMode()
{
	return _mode;
}

//set pressure range for pumps
void Pentair::setRange(int High)
{
	_inHigh = High * 10;	
}

//get range of PSI
byte Pentair::getRange()
{
	return _inHigh/10;
}

//get current pressure
double Pentair::getPressure()
{
	return _psi;
}

boolean Pentair::fault()
{
	return _fault;
}

//enable the pump
void Pentair::begin()
{
//VF: FF 00 FF A5 00 60 10 05 01 06 10 21	
//VS: FF 00 FF A5 00 61 10 06 01 0A 01 26	
					   //VS   //VF
	byte output[12] = {0xFF,  //0  
					   0x00,  //1
					   0xFF,  //2 //require first two?
					   0xA5,  //3
					   0x00,  //4
					   0x60,  //5
					   0x10,  //6
					   0x06,  //7 0x05
					   0x01,  //8
					   0x0A,  //9 0x06
					   0x01,  //10
					   0x26}; //11 0x21	
	_integral = Integral;
	_derivative = Derivative;
	if(_pType == 2) //VF
	{	
		output[7] = 0x05;
		output[9] = 0x06; //priming
		output[11] = 0x21;
		_derivative += Derivative1; //broader window
	}
	output[5] = basePump + _address;
	output[11] += _address;
	_length = 12;	
	for(byte i = 0; i < _length; i++)
		pOut[i] = output[i];	
	_fault = false;			
}

//e-Stop the pump
void Pentair::end()
{	
//VF: FF 00 FF A5 00 60 10 05 00 01 1A	
//VS: FF 00 FF A5 00 61 10 06 01 04 01 21					
	byte length;
	length = 12; //default for VS
					 //VS   //VF
	byte output[] = {0xFF,  //0 
					 0x00,  //1
					 0xFF,  //2 //require first two?
					 0xA5,  //3
					 0x00,  //4
					 0x60,  //5
					 0x10,  //6
					 0x06,  //7 //0x05
					 0x01,  //8 //0x00
					 0x04,  //9 //0x01
					 0x01,  //10 //0x1A
					 0x20 };//11	
	output[5] = basePump + _address;						
	if(_pType == 2) //VF
	{
		length = 11;	
		output[7] = 0x05;
		output[8] = 0x00;
		output[9] = 0x01;
		output[10] = 0x1A + _address;		
	}
	else
		output[11] += _address;	
	_length = length;		
	for(byte i = 0; i < _length; i++)
		pOut[i] = output[i];		
	velocity = 0; //reset 	
}

//set up Pentair with initial PSI setpoint
void Pentair::init() //ignore for now
{
	if(_setpoint < 0 || _setpoint > _inHigh)
		_setpoint = _inHigh/2;
} 

//setpoint of the pump or psi?
void Pentair::setSetpoint(int setpoint)
{
	_setpoint = setpoint * 10;	
}

//setpoint of the pump or psi?
byte Pentair::getSetpoint()
{
	return _setpoint/10; //return psi
}

//read pump status
uint32_t Pentair::read(uint8_t ch)
{ 
	//need to implement!
	return ch;
} 

//using PID to calculate speed (RPM/GPM) 
int Pentair::getSpeed()
{
	boolean fault = false;
	_pressure = map(analogRead(_channel), _inLow, inHigh, 0, _inHigh); //read and scale	
	_psi = (double)(_pressure/10.0);
	velocity += (_setpoint - _pressure)*_integral/_derivative; //fastest and simplest PID algorithm I can think of
	if(_pressure < 5 || _pressure > _inHigh - 5) //if pressure falls below 0.5 psi or pressure is greater than (maxPSI - 0.5psi)
	{
		fault = true;
		velocity = (4*_inHigh)/5; //run at 80%
/* 		if(_pType)
			velocity /= 2; //run at 40% instead? */
	}		
	else if(velocity < 0)
	{
		velocity = 0;
		fault = true;		
	}
	else if(velocity >= _inHigh)
	{
		velocity = _inHigh;
		fault = true;
	}
	_fault = fault;
	_speed = map(velocity, 0, _inHigh, _outLow, _outHigh);
	if(_pType == 1) //VS
		_speed *= 10; //multiples of 10				
	return _speed;
}

void Pentair::setSpeed(int speed)
{
	if(_mode) //manual
	{
		if(speed < _outLow)
			speed = _outLow; 
		else if(speed > _outHigh) //outside range
			speed = _outHigh;
	}
	else //false for automatic
		speed = getSpeed(); //get speed from PSI sensing	
	_speed = speed;
	calculate(_speed);
}

int Pentair::readSpeed()
{
	return _speed;
}

String Pentair::report()
{
	String output = "";
	char buffer[6]; //-99.9 psi
	byte length = 4;
	if(_psi < 10.0 && _psi > 0)
		length = 3;
	output += "#";
	output += _address + 1;
	output += ": ";
	output += dtostrf(_psi, length, 1, buffer);
	output += " PSI, ";
	output += readSpeed(); //second call, first is setSpeed
	if(_pType == 2) //VF
		output += " GPM, VF\n"; 
	else if (_pType == 1)
		output += " RPM, VS\n";
	return output;
}

//return as byte array
void Pentair::calculate(int speed)
{	
	byte output[] = {0XFF,	//0
					 0X00, 	//1
					 0XFF, 	//2 //require first two?
					 0XA5, 	//3
					 0X00, 	//4
					 0X60, 	//5 PUMP ADDRESS
					 0X10, 	//6
					 0X01, 	//7
					 0X04, 	//8
					 0X02, 	//9
					 0XC4, 	//10 E4 for VF
					 0X01, 	//11 UPPER BYTE OF speed, //00 for VF
					 0XF4,  //12 LOWER BYTE OF speed //speed for VF
					 0X02,  //13
					 0XD5}; //14 ENCRYPTION KEY
    output[5] = basePump + _address;
	output[11] = (speed >> 8) & 0xFF; //will be 0 for VF 
	output[12] = speed & 0xFF;
	if(_pType == 2) //VF
	{
		output[10] = 0xE4;
		output[14] = output[12] + _address;		
	}
	else if(_pType == 1)//VS
		output[14] = ((baseKey + _address + 0x01 + speed - 400) % 0xFF); //cleaner, but is way slower
	//something to think about:
	// output[14] = ((baseKey + _address + 0x01 + speed - 400);
	// output[14] = (output[14] & 0xff) + (output[14]>>8); //should be equivalent, but faster 
	_length = 15;	
	for(byte i = 0; i < _length; i++)
		pOut[i] = output[i];	
}

byte Pentair::getType()
{
	return _pType;
}

void Pentair::setType(byte type)
{
	_pType = type;
	if(_pType == 2) //VF: GPM
	{
		_outLow = 15;
		_outHigh = 130;	
	}
	else if(_pType == 1) //VS: RPM
	{
		_outLow = 40; //since need to be multiples of 10
		_outHigh = 345;	
	}	
}

byte Pentair::pLength()
{
	return _length;
}

void Pentair::setSensor(byte input)
{
	_sType = input;
	_inLow = 205; //offset
	if(input < 1) //1-5, and 4-20 require offset
		_inLow = 0;
}

byte Pentair::getSensor()
{
	return _sType;
}
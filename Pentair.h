#ifndef Pentair_h
#define Pentair_h

#if (ARDUINO >= 100)
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define VS 0
#define VF 1
#define basePump 0x60
#define baseKey 0x70
#define Integral 4
#define Derivative 10
#define Derivative1 6
#define inHigh 1023

class Pentair
{
	public:
		//initialize
		Pentair();
		Pentair(uint8_t address);
		Pentair(uint8_t address, byte type, byte status);
		void init(); //initialize Pentair
		void begin(); //		
		//type of pump: VS/VF
		byte getType();
		void setType(byte type);
		//range of pressure sensor
		void setRange(int High);
		byte getRange();
		//
		double getPressure(); //return current PSI
		boolean fault();
		//auto or manual, needs to be implemented
		void setMode(boolean type);
		boolean getMode();
		//
		void setSetpoint(int setpoint);
		byte getSetpoint();
		int getSpeed();
		void setSpeed(int input);
		byte pOut[15]; //max length of all transmissions...hopefully
		byte pLength(); //get length for pointer
		//ADC channel to read from
		void setChannel(uint8_t channel);
		byte getChannel();
		//if we want to assign timer for each pump
		void setSampling(int value);
		int getSampling();
		//enable or disable pump
		void setStatus(byte enable);
		byte getStatus();
		void end(); //stop pump, like e-Stop		
		uint32_t read(uint8_t pump); //read pump status: need to implement	
		String report();
		void setSensor(byte input);
		byte getSensor(); //return the sensor type
		int readSpeed();
		int _speed;	
		byte _length;		
		double _psi;			
	private:
		uint8_t _address; //base address + offset address of pump
		uint8_t _channel;
		uint8_t _integral;
		uint8_t _derivative;
		int _pressure;
		int _sampling;
		int _setpoint;	
		boolean _mode;
		byte _enabled;
		boolean _fault;	
		byte _pType; //pump type	
		//pressure setup
		int _inHigh;
		//RPM/GPM setup
		int _inLow;
		int _outLow; 
		int _outHigh;
		byte _sType; //sensor type
		int velocity;
		void calculate(int input);			
};
#endif
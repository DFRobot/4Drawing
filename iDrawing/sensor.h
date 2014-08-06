/******************************************************************************

                        MCU Board Testing Software
  
  Copyright (C) <2014>  <DFRobot>   www.dfrobot.com
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Contact: leo.yan.cn@gmail.com

 ******************************************************************************
  Author        : Leo Yan
  Created       : 2014/6
  Last Modified :
  Description   :  The library is depend on Arduino environment. 
  Function List :

******************************************************************************/


#ifndef _SENSOR_H
#define _SENSOR_H

#include "Arduino.h"
#include "public.h"


#define SENSOR_MAXNUM 8 



class Sensor 
{
public:
    Sensor();
    Sensor* creatObject( uint8_t ID, uint8_t pin );
    Sensor* getObject( uint8_t pin );

    virtual void start();
    virtual void stop();    
    virtual void run();
    virtual uint8_t getValue(){ return 255; };
    
protected:

    uint8_t attachPin; 
    uint8_t value;       
    boolean runEnable; 

private:


    typedef struct
    {
        uint8_t attachPin;   //keyword
        Sensor *pObject;
    }SensorObject_stru;  

    uint8_t objectNum;
    SensorObject_stru objectList[SENSOR_MAXNUM];


    void rigisterObject( uint8_t pin, Sensor *pObject );


};



class MCUTime : public Sensor 
{
public:
    MCUTime(uint8_t pin);

    void start();
    void run();
    void stop();
    uint8_t getValue();

private:
    uint8_t  secondFromStart;    
    uint16_t startTime;  //unit: ms
};


class PinDigitalSensor : public Sensor 
{
public:
    PinDigitalSensor(uint8_t pin);

    void start();
    void run();
    void stop();
    uint8_t getValue();

private:
    uint8_t lastValue;

};


class PinAnalogSensor : public Sensor 
{
    public:
        PinAnalogSensor(uint8_t pin);
    
        void start();
        void run();
        void stop();
        uint8_t getValue();

};

#endif

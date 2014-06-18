/******************************************************************************
              4Drawing software - Actuator
 
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


#ifndef _ACTUATOR_H
#define _ACTUATOR_H


#include "arduino.h"
#include "public.h"
#include "Timer.h"
#include "Player.h"

#define ACTUATOR_MAXMUM 16


#define DIGITAL_WRITE drawing_DigitalWrite
#define ANALOG_WRITE drawing_AnalogWrite



class Actuator 
{
public:
    Actuator();
    Actuator* creatObject( uint8_t ID, uint8_t pin );
    Actuator* getObject( uint8_t pin );

    virtual void start( void *pData, byte dataLength ){};
    virtual void stop();
    virtual void run();
    boolean getRunEnable(){return runEnable;};

    void setMakeMode( uint8_t mode ){ makeMode = mode;};

#if _DEBUG
    virtual void printAttribute(){};
#endif

protected:
    uint8_t attachPin; 
    boolean runEnable; 

    uint8_t makeMode;  //accord to switch state 


private:
    typedef struct
    {
        uint8_t attachPin;   //keyword
        Actuator *pObject;
    }ActuatorObject_stru;  

    uint8_t objectNum;
    ActuatorObject_stru objectList[ACTUATOR_MAXMUM];

    void rigisterObject( uint8_t pin, Actuator *pObject );


};



class LED : public Actuator 
{
public:
    LED( uint8_t pin );

    void start( void *pData, byte dataLength );
    void stop();
    void run();

#if _DEBUG
    void printAttribute();
#endif

private:
    uint8_t lightMode;    
    uint16_t halfPeriod;  //unit: ms

    int16_t lightDuty;  //range: 0-255,   using signed to check over
    uint8_t maxDuty;  //
    uint8_t numPerStep;  //the duty change value every time
    uint8_t stepsPerRun;  //
    uint8_t stepCnt;  //
    int8_t stepDirection;  //direction: -1 or 1

    uint8_t initState;  //HIGH or LOW

    
    void setAttribute( void *pData, byte dataLength );

};


/****/
#define USART_BAUD 9600


class MP3Music : public Actuator 
{
public:
    MP3Music( uint8_t pin, uint8_t playerID );

    void start( void *pData, byte dataLength );
    void run();
    void stop();
#if _DEBUG
    void printAttribute();
#endif


private:
    Player * pPlayerObj;
    uint8_t playMode;     //specify number;   random 
    uint16_t digitalSongName;  //unit: ms

    void setAttribute( void *pData, byte dataLength );
};


/******/
class PinDigitalActuator: public Actuator 
{
public:
    PinDigitalActuator( uint8_t pin );

    void start( void *pData, byte dataLength );
    void stop();
    void run();

#if _DEBUG
    void printAttribute();
#endif

private:

    uint8_t mode;    
    uint8_t pulseCnt;    

    void setAttribute( void *pData, byte dataLength );

};



extern void drawing_DigitalWrite( uint8_t pin, uint8_t val );
extern void drawing_AnalogWrite( uint8_t pin, uint8_t duty );


#endif

/******************************************************************************

                        4Drawing software - Sensor
  
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



#include "common.h"
#include "sensor.h"

Sensor::Sensor()
{
    runEnable = false;
    objectNum = 0;
}

Sensor* Sensor::creatObject( uint8_t ID, uint8_t pin)
{
    Sensor *pObject = NULL;


    /**search**/
    pObject = getObject( pin );

    if ( pObject )
    {
        return pObject;
    }

    if ( objectNum >= SENSOR_MAXNUM )
    {
        return NULL;
    }

    /**creat **/
    
    switch ( ID )
    {
        case FUNCTION_SENSOR_VIRTUALTIME:
            pObject = new MCUTime( PALETTE_PIN_VIRTUAL_TIME );
            
            break;

        case FUNCTION_SENSOR_DIGITAL:
            pObject = new PinDigitalSensor( pin );
            
            break;
            
        case FUNCTION_SENSOR_ANALOG:
            pObject = new PinAnalogSensor( pin );
            
            break;


        default:
           ; //nothing

    }


    /**store **/

    if ( pObject )
    {
        rigisterObject( pin, pObject );
        objectNum++;  //must be increase after Rigister
    }
    
    return pObject;  
}


Sensor* Sensor::getObject( uint8_t pin )
{
    uint8_t i;
    Sensor *pObject = NULL;
    

    for ( i = 0; i < objectNum ; i++ )
    {
        if ( pin == objectList[i].attachPin )
        {
            pObject = objectList[i].pObject;
            break;
        }
    }

    return pObject;

}


void Sensor::rigisterObject( uint8_t pin, Sensor *pObject )
{

    if ( objectNum >= SENSOR_MAXNUM )
    {
        return;
    }

    objectList[objectNum].attachPin = pin;
    objectList[objectNum].pObject = pObject;
}

void Sensor::start()
{
    uint8_t i;
    Sensor *pObject = NULL;


    for ( i = 0; i < objectNum ; i++ )
    {
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            pObject->start();
        }
    }

    runEnable = true;

}


void Sensor::run()
{
    uint8_t i;

    Sensor *pObject = NULL;

    if ( !runEnable )
    {
        return;
    }

    for ( i = 0; i < objectNum ; i++ )
    {
        //todo,  add balance
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            pObject->run();
        }
    }

    
}


void Sensor::stop()
{
    uint8_t i;   
    Sensor *pObject = NULL;

    //runEnable = false;

    for ( i = 0; i < objectNum ; i++ )
    {
        //todo,  add balance
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            pObject->stop();
        }
    }
}


/****/

MCUTime::MCUTime(uint8_t pin)
{
    attachPin = pin;
    secondFromStart = 0;
    startTime = (uint16_t)millis();
}

void MCUTime::start()
{
    secondFromStart = 0;
    startTime = (uint16_t)millis();
    runEnable = true;
}

void MCUTime::run()
{
    if ( runEnable )
    {
        secondFromStart = ( ((uint16_t)millis() - startTime)/1000 );
    }
}

void MCUTime::stop()
{
}


uint8_t MCUTime::getValue()
{
    return secondFromStart;
}


/******/
PinDigitalSensor::PinDigitalSensor(uint8_t pin)
{
    attachPin = pin;
}


void PinDigitalSensor::start()
{
    pinMode( attachPin, INPUT );

    value = digitalRead(attachPin);
    lastValue = value;
    runEnable = true;
}

void PinDigitalSensor::run()
{   
    uint8_t var;

    /*filter*/
    var = digitalRead(attachPin);
    
    if ( var == lastValue )
    {
        value = var;
    }

    lastValue = var;

}

void PinDigitalSensor::stop()
{
}


uint8_t PinDigitalSensor::getValue()
{
    return value;
}


/******/
PinAnalogSensor::PinAnalogSensor(uint8_t pin)
{
    attachPin = pin;
}


void PinAnalogSensor::start()
{
    runEnable = true;
}

void PinAnalogSensor::run()
{   
    uint16_t var;

    var = analogRead(attachPin);
    
    value = map(var, 0, 1023, 0, 100);   
}

void PinAnalogSensor::stop()
{
}


uint8_t PinAnalogSensor::getValue()
{
    return value;
}



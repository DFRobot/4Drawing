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



#include "common.h"
#include "actuator.h"




Actuator::Actuator( )
{
    objectNum = 0;
    runEnable = false;
    makeMode = SW_RUN;
}



Actuator* Actuator::creatObject( uint8_t ID, uint8_t pin)
{
    Actuator *pObject = NULL;

    /**search**/
    pObject = getObject( pin );

    if ( pObject )
    {
        return pObject;
    }

    if ( objectNum >= ACTUATOR_MAXMUM )
    {
        return NULL;
    }

    /**creat **/
    
    switch ( ID )
    {
        case FUNCTION_LED:
            pObject = new LED( pin );
            
            break;
            
        case FUNCTION_RGB:
            break;
            
        case FUNCTION_PLAYER_MINI:
            if ( PALETTE_PIN_UART == pin)
            {
                pObject = new MP3Music( pin, FUNCTION_PLAYER_MINI );
            }
            break;

        case FUNCTION_ACTUATOR_DIGITAL:
            pObject = new PinDigitalActuator( pin );


    }

    /**store **/

    if ( pObject )
    {
        rigisterObject( pin, pObject );
        objectNum++;
    }
    
    return pObject;

}


Actuator* Actuator::getObject( uint8_t pin )
{
    uint8_t i;
    Actuator *pObject = NULL;
    

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


void Actuator::rigisterObject( uint8_t pin, Actuator *pObject )
{

    if ( objectNum >= ACTUATOR_MAXMUM )
    {
        return;
    }

    objectList[objectNum].attachPin = pin;
    objectList[objectNum].pObject = pObject;
}


void Actuator::run()
{
    uint8_t i;
    Actuator *pObject = NULL;

    for ( i = 0; i < objectNum ; i++ )
    {     
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            pObject->run();
        }
    }
}


void Actuator::stop()
{
    uint8_t i;   
    Actuator *pObject = NULL;


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



/*****/

LED::LED( uint8_t pin )
{   
    attachPin = pin;

    /*default value*/    
    lightMode = LED_LIGHTMETHOD_FLASH;
    halfPeriod = random(150,500);
    initState = LOW;
}


void LED::setAttribute( void *pData, byte dataLength )
{
    uint8_t type, length, totalLength;
    uint8_t *pDataByte;

    /****/
    
    /****/
    if ( NULL == pData)
    {
        return;
    }
    
    pDataByte = (uint8_t *)pData;
    totalLength = 0;

    do
    {
        type = *pDataByte++;
        length = *pDataByte++;

        totalLength += length+2;

        if( totalLength > dataLength )
        {
            break;
        }

        switch ( type )
        {

            case PARATYPE_RUNMETHOD_ID:
                #define DATA_TYPE uint8_t
                if ( sizeof(DATA_TYPE) == length )
                {
                    lightMode = *(DATA_TYPE*)pDataByte;
                }
                #undef DATA_TYPE
                break;

            case PARATYPE_PERIOD_ID:
                #define DATA_TYPE uint16_t

                if ( sizeof(DATA_TYPE) == length )
                {
                    halfPeriod = *(DATA_TYPE*)pDataByte;
                    halfPeriod >>= 1;

                    DBG_ASSERT( halfPeriod >= PALETTE_TIMER_PERIOD );
                    DBG_ASSERT( halfPeriod < (PALETTE_TIMER_PERIOD * 255) );
                }
                #undef DATA_TYPE
                break;

             case PARATYPE_INIT_STATE_ID:
                #define DATA_TYPE uint8_t
                if ( sizeof(DATA_TYPE) == length )
                {
                    initState = *(DATA_TYPE*)pDataByte;
                }
                #undef DATA_TYPE
                break;

               
             default:
                ;//nothing
        }

        pDataByte += length;

    }while(totalLength < dataLength);
    
}

#if _DEBUG
void LED::printAttribute( )
{
    Serial.print( "LED: pin=" );
    Serial.print( attachPin );
    Serial.print( ",halfPeriod=");
    Serial.print( halfPeriod );
    Serial.println("");
}
#endif


#define LED_FADE_STEPS 1  //it is the divisor of 255
#define LED_FLASH_STEPS 255  //it is the divisor of 255


void LED::start( void *pData, byte dataLength )
{
    uint8_t steps;

    
    setAttribute( pData, dataLength );

    maxDuty = (SW_MAKE == makeMode)? 150 : 255;

    lightDuty = (LOW == initState)? 0 : maxDuty;
    
    stepCnt = 0;

    steps = (halfPeriod/PALETTE_TIMER_PERIOD);

    if (LED_LIGHTMETHOD_FLASH == lightMode)
    {
        stepsPerRun = steps;
    }
    else
    {
        stepsPerRun = 1; 
    }

    numPerStep = (uint16_t)(maxDuty*(uint16_t)stepsPerRun)/steps ;
    
    DBG_PRINTLN_VAR(maxDuty, DEC);

    DBG_ASSERT( numPerStep > 0 );

    runEnable = true;

   
}

void LED::stop()
{   
    runEnable = false;
    DIGITAL_WRITE( attachPin, LOW );
}

void LED::run()
{
    uint8_t duty;
    
    if ( !runEnable )
    {
        return;
    }
    
    stepCnt++;
    if ( stepCnt < stepsPerRun )
    {
        return;
    }
    
    stepCnt = 0;
    
#if _DEBUG
    //Serial.print(" Pin=");
    //Serial.print(attachPin);
    //Serial.print(":lightDuty=");
    //Serial.println(lightDuty);
#endif
    
    // todo: maxDuty = (SW_MAKE == g_switchState) ? MAKING_LED_DUTY : 255;

    duty = lightDuty;

    ANALOG_WRITE( attachPin, duty );
    
    
    if ( lightDuty <= 0 ) 
    {
        stepDirection = 1; 
    }
    else if ( lightDuty >= maxDuty )
    {
        stepDirection = -1; 
    }
    else
    {
        //nothing
    }

    lightDuty = lightDuty + (stepDirection * numPerStep);

    if ( lightDuty < 0 )
    {
        lightDuty = 0;
    }
    else if ( lightDuty > maxDuty )
    {
        lightDuty = maxDuty;
    }


}

/**MP3 module**/

MP3Music::MP3Music( uint8_t pin, uint8_t playerID)
{
    HardwareSerial* pPlayerSerial = NULL;

#if defined(UBRR1H)
    pPlayerSerial = &Serial1;
#else
    pPlayerSerial = &Serial;
#endif

    attachPin = pin;

    
    switch( playerID )
    {
        case FUNCTION_PLAYER_MINI:
            pPlayerObj = new DFPlayerMini(pPlayerSerial, USART_BAUD);
            break;

        default:
            pPlayerObj = NULL;
            break;
    }
           
    playMode = PLAYMODE_RANDOM;   

}



void MP3Music::setAttribute(void *pData, uint8_t dataLength)
{

    uint8_t type, length, totalLength;
    uint8_t *pDataByte;

    /****/
    if ( NULL == pData)
    {
        return;
    }

    pDataByte = (uint8_t *)pData;
    totalLength = 0;

    do
    {
        type = *pDataByte++;
        length = *pDataByte++;

        totalLength += length+2;

        if( totalLength > dataLength )
        {
            break;
        }

        switch ( type )
        {

            case PARATYPE_RUNMETHOD_ID:
                #define DATA_TYPE uint8_t
                if ( sizeof(DATA_TYPE) == length )
                {
                    playMode = *(DATA_TYPE*)pDataByte;
                }
                #undef DATA_TYPE
                break;

            case PARATYPE_INDEX_ID:
                #define DATA_TYPE uint8_t

                if ( sizeof(DATA_TYPE) == length )
                {
                    digitalSongName = *(DATA_TYPE*)pDataByte;
                }
                #undef DATA_TYPE
                break;

            default:
                ;//nothing

        }


        pDataByte += length;

    }while(totalLength < dataLength);



    
}



void MP3Music::start( void *pData, byte dataLength )
{

#if defined(UBRR1H)
    Serial1.begin(USART_BAUD);
#else
    Serial.begin(USART_BAUD);
#endif
        
    setAttribute( pData, dataLength);

    pPlayerObj->setVolume( (SW_MAKE == makeMode)? 60 : 98 );

    delay(DELAY_BETWEENCMD);


    /**if mode **/
    if ( RUNMETHOD_RANDOM == playMode )
    {
        pPlayerObj->setMode(PLAYMODE_RANDOM);
        pPlayerObj->play();
    }
    else
    {
        pPlayerObj->setMode(PLAYMODE_NORMAL);
        pPlayerObj->play( String(digitalSongName) );
    }


    runEnable = true;
}

void MP3Music::run()
{

    if ( !runEnable )
    {
        return;
    }

    pPlayerObj->receiveProc();
 
}


void MP3Music::stop()
{
    runEnable = false;

    pPlayerObj->stop();


#if defined(UBRR1H)
    Serial1.end();
#else
    Serial.end();
#endif
    
}

#if _DEBUG
void MP3Music::printAttribute()
{ 
    return; 
}
#endif


/**Digital module**/
PinDigitalActuator::PinDigitalActuator( uint8_t pin)
{
    attachPin = pin;
    mode = RUNMETHOD_PULSE_POSITIVE;
}



void PinDigitalActuator::setAttribute(void *pData, uint8_t dataLength)
{

    uint8_t type, length, totalLength;
    uint8_t *pDataByte;

    /****/
    if ( NULL == pData)
    {
        return;
    }

    pDataByte = (uint8_t *)pData;
    totalLength = 0;

    do
    {
        type = *pDataByte++;
        length = *pDataByte++;

        totalLength += length+2;

        if( totalLength > dataLength )
        {
            break;
        }

        switch ( type )
        {

            case PARATYPE_RUNMETHOD_ID:
                #define DATA_TYPE uint8_t
                if ( sizeof(DATA_TYPE) == length )
                {
                    mode = *(DATA_TYPE*)pDataByte;
                }
                #undef DATA_TYPE
                break;

            default:
                ;//nothing

        }


        pDataByte += length;

    }while(totalLength < dataLength);



    
}



void PinDigitalActuator::start( void *pData, byte dataLength )
{
    setAttribute( pData, dataLength);

    if ( (RUNMETHOD_PULSE_POSITIVE == mode)
         || (RUNMETHOD_LOW == mode) )
    {
        DIGITAL_WRITE( attachPin, LOW );
    }
    else if( (RUNMETHOD_PULSE_NEGATIVE == mode)
         || (RUNMETHOD_HIGH == mode) )
    {
        DIGITAL_WRITE( attachPin, HIGH );
    }

    pinMode( attachPin, OUTPUT );


    pulseCnt = 0;

    runEnable = true;
}

void PinDigitalActuator::run()
{

    if ( !runEnable )
    {
        return;
    }


    /**mode **/

    if ( RUNMETHOD_PULSE_POSITIVE == mode )   
    {
        if ( pulseCnt < 2) //todo: magic number
        {
            pulseCnt++;
            DIGITAL_WRITE( attachPin, ( (1 == pulseCnt)? HIGH : LOW ) );  
        }
    }
    else if ( RUNMETHOD_PULSE_NEGATIVE == mode )
    {
        if ( pulseCnt < 2) //todo: magic number
        {
            pulseCnt++;
            DIGITAL_WRITE( attachPin, ( (1 == pulseCnt)? LOW : HIGH ) );  
        }
    }

}


void PinDigitalActuator::stop()
{
    runEnable = false;

    digitalWrite( attachPin, LOW);  //set pin to high-impendance
    pinMode( attachPin, INPUT );
}

#if _DEBUG
void PinDigitalActuator::printAttribute()
{ 
    return; 
}
#endif




void drawing_DigitalWrite( uint8_t pin, uint8_t val )
{
    if( (5 == pin) || (6 == pin) || (11 == pin) )
    {
        digitalWrite( pin, !val );
    }
    else
    {
        digitalWrite( pin, val );
    }
}

void drawing_AnalogWrite( uint8_t pin, uint8_t duty )
{
    if( (5 == pin) || (6 == pin) || (11 == pin) )
    {
        duty = 255 - duty;
    }
    
    analogWrite( pin, duty );
}


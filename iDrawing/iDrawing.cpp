/******************************************************************************

                        4Drawing software - main
  
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



#include <avr/sleep.h>
#include "public.h"
#include "Timer.h"

#include "common.h"
#include "sensor.h"
#include "actuator.h"
#include "iDrawing.h"


extern void dettachUSB();
extern void PCattachInterrupt(uint8_t pin);
extern void PCdetachInterrupt(uint8_t pin);
extern void softBlink( uint16_t period, uint8_t times );

/*** ***/

Sensor *pRunStateTime = NULL;   //time is not timer

Timer timer;
Actuator actuator;
Sensor sensor;





/***   ***/

void timerHandle()
{
    sensor.run();
    actuator.run();
}



const uint8_t Palette::ioPort[PALETTE_PORT_NUM] = { 3,  6, 2, 4 };   //close MVCC last
const uint8_t Palette::ioDisableMask[PALETTE_PORT_NUM] = { 0b0, 0b0, 0b00010000, 0b0 };  //some pin doesn't need to disable



Palette::Palette()
{
    uint8_t i;

   
    for( i = 0; i < PALETTE_RULE_MAXNUM; i++ )
    {
        paletteRule[i].valid = false;
    }

    runDuration = 20;
    runOvertimeFlag = false;
    wakeCondition = WAKE_CONDITION;
    wakeFlag = false;


    switchState = SW_MAKE;

    sysState = SYS_NORMAL;

    /****/
     
}

void Palette::setDuration( uint8_t duration )
{
    runDuration = duration;
}

void Palette::setWakeCondition( uint8_t condition )
{
    wakeCondition = condition;
}

boolean Palette::setRule( PaletteRuleConfigRec_stru *pRuleConfig, uint8_t number )
{
    uint8_t i, j;
    uint8_t actionType;  //start,stop
    uint8_t attachPin, functionID;

    boolean ret;
    char strBuf[64];

    PaletteRuleConfigRec_stru *pConfig = NULL;

    /**get task from eeprom**/
    /*if nodata in eeprom*/
    
    if ( number > PALETTE_RULE_MAXNUM )
    {
        sprintf( strBuf, "Error:The number of Rules must less than %u", PALETTE_RULE_MAXNUM);
        showErrInfo( strBuf );

        return false;
    }

    for ( i = 0; i < number; i++ )
    {
        pConfig = pRuleConfig + i;
        actionType = pConfig->actionType;
        attachPin = pConfig->actuator.attachPin;
        functionID = pConfig->actuator.ID;

        if ( ACTION_RUN == actionType )
        {

            for ( j = i+1; j < number; j++ )
            {
                pConfig = pRuleConfig + j;

                if ( ACTION_RUN == pConfig->actionType )
                {
                    if ( (attachPin == pConfig->actuator.attachPin) && (functionID != pConfig->actuator.ID) )
                    {
                        sprintf( strBuf, "Error @Rule No.%u: same PIN with Rule No.%u", j+1, i+1);
                        showErrInfo( strBuf );

                        return false;
                    }
                }

            }
        }

    }

    ret = true;

    DBG_PRINTLN("*Set Rule*");

    
    for ( i = 0; i < number; i++, pRuleConfig++ )
    {

        DBG_PRINTLN_VAR(i, DEC);
       
        paletteRule[i].pTrigger = sensor.creatObject( pRuleConfig->trigger.ID, pRuleConfig->trigger.attachPin );
        DBG_PRINTLN_VAR((word)paletteRule[i].pTrigger, HEX);

        if( NULL == paletteRule[i].pTrigger )
        {
            ret = false;
            break;
        }


        paletteRule[i].triggerValue = pRuleConfig->triggerValue;
        paletteRule[i].triggerRelation = pRuleConfig->relation; 

        actionType = pRuleConfig->actionType;
        paletteRule[i].actionType = actionType; 

        if ( ACTION_RUN == actionType )
        {
            paletteRule[i].pActuator = actuator.creatObject(pRuleConfig->actuator.ID, pRuleConfig->actuator.attachPin);  
            paletteRule[i].actuatorAttributeLen = pRuleConfig->actuator.paraLen;
            paletteRule[i].pActuatorAttribute = pRuleConfig->actuator.pPara;
        }
        else if( ACTION_STOP == actionType )
        {
            paletteRule[i].pActuator = actuator.getObject(pRuleConfig->actuator.attachPin);  
        }
        else
        {
            paletteRule[i].pActuator = NULL;
        }

        DBG_PRINTLN_VAR((word)paletteRule[i].pActuator, HEX);

        if ( ACTION_SYS_SLEEP != actionType )
        {
            if( NULL == paletteRule[i].pActuator )
            {
                ret = false;
                break;
            }
        }
        
        paletteRule[i].valid = true;
        paletteRule[i].state = RULE_STATE_INIT; 
       

    }

    if ( false == ret )
    {
        sprintf( strBuf, "Error: Rule No.%u", i+1 );
        showErrInfo( strBuf );
    }

    return ret;

}


void Palette::init(PaletteRuleConfigRec_stru *pRuleConfig, uint8_t number)
{
    /**power on blink**/
    softBlink( 500, 3 );

    /**set rules**/
    (void)setRule(pRuleConfig, number);

    /** **/
    pRunStateTime = sensor.creatObject(FUNCTION_SENSOR_VIRTUALTIME, PALETTE_PIN_VIRTUAL_TIME);

    DBG_ASSERT( NULL != pRunStateTime );

    /** system module init **/      
    pinMode( PALETTE_PIN_PIR, INPUT );
    pinMode( PALETTE_PIN_SW, INPUT_PULLUP ); 

    delay(10);

    switchState = digitalRead(PALETTE_PIN_SW);

    enablePower();

    pinMode( PALETTE_PIN_BLK, OUTPUT );
    digitalWrite( PALETTE_PIN_BLK, HIGH);

    sensor.start();

    timer.every( PALETTE_TIMER_PERIOD, timerHandle );
}


void Palette::run()
{
    uint8_t i;
    uint8_t runNowTime;


    timer.update();

    /**Event process**/
    runNowTime = pRunStateTime->getValue();  //todo


    if( runNowTime >= runDuration )
    {
        runOvertimeFlag = true;
    }

    
    /**System state process**/
    /*rule state at ACTION_RUN:  INIT-(Trigger)->READY-(start Actuator)->DOING-( Actuator is stopped)->INIT */
    /*rule state at ACTION_STOP:  INIT-(Trigger)->READY-(stop Actuator)->DOING-( Actuator is started)->INIT */

    if( SYS_NORMAL == sysState )
    {
        /*execute user task */
        uint8_t realTriggerValue;
        uint8_t ruleTriggerValue;  //equal, more than, less than, transfer
        uint8_t ruleActionType;    
        Actuator *pActuator = NULL;  
       
        for( i = 0; i < PALETTE_RULE_MAXNUM; i++ )
        {

            if ( false == paletteRule[i].valid )
            {
                break;
            }

            
            uint8_t &ruleState = paletteRule[i].state;
           
            realTriggerValue = paletteRule[i].pTrigger->getValue();
            ruleTriggerValue = paletteRule[i].triggerValue;
            ruleActionType = paletteRule[i].actionType;
            pActuator = paletteRule[i].pActuator;


            if ( RULE_STATE_DOING == ruleState ) 
            {
                if ( NULL != pActuator )
                {

                    if ( ACTION_RUN == ruleActionType )
                    {
                        if ( !pActuator->getRunEnable())
                        {
                            ruleState = RULE_STATE_INIT;
                        }               
                    }
                    else // ( ACTION_STOP == ruleActionType )
                    {
                        if ( pRunStateTime != paletteRule[i].pTrigger )
                        { //time is only stop once
                            if ( pActuator->getRunEnable())
                            {
                                ruleState = RULE_STATE_INIT;
                            } 
                        }
                    }
                }
                else
                {
                    DBG_WARNING(0);
                }


            }
            else if ( RULE_STATE_INIT == ruleState )
            {

                switch (  paletteRule[i].triggerRelation )
                {
                    case RELATION_EQUAL:
                        if ( realTriggerValue == ruleTriggerValue )
                        {
                            ruleState = RULE_STATE_READY;
                        }
                        break;
                    case RELATION_MORE:
                        if ( realTriggerValue > ruleTriggerValue )
                        {
                            ruleState = RULE_STATE_READY;
                        }
                        break;
                    case RELATION_LESS:
                        if ( realTriggerValue < ruleTriggerValue )
                        {
                            ruleState = RULE_STATE_READY;
                        }
                        break;
                    case RELATION_ANY:
                        ruleState = RULE_STATE_READY;
                        break;
                    default:
                        DBG_WARNING(0);
                }

            }
            else if ( RULE_STATE_READY == ruleState )
            {

                DBG_PRINT(" Execute ruler-");
                DBG_PRINTLN(i);


                if ( ACTION_RUN == ruleActionType )
                {
                    pActuator->setMakeMode(switchState);
                    pActuator->start(paletteRule[i].pActuatorAttribute, paletteRule[i].actuatorAttributeLen);

                    DBG_RUNCODE(pActuator->printAttribute());
                }
                else if ( ACTION_STOP == ruleActionType )
                {
                    pActuator->stop();
                }
                else //ACTION_SYS_SLEEP
                {
                    runOvertimeFlag = true;
                    break;
                }

                ruleState = RULE_STATE_DOING;

            }
            else
            {
                //nothing
            }
        
        }

        /*state change*/
        if( runOvertimeFlag )
        {
            //todo, close peripheral ports and power
            sensor.stop();
            actuator.stop();

            disablePeripheral();
            sysState = SYS_LOCK;
            DBG_PRINTLN_VAR(sysState,HEX);
        }    
    }
    else if( SYS_LOCK == sysState )
    {
        if( runNowTime >= (runDuration + LOCK_TIME) )
        {
            if ( (SW_RUN == digitalRead(PALETTE_PIN_SW)) && (SW_MAKE == switchState) )
            {
#if !_DEBUG                
                dettachUSB();
#endif
                switchState = SW_RUN;
            }

            sysState = SYS_SLEEP;
            DBG_PRINTLN_VAR(sysState,HEX);
        }    
    }
    else //SYS_SLEEP
    {
        sleep();

        /*state change*/
        if( wakeFlag )
        {
            //recovery ports and open peripheral power
            enablePeripheral();
            sensor.start();

            runOvertimeFlag = false;

            for ( i = 0; i < PALETTE_RULE_MAXNUM; i++ )
            {
                if ( false == paletteRule[i].valid )
                {
                    break;
                }

                paletteRule[i].state = RULE_STATE_INIT;   
            }

            switchState = digitalRead(PALETTE_PIN_SW);           

            sysState = SYS_NORMAL;
            DBG_PRINTLN_VAR(sysState,HEX);
        }

        wakeFlag = false;
    }

 
}



void softBlink( uint16_t period, uint8_t times )
{
    uint8_t i, j;
    uint16_t halfPeriod;

    halfPeriod = period>>1;
    
    pinMode( PALETTE_PIN_BLK, OUTPUT );

    for ( i = 1; i <= times ; i++ )
    {
        for ( j = 0; j < 2; j++ )
        {
          digitalWrite( PALETTE_PIN_BLK, j );
          delay( halfPeriod );
        }
    }

    digitalWrite( PALETTE_PIN_BLK, LOW );
    pinMode( PALETTE_PIN_BLK, INPUT );
    
}


void Palette::showErrInfo( char *pInfo )
{
    uint8_t i;
    boolean state;
   
    pinMode( PALETTE_PIN_BLK, OUTPUT );

    Serial.begin(9600);

    state = HIGH;
    
    while(1)
    {
        for ( i = 0; i < 8; i++ )
        {
          digitalWrite( PALETTE_PIN_BLK, state );
          delay( 100 );
          state = !state;
        }

        if ( Serial )
        {
            Serial.println(pInfo);
        }
    }    
}




void Palette::disablePeripheral()
{
    uint8_t i;
	volatile uint8_t *out, *ddr;

    delay(50);
    
    /**close ports**/    
    for( i = 0; i < PALETTE_PORT_NUM; i++ )
    {
        ddr = portModeRegister(ioPort[i]);
        out = portOutputRegister(ioPort[i]);

        ioPreDDRValue[i] = *ddr;
        ioPrePORTValue[i] = *out;

        //DBG_PRINT(i);
        //DBG_PRINTLN_VAR(ioPreDDRValue[i],BIN);
        //DBG_PRINTLN_VAR(ioPrePORTValue[i],BIN);
        
        (*ddr) &= ioDisableMask[i];
        (*out) &= ioDisableMask[i];

    }


}

/**recovery MVCC**/
void Palette::enablePower()
{
    digitalWrite( PALETTE_PIN_MVCC, LOW );  
    pinMode( PALETTE_PIN_MVCC, OUTPUT );
    delay(1000);  //waiting power stable
}

void Palette::enablePeripheral()
{
    uint8_t i;
    volatile uint8_t *out, *ddr;

    enablePower();

    /**recovery ports**/    
    for( i = 0; i < PALETTE_PORT_NUM; i++ )
    {
        ddr = portModeRegister(ioPort[i]);
        out = portOutputRegister(ioPort[i]);

        *ddr = ioPreDDRValue[i];
        *out = ioPrePORTValue[i];
    } 

#if defined(USBCON)
    if( SW_RUN == switchState )
    {
    	USBDevice.attach();
    }
#endif   


}
  



void Palette::sleep()         // here we put the arduino to sleep
{

    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we 
     * choose the according 
     * sleep mode: SLEEP_MODE_PWR_DOWN
     * 
     */

     if( SW_RUN == switchState )
     {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
        
        sleep_enable();          // enables the sleep bit in the mcucr register

                             // so sleep is possible. just a safety pin 
     }                       

    /* Now it is time to enable an interrupt. We do it here so an 
     * accidentally pushed interrupt button doesn't interrupt 
     * our running program. if you want to be able to run 
     * interrupt code besides the sleep function, place it in 
     * setup() for example.
     * 
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
     * 
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */

    PCattachInterrupt( PALETTE_PIN_PIR );  

    if( SW_MAKE == switchState )
    {
        do
        {
            delay(1);
        }while(!wakeFlag);        
    }
    else
    {
        do
        {
            sleep_mode();   // here the device is actually put to sleep!!
	                    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
            delay(1);  //if there is no delay, wakeFlag always false, why? 
        }while(!wakeFlag);           
                   
        sleep_disable();         // first thing after waking from sleep:  disable sleep...                           
    }

    PCdetachInterrupt( PALETTE_PIN_PIR );  
}



void Palette::wake()
{
    if ( digitalRead(PALETTE_PIN_PIR) == wakeCondition )
    {
        wakeFlag = true;   
    }
}




/*** public funtion ***/




/*
 * attach an interrupt to a specific pin using pin change interrupts.
    pin =  arduino digital pin number
 */
void PCattachInterrupt(uint8_t pin) {
  uint8_t bit;
  uint8_t port;

  port = digitalPinToPort(pin);
  // map pin to PCIR register
  if (port != 2) {
    return;
  } 

  // set the mask
  bit = digitalPinToBitMask(pin);
  PCMSK0 |= bit;

  // enable the interrupt
  PCICR |= 0x01 << 0;
}

void PCdetachInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);

  // map pin to PCIR register
  if (port != 2) {
    return;
  } 


  // disable the mask.
  PCMSK0 &= ~bit;
  // if that's the last one, disable the interrupt.
  if ( 0 == PCMSK0 ) {
    PCICR &= ~(0x01 << 0);
  }
}


/***ISR thread***/

Palette Drawing;


ISR(PCINT0_vect)
{
    Drawing.wake();   
}



void dettachUSB()
{
    
#ifdef UDCON
    DBG_END;

	UDCON = 1;							// disable attach resistor
    USBCON = 1<<FRZCLK;                 // Disable USB interface & Disable PLL
	PLLCSR = 0;		   				    // Disable PLL
	UHWCON = 0;                      //Disable USB pad regulator
#endif
}




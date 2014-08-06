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


#ifndef _IDRAWING_H
#define _IDRAWING_H

#include "Arduino.h"
#include "public.h"
#include "Timer.h"
#include "sensor.h"
#include "actuator.h"




/**user can redefine**/
#define PALETTE_RULE_MAXNUM 16


#define WAKE_CONDITION HIGH         //HIGH or LOW
#define RUN_OVERTIME_DEFAULT 20  //[3-255], unit: s


/********************************************************************************/
#define LOCK_TIME 2              //[0-255], unit: s

#define MAKING_LED_DUTY 180 
#define MAKING_MP3_VOL 16

#define PALETTE_PORT_NUM 4


/**System state**/
typedef enum
{
    SYS_NORMAL,  //Running normaly. 
    SYS_LOCK,    //in order to prevent trigger frequently 
    SYS_SLEEP    //shutdown the power of peripheral and close all port of MCU.    substate: powerdown or idle according to SW state. 
}SysState_enum;



/**wake event define**/

//todo
#define WAKE_CONDITION_LOW  LOW  
#define WAKE_CONDITION_HIGH HIGH 
#define WAKE_CONDITION_CHANGE 0XFF 


/**Sensor Define**/
#define FUNCTION_NULL (255)

typedef struct
{
    uint8_t attachPin;   //keyword
    uint8_t ID;
}SensorConfigRec_stru;


/**Actuator Define**/

typedef struct
{
    uint8_t attachPin;   //keyword   
    uint8_t ID;
    uint8_t paraLen;
    void *pPara; //type,len, data 
}ActuatorConfigRec_stru;


/**Rulers,  the relation between Trigger and Actuator and Input**/
typedef struct
{
    SensorConfigRec_stru trigger;
    uint8_t triggerValue;
    uint8_t relation;  //equal, more than, less than, transfer
    uint8_t actionType;  //start,stop
    ActuatorConfigRec_stru actuator;
}PaletteRuleConfigRec_stru;



#define RELATION_EQUAL 0
#define RELATION_MORE 1
#define RELATION_LESS 2
#define RELATION_ANY 3

#define RULE_STATE_INIT 0
#define RULE_STATE_READY 1
#define RULE_STATE_DOING 2


#define ACTION_RUN 1
#define ACTION_STOP 0
#define ACTION_SYS_SLEEP 2



/**running Rulers**/

typedef struct
{
    boolean valid;        //false = blank record
    uint8_t state;    //state of ruler    

    Sensor *pTrigger;    //(triggerPin and event) is keyword
    uint8_t triggerValue;
    uint8_t triggerRelation;  //equal, more than, less than, transfer

    uint8_t actionType;  //run,stop

    Actuator *pActuator;  //
    uint8_t actuatorAttributeLen;
    void *pActuatorAttribute; 


}PaletteRuleRec_stru;


                       

/**global variate**/









class Palette 
{
public:
    Palette();
    void setDuration( uint8_t duration );
    void setWakeCondition( uint8_t condition );
    void init(PaletteRuleConfigRec_stru *pRuleConfig, uint8_t number);
    void run();
    void sleep();

    void wake();

    
protected:



private:

    uint8_t wakeCondition;   //HIGH or LOW
    uint8_t runDuration;
    boolean runOvertimeFlag;


    uint8_t switchState;
    boolean wakeFlag;

    SysState_enum sysState;


    PaletteRuleRec_stru paletteRule[PALETTE_RULE_MAXNUM];

    //uint8_t peripheralPinlist[] = { SCL, SDA, A0, A1, A2, 9, 0, 1, A3, A4, A5, 10 };
    static const uint8_t ioPort[PALETTE_PORT_NUM];   
    static const uint8_t ioDisableMask[PALETTE_PORT_NUM]; 
    
    uint8_t ioPreDDRValue[PALETTE_PORT_NUM];
    uint8_t ioPrePORTValue[PALETTE_PORT_NUM];


    boolean setRule( PaletteRuleConfigRec_stru *pRuleConfig, uint8_t number );
    void enablePeripheral();
    void disablePeripheral();
    void enablePower();
    void showErrInfo( char *pInfo );


};


/****/

extern Palette Drawing;



#endif

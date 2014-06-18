/******************************************************************************

                        4Drawing Demo
  
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
  Description   :  
  Function List :

******************************************************************************/

#include "public.h" 
#include "common.h"
#include "Timer.h"
#include "Player.h"
#include "iDrawing.h"




uint8_t actuatorLED5[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FLASH), PARATYPE_PERIOD(450)};
uint8_t actuatorLED6[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FLASH), PARATYPE_PERIOD(250)};
uint8_t actuatorLED11[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FADE), PARATYPE_PERIOD(2000)};
uint8_t actuatorLEDA3[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FLASH), PARATYPE_PERIOD(200)};
uint8_t actuatorLEDA4[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FLASH), PARATYPE_PERIOD(200), PARATYPE_INIT_STATE(HIGH)};
uint8_t actuatorLEDA5[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FLASH), PARATYPE_PERIOD(450), PARATYPE_INIT_STATE(HIGH)};
uint8_t actuatorLED10[] = { PARATYPE_RUNMETHOD(LED_LIGHTMETHOD_FADE), PARATYPE_PERIOD(1000), PARATYPE_INIT_STATE(HIGH)};
uint8_t actuatorplayer1[] = { PARATYPE_RUNMETHOD(RUNMETHOD_RANDOM)};





void setup() 
{
    DBG_BEGIN(9600);
    
    PaletteRuleConfigRec_stru defaultRule[] =
    {
        { {A0, FUNCTION_SENSOR_ANALOG }, 1, RELATION_LESS, ACTION_SYS_SLEEP },

        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 0, RELATION_EQUAL, ACTION_RUN, 
                {A3, FUNCTION_LED, sizeof(actuatorLEDA3), actuatorLEDA3} },

        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 0, RELATION_EQUAL, ACTION_RUN, 
                {A4, FUNCTION_LED, sizeof(actuatorLEDA4), actuatorLEDA4} },

        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 0, RELATION_EQUAL, ACTION_RUN, 
                {A5, FUNCTION_LED, sizeof(actuatorLEDA5), actuatorLEDA5} },
                
        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 0, RELATION_EQUAL, ACTION_RUN, 
                {10, FUNCTION_LED, sizeof(actuatorLED10), actuatorLED10} },

        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 1, RELATION_EQUAL, ACTION_RUN, 
                { 5, FUNCTION_LED, sizeof(actuatorLED5), actuatorLED5 } },
    
        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 1, RELATION_EQUAL, ACTION_RUN, 
                { 6, FUNCTION_LED, sizeof(actuatorLED6), actuatorLED6 } },
    
        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 1, RELATION_EQUAL, ACTION_RUN, 
                {11, FUNCTION_LED, sizeof(actuatorLED11), actuatorLED11} },

        { {PALETTE_PIN_VIRTUAL_TIME, FUNCTION_SENSOR_VIRTUALTIME }, 2, RELATION_EQUAL, ACTION_RUN, 
                {PALETTE_PIN_UART, FUNCTION_PLAYER_MINI, sizeof(actuatorplayer1), actuatorplayer1} },
                
    };


    Drawing.setWakeCondition(HIGH);
    Drawing.setDuration(20);

    Drawing.init( defaultRule, sizeof(defaultRule)/sizeof(PaletteRuleConfigRec_stru) );

}

void loop() 
{  
    Drawing.run();
}



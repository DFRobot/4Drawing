/******************************************************************************

                        DFPlayer Mini  demo
  
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


#include "common.h"
#include "Timer.h"
#include "Player.h"



Player * miniPlayer = new DFPlayerMini(&Serial1, 9600);
Timer time;


void play1()
{
    miniPlayer->play("6");
}

void play2()
{
   miniPlayer->play("1234");
}

void stop()
{
   miniPlayer->stop();
}


void setup() { 
    DBG_BEGIN(9600);  
    delay(500);
    
    time.after(2, play1);
    time.after(12,play2);
    time.after(22, stop);
    
    miniPlayer->setVolume(40);
}

// the loop routine runs over and over again forever:
void loop() {

    miniPlayer->receiveProc();
    
    time.update();
}





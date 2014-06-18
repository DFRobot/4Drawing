/******************************************************************************

                        DFPlayer Mini  software
  
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


#ifndef _PLAYER_H
#define _PLAYER_H

#include "arduino.h"
#include "public.h"





typedef enum PLAYMODE
{
    PLAYMODE_NORMAL,
    PLAYMODE_RANDOM,
    PLAYMODE_SINGLECYCLE,
    PLAYMODE_ALLCYCLE                 
}PLAYMODE_ENUM;




class Player
{

public :
    Player( HardwareSerial* , uint32_t baud);
    virtual void setMode (PLAYMODE_ENUM mode);
    virtual void setVolume( byte vol)= 0;
    virtual void play()= 0; 
    virtual void play(String name)= 0;
    virtual void playNext ()= 0; 
    virtual void playPrevious()= 0; 
    virtual void pause()= 0; 
    virtual void stop()= 0; 
    virtual uint8_t getSongTotalNum()= 0;   
    virtual void receiveProc()= 0;

protected:
    HardwareSerial* commSerial;
    PLAYMODE_ENUM playMode;

};





/****/
#define DELAY_BETWEENCMD 50

#define CMD_PLAYNEXT  0X01
#define CMD_PLAYPREVIOUS  0X02
#define CMD_PLAYBYINDEX  0X03
#define CMD_SET_VOLUME   0X06
#define CMD_PLAYBYINDEX_REPEAT  0X08
#define CMD_PLAY         0X0D
#define CMD_PAUSE        0X0E
#define CMD_PLAY_ALLCYCLE      0X11
#define CMD_PLAYBYNAME_MP3DIR  0X12
#define CMD_STOP         0X16

#define CMD_PLAY_RANDOM  0X18
#define CMD_PLAY_SINGLECYCLE  0X19


#define QUERY_FILENUM_TF   0X48

#define ACK_PLAYEND_TF     0X3D
#define ACK_ERROR          0X40
#define ACK_SUCCESS        0X41


typedef struct
{
    uint8_t start;
    uint8_t version;
    uint8_t dataLen;
    uint8_t command;
    boolean isFeedback;  //true: yes
    uint16_t para;
    uint16_t checkSum;
    uint8_t end;
}Frame_stru;



class DFPlayerMini : public Player
{

public :
    DFPlayerMini(HardwareSerial*, uint32_t baud);
    void setMode (PLAYMODE_ENUM mode);
    void setVolume( byte vol);
    void play(); 
    void play(String name);
    void playNext (); 
    void playPrevious(); 
    void pause(); 
    void stop(); 

    uint8_t getSendState();
    void receiveProc();

    uint8_t getSongTotalNum();


private:
    static uint8_t frameLen;
    uint8_t sendState;
    uint8_t receiveState;


    Frame_stru sendFrameData;   
    Frame_stru receiveFrameData;  

    	
    void sendCmd (uint8_t cmd, uint16_t para = 0 );

};






#endif




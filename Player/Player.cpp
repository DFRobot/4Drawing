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


#include "Player.h"

Player::Player(HardwareSerial* theSerial, uint32_t baud)
{
	commSerial = theSerial;
    commSerial->begin(baud);
    
    playMode = PLAYMODE_NORMAL;
}

void Player::setMode(PLAYMODE_ENUM mode)
{
    playMode = mode;
    DBG_PRINTLN_VAR(playMode,DEC);
}


/*** ***/


uint8_t DFPlayerMini::frameLen = sizeof(Frame_stru);

//
DFPlayerMini::DFPlayerMini (HardwareSerial* theSerial, uint32_t baud):Player(theSerial, baud)
{

    frameLen = sizeof(sendFrameData);
        
    sendFrameData.start = 0x7E;
    sendFrameData.version = 0xFF;
    sendFrameData.dataLen = 0x6;
    sendFrameData.isFeedback = (0 == _DEBUG) ? false : true;
    sendFrameData.end = 0xEF;
    
}

void DFPlayerMini::setMode(PLAYMODE_ENUM mode)
{ 
    stop();
    Player::setMode(mode);   
    delay(DELAY_BETWEENCMD);
}

//


void DFPlayerMini::sendCmd (uint8_t cmd, uint16_t para ) 
{
    uint8_t i;
    uint8_t *pByte = NULL;
	uint16_t checksum = 0;
    /****/
    sendFrameData.command = cmd;
    sendFrameData.para = para;

    
    /**checksum = complemental code of sum **/
    pByte = &sendFrameData.version;
    
    checksum = 0;
	for ( ; pByte < (uint8_t*)&sendFrameData.checkSum ; pByte++) 
    {
		checksum += *pByte;
	}
    
	sendFrameData.checkSum = ~checksum + 1;

    /****/
    HTON16(sendFrameData.para);
    HTON16(sendFrameData.checkSum);

    /****/

    pByte = (uint8_t*)&sendFrameData;

    DBG_PRINT("Send:");

	for ( i=0; i<frameLen; i++, pByte++ ) 
    {
        
        DBG_PRINT(*pByte);
        DBG_PRINT(" ");

		commSerial->write(*pByte);
        
	}

    DBG_PRINTLN("");
    
}

//
void DFPlayerMini::play() 
{
    switch ( playMode )
    {
        case PLAYMODE_NORMAL:
    	    sendCmd (CMD_PLAY);
            break;
            
        case PLAYMODE_RANDOM:
            sendCmd (CMD_PLAY_RANDOM);
            delay(DELAY_BETWEENCMD);
            playNext();

            break;
            
        case PLAYMODE_ALLCYCLE:
            sendCmd (CMD_PLAY_ALLCYCLE, 1);
            break;

        case PLAYMODE_SINGLECYCLE:
            sendCmd (CMD_PLAY);
            delay(DELAY_BETWEENCMD);
            sendCmd (CMD_PLAY_SINGLECYCLE, 0);
            break;

            
        default:
    	    sendCmd (CMD_PLAY);
            break;                       
    }
}

//
void DFPlayerMini::play(String name) 
{
    int32_t digitalName;
    
    digitalName = name.toInt();
    if ( (0 < digitalName) && (digitalName <= 0xFFFF ) )
    {
        sendCmd(CMD_PLAYBYNAME_MP3DIR, (uint16_t)digitalName);
        
        if( PLAYMODE_SINGLECYCLE == playMode )
        {
            delay(DELAY_BETWEENCMD);
            sendCmd(CMD_PLAY_SINGLECYCLE, 0);
        }      
    }
}

//
void DFPlayerMini::pause() {
	sendCmd (CMD_PAUSE);
}

//
void DFPlayerMini::stop() {
	sendCmd(CMD_STOP);
}

//
void DFPlayerMini::playNext () {
	sendCmd(CMD_PLAYNEXT);
}

//
void DFPlayerMini::playPrevious() {
	sendCmd (CMD_PLAYPREVIOUS);
}

void DFPlayerMini::setVolume( byte vol) 
{
    uint8_t val;
	
	if ( vol > 100 ) return;

    val = map(vol, 0, 100, 0, 30 );

    DBG_PRINT("vol:");
    DBG_PRINTLN(val);

	
	sendCmd(CMD_SET_VOLUME, val);
}


uint8_t DFPlayerMini::getSongTotalNum()
{
    sendCmd(QUERY_FILENUM_TF);
    return 0;
}


uint8_t DFPlayerMini::getSendState()
{
    return sendState;
}


void DFPlayerMini::receiveProc()
{
    
#if _DEBUG
    uint8_t inByte;

    while (commSerial->available() > 0) 
    {
      inByte = commSerial->read();

      DBG_PRINT("-Receive:");
      DBG_PRINTLN(inByte);
    }
#endif    
}


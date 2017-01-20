/**
* \file CRADA.h
* \brief CRADA master include file

*
* Copyright (c) Dynamax Inc, all rights reserved, 6/25/2013
*
* Created: 6/25/2014
* \author: Peter van Bavel
*
*/
/**
* \ingroup CRADA_Dynamax_Main
* 
*
* @{
	*/
#define ON true
#define OFF false


#define EEPROM_SERIAL_NUMBER 0		//32bit serial number
#define EEPROM_SENSOR_ID 4			//node ID, coded as two ascii digits, node id  from 0-255
#define EEPROM_SERIAL_COMMAND_MODE 6  //ff undefined, 00= not command mode, 01= command mode turned on
#define EEPROM_SOLAR_PANEL 7  //bool FF undefined, 00= trickle charge, 01= TopOff fast charge, 02 = No Charge 
#define EEPROM_UPDATE_INTERVAL 8 //int16 FFFF UNDEFINED, in minutes up to 60minutes  
#define EEPROM_VBAT_CALIBRATION 10// int16 FFFF undefined, 0x5929 nominal. 
#define EEPROM_DAC_OUTPUT 12// BOOL ff UNDEFINED, 00= don't update DAC, 01= do update DAC 
#define EEPROM_RADIO_OUTPUT 13// BOOL ff undefined 00= don't send data on radio, 01 do send it
#define EEPROM_DAC_OFFSET 14// int16 amount to be subtracted from DAC telemetry output in milidegrees, nominal 0x001e
#define EEPROM_SDI_MODE 16 //bool ff undefined, 00 no SDI mode, ordinary radio or telemetry analog, 01 = SDI serial mode
#define EEPROM_SDI_ADDRESS 17 //SINGLE BYTE ASCII ADDRESS
#define EEPROM_THERMISTOR 18 // if 1, thermistor is in battery 00 no battery
/*********  WARNING!!!!! update EEPROM Calculator spreadsheet EVERY time variables are added right before releasing to laboratory!!!*/


#define EMULATE 1 // is true so emulator is allowed to work,  otherwise for lowest power consumption comment out
#define WATCHCRYSTAL 1   //1 is true so using 32khz watch crystal for sleep clock
enum charge_mode_type
{
	charger_off=0,
	trickle=1,
	top_off=2,
	fast=3
};
enum cmd_type
{   //all supported types
	a=0,aI=1,question=2,aAb=3,aM=4,aDx=5,aMC=6,aXF=7,aXP=8,aXX=9,aXR=10,INVALID=11
	}; 
// -*- C++ -*-
/*!
 * @file  RoboCar.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "RoboCar.h"  
#include "robocarDll.h"
#define STRICT

#include <tchar.h>
#include <windows.h>
#include <windowsx.h>
#include <basetsd.h>
#include <mmsystem.h>
#include <math.h>
#include <stdio.h>  
#include "resource.h"
#include "PCANBasic.h"
#include <iostream>

using namespace std;

//デバッグ用変数
int tmpA,tmpB,tmpC;
unsigned char tma,tmb,tmc,tmd,tme;

//CAN関係
HANDLE CAN1Thread, CAN2Thread;

FILE *fp;
int logcounter = 0;


//Prius CAN関係(プリウスから出力する)
struct Prius_CAN
{
	int Sideway_acceleration_sensor;	// -:左方向,+:右方向
	int Acceleration_sensor;			// -:手前方向,+:踏込方向
	int Steering_Angle;				// 
	int Brakes_Pressflg;				// bit#7 : 0=pressed 1=released  ( 0x84 Not pressed / 0x04 Pressed )
	int Brakes_Strength;				// 0:out, 0x7F:pushed in 
	int EM_Current1;					// 12 bit signed number ( + discharge ) unit is 0.1A
	int EM_Current2;					// battery pack voltage, unit is 1V
	int Front_Wheels_R;				// value is speed in km/h * 100
	int Front_Wheels_L;				// value is speed in km/h * 100
	int Rear_Wheels_R;				// value is speed in km/h * 100
	int Rear_Wheels_L;				// value is speed in km/h * 100
	int Speed;						// 0x400 for every 10km/h, including signed value for backwards
	int Drive_Mode_Cruiseflg;			// 0x50=D 0x51=B 0x4D=P 0x4E=R 0x4F=N MSB=CruiseON/off 0x49|0x4B = Standby
	int Drive_Mode_Shiftflg;			// 0x13&0x14=D 0x10=P 0x11=R 0x12=N
	int Drive_Mode_Powerflg;			// 0x04=Powered 0x00=Standby
	int Gas_Pedal_Speed;				// Speed related ( ~0x150=10km/h ~0x300=20km/h ~0x700=50km/h )  ALSO, negative, when backing up !!!
	int Gas_Pedal;					// Gas Pedal ( 0x00 ~ 0xC8 )
	int ICE_Throttle;					// Throttle related ( requested rpm ?) ~MPG=(KMH*10000)/value 
	int ICE_Powerflg;					// 0 when no power by ICE.
	int ICE_Rpm_Target;				// Target RPM ( unit is 32 rpm )
	int Speed2;						// Value in km/h
	int Battery_Max_Amps_Discharge;	// max current for the battery discharge in Amps.
	int Battery_Max_Amps_Charge;		// max current for the battery charge in Amps.
	int Battery_SOC;					// 16 bit = SOC Value, unit is 0.5%
	int Battery_Min_Temp;				// Batt Temp Min. (C)
	int Battery_Max_Temp;				// Batt Temp Max.
	int Battery_Fault_Code;
	int Battery_Pack_Voltage;			// 16 bit pack voltage
	int EV_MODE;						// 0x00=Normal Drive  0x40=EVMode  0x80=EVDenied  Other=Cancelled
	int Interlock;					// bit#7 pressed/depressed
	int Engine_Coolant_Temp;			// value in Celsius, unit is 0.5C
	int Shift_Lever;					// 00=B 10=D 20=N 40=R 80=P
	int Shift_Transition;				// bit#7 transition
	int Lights_Front;					// 00=Off 10=Park 30=On 38=HighBeam
	int Lights_Transition;			// bit#7 transition
	int Lights_Instruments;			// bit#3 instruments 1=dimmed 0=normal ( 0x18/0x10 )
	int Gas_Gauge;					// value ( my recorded max is 0x28 )
	int Doors_flg;					// 00=Closed 04=Rear 40=Passenger 80=Driver open
	int Doors_Transition;				// bit#7 transition
	int Cruise_flg;					// 00=Off 10=On
	int Cruise_Transition;			// bit#7 transition
}Prius;


//HV Controller受信関係(get"出力"、set"入力する")
struct HV_Controller_CAN_GET
{
	int Priority_Error_Level;
	int Priority_Error_Code;
	int Drive_Mode_Drive;			//0x00 マニュアルモード, 0x10 プログラムモード
	int Drive_Mode_Cont;			//0x00 速度制御, 0x10 ペダルストローク制御
	int Drive_Mode_Override;		//0x00 オーバーライドEnable, 0x10 オーバーライドDisable
	int Drive_Mode_Servo;			//0x00 サーボOFF, 0x10 サーボON
	int Drive_Pedal_Input;			//アクセル入力値 0～4095 (0～1.0)
	int Drive_Pedal_Target;			//アクセル指令値 0～4095 (0～1.0)
	int Drive_Pedal_Actual;			//アクセル現在値 0～4095 (0～1.0)
	int Drive_Shift_Input;			//シフト入力値 0x00 B, 0x10 D, 0x20 N, 0x40 R, 0x80 P, 0x0F Start, 0xFF Unknown
	int Drive_Shift_Target;			//シフト指令値 0x00 B, 0x10 D, 0x20 N, 0x40 R, 0x80 P, 0x0F Start, 0xFF Unknown
	int Drive_Shift_Actual;			//シフト現在値 0x00 B, 0x10 D, 0x20 N, 0x40 R, 0x80 P, 0x0F Start, 0xFF Unknown
	int Drive_Speed_Target;			//速度指令値 0～18000 (0～180[km/h])
	int Drive_Speed_Actual;			//速度現在値 0～18000 (0～180[km/h])
	int Drive_Pedal_Raw1;			//アクセルストロークセンサ VPA1 [mV]
	int Drive_Pedal_Raw2;			//アクセルストロークセンサ VPA2 [mV]
	int Drive_Shift_Raw1;			//VSX1 [mV]
	int Drive_Shift_Raw2;			//VSX2 [mV]
	int Drive_Shift_Raw3;			//VSX3 [mV]
	int Drive_Shift_Raw4;			//VSX4 [mV]
	int Steer_Mode_Drive;			//0x00 マニュアルモード, 0x10 プログラムモード
	int Steer_Mode_Cont;			//0x00 トルク制御, 0x10 角度制御
	int Steer_Mode_Override;		//0x00 オーバーライドEnable, 0x10 オーバーライドDisable
	int Steer_Mode_Servo;			//0x00 サーボOFF, 0x10 サーボON
	int Steer_Angle_Target;			//角度指令値 -6660～+6660 (-666°～ +666°) 左がプラス
	int Steer_Angle_Actual;			//角度現在値 -6660～+6660 (-666°～ +666°) 左がプラス
	int Steer_Torque_Target;		//トルク指令値 -4096～4095 (-1.0～1.0) 左がプラス
	int Steer_Torque_Actual;		//トルク現在値 -4096～4095 (-1.0～1.0) 左がプラス
	int Steer_Torque_Raw1;			//TRQ1 [mV]
	int Steer_Torque_Raw2;			//TRQ2 [mV]
	int Brake_Pedal_Input;			//ブレーキ入力値 0～4095 (0～1.0)
	int Brake_Pedal_Target;			//ブレーキ指令値 0～4095 (0～1.0)
	int Brake_Pedal_Actual;			//ブレーキ現在値 0～4095 (0～1.0)
	int Brake_Pedal_SKS1;			//SKS1 [mV]
	int Brake_Pedal_SKS2;			//SKS2 [mV]
	int Brake_Pedal_PMC1;			//PMC1 [mV]
	int Brake_Pedal_PMC2;			//PMC2 [mV]
	int Brake_Pedal_PRL;			//PRL [mV]
	int Brake_Pedal_PFL;			//PFL [mV]
	int Brake_Pedal_PRR;			//PRR [mV]
	int Brake_Pedal_PFR;			//PFR [mV]
	int Common_Echo_Controller;		//コントローラ種別
	int Common_Echo_No;				//番号
	int Common_Config_Index;		//開始Index
	int Common_Config_Value1;		//値1
	int Common_Config_Value2;		//値2
	int Common_Config_Value3;		//値3
	int Common_Error_Level;
	int Common_Error_Code;
}HV_GET;


//HV Controller送信関係
struct HV_Controller_CAN_SET
{
	int Drive_Mode_Drive;			//0 マニュアルモード, 1 プログラムモード
	int Drive_Mode_Cont;			//0 速度制御, 1 ペダルストローク制御
	int Drive_Mode_Override;		//0 オーバーライドEnable, 1 オーバーライドDisable
	int Drive_Mode_Servo;			//0 サーボOFF, 1 サーボON
	double Drive_Velocity;			//0.00-180.00[km/h]
	double Drive_Pedal;				//0.000-1.000[-]
	int Drive_Shift;				//0: Start, 1: B, 2: D, 3: N, 4: R 
	int Steer_Mode_Drive;			//0 マニュアルモード, 1 プログラムモード
	int Steer_Mode_Cont;			//0 トルク制御, 1 角度制御
	int Steer_Mode_Override;		//0 オーバーライドEnable, 1 オーバーライドDisable
	int Steer_Mode_Servo;			//0 サーボOFF, 1 サーボON
	double Steer_Torque;			//-1.000-1.000[-]
	double Steer_Angle;				//-666.0-666.0[deg]
	double Brake_Strength;			//0.000-1.000[-]
	int Common_GET_Config_Index;	//開始Index
	int Common_GET_Config_num;		//数
	int Common_SET_Config_Index;	//開始Index
	int Common_SET_Config_Value;	//値
}HV_SET, HV_SET_OLD;



union short_to_BYTE{
        BYTE b[2];
        short s;
}S2B;
union WORD_to_BYTE{
        BYTE b[2];
        WORD w;
}W2B;


//システム用変数
static double etime=0,gtime=0,pgtime=0,looptime=0,mgtime=0;
static long cnt=0;
static int taketime[20];
static int sch=0;
static int writeflg=0;
static int firstff=0;
FILE *fparaco;
static char key;

char szClassName[]="Skeleton";
static HWND      hWnd;
static HDC  hDC,memDC,memDC_map;
static PAINTSTRUCT ps;
static HBITMAP hBitmap,hBit_map;
static HINSTANCE  hInst;

static UINT	TimerPeriod = 0;
static HWND		hWndPeriod;
BOOL	f_MMTimerPost = TRUE;
static MMRESULT	mID;
static TIMECAPS TimerCaps = { 0, 0 };

/*
//使用関数定義
void main();
void drawsysval(int);
unsigned long GetTimer();
void timing(int);
int TreadStart();
LRESULT CALLBACK WindowProc(HWND,UINT,WPARAM,LPARAM);
extern void CALLBACK TimerPeriodProc(UINT wID, UINT wUser, DWORD dwUser, DWORD dw1, DWORD dw2); //堀が勝手にstaticをexternに変えた

//PCAN使用関数定義
void CAN1Func();
void CAN2Func();
void drawCAN(int);



void CAN_init(){
	TPCANStatus result;
	char strMsg[256];
	DWORD dwParam;
	
	//Prius CAN 初期化
	result = CAN_Initialize(PCAN_PCIBUS1, PCAN_BAUD_500K);
	if(result != PCAN_ERROR_OK)
	{
		// An error occurred, get a text describing the error and show it
		memset(strMsg, 0 ,sizeof(strMsg));
		CAN_GetErrorText(result, 2, strMsg);
		MessageBox(NULL, strMsg, "CAN_init Error", MB_ICONERROR | MB_OK);
	}
	else
	{
		MessageBox(NULL, "PCAN-PCI (Ch-1) was initialized", "CAN_init Success", MB_ICONINFORMATION | MB_OK);
		CAN1Thread = CreateThread(NULL , 0 , (LPTHREAD_START_ROUTINE)CAN1Func , hWnd , 0 , &dwParam);
	}
}

//HV Controller 初期化
void HV_init(){    //←勝手に足したらエラーが消えた
	TPCANStatus result;
	char strMsg[256];
	DWORD dwParam;
	result = CAN_Initialize(PCAN_PCIBUS2, PCAN_BAUD_1M);
	if(result != PCAN_ERROR_OK)
	{
		// An error occurred, get a text describing the error and show it
		CAN_GetErrorText(result, 9, strMsg);
		MessageBox(NULL, strMsg, "CAN_init Error", MB_ICONERROR | MB_OK);
	}
	else
	{
		MessageBox(NULL, "PCAN-PCI (Ch-2) was initialized", "CAN_init Success", MB_ICONINFORMATION | MB_OK);
		memset(&HV_SET, 0, sizeof(HV_SET));
		memset(&HV_SET_OLD, 0, sizeof(HV_SET_OLD));
		CAN2Thread = CreateThread(NULL , 0 , (LPTHREAD_START_ROUTINE)CAN2Func , hWnd , 0 , &dwParam);
	}
}



void CAN1Func(){//Prius CAN 読込処理
	TPCANMsg msg;
	TPCANTimestamp timestamp;
	TPCANStatus result;
	char strMsg[256];

	WORD WORDdami;
	

	do{
		result = CAN_Read(PCAN_PCIBUS1,&msg,&timestamp);
		
		if(result == PCAN_ERROR_OK){
			switch(msg.ID){//"Almost Sure"まで実装
				case 0x0022://Sideway acceleration sensor
					Prius.Sideway_acceleration_sensor = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]) - (int)0x0200;
					break;
				case 0x0023://Acceleration sensor
					Prius.Acceleration_sensor = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]) - (int)0x0200;
					break;
				case 0x0025://Steering
					Prius.Steering_Angle = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					break;
				case 0x0030://Brakes
					{Prius.Brakes_Pressflg = (int)msg.DATA[7];
					Prius.Brakes_Strength = (int)msg.DATA[4];
					break;}
				case 0x003B://EM Current
					{Prius.EM_Current1 = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					Prius.EM_Current2 = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;}
				case 0x00B1://Front wheels
					{Prius.Front_Wheels_R = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					Prius.Front_Wheels_L = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;}
				case 0x00B3://Rear wheels
					{Prius.Rear_Wheels_R = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					Prius.Rear_Wheels_L = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;}
				case 0x00B4://Speed
					{Prius.Speed = (((int)msg.DATA[6] * 256) + (int)msg.DATA[5]);
					break;}
				case 0x0120://Drive Mode
					{Prius.Drive_Mode_Cruiseflg = (int)msg.DATA[4];
					Prius.Drive_Mode_Shiftflg = (int)msg.DATA[5];
					Prius.Drive_Mode_Powerflg = (int)msg.DATA[6];
					break;}
				case 0x0244://Gas Pedal / Speed
					{Prius.Gas_Pedal_Speed = (((int)msg.DATA[5] * 256) + (int)msg.DATA[4]);
					Prius.Gas_Pedal = (int)msg.DATA[6];
					break;}
				case 0x0348://ICE
					{Prius.ICE_Throttle = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					Prius.ICE_Powerflg = (int)msg.DATA[4];
					break;}
				case 0x03C8://ICE rpm
					{Prius.ICE_Rpm_Target = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;}
				case 0x03CA://Speed
					{Prius.Speed2 = (int)msg.DATA[2];
					break;}
				case 0x03CB://Battery
					{Prius.Battery_Max_Amps_Discharge = (int)msg.DATA[0];
					Prius.Battery_Max_Amps_Charge = (int)msg.DATA[1];
					Prius.Battery_SOC = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					Prius.Battery_Min_Temp = (int)msg.DATA[4];
					Prius.Battery_Max_Temp = (int)msg.DATA[5];
					break;}
				case 0x03CD://Battery
					{Prius.Battery_Fault_Code = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					Prius.Battery_Pack_Voltage = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;}
				case 0x0529://(Interlock) / EV MODE
					{Prius.EV_MODE = (int)msg.DATA[4];
					Prius.Interlock = (int)msg.DATA[0];
					break;}
				case 0x052C://Engine coolant temp
					{Prius.Engine_Coolant_Temp = (int)msg.DATA[1];
					break;}
				case 0x0540://Shift lever
					{Prius.Shift_Lever = (int)msg.DATA[1];
					Prius.Shift_Transition = (int)msg.DATA[0];
					break;}
				case 0x057F://Lights
					{Prius.Lights_Front = (int)msg.DATA[1];
					Prius.Lights_Transition = (int)msg.DATA[0];
					Prius.Lights_Instruments = (int)msg.DATA[2];
					break;}
				case 0x05A4://Gas Gauge
					{Prius.Gas_Gauge = (int)msg.DATA[1];
					break;}
				case 0x05B6://Doors
					{Prius.Doors_flg = (int)msg.DATA[2];
					Prius.Doors_Transition = (int)msg.DATA[0];
					break;}
				case 0x05C8://Cruise
					{Prius.Cruise_flg = (int)msg.DATA[2];
					Prius.Cruise_Transition = (int)msg.DATA[0];
					break;}
				default:
					break;
			}
		}
	}while((result & !PCAN_ERROR_QRCVEMPTY) == PCAN_ERROR_OK);

	CAN_GetErrorText(result, 9, strMsg);
	MessageBox(NULL, strMsg, "CAN_Read Error", MB_ICONINFORMATION | MB_OK);
}


void CAN2Func(){
	TPCANMsg msg;
	TPCANTimestamp timestamp;
	TPCANStatus result;
	char strMsg[256];


	do{
		result = CAN_Read(PCAN_PCIBUS2,&msg,&timestamp);
		
	
		if(result == PCAN_ERROR_OK){
			switch(msg.ID){
				case 0x0001://エラー通知
					HV_GET.Priority_Error_Level = (int)msg.DATA[0];
					HV_GET.Priority_Error_Code = (((int)msg.DATA[2] * 256) + (int)msg.DATA[1]);
					break;
				case 0x0061://ドライブ内部状態通知（制御モード）
					HV_GET.Drive_Mode_Drive = (int)msg.DATA[0];
					HV_GET.Drive_Mode_Cont = (int)msg.DATA[1];
					HV_GET.Drive_Mode_Override = (int)msg.DATA[2];
					HV_GET.Drive_Mode_Servo = (int)msg.DATA[3];
					break;
				case 0x0062://ドライブ内部状態通知（ペダル指示）
					HV_GET.Drive_Pedal_Input = (((int)msg.DATA[0] * 256) + (int)msg.DATA[1]);
					HV_GET.Drive_Pedal_Target = (((int)msg.DATA[2] * 256) + (int)msg.DATA[3]);
					HV_GET.Drive_Pedal_Actual = (((int)msg.DATA[4] * 256) + (int)msg.DATA[5]);
					break;
				case 0x0063://ドライブ内部状態通知（シフト）
					HV_GET.Drive_Shift_Input = (int)msg.DATA[0];
					HV_GET.Drive_Shift_Target = (int)msg.DATA[1];
					HV_GET.Drive_Shift_Actual = (int)msg.DATA[2];
					break;
				case 0x0064://ドライブ内部状態通知（速度指示）
					HV_GET.Drive_Speed_Target = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Drive_Speed_Actual = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;
				case 0x0065://ドライブ内部状態通知（アクセルセンサ）
					HV_GET.Drive_Pedal_Raw1 = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Drive_Pedal_Raw2 = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;
				case 0x0066://ドライブ内部状態通知（シフトRaw値）
					HV_GET.Drive_Shift_Raw1 = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Drive_Shift_Raw2 = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					HV_GET.Drive_Shift_Raw3 = (((int)msg.DATA[5] * 256) + (int)msg.DATA[4]);
					HV_GET.Drive_Shift_Raw4 = (((int)msg.DATA[7] * 256) + (int)msg.DATA[6]);
					break;
				case 0x00A1://ステアリング内部状態通知（制御モード）//使う
					HV_GET.Steer_Mode_Drive = (int)msg.DATA[0];
					HV_GET.Steer_Mode_Cont = (int)msg.DATA[1];
					HV_GET.Steer_Mode_Override = (int)msg.DATA[2];
					HV_GET.Steer_Mode_Servo = (int)msg.DATA[3];
					break;
				case 0x00A2://ステアリング内部状態通知（角度制御）
					HV_GET.Steer_Angle_Target = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Steer_Angle_Actual = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;
				case 0x00A3://ステアリング内部状態通知（トルク制御）
					HV_GET.Steer_Torque_Target = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Steer_Torque_Actual = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;
				case 0x00A4://ステアリング内部状態通知（トルクセンサ）
					HV_GET.Steer_Torque_Raw1 = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Steer_Torque_Raw2 = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					break;
				case 0x00E1://ブレーキ内部状態通知（ペダル指示）
					HV_GET.Brake_Pedal_Input = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Brake_Pedal_Target = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					HV_GET.Brake_Pedal_Actual = (((int)msg.DATA[5] * 256) + (int)msg.DATA[4]);
					break;
				case 0x00E2://ブレーキ内部状態通知（ペダルセンサ）
					HV_GET.Brake_Pedal_SKS1 = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Brake_Pedal_SKS2 = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					HV_GET.Brake_Pedal_PMC1 = (((int)msg.DATA[5] * 256) + (int)msg.DATA[4]);
					HV_GET.Brake_Pedal_PMC2 = (((int)msg.DATA[7] * 256) + (int)msg.DATA[6]);
					break;
				case 0x00E3://ブレーキ内部状態通知（ブレーキセンサ）
					HV_GET.Brake_Pedal_PRL = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Brake_Pedal_PFL = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					HV_GET.Brake_Pedal_PRR = (((int)msg.DATA[5] * 256) + (int)msg.DATA[4]);
					HV_GET.Brake_Pedal_PFR = (((int)msg.DATA[7] * 256) + (int)msg.DATA[6]);
					break;
				case 0x0621://エコー応答
					HV_GET.Common_Echo_Controller = (int)msg.DATA[0];
					HV_GET.Common_Echo_No = (int)msg.DATA[1];
					break;
				case 0x0622://コンフィグ読み出し応答
					HV_GET.Common_Config_Index = (((int)msg.DATA[1] * 256) + (int)msg.DATA[0]);
					HV_GET.Common_Config_Value1 = (((int)msg.DATA[3] * 256) + (int)msg.DATA[2]);
					HV_GET.Common_Config_Value2 = (((int)msg.DATA[5] * 256) + (int)msg.DATA[4]);
					HV_GET.Common_Config_Value3 = (((int)msg.DATA[7] * 256) + (int)msg.DATA[6]);
					break;
				case 0x0623://エラー状態の取得応答
					HV_GET.Common_Error_Level = (int)msg.DATA[0];
					HV_GET.Common_Error_Code = (((int)msg.DATA[2] * 256) + (int)msg.DATA[1]);
					break;
				default:
					break;
			}
		}
		
	}while((result & !PCAN_ERROR_QRCVEMPTY) == PCAN_ERROR_OK);
	CAN_GetErrorText(result, 9, strMsg);
	MessageBox(NULL, strMsg, "CAN_Read Error", MB_ICONINFORMATION | MB_OK);
}


void CAN2_send(){

	TPCANMsg msg;
	TPCANStatus result;	
	
	if(HV_SET.Drive_Mode_Drive != HV_SET_OLD.Drive_Mode_Drive){//0 マニュアルモード, 1 プログラムモード
		msg.ID = 0x041;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Drive_Mode_Drive == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Drive_Mode_Cont != HV_SET_OLD.Drive_Mode_Cont){//0 速度制御, 1 ペダルストローク制御
		msg.ID = 0x042;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Drive_Mode_Cont == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Drive_Mode_Override != HV_SET_OLD.Drive_Mode_Override){//0 オーバーライドEnable, 1 オーバーライドDisable
		msg.ID = 0x043;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Drive_Mode_Override == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Drive_Mode_Servo != HV_SET_OLD.Drive_Mode_Servo){//0 サーボOFF, 1 サーボON
		msg.ID = 0x044;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Drive_Mode_Servo == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Drive_Velocity != HV_SET_OLD.Drive_Velocity){//0.00-180.00[km/h]
		msg.ID = 0x045;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 2;
		W2B.w = (WORD)(HV_SET.Drive_Velocity * 100.0);
		msg.DATA[0] = W2B.b[1];
		msg.DATA[1] = W2B.b[0];
	
		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Drive_Pedal != HV_SET_OLD.Drive_Pedal){//0.000-1.000[-]
		msg.ID = 0x046;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 2;
		W2B.w = (WORD)(HV_SET.Drive_Pedal * 4095.0);
		msg.DATA[0] = W2B.b[1];
		msg.DATA[1] = W2B.b[0];
	
		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Drive_Shift != HV_SET_OLD.Drive_Shift){//0: Start, 1: B, 2: D, 3: N, 4: R 
		msg.ID = 0x047;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Drive_Shift == 0)msg.DATA[0] = 0x0F;
		else if(HV_SET.Drive_Shift == 1)msg.DATA[0] = 0x00;
		else if(HV_SET.Drive_Shift == 2)msg.DATA[0] = 0x10;
		else if(HV_SET.Drive_Shift == 3)msg.DATA[0] = 0x20;
		else if(HV_SET.Drive_Shift == 4)msg.DATA[0] = 0x40;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Steer_Mode_Drive != HV_SET_OLD.Steer_Mode_Drive){//0 マニュアルモード, 1 プログラムモード
		msg.ID = 0x081;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Steer_Mode_Drive == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Steer_Mode_Cont != HV_SET_OLD.Steer_Mode_Cont){//0 トルク制御, 1 角度制御
		msg.ID = 0x082;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Steer_Mode_Cont == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Steer_Mode_Override != HV_SET_OLD.Steer_Mode_Override){//0 オーバーライドEnable, 1 オーバーライドDisable
		msg.ID = 0x083;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Steer_Mode_Override == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Steer_Mode_Servo != HV_SET_OLD.Steer_Mode_Servo){//0 サーボOFF, 1 サーボON
		msg.ID = 0x084;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 1;
		if(HV_SET.Steer_Mode_Servo == 0)msg.DATA[0] = 0x00;
		else msg.DATA[0] = 0x10;

		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Steer_Torque != HV_SET_OLD.Steer_Torque){//-1.000-1.000[-]
		msg.ID = 0x085;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 2;
		S2B.s = (short)(HV_SET.Steer_Torque * 4095.0);
		msg.DATA[0] = S2B.b[1];
		msg.DATA[1] = S2B.b[0];
	
		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Steer_Angle != HV_SET_OLD.Steer_Angle){//-666.0-666.0[deg]
		msg.ID = 0x086;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 2;
		S2B.s = (short)(HV_SET.Steer_Angle * 10.0);
		msg.DATA[0] = S2B.b[1];
		msg.DATA[1] = S2B.b[0];
	
		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}

	if(HV_SET.Brake_Strength != HV_SET_OLD.Brake_Strength){//0.000-1.000[-]
		msg.ID = 0x0C1;
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		msg.LEN = 2;
		W2B.w = (WORD)(HV_SET.Brake_Strength * 4095.0);
		msg.DATA[0] = W2B.b[1];
		msg.DATA[1] = W2B.b[0];
	
		result = CAN_Write(PCAN_PCIBUS2, &msg);
	}
	memcpy(&HV_SET_OLD, &HV_SET, sizeof(HV_SET));
}
*/


// Module specification
// <rtc-template block="module_spec">
static const char* robocar_spec[] =
  {
    "implementation_id", "RoboCar",
    "type_name",         "RoboCar",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "VenderName",
    "category",          "Category",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.drivemodedrive", "0",
    "conf.default.drivemodecont", "0",
    "conf.default.drivemodeoverride", "0",
    "conf.default.drivemodeservo", "0",
    "conf.default.drivevelocity", "0",
    "conf.default.drivepedal", "0",
    "conf.default.driveshift", "0",
    "conf.default.steermodedrive", "0",
    "conf.default.steermodecont", "0",
    "conf.default.steermodeoveride", "0",
    "conf.default.steermodeservo", "0",
    "conf.default.steertorque", "0",
    "conf.default.steerangle", "0",
    "conf.default.brakestrength", "0",
    // Widget
    "conf.__widget__.drivemodedrive", "text",
    "conf.__widget__.drivemodecont", "text",
    "conf.__widget__.drivemodeoverride", "text",
    "conf.__widget__.drivemodeservo", "text",
    "conf.__widget__.drivevelocity", "text",
    "conf.__widget__.drivepedal", "text",
    "conf.__widget__.driveshift", "text",
    "conf.__widget__.steermodedrive", "text",
    "conf.__widget__.steermodecont", "text",
    "conf.__widget__.steermodeoveride", "text",
    "conf.__widget__.steermodeservo", "text",
    "conf.__widget__.steertorque", "text",
    "conf.__widget__.steerangle", "text",
    "conf.__widget__.brakestrength", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RoboCar::RoboCar(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_GetDataIn("GetData", m_GetData),
    m_GamepadIn("Gamepad", m_Gamepad),
    m_csvOut("csv", m_csv)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RoboCar::~RoboCar()
{
}



RTC::ReturnCode_t RoboCar::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("GetData", m_GetDataIn);
  addInPort("Gamepad", m_GamepadIn);
  
  // Set OutPort buffer
  addOutPort("csv", m_csvOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("drivemodedrive", m_drivemodedrive, "0");
  bindParameter("drivemodecont", m_drivemodecont, "0");
  bindParameter("drivemodeoverride", m_drivemodeoverride, "0");
  bindParameter("drivemodeservo", m_drivemodeservo, "0");
  bindParameter("drivevelocity", m_drivevelocity, "0");
  bindParameter("drivepedal", m_drivepedal, "0");
  bindParameter("driveshift", m_driveshift, "0");
  bindParameter("steermodedrive", m_steermodedrive, "0");
  bindParameter("steermodecont", m_steermodecont, "0");
  bindParameter("steermodeoveride", m_steermodeoveride, "0");
  bindParameter("steermodeservo", m_steermodeservo, "0");
  bindParameter("steertorque", m_steertorque, "0");
  bindParameter("steerangle", m_steerangle, "0");
  bindParameter("brakestrength", m_brakestrength, "0");
  // </rtc-template>


  
  RobocarDll::MyRobocar::CAN_init();
  RobocarDll::MyRobocar::HV_init();
  
  m_Gamepad.data.length(3); 
  m_csv.data.length(108);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoboCar::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoboCar::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoboCar::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RoboCar::onActivated(RTC::UniqueId ec_id)
{
	if(m_GamepadIn.isNew()){
		m_GamepadIn.read();
	}

	if(m_GetDataIn.isNew()){
		m_GetDataIn.read();
	}

	
	
		
  return RTC::RTC_OK;

}


RTC::ReturnCode_t RoboCar::onDeactivated(RTC::UniqueId ec_id)
{
	CAN_Uninitialize(PCAN_NONEBUS);
	fclose(fp);
	

  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoboCar::onExecute(RTC::UniqueId ec_id)
{
	system("cls");
	if(m_GamepadIn.isNew() ){
		m_GamepadIn.read();
	
		//RobocarDll::MyRobocar::

	HV_SET.Drive_Pedal = m_Gamepad.data[0];
	HV_SET.Brake_Strength = m_Gamepad.data[1];
	HV_SET.Steer_Angle = -m_Gamepad.data[2];  
	}

	if(m_GetDataIn.isNew())
	{


	}
	
	cout << "マニュアル...0 , プログラム...1" <<endl;

	
	HV_SET.Drive_Mode_Drive = m_drivemodedrive;
	HV_SET.Drive_Mode_Override = m_drivemodeoverride;
	HV_SET.Drive_Mode_Servo = m_drivemodeservo;
	HV_SET.Drive_Mode_Cont = m_drivemodecont;
	HV_SET.Steer_Mode_Drive = m_steermodedrive;
	HV_SET.Steer_Mode_Cont = m_steermodecont;
	HV_SET.Steer_Mode_Override = m_steermodeoveride;
	HV_SET.Steer_Mode_Servo = m_steermodeservo;
	//HV_SET.Drive_Pedal = m_drivepedal;
	HV_SET.Drive_Velocity = m_drivevelocity;
	//HV_SET.Brake_Strength = m_brakestrength;
	//HV_SET.Steer_Angle = m_steerangle;
	HV_SET.Steer_Torque = m_steertorque;
	HV_SET.Drive_Shift = m_driveshift;

	
	RobocarDll::MyRobocar::CAN2_send();

	cout << "車のモードは" << HV_GET.Drive_Mode_Drive << endl;
	cout << "車のorverrideは" << HV_GET.Drive_Mode_Override << endl;
	cout << "車のservoは" << HV_GET.Drive_Mode_Servo << endl;
	cout << "車のSHIftは" << HV_GET.Drive_Shift_Actual << endl; 
	cout << "ステアリングのモードは" << HV_GET.Steer_Mode_Drive << endl;
	cout << "ステアリングの制御は" << HV_GET.Steer_Mode_Cont << endl;
	cout << "ステアリングのオーバーライドは" << HV_GET.Steer_Mode_Override << endl;
	cout << "ステアリングのモードは" << HV_GET.Steer_Mode_Servo << endl;
	cout << "ステアリングの角度は" << HV_GET.Steer_Angle_Actual << endl;
	cout << "プリウスの速度は"  << HV_GET.Drive_Speed_Actual <<endl;
	cout << "プリウスのdrivemodecontは"  << HV_GET.Drive_Mode_Cont <<endl;

	cout << "ゲームパッドのSteerAngleは"<<HV_SET.Steer_Angle << endl;
	cout << "ゲームパッドのDrivePedalは"<<HV_SET.Drive_Pedal << endl;
	cout << "ゲームパッドのBrakeStrengthは"<<HV_SET.Brake_Strength << endl;


	//記録用コンポーネントに送信
	m_csv.data[0] = Prius.Sideway_acceleration_sensor;	// -:左方向,+:右方向
	m_csv.data[1] = Prius.Acceleration_sensor;			// -:手前方向,+:踏込方向
	m_csv.data[2] = Prius.Steering_Angle;				// 
	m_csv.data[3] = Prius.Brakes_Pressflg;				// bit#7 : 0=pressed 1=released  ( 0x84 Not pressed / 0x04 Pressed )
	m_csv.data[4] = Prius.Brakes_Strength;				// 0:out, 0x7F:pushed in 
	m_csv.data[5] = Prius.EM_Current1;					// 12 bit signed number ( + discharge ) unit is 0.1A
	m_csv.data[6] = Prius.EM_Current2;					// battery pack voltage, unit is 1V
	m_csv.data[7] = Prius.Front_Wheels_R;				// value is speed in km/h * 100
	m_csv.data[8] = Prius.Front_Wheels_L;				// value is speed in km/h * 100
	m_csv.data[9] = Prius.Rear_Wheels_R;				// value is speed in km/h * 100
	m_csv.data[10] = Prius.Rear_Wheels_L;				// value is speed in km/h * 100
	m_csv.data[11] = Prius.Speed;						// 0x400 for every 10km/h, including signed value for backwards
	m_csv.data[12] = Prius.Drive_Mode_Cruiseflg;			// 0x50=D 0x51=B 0x4D=P 0x4E=R 0x4F=N MSB=CruiseON/off 0x49|0x4B = Standby
	m_csv.data[13] = Prius.Drive_Mode_Shiftflg;			// 0x13&0x14=D 0x10=P 0x11=R 0x12=N
	m_csv.data[14] = Prius.Drive_Mode_Powerflg;			// 0x04=Powered 0x00=Standby
	m_csv.data[15] = Prius.Gas_Pedal_Speed;				// Speed related ( ~0x150=10km/h ~0x300=20km/h ~0x700=50km/h )  ALSO, negative, when backing up !!!
	m_csv.data[16] = Prius.Gas_Pedal;					// Gas Pedal ( 0x00 ~ 0xC8 )
	m_csv.data[17] = Prius.ICE_Throttle;					// Throttle related ( requested rpm ?) ~MPG=(KMH*10000)/value 
	m_csv.data[18] = Prius.ICE_Powerflg;					// 0 when no power by ICE.
	m_csv.data[19] = Prius.ICE_Rpm_Target;				// Target RPM ( unit is 32 rpm )  20
	m_csv.data[20] = Prius.Speed2;						// Value in km/h
	m_csv.data[21] = Prius.Battery_Max_Amps_Discharge;	// max current for the battery discharge in Amps.
	m_csv.data[22] = Prius.Battery_Max_Amps_Charge;		// max current for the battery charge in Amps.
	m_csv.data[23] = Prius.Battery_SOC;					// 16 bit = SOC Value, unit is 0.5%
	m_csv.data[24] = Prius.Battery_Min_Temp;				// Batt Temp Min. (C)
	m_csv.data[25] = Prius.Battery_Max_Temp;				// Batt Temp Max.
	m_csv.data[26] = Prius.Battery_Fault_Code;
	m_csv.data[27] = Prius.Battery_Pack_Voltage;			// 16 bit pack voltage
	m_csv.data[28] = Prius.EV_MODE;						// 0x00=Normal Drive  0x40=EVMode  0x80=EVDenied  Other=Cancelled
	m_csv.data[29] = Prius.Interlock;					// bit#7 pressed/depressed
	m_csv.data[30] = Prius.Engine_Coolant_Temp;			// value in Celsius, unit is 0.5C
	m_csv.data[31] = Prius.Shift_Lever;					// 00=B 10=D 20=N 40=R 80=P
	m_csv.data[32] = Prius.Shift_Transition;				// bit#7 transition
	m_csv.data[33] = Prius.Lights_Front;					// 00=Off 10=Park 30=On 38=HighBeam
	m_csv.data[34] = Prius.Lights_Transition;			// bit#7 transition
	m_csv.data[35] = Prius.Lights_Instruments;			// bit#3 instruments 1=dimmed 0=normal ( 0x18/0x10 )
	m_csv.data[36] = Prius.Gas_Gauge;					// value ( my recorded max is 0x28 )
	m_csv.data[37] = Prius.Doors_flg;					// 00=Closed 04=Rear 40=Passenger 80=Driver open
	m_csv.data[38] = Prius.Doors_Transition;				// bit#7 transition
	m_csv.data[39] = Prius.Cruise_flg;					// 00=Off 10=On  40
	m_csv.data[40] = Prius.Cruise_Transition;			// bit#7 transition
	m_csv.data[41] = HV_GET.Priority_Error_Level;
	m_csv.data[42] = HV_GET.Priority_Error_Code;
	m_csv.data[43] = HV_GET.Drive_Mode_Drive;			//0x00 マニュアルモード, 0x10 プログラムモード
	m_csv.data[44] = HV_GET.Drive_Mode_Cont;			//0x00 速度制御, 0x10 ペダルストローク制御
	m_csv.data[45] = HV_GET.Drive_Mode_Override;		//0x00 オーバーライドEnable, 0x10 オーバーライドDisable
	m_csv.data[46] = HV_GET.Drive_Mode_Servo;			//0x00 サーボOFF, 0x10 サーボON
	m_csv.data[47] = HV_GET.Drive_Pedal_Input;			//アクセル入力値 0～4095 (0～1.0)
	m_csv.data[48] = HV_GET.Drive_Pedal_Target;			//アクセル指令値 0～4095 (0～1.0)
	m_csv.data[49] = HV_GET.Drive_Pedal_Actual;			//アクセル現在値 0～4095 (0～1.0)
	m_csv.data[50] = HV_GET.Drive_Shift_Input;			//シフト入力値 0x00 B, 0x10 D, 0x20 N, 0x40 R, 0x80 P, 0x0F Start, 0xFF Unknown
	m_csv.data[51] = HV_GET.Drive_Shift_Target;			//シフト指令値 0x00 B, 0x10 D, 0x20 N, 0x40 R, 0x80 P, 0x0F Start, 0xFF Unknown
	m_csv.data[52] = HV_GET.Drive_Shift_Actual;			//シフト現在値 0x00 B, 0x10 D, 0x20 N, 0x40 R, 0x80 P, 0x0F Start, 0xFF Unknown
	m_csv.data[53] = HV_GET.Drive_Speed_Target;			//速度指令値 0～18000 (0～180[km/h])
	m_csv.data[54] = HV_GET.Drive_Speed_Actual;			//速度現在値 0～18000 (0～180[km/h])
	m_csv.data[55] = HV_GET.Drive_Pedal_Raw1;			//アクセルストロークセンサ VPA1 [mV]
	m_csv.data[56] = HV_GET.Drive_Pedal_Raw2;			//アクセルストロークセンサ VPA2 [mV]
	m_csv.data[57] = HV_GET.Drive_Shift_Raw1;			//VSX1 [mV]
	m_csv.data[58] = HV_GET.Drive_Shift_Raw2;			//VSX2 [mV]
	m_csv.data[59] = HV_GET.Drive_Shift_Raw3;			//VSX3 [mV]  60
	m_csv.data[60] = HV_GET.Drive_Shift_Raw4;			//VSX4 [mV]
	m_csv.data[61] = HV_GET.Steer_Mode_Drive;			//0x00 マニュアルモード, 0x10 プログラムモード
	m_csv.data[62] = HV_GET.Steer_Mode_Cont;			//0x00 トルク制御, 0x10 角度制御
	m_csv.data[63] = HV_GET.Steer_Mode_Override;		//0x00 オーバーライドEnable, 0x10 オーバーライドDisable
	m_csv.data[64] = HV_GET.Steer_Mode_Servo;			//0x00 サーボOFF, 0x10 サーボON
	m_csv.data[65] = HV_GET.Steer_Angle_Target;			//角度指令値 -6660～+6660 (-666°～ +666°) 左がプラス
	m_csv.data[66] = HV_GET.Steer_Angle_Actual;			//角度現在値 -6660～+6660 (-666°～ +666°) 左がプラス
	m_csv.data[67] = HV_GET.Steer_Torque_Target;		//トルク指令値 -4096～4095 (-1.0～1.0) 左がプラス
	m_csv.data[68] = HV_GET.Steer_Torque_Actual;		//トルク現在値 -4096～4095 (-1.0～1.0) 左がプラス
	m_csv.data[69] = HV_GET.Steer_Torque_Raw1;			//TRQ1 [mV]
	m_csv.data[70] = HV_GET.Steer_Torque_Raw2;			//TRQ2 [mV]
	m_csv.data[71] = HV_GET.Brake_Pedal_Input;			//ブレーキ入力値 0～4095 (0～1.0)
	m_csv.data[72] = HV_GET.Brake_Pedal_Target;			//ブレーキ指令値 0～4095 (0～1.0)
	m_csv.data[73] = HV_GET.Brake_Pedal_Actual;			//ブレーキ現在値 0～4095 (0～1.0)
	m_csv.data[74] = HV_GET.Brake_Pedal_SKS1;			//SKS1 [mV]
	m_csv.data[75] = HV_GET.Brake_Pedal_SKS2;			//SKS2 [mV]
	m_csv.data[76] = HV_GET.Brake_Pedal_PMC1;			//PMC1 [mV]
	m_csv.data[77] = HV_GET.Brake_Pedal_PMC2;			//PMC2 [mV]
	m_csv.data[78] = HV_GET.Brake_Pedal_PRL;			//PRL [mV]
	m_csv.data[79] = HV_GET.Brake_Pedal_PFL;			//PFL [mV]  80
	m_csv.data[80] = HV_GET.Brake_Pedal_PRR;			//PRR [mV]
	m_csv.data[81] = HV_GET.Brake_Pedal_PFR;			//PFR [mV]
	m_csv.data[82] = HV_GET.Common_Echo_Controller;		//コントローラ種別
	m_csv.data[83] = HV_GET.Common_Echo_No;				//番号
	m_csv.data[84] = HV_GET.Common_Config_Index;		//開始Index
	m_csv.data[85] = HV_GET.Common_Config_Value1;		//値1
	m_csv.data[86] = HV_GET.Common_Config_Value2;		//値2
	m_csv.data[87] = HV_GET.Common_Config_Value3;		//値3
	m_csv.data[88] = HV_GET.Common_Error_Level;
	m_csv.data[89] = HV_GET.Common_Error_Code;

	m_csv.data[90] = HV_SET.Drive_Mode_Drive;			//0 マニュアルモード, 1 プログラムモード
	m_csv.data[91] = HV_SET.Drive_Mode_Cont;			//0 速度制御, 1 ペダルストローク制御
	m_csv.data[92] = HV_SET.Drive_Mode_Override;		//0 オーバーライドEnable, 1 オーバーライドDisable
	m_csv.data[93] = HV_SET.Drive_Mode_Servo;			//0 サーボOFF, 1 サーボON
	m_csv.data[94] = HV_SET. Drive_Velocity;			//0.00-180.00[km/h]
	m_csv.data[95] = HV_SET. Drive_Pedal;				//0.000-1.000[-]
	m_csv.data[96] = HV_SET.Drive_Shift;				//0: Start, 1: B, 2: D, 3: N, 4: R 
	m_csv.data[97] = HV_SET.Steer_Mode_Drive;			//0 マニュアルモード, 1 プログラムモード
	m_csv.data[98] = HV_SET.Steer_Mode_Cont;			//0 トルク制御, 1 角度制御
	m_csv.data[99] = HV_SET.Steer_Mode_Override;		//0 オーバーライドEnable, 1 オーバーライドDisable  100
	m_csv.data[100] = HV_SET.Steer_Mode_Servo;			//0 サーボOFF, 1 サーボON
	m_csv.data[101] = HV_SET. Steer_Torque;			//-1.000-1.000[-]
	m_csv.data[102] = HV_SET. Steer_Angle;				//-666.0-666.0[deg]
	m_csv.data[103] = HV_SET. Brake_Strength;			//0.000-1.000[-]
	m_csv.data[104] = HV_SET.Common_GET_Config_Index;	//開始Index
	m_csv.data[105] = HV_SET.Common_GET_Config_num;		//数
	m_csv.data[106] = HV_SET.Common_SET_Config_Index;	//開始Index
	m_csv.data[107] = HV_SET.Common_SET_Config_Value;	//値   108

	m_csvOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoboCar::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoboCar::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoboCar::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoboCar::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoboCar::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void RoboCarInit(RTC::Manager* manager)
  {
    coil::Properties profile(robocar_spec);
    manager->registerFactory(profile,
                             RTC::Create<RoboCar>,
                             RTC::Delete<RoboCar>);
  }
  
};



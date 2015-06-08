/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
//===== LIN_Master =====//
#include <project.h>
#include "lin_master.h"
#include "ps2_controller.h"
#include "servo.h"
#include <stdio.h>
#include <math.h>

int    g_timerFlag;
double ROOT3;

Lindata  lindata;
order    control;

//--- 10ms立った時にflagを上げる ---//
CY_ISR(motor_isr)
{	
	g_timerFlag = 1;
}

// オム二速度計算関数　//
void calcOmuni(int16 Vx ,int16 Vy ,int16 roll)
{
    int16  omuni1 = 0;
    int16  omuni2 = 0;
    int16  omuni3 = 0;
    double runRatio = 1;
    double turnRatio = 1;
    double constant = 3.937;//6.063;
   
    // 各モータ速度算出　//
    if((Vx != 0)||(Vy != 0))
    {
        omuni1 = (Vx * runRatio) * constant;
        omuni2 = -(((0.5 * Vx) - ((ROOT3 * 0.5) * Vy)) * runRatio)  * constant;
        omuni3 = -(((0.5 * Vx) + ((ROOT3 * 0.5) * Vy)) * runRatio)  * constant;
    }
    else
    {
        omuni1 = roll * constant;
        omuni2 = roll * constant;
        omuni3 = roll * constant;
    }
 
    // 送信データ整形　//
    lindata.data1 = omuni1;
    lindata.data2 = omuni2;
    lindata.data3 = omuni3;
    
    /*
    if(UART_1_ReadTxStatus()&UART_1_TX_STS_FIFO_EMPTY)
    {
        // (Slave番号)　[送信データ]　//
        sprintf(Data,"(1)[%5d] (2)[%5d] (3)[%5d] \n"
            ,(int)omuni1,(int)omuni2,(int)omuni3); 
        UART_1_PutString(Data);
    }
    */
}

//--- 初期設定 ---//
void init()
{
    PS2_Start();
    
    UART_1_Start();
    UART_1_EnableTxInt();
    
    isr_2_StartEx(motor_isr);
}

int main()
{
    //--- 変数定義 ---//
    char  Data[255] = {0};
    
    int8  ps2_LY = 0;
    int8  ps2_LX = 0;
    int8  ps2_RY = 0;
    int8  ps2_RX = 0;
    
    uint8 container = 0;
    uint8 grabFlag = 1;
    uint8 selectFlag = 1;
    uint8 positionFlag = 1;
    uint8 emissionRedFlag = 1;
    uint8 emissionBlueFlag = 1;
    uint8 emissionYellowFlag = 1;
    uint8 L2Flag = 0;
   
    //--- 構造体定義　---//
    PS2Controller psData;
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */    
    
    //--- 割り込み許可　---//
    CyGlobalIntEnable;
    
    //--- 初期化 ---//
    init();
    initLin();
    //-- サーボ初期化 ---//
    servoInit();
    LinSendData(&lindata);
    
    /* Uncomment this line to enable global interrupts. */
    
    //--- analog押すまで待機 ---//
    while (!PS2_Analog_Flag());
    
    //--- start押すまで待機 ---//
    while (!PS2_Controller_get().START);
    
    //--- √３計算 ---//
    ROOT3 = pow(3,0.5);
    
    //--- 排出機構展開 ---//
    control.servo.command.pad = 1;
    lindata.data4 = control.servo.Trans;
    LinSendData(&lindata);
    
    //--- 処理開始　---//
    for(;;)
    {
    //--- 割り込み判定　---//
		if(g_timerFlag == 1)
        {
            
        //--- PS2Controller値取得 ---//
            psData = PS2_Controller_get();

        //--- コントローラ信号整形　---//
            // 初期化 //
            if(!PS2_Analog_Flag() || PS2_Timeout_Flag())
            {                
                ps2_LY = 0;
                ps2_LX = 0;
                ps2_RY = 0;
                ps2_RX = 0;               
            }
            // 左スティック　//
            if((psData.ANALOG_LY!= 0)||(psData.ANALOG_LX != 0))
            {
                // Y軸 //
                if(psData.ANALOG_LY > (127.0 + 40.0))
                {                    
                    ps2_LY = psData.ANALOG_LY;
                    ps2_LY = -(ps2_LY - 128.0);               
                }
                else if(psData.ANALOG_LY < (127.0 - 40.0))
                {                   
                    ps2_LY = psData.ANALOG_LY;
                    ps2_LY = 127.0 - ps2_LY;                   
                }
                else
                {                   
                    ps2_LY = 0;                  
                }
                // X軸 //
                if(psData.ANALOG_LX > (127.0 + 40.0))
                {
                    ps2_LX = psData.ANALOG_LX;
                    ps2_LX = ps2_LX - 128.0;               
                }                
                else if(psData.ANALOG_LX < (127.0 - 40.0))
                {
                    ps2_LX = psData.ANALOG_LX;
                    ps2_LX = -(127.0 - ps2_LX);      
                }               
                else
                {                    
                    ps2_LX = 0;                  
                }               
            }
            // 右スティック　//
            if((psData.ANALOG_RY!= 0)||(psData.ANALOG_RX != 0))
            {
                // Y軸 //
                if(psData.ANALOG_RY > (127.0 + 40.0))
                {                   
                    ps2_RY = psData.ANALOG_RY;
                    ps2_RY = -(ps2_RY - 128.0);                
                }                
                else if(psData.ANALOG_RY < (127.0 - 40.0))
                {                   
                    ps2_RY = psData.ANALOG_RY;
                    ps2_RY = 127.0 - ps2_RY;                   
                }               
                else
                {
                    ps2_RY = 0;                 
                }
                // X軸 //
                if(psData.ANALOG_RX > (127.0 + 40.0))
                {                   
                    ps2_RX = psData.ANALOG_RX;
                    ps2_RX = ps2_RX - 128.0;               
                }               
                else if(psData.ANALOG_RX < (127.0 - 40.0))
                {   
                    ps2_RX = psData.ANALOG_RX;
                    ps2_RX = -(127.0 - ps2_RX);    
                }
                else
                {                    
                    ps2_RX = 0;                 
                }       
            }
        //--- 足回りデータ整形　---//
            if((ps2_LY != 0)||(ps2_LX != 0)||(ps2_RX != 0))
            {
                calcOmuni(ps2_LX,ps2_LY,ps2_RX);
            }
            else
            {
                calcOmuni(0,0,0);
            }
           
        //--- servoデータ整形 ---//
            
            //  掴むOR離す　//
            if(psData.L2)
            {
                L2Flag=1;
            }
            if(L2Flag==1)
            {
                L2Flag=catchBallAuto(1);
            }          
            else
            { 
                if(psData.L1)
                {
                    if(grabFlag == 1)
                    {
                        grabFlag = 0;   
                        control.servo.command.grabBall = ~control.servo.command.grabBall;
                    }
                }
                else
                {
                    grabFlag = 1;
                }
                // アーム姿勢 //
                if((psData.R1)&&(control.servo.command.selectContainer == 0))
                {
                    if(positionFlag == 1)
                    {
                        positionFlag = 0;
                        control.servo.command.position = ~control.servo.command.position;
                    }
                }
                else
                {
                    positionFlag = 1;
                }
                // 格納コンテナ選択　//
                if(control.servo.command.position == 0)
                {
                    if(psData.R2)
                    {
                        if((selectFlag == 1)&&(container < 3))
                        {
                            selectFlag = 0;
                            container++;
                        }
                        if(container == 3)
                        {
                            container = 0;
                        }
                    }
                    else
                    {
                        selectFlag = 1;
                    }
                    // 初期位置(コンテナ黄)　//
                    if(container == 0)
                    {
                        control.servo.command.selectContainer = 0;
                    }else 
                    // コンテナ赤 //
                    if(container == 1)
                    {
                        control.servo.command.selectContainer = 1;
                    }else
                    // コンテナ青　//
                    if(container == 2)
                    {
                        control.servo.command.selectContainer = 2;
                    }else
                    //　コンテナ青(あまり)　//
                    if(container == 3)
                    {
                        control.servo.command.selectContainer = 3;
                    }
               }
                // ボール赤排出　//
                if(psData.CIRCLE)
                {
                    if(emissionRedFlag == 1)
                    {
                        emissionRedFlag = 0;
                        control.servo.command.emissionRed = ~control.servo.command.emissionRed;
                    }
                }
                else
                {
                    emissionRedFlag = 1;
                }
                // ボール青排出 //
                if(psData.SQUARE)
                {
                    if(emissionBlueFlag == 1)
                    {
                        emissionBlueFlag = 0;
                        control.servo.command.emissionBlue = ~control.servo.command.emissionBlue;
                    }
                }
                else
                {
                    emissionBlueFlag = 1;
                }
                // ボール黄排出 //
                if(psData.CROSS)
                {
                    if(emissionYellowFlag == 1)
                    {
                        emissionYellowFlag = 0;
                        control.servo.command.emissionYellow = ~control.servo.command.emissionYellow;
                    }
                }
                else
                {
                    emissionYellowFlag = 1;
                }
                
                lindata.data4 = control.servo.Trans;
            }
        /* Place your application code here. */
            
        //--- LINデータ送受信 ---//
            // Slaveへデータ送信 [関数呼び出し] //
            LinSendData(&lindata);
            
//        //--- デバック ---//
//            if(UART_1_ReadTxStatus()&UART_1_TX_STS_FIFO_EMPTY)
//            {
//                // (Slave番号)　[送信データ]　//
//                sprintf(Data,"(1)[%5d] (2)[%5d] (3)[%5d] (4)[%x]\n"
//                    ,(int)lindata.data1,(int)lindata.data2
//                    ,(int)lindata.data3,(int)lindata.data4); 
//                UART_1_PutString(Data);
//            }
            
        //---　タイマーフラグ リセット　---//
            g_timerFlag = 0;
        }
    }
}

/* [] END OF FILE */


#include <project.h>
#include <stdio.h>
#include <math.h>
#include "servo.h"

//--- LINデータ送信関数 ---//
void LinSendData(Lindata* lindata)
{
    static uint16 data1_data_old;
    static uint16 data2_data_old;
    static uint16 data3_data_old;
    static uint16 data4_data_old;
    uint8 send_flag = 0;
    static uint8 put_data[6] = {0};

    // 変化判定 //
    if(data1_data_old != lindata->data1)
    {        
        send_flag = 1;  
    }    
    data1_data_old = lindata->data1;
    
    if(data2_data_old != lindata->data2)
    {       
        send_flag = 1;
    }    
    data2_data_old = lindata->data2;
    
    if(data3_data_old != lindata->data3)
    {        
        send_flag = 1;  
    }    
    data3_data_old = lindata->data3;
    
    if(data4_data_old != lindata->data4) 
    {        
        send_flag = 1;  
    }    
    data4_data_old = lindata->data4;
    
    if(LIN_Master_ReadTxStatus() != LIN_TX_SEND)
    {        
        if(send_flag == 1)
        {         
            // 16bitデータを2個の8bitに変換 //
            put_data[0] = 0xff & lindata->data1;
            put_data[1] = 0xff & (lindata->data1 >> 8);
            put_data[2] = 0xff & lindata->data2;
            put_data[3] = 0xff & (lindata->data2 >> 8);
            put_data[4] = 0xff & lindata->data3;
            put_data[5] = 0xff & (lindata->data3 >> 8);
            // slave4は最初から8bitデータ　//
            put_data[6] = 0xff & lindata->data4;
            LIN_Master_PutArray(2,7,put_data);            
        }        
    } 
}

void servoInit()
{
    control.servo.command.grabBall        = 1;
    control.servo.command.position        = 0;
    control.servo.command.selectContainer = 0;
    control.servo.command.emissionRed     = 0;
    control.servo.command.emissionBlue    = 0;
    control.servo.command.emissionYellow  = 0;
    control.servo.command.pad             = 0;
    lindata.data4 = control.servo.Trans;
}

//0黄1赤2青
uint8 catchBallAuto(uint8 color){
    static uint16 step  = 0;
    static uint16 count = 0;
           uint16 limit = 0;
           uint8 catchFlag = 0;
    char Data[200];
    catchFlag = Arm_Read();

    if(step==0)
    {       
        control.servo.command.selectContainer = 0;
        limit = 10;
    }else
    if(step==1)
    {
        control.servo.command.grabBall = 0;
        control.servo.command.position = 1;
 
        limit=100;
    }else
    if(step==2)
    {
        control.servo.command.grabBall = 1;
        limit = 60;
    }else
    if(step == 3)
    {
        if(catchFlag==1){//キャッチ失敗
            control.servo.command.position = 0;
            control.servo.command.selectContainer = 0;
            count = 0;
            step = 0;
            return 2;
        }
        else
        {
            control.servo.command.position = 0;
            control.servo.command.selectContainer = color;//0黄,1赤,2青
            limit = 100;
        }
    }
    else
    if(step == 4)
    {
        control.servo.command.grabBall = 0;
        limit = 100;
    }
    else
    if(step == 5)
    {
        control.servo.command.selectContainer = 0;
        control.servo.command.grabBall = 1;
        limit = 50;
    }
    else
    if(step == 6)
    {
        count = 0;
        step=0;
        return 0;
    }
    count++;
    if(limit == count)
    {
        step++;
        count = 0;
    }
    lindata.data4 = control.servo.Trans;
    return 1;
}

/* [] END OF FILE */
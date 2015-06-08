#include <project.h>
#include "lin_master.h"

//---　データ:送信用 ---//
typedef struct
{    
    uint16 data1;
    uint16 data2;
    uint16 data3;  
    uint16 data4;
}Lindata;

//--- サーボモーション用 ---//
union Motion
{
	uint8 Trans;
	struct
	{
        uint8  grabBall        : 1;
        uint8  position        : 1;
        uint8  selectContainer : 2;
        uint8  emissionRed     : 1;
        uint8  emissionYellow  : 1;
		uint8  emissionBlue    : 1;
		uint8  pad             : 1;	
	}command;
};

typedef struct
{
	union Motion servo;
} order;

extern order    control;
extern Lindata  lindata;

void LinSendData(Lindata* lindata);
void servoInit();
uint8 catchBallAuto(uint8 color);

/* [] END OF FILE */
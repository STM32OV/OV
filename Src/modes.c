#include "main.h"

extern int8_t current[128];
extern int8_t volume[128];
extern int8_t pressure[128];
extern int8_t flow[128];

//int8_t params[4]={1,12,45,20};
//int8_t params_min[4]={0,4,25,10};
//int8_t params_max[4]={1,20,99,30};
//char* modes[2]={"NF","AC"};


extern uint8_t piezo_alarm;
extern uint8_t keypressed[5];

extern uint16_t timer_frame;

extern TIM_HandleTypeDef    TimHandlePWM;
extern ADC_HandleTypeDef    AdcHandle;



typedef struct
{
	char name[4];
	uint8_t nb_param;
	uint8_t param_default[4];
	uint8_t param_max[4];
	uint8_t param_min[4];
	uint8_t param_disp_mul[4];
	uint8_t param[4];
	char param_name[4][4];
}ModeParam;

int8_t  mode=0;
int8_t id_param=0;
ModeParam modeparams[2];


void InitParams()
{
#define NB_MODES 2

#define MODE_AC 0
#define MODE_AC_PARAM_R 0
#define MODE_AC_PARAM_V 1
#define MODE_AC_PARAM_P 2

	modeparams[MODE_AC].nb_param=3;

	strcpy(modeparams[MODE_AC].name, "AC");
	strcpy(modeparams[MODE_AC].param_name[0], "R:");
	strcpy(modeparams[MODE_AC].param_name[1], "V:");
	strcpy(modeparams[MODE_AC].param_name[2], "Pmx:");

	modeparams[MODE_AC].param_default[MODE_AC_PARAM_R]=12;
	modeparams[MODE_AC].param[MODE_AC_PARAM_R]=12;
	modeparams[MODE_AC].param_max[MODE_AC_PARAM_R]=18;
	modeparams[MODE_AC].param_min[MODE_AC_PARAM_R]=6;
	modeparams[MODE_AC].param_disp_mul[MODE_AC_PARAM_R]=1;

	modeparams[MODE_AC].param_default[MODE_AC_PARAM_V]=45;
	modeparams[MODE_AC].param[MODE_AC_PARAM_V]=45;
	modeparams[MODE_AC].param_max[MODE_AC_PARAM_V]=80;
	modeparams[MODE_AC].param_min[MODE_AC_PARAM_V]=20;
	modeparams[MODE_AC].param_disp_mul[MODE_AC_PARAM_V]=10;

	modeparams[MODE_AC].param_default[MODE_AC_PARAM_P]=20;
	modeparams[MODE_AC].param[MODE_AC_PARAM_P]=20;
	modeparams[MODE_AC].param_max[MODE_AC_PARAM_P]=40;
	modeparams[MODE_AC].param_min[MODE_AC_PARAM_P]=12;
	modeparams[MODE_AC].param_disp_mul[MODE_AC_PARAM_P]=1;


#define MODE_NF 1
#define MODE_NF_PARAM_R 0
#define MODE_AC_PARAM_V 1

	modeparams[MODE_NF].nb_param=2;

	strcpy(modeparams[MODE_NF].name, "NF:");
	strcpy(modeparams[MODE_NF].param_name[0], "R:");
	strcpy(modeparams[MODE_NF].param_name[1], "V:");

	modeparams[MODE_NF].param_default[MODE_AC_PARAM_R]=12;
	modeparams[MODE_NF].param[MODE_AC_PARAM_R]=12;
	modeparams[MODE_NF].param_max[MODE_AC_PARAM_R]=18;
	modeparams[MODE_NF].param_min[MODE_AC_PARAM_R]=6;
	modeparams[MODE_NF].param_disp_mul[MODE_AC_PARAM_R]=1;

	modeparams[MODE_NF].param_default[MODE_AC_PARAM_V]=45;
	modeparams[MODE_NF].param[MODE_AC_PARAM_V]=45;
	modeparams[MODE_NF].param_min[MODE_AC_PARAM_V]=20;
	modeparams[MODE_NF].param_max[MODE_AC_PARAM_V]=80;
	modeparams[MODE_NF].param_disp_mul[MODE_AC_PARAM_V]=10;
}


uint8_t GetColor(uint8_t id)
{
	if(id==id_param && timer_frame%100<50)return 1;
	return 2;
}


void DrawMenu()
{
	ModeParam* activeparam=&modeparams[mode];

	if(keypressed[BTN_RIGHT])
	{
		id_param++;
		uint8_t id_max=activeparam->nb_param;
		if(id_param>id_max)id_param=id_max;
	}
	if(keypressed[BTN_LEFT])
	{
		id_param--;
		if(id_param<0)id_param=0;
	}

	if(keypressed[BTN_UP])
	{
		if(id_param==0)
		{
			mode++;
			if(mode>=NB_MODES)mode=NB_MODES-1;
		}
		else
		{
			activeparam->param[id_param-1]++;
			if(activeparam->param[id_param-1]>activeparam->param_max[id_param-1])activeparam->param[id_param-1]=activeparam->param_max[id_param-1]-1;
		}
	}
	if(keypressed[BTN_DOWN])
	{
		if(id_param==0)
		{
			mode--;
			if(mode<0)mode=0;
		}
		else
		{
			activeparam->param[id_param-1]--;
			if(activeparam->param[id_param-1]<activeparam->param_min[id_param-1])activeparam->param[id_param-1]=activeparam->param_min[id_param-1];
		}
	}

	{
		st7565_drawstringmulti(0,7,activeparam->name, GetColor(0));
	}

	int x_pos=3*6;
	for(int i=0;i<activeparam->nb_param;i++)
	{
		char buffer[12];
		int n=sprintf (buffer, "%s%i\0", activeparam->param_name[i], ((int16_t)activeparam->param[i]*(int16_t)activeparam->param_disp_mul[i]));
		st7565_drawstringmulti(x_pos,7,buffer, GetColor(i+1));
		x_pos+=6*6;
	}

}


int pos_min=0;
int pos_max=1000;
int pos_curr=0;
int pos_target=0;

int trigger=0; //Trigger will be set to TIME_INSPIRE when a cycle is triggered (in ms).
int timeout=0; //Timeout in ms.
int pos_curr_mlx10;

void UpdateAC(uint32_t ya, uint32_t yb, uint32_t yc, uint32_t yw)
{
	ModeParam* activeparam=&modeparams[MODE_AC];

	pos_target = pos_min;
	if(trigger>0)
	{
		pos_target=10000;
		timeout=0;
	}
	trigger-=5;

	timeout+=5;
	if(timeout > 60000/(int16_t)activeparam->param[MODE_AC_PARAM_R] - MODEAC_TIME_INSPIRE) //->4000 is 9 sec.
	{
		trigger = MODEAC_TIME_INSPIRE;
	}

	if(pos_curr<pos_target)pos_curr+=5*10000/MODEAC_TIME_INSPIRE; //pos target is 10 000, corresponds to full inspiration. It is corrected later to account for volume.
	if(pos_curr>pos_target)pos_curr-=5*10000/MODEAC_TIME_INSPIRE; //It will go up to 10 000 in TIME_INSPIRE ms.

	pos_curr_mlx10 = pos_curr*((uint16_t)activeparam->param[MODE_AC_PARAM_V])/100; //To get ml you should divide by 10000. But Params[2] is in fact ml target/10 (45 for 450ml). So we need to divide by 100.

	if(yw<170 && trigger<-MODEAC_TIME_INSPIRE - MODEAC_TIMEOUT_TRIGGER)trigger=MODEAC_TIME_INSPIRE;

	piezo_alarm=0;
	if(yw>activeparam->param[MODE_AC_PARAM_P]*80)piezo_alarm=1;

	if(ya>300)piezo_alarm=1;
	if(yb>300)piezo_alarm=1;
	if(yc>300)piezo_alarm=1;
}

void UpdateNF(uint32_t ya, uint32_t yb, uint32_t yc, uint32_t yw)
{

}

extern uint16_t currid;
extern UART_HandleTypeDef UartHandle;
uint8_t aRxBuffer[64];






//This function is called 5ms
void UpdateServoPWM()
{
	uint32_t pos_in_buf = ADCSAMPLECOUNT*4 - (uint32_t)__HAL_DMA_GET_COUNTER(AdcHandle.DMA_Handle);
	pos_in_buf/=4;	//there are 3 values interleaved. Just read the first one.
	pos_in_buf*=4;	//Round up to the 3rd closest.

	uint32_t ya = ComputeAverage(pos_in_buf-4);
	uint32_t yb = ComputeAverage(pos_in_buf-3);
	uint32_t yc = ComputeAverage(pos_in_buf-2);
	uint32_t yw = ComputeAverage(pos_in_buf-1);

	if(mode==0)UpdateAC(ya, yb, yc, yw);
	if(mode==1)UpdateNF(ya, yb, yc, yw);

	//ServoK is in 0.1ml per microsecond (what volume is expelled for a 1 microsecond difference in PWM output)
	TimHandlePWM.Instance->CCR1 = 2930 - pos_curr_mlx10/MODEAC_SERVO_K-60;
	TimHandlePWM.Instance->CCR2 = 2930 - pos_curr_mlx10/MODEAC_SERVO_K+20;
	TimHandlePWM.Instance->CCR3 = 2930 - pos_curr_mlx10/MODEAC_SERVO_K-220;

	if(timer_frame%20==0)
	{
		char buffer[50];
		int n=sprintf (buffer, "%i, %i, %i, %i, %i, %i, %i\n", pos_curr/20, ya, yb, yc, yw, (uint16_t)aRxBuffer[0], (uint16_t)aRxBuffer[1]);

		int vvolprec = pos_curr_mlx10/500;
		if(vvolprec>126)vvolprec=126;
		if(vvolprec<-126)vvolprec=-126;
		currid=(timer_frame/20)%128;
		volume[currid]=vvolprec;
		for(int i=1;i<10;i++)volume[(currid+i)%128]=127; //Delete points in advance

		int vcurrprec = (ya+yb+yc)/50;
		if(vcurrprec>126)vcurrprec=126;
		if(vcurrprec<-126)vcurrprec=-126;
		current[currid]=vcurrprec;
		for(int i=1;i<10;i++)current[(currid+i)%128]=127; //Delete points in advance

		int vpressprec = (yw)/200;
		if(vpressprec>126)vpressprec=126;
		if(vpressprec<-126)vpressprec=-126;
		pressure[currid]=vpressprec;
		for(int i=1;i<10;i++)pressure[(currid+i)%128]=127; //Delete points in advance

		HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, n , 5000);
	}
}




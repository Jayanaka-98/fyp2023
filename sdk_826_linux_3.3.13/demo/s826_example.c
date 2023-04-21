////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSORAY MODEL 826 PROGRAMMING EXAMPLES
// This file contains simple functions that show how to program the 826.
// Copyright (C) 2012 Sensoray
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MSc setup - Force Predition
// Praveena Dewapura
// 2022.03.30
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Header functions declarations

#ifndef _LINUX
#include <windows.h>
#include <conio.h>
#include <string.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#define PI	3.14159265

#ifndef _LINUX
#include "..\826api.h"
#else
#include "826api.h"
#endif


// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          			// convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   			// convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    			// extract dio channel's boolean state from uint[2] array

////////////////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLING
// These examples employ very simple error handling: if an error is detected, the example 
// functions will immediately return an error code. This behavior may not be suitable for some 
// real-world applications but it makes the code easier to read and understand. In a real
// application, it's likely that additional actions would need to be performed. The examples use
// the following X826 macro to handle API function errors; it calls an API function and stores
// the returned value in errcode, then returns immediately if an error was detected.

#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

// end of hearder fuction declarations


////////////////////////////////////////////////////////////////////////////////////////////////
// Globale veriable declaration

// Encoder read

uint counts = 0;								// encoder counts when the snapshot was captured
uint counts_pre =0;
uint timestamp;								// time the snapshot was captured in us. converted to s by /10^6
uint timestamp_start;
uint timestamp_previous;							// time the previous snapshot was captured 
uint aout = 3;        								// aout= analogue output channel no.2
uint setpoint;
uint range;
int errcode = S826_ERR_OK;
int steps = 0;

//for loop oparation
long long int i ;

volatile double function_exicution_time = 0.0;	
static double dt = 0.0;
static double F_ref = 0.0;								
static double F_ref_max = 0.0;
static double F_ref_min = 0.0;
static double ramp_rate = 0.01;
static double X_res = 0.0;
static double X_ref = 0.0;
static double X_sum = 0.0;

// motor drive oparation	
unsigned int analog_out_motor;	
static double I_a_ref = 0.0;
static double I_motor = 0.0;

// motor parameters
static double K_fn = 47.0;								// motor force constent
static double M_n = 2.817;								// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green 											//bearing and shft connector)=600g and (whight hammer head)=55g .   Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g . Changed the hammer head mass to 43 from 57 from elctronic balance, thus the new mass =  0.655 - 55 + 43 =0.643g 
static double friction = 0.0;								// +++ Eventhough friction is assumed as 0 there is a 1N, the motor shaft doesn't move tll there is 1N value shown in the 											//spring balance

// Velocity mesurment
static double velocity_sum = 0.0;
static double velocity = 0.0;
static double velocity_prev = 0.0;								// motor velocity
static double g_velocity = 300.0;							// velocity filter constent
static double position_x = 0.0;							// motor physical position
static double position_x_prevous = 0.0;
static double position_dx = 0.0;

// DOb
static double dob_input = 0.0;
static double dob_filter_input = 0.0;
static double dob_filter_output = 0.0;
int const g_dis = 200.0;							//Consider g_rec = g_dis
static double dob_force = 0.0;

// RTOB
static double rtob_filter_input = 0.0;
static double rtob_filter_output = 0.0;
static double rtob_force = 0.0;

// PID
static double K_p = 2000.0; //1900.0; //1500.0; //1200; //1000.0;
static double K_I = 150.0;
static double K_d = 200.0; //100.0;

//Generate force response
static double K_s = 2.0;	

//create array
char* Value ;

// end of veriable declarations

////////////////////////////////////////////////////////////////////////////////////////////////
// Function declaration

static float VelocityCalculation();
static int ControlLoop(uint board);
static int DemoWatchdog(uint board);
static int MotorOutDAC(uint board);
static int DoB();
static int RToB();
static int dtCalculation();

// end of function declaration

////////////////////////////////////////////////////////////////////////////////////////////////
// Main function.
int main(int argc, char **argv)
{
	uint board      = 0;							// 1 board, named as 0, change this if you want to use other than board number 0
	int errcode     = S826_ERR_OK;	
	int boardflags  = S826_SystemOpen();					// open 826 driver and find all 826 boards

	if (argc > 1)
        board = atoi(argv[1]);

	if (boardflags < 0)
        	errcode = boardflags;                       						// problem during open
    	else if ((boardflags & (1 << board)) == 0) 
	{
		int i;
        	printf("TARGET BOARD of index %d NOT FOUND\n",board);				// driver didn't find board you want to use
        	for (i = 0; i < 8; i++)
		{
			if (boardflags & (1 << i))
			{
				printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            		}
        	}
	} else  
	{
		// running functions
		X826( ControlLoop(board)			);					// read the endocer value, output anlog value and file write
		X826( DemoWatchdog(board)           );					// watchdog timer
	}

	switch (errcode)
	{
		case S826_ERR_OK:           break;
		case S826_ERR_BOARD:        printf("Illegal board number"); break;
		case S826_ERR_VALUE:        printf("Illegal argument"); break;
		case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
		case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
		case S826_ERR_DRIVER:       printf("Driver call failed"); break;
		case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
		case S826_ERR_DUPADDR:      printf("Two boards have same number");break;S826_SafeWrenWrite(board, 0x02);
		case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
		case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
		case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
		default:                    printf("Unknown error"); break;
	}
    
#ifndef _LINUX	
	printf("\nKeypress to exit ...\n\n");
	while (!_kbhit());
	_getch();
#endif

	S826_SystemClose();
	return 0;
}
// end of main function


////////////////////////////////////////////////////////////////////////////////////////////////
// ENCODER READ ANLOG OUT AND FILE WRITE FUCNTION

static int ControlLoop(uint board)
{
	// ***Configure interfaces and start them running.
		
	X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_0_5, 0));		// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation , 
	
	X826( S826_CounterModeWrite(board, 3, S826_CM_K_QUADX4));  			// Configure counter0 as incremental encoder interface. quadrature

	X826( S826_CounterStateWrite(board, 3, 1));  						// Start tracking encoder position.	
	
	I_motor = 0.0;	
	position_x = 0.0; 													//stop the mortor input current
	
	MotorOutDAC(board);  //- remove
	X826( S826_CounterSnapshot(board, 3) );     					    				// Trigger snapshot on counter 4.
	X826( S826_CounterSnapshotRead(board, 3, &counts, &timestamp_start, NULL, 0));      // Read the snapshot: from the counter4
	

	
	// First force loop
	F_ref_max = 3.0 ;		// 6 , 9 , 12 , 15 
	F_ref_min = 0.0;
	
	ramp_rate = 1;		// 2.1 , 2.4 , 2.7 , 3.0				// to change the force ramping 
	F_ref = 0.0;
	I_a_ref = 0.0;
	
	
	timestamp_previous = timestamp_start;
	
	X826( S826_CounterSnapshot(board, 3) 						);     // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, 3, &counts, &timestamp, NULL, 0) 	);     // Read the snapshot:receive the snapshot info here;no need to wait for snapshot;it's already been captured
	
	// Important to calculate dt, velocity before dob, rtob calculations			
	dtCalculation();	
	VelocityCalculation();
	

	
//////////////////////////F=3/////////////////////////////////////////////////////

	//zero
	F_ref_max = 9.0 ;		// 6 , 9 , 12 , 15 
	// ****file write operation
	
//	fp = fopen("testdata_V02_3.txt", "a+");
	
	float reading[335884];
	int i = 0;
	FILE *fp;										// file pointer file file operation
	
	if (fp = fopen("sliced_data.txt", "r"))
	{
		while(fscanf(fp, "%f", &reading[i]) != EOF)
		{
			i++;
		}
		fclose(fp);
		reading[i] = '\0';
	}
	
	
	//fprintf(fp,"timestamp_start,timestamp,timestamp_previous,function_exicution_time,dt,F_ref,rtob_force,dob_force,counts,steps,position_x,velocity,I_motor,I_a_ref,analog_out_motor\n");
	// double X_res
	// double X_ref

	//K_p = 500;
	//K_I = 10; //5.0;
	//K_d = 0.1; //3.16/10;
	FILE *fp2 = fopen("/home/linux/Documents/fyp2023-1/Data/Large_Motor_26.txt","a+");


	for(i=0;i<800000;i++)
	{

//		X_ref = reading[i]/1000000*10;	
//		printf("%f\n",X_ref);
//		X_ref = 0;
//		X_ref = 0.01*sin(2*PI*1*(i*0.00002));
		X_ref = X_ref + (dt * 0.001);
		
//		if(i<200000)
//		{
//			dtCalculation();  							// calculate dt, velocity, rotob_force, dob_force according to the existing force command
//			VelocityCalculation();	
//			DoB();
//			RToB();	
//			continue;
//		}

//		printf("%d\n",i);
		
		
		dtCalculation();  							// calculate dt, velocity, rotob_force, dob_force according to the existing force command
		VelocityCalculation();	
		DoB();
//		RToB();	
		
																
		X826( S826_CounterSnapshot(board, 3) 									);     	// Trigger snapshot on counter 0.
		X826( S826_CounterSnapshotRead(board, 3, &counts, &timestamp, NULL, 0) 	);		// Read the snapshot:receive the snapshot info here; no need to wait for snapshot; it's already been captured
		
		steps = counts;
//		F_ref = 3.0;
//		I_a_ref = K_p*(F_ref - rtob_force)*(M_n/K_fn);
//		I_motor = I_a_ref + (dob_force/K_fn);
//		I_motor = 0.2;

		//Position Controller
		X_res = steps*0.000001;
		X_sum = X_sum + (X_ref - X_res)*dt;
		I_a_ref = (K_p*(X_ref - X_res)+ K_I*X_sum + K_d*(velocity-velocity_prev))*(M_n/K_fn);
//		//if(i>10000){
		I_motor = -I_a_ref;// + (dob_force/K_fn);
		//} else {
		//	I_motor = I_a_ref;
		//}
		
//		printf("%f, %f\n", I_a_ref, X_res-X_ref);
		printf("%f,%d,%f,%f\n", I_motor,steps,X_ref,X_res);
		fprintf(fp2,"%d,%f,%f,%f\n",timestamp,X_ref,X_res,dt);
		MotorOutDAC(board);						// the mortor input current
		
	}

	fclose(fp2);
	
	
	
	
////////////////////////////////////////End F=3 /////////////////////

		
	
	// Injecting nevagive current, stopping cycle
	F_ref = 0.0;
	I_motor = 0.0;
	I_a_ref = 0.0;									//stop the mortor input current
	MotorOutDAC(board);
	X826( S826_CounterStateWrite(board, 3, 0)	); 				// Stop tracking encoder position.

	return errcode;
}
// end of main function
////////////////////////////////////////////////////////////////////////////////////////////////


// dt calculation 
static int dtCalculation()
{
//	dt = 20/1000000;
	function_exicution_time = (float)(timestamp - timestamp_start)/1000000;     //Timestamp in us , /10^6 to convert to s
	dt =(float)(timestamp - timestamp_previous)/1000000;
	timestamp_previous = timestamp;
	return 0;
}

// velocitycalculation fucntions with function
static float VelocityCalculation()
{
	steps = counts;								// Correcting the encorder read , + to the direction of squeezing 
	position_x = steps*0.000005;
	velocity_sum = velocity_sum + velocity*dt;
	velocity = g_velocity * (position_x - velocity_sum);
	velocity_prev = velocity;
	counts_pre = counts;
	return 0;
}

// DOB
static int DoB()
{
	dob_input = velocity * g_dis * M_n;
	dob_filter_input = K_fn * I_motor + dob_input;
	dob_filter_output = dob_filter_output + g_dis*(dob_filter_input - dob_filter_output)*dt;
	dob_force = dob_filter_output - dob_input;
	return 0;
}


// RTOB
static int RToB()
{
	rtob_filter_input = dob_filter_input - friction;
	rtob_filter_output = rtob_filter_output + g_dis*(rtob_filter_input - rtob_filter_output)*dt;
	rtob_force = rtob_filter_output - dob_input;
	return 0;
}

// Motor drive input volate genarating function, using dac
static int MotorOutDAC(uint board)
{	
	if (I_motor < -0.7)
	{
		I_motor = -0.7;
	}else if(I_motor > 0.7)
	{
		I_motor = 0.7;
	}
	analog_out_motor = (unsigned int)((65535*0.5) + ((I_motor*65535)/(0.6*2)));	// DAC senstivity 16 bit (2**16)movo zero vlaue 0 --> 0V out in dac (-5 - 5V)
	//analog_out_motor = (unsigned int)(32768 + (I_motor/0.6)*32768);		// DAC senstivity 16 bit (2**16)movo zero vlaue 0 -->  2.5V out in dac
	X826( S826_DacDataWrite(board, aout, analog_out_motor, 0) );			// Analog value out from dac
	//X826( S826_DacDataWrite(board, chan, analog_out_motor, 0) );		// Analog value out from dac

	

	/*
	X826( S826_DacRead(board, aout, &range, &setpoint, 0) );
	
	printf("/////\n");
	printf("analog_out_motor :%d \n",analog_out_motor);
	printf("----- 	\n");
	printf("aout :%d \n",aout);
	printf("setpoint :%d \n",setpoint);
	printf("range :%d \n",range);
	X826( S826_DacRead(board, chan, &range, &setpoint, 0) );
	printf("chan :%d \n",chan);
	printf("setpoint :%d \n",setpoint);
	printf("range :%d \n",range);
	*/

	return 0;									// negativa current - negative cont, Positive current positive counter incrimet
}



////////////////////////////////////////////////////////////////////////////////////////////////
// The demo.
static int DemoWatchdog(uint board)
{
	int rc;
	uint timing[5];
	// set timer 1 time-out to around 1 second
	timing[0] = 55000000;
	printf("\nDemoWatchdog\n");
	// enable writing to watchdog
	rc = S826_SafeWrenWrite(board, 0x02);
	if (rc != 0)
	{
		printf("failed to enable wren for watchdog\n");
		return rc;
	}
	rc = S826_WatchdogConfigWrite(board, 0x00, timing);
	
	if (rc != 0)
	{
		printf("failed to configure WD\n");
		return rc;
	}

	rc = S826_WatchdogEnableWrite(board, 1);
	if (rc != 0)
	{
		printf("failed to enable WD\n");
		return rc;
	}
    
	// commented out.  for testing watchdog cancel
	// CreateThread(NULL, 4096, testThread, board, 0, NULL);

	// watch indefinitely for watchdog
	rc = S826_WatchdogEventWait(board, S826_WAIT_INFINITE);

	switch (rc)
	{
		case 0:
			printf("watchdog successfully expired\n");
			break;
		case S826_ERR_NOTREADY:
			printf("WD wait timed out\n");
			break;
		case S826_ERR_CANCELLED:
			printf("WD wait cancelled\n");
			break;
		default:
		printf("error %d\n", rc);
        
	}
	// disable watchdog
	rc = S826_WatchdogEnableWrite(board, 0);
	if (rc != 0)
	{
		printf("failed to disable WD\n");
		return rc;
	}
	return 0;
}


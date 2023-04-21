////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSORAY MODEL 826 PROGRAMMING EXAMPLES
// This file contains simple functions that show how to program the 826.
// Copyright (C) 2012 Sensoray
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FYP project setup - R0 
// K.D.M. Jayawardhana
// 21.02.2023
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Header functions declarations

#ifndef _LINUX
#include <windows.h>
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

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

// Counter ports initialization
uint counter_platform_motor = 3;			
uint counter_rotory_motor = 5;
uint counter_master_motor = 4;


// Encoder count 
uint counts_plotform_motor = 0;							// encoder counts when the snapshot was captured in platform motor
uint counts_rotory_motor = 0;
uint counts_master_motor = 0;

// 
uint timestamp;											// time the snapshot was captured
uint timestamp_pltform;
uint timestamp_rotory;
uint timestamp_start;
uint timestamp_previous;								// time the previous snapshot was captured 
uint aout = 0;        									// output duplicate waveform on this dac channel
uint aout_M = 2;
int errcode = S826_ERR_OK;
int steps = 0;
int steps_rot = 0;

//for loop oparation
long long int i ;

static double function_exicution_time = 0.0;
static double dt = 0.0;
static double F_ref = 0.0;
static double F_ref_max = 0.0;
static double F_ref_min = 0.0;
static double ramp_rate = 0.0;
static double X_ref = 0;
static double X_res = 0;
static double X_sum = 0;

// motor drive oparation	
unsigned int analog_out_motor;
unsigned int analog_out_master;	
static double I_a_ref = 0.0;
static double I_motor = 0.0;
static double I_motor_M = 0.0;

// motor parameters
static double K_tn = 24.0;								// motor force constent
static double M_n = 0.643;								// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green 											//bearing and shft connector)=600g and (whight hammer head)=55g .   Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g . Changed the hammer head mass to 43 from 57 from elctronic balance, thus the new mass =  0.655 - 55 + 43 =0.643g 
static double friction = 0.0;

// Velocity mesurment
static double velocity_sum = 0.0;
static double velocity = 0.0;										// motor velocity
static double velocity_prev = 0.0;
static double g_velocity = 300.0;									// velocity filter constent
static double position_x = 0.0;										// motor physical position
static double position_x_prevous = 0.0;
static double position_dx = 0.0;

// DOb
static double dob_input = 0.0;
static double dob_filter_input = 0.0;
static double dob_filter_output = 0.0;
int const g_dis = 200.0;
static double dob_torque = 0.0;

// RTOB
static double rtob_filter_input = 0.0;
static double rtob_filter_output = 0.0;
static double rtob_torque = 0.0;

// PID
const double K_p = 1400.0;	//900.0;
const double K_I = 0.0;		//0.0;
const double K_D = 120.0;	//120.0;

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
	uint board      = 0;	// change this if you want to use other than board number 0
	int errcode     = S826_ERR_OK;	
	int boardflags  = S826_SystemOpen();	// open 826 driver and find all 826 boards

	if (argc > 1)
        board = atoi(argv[1]);

	if (boardflags < 0)
        	errcode = boardflags;                       // problem during open
    	else if ((boardflags & (1 << board)) == 0) 
	{
		int i;
        	printf("TARGET BOARD of index %d NOT FOUND\n",board);// driver didn't find board you want to use
        	for (i = 0; i < 8; i++)
		{
			if (boardflags & (1 << i))
			{
				printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            		}
        	}
	} else  
	{	// running functions
		X826( ControlLoop(board)); // read the endocer value, output anlog value and file write
		X826( DemoWatchdog(board)); // watchdog timer
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
		case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
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
// JKD - 21.02.2023

//	uint counter_platform_motor = 3;			
//	uint counter_rotory_motor = 5;
//	uint counter_master_motor = 4;



// Encoder count 
//	uint counts_plotform_motor = 0;							// encoder counts when the snapshot was captured in platform motor
//	uint counts_rotory_motor = 0;
//	uint counts_master_motor = 0;

static int ControlLoop(uint board)
{	// ***Configure interfaces and start them running.
	//X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_10_10, 0)); // program dac output range: -10V to +10V
	X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_0_5, 0));// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation
	X826( S826_DacRangeWrite(board, aout_M, S826_DAC_SPAN_0_5, 0));// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation
	
	X826( S826_CounterModeWrite(board, counter_master_motor, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_master_motor, 1) ); // Start tracking encoder position.
	X826( S826_CounterModeWrite(board, counter_rotory_motor, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_rotory_motor, 1) ); // Start tracking encoder position.	

	I_motor = 0.0;	//stop the motor input current
	I_motor_M = 0.0;
	MotorOutDAC(board);
	
	X826( S826_CounterSnapshot(board, counter_master_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_master_motor, &counts_master_motor, &timestamp_start, NULL, 0));// Read the snapshot:
	printf("Timestamp:%d \n",timestamp_start);
	printf("counts:%d \n",counts_plotform_motor);

	X826( S826_CounterSnapshot(board, counter_rotory_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_rotory_motor, &counts_rotory_motor, &timestamp, NULL, 0));// Read the snapshot:
	printf("Timestamp:%d \n",timestamp);
	printf("counts:%d \n",counts_plotform_motor);

	dtCalculation();

	// ****file write operation
	FILE *fp;// file pointer file file operation
	fp = fopen("V_9.txt", "a+");


//	To remove the startup error we can can run 10 cycles of data

//	for(i=0;i<5;i++)
//	{
//		dtCalculation();
//		VelocityCalculation();
		
//		MotorOutDAC(board); // the mortor input current
//		X826( S826_CounterSnapshot(board, counter_rotory_motor)); 									// Trigger snapshot on counter counter_rotory_motor.
//		X826( S826_CounterSnapshotRead(board, 0, &counts_rotory_motor, &timestamp, NULL, 0));		// read the data 
//	}

	// 1

//	K_p = 500.0;
//	K_I = 10.0; //5.0;
//	K_D = 0.1; //3.16/10;


	for(i=0;i<500000;i++)
	{	
//		F_ref = 1;
//
//		if (F_ref < F_ref_max)
//		{
//			F_ref = F_ref + dt * ramp_rate;
//		}else
//		{
//			F_ref = F_ref;
//		}
//		I_a_ref = K_p*(F_ref - rtob_torque)*(M_n/K_tn);
//		I_motor = I_a_ref + (dob_torque/K_tn);
//		DoB();
//		RToB();
//		dtCalculation();
//		VelocityCalculation();
		I_motor = 0.1;
	
		MotorOutDAC(board);	// the mortor input current

		X826( S826_CounterSnapshot(board, counter_master_motor)); // Trigger snapshot on counter 0.
		X826( S826_CounterSnapshot(board, counter_rotory_motor)); // Trigger snapshot on counter 0.

		X826( S826_CounterSnapshotRead(board, counter_master_motor, &counts_master_motor, &timestamp, NULL, 0));// Read the snapshot:
		
		X826( S826_CounterSnapshotRead(board, counter_rotory_motor, &counts_rotory_motor, &timestamp, NULL, 0));// Read the snapshot:
		

		steps_rot = counts_rotory_motor;
		
		steps = counts_master_motor;

		dtCalculation();
		VelocityCalculation();
		DoB();
		RToB();
		
		// Linear Master Position Control
		
		X_ref = steps_rot*0.000005*10;
		X_res = steps*0.000005;
		X_sum = X_sum + (X_ref - X_res)*dt;
		I_a_ref = (K_p*(X_ref - X_res)+ K_I*X_sum+ K_D*(velocity-velocity_prev))*(M_n/K_tn);
		I_motor_M = I_a_ref + (dob_torque/K_tn);
//		printf("%d,%d\n",analog_out_master,analog_out_motor);
		fprintf(fp,"%d,%d,%d,%f\n",timestamp,counts_rotory_motor,counts_master_motor,dob_torque);

		MotorOutDAC(board);
		velocity_prev = velocity;
//		printf("%f\n",I_a_ref);

	}

	

	fclose(fp);

	// Injecting nevagive current, stopping cycle
	F_ref = 0.0;
	I_motor = 0.0;
	I_motor_M = 0.0;
	I_a_ref = 0.0;//stop the motor input current
	MotorOutDAC(board);
	X826( S826_CounterStateWrite(board, counter_master_motor, 0)); // Stop tracking encoder position.
	X826( S826_CounterStateWrite(board, counter_rotory_motor, 0)); // Stop tracking encoder position.

	return errcode;
} // end of main function
////////////////////////////////////////////////////////////////////////////////////////////////

// Motor drive input volate genarating function, using dac
static int MotorOutDAC(uint board)
{	
	if (I_motor < 0)
	{
		I_motor = 0;
	}else if(I_motor > 1.0)
	{
		I_motor = 1.0;
	}

		if (I_motor_M < -0.59)
	{
		I_motor_M = -0.59;
	}else if(I_motor_M > 0.59)
	{
		I_motor = 0.59;
	}
//	analog_out_motor = (unsigned int)(32768 + (I_motor/0.6)*32768);			// DAC senstivity 16 bit (2**16)movo zero vlaue 0 -->  2.5V out in dac
	analog_out_motor = (unsigned int)(I_motor*65635/1.5);
	analog_out_master = (unsigned int)((65535*0.5) + ((I_motor_M*65535)/(0.6*2)));					// DAC senstivity 16 bit (2**16)movo zero vlaue 0 -->  0V out in dac
	X826( S826_DacDataWrite(board, aout, analog_out_motor, 0) );		// Analog value out from dac
	X826( S826_DacDataWrite(board, aout_M, analog_out_master, 0) );
	return 0;															// negativa current - negative cont, Positive current positive counter incrimet
}

// velocitycalculation fucntions with function
static float VelocityCalculation()
{
	position_x = steps*0.000005;
	velocity_sum = velocity_sum + velocity*dt;
	velocity = g_velocity * (position_x - velocity_sum);
	
	return 0;
}


// DOB
static int DoB()
{
	dob_input = velocity * g_dis * M_n;
	dob_filter_input = K_tn * I_motor_M + dob_input;
	dob_filter_output = dob_filter_output + g_dis*(dob_filter_input - dob_filter_output)*dt;
	dob_torque = (dob_filter_output - dob_input);
	return 0;
}


// RTOB
static int RToB()
{
	rtob_filter_input = dob_filter_input - friction;
	rtob_filter_output = rtob_filter_output + g_dis*(rtob_filter_input - rtob_filter_output)*dt;
	rtob_torque = rtob_filter_output - dob_input;
	return 0;
}

// dt calculation 
static int dtCalculation()
{
	// function_exicution_time = (float)(timestamp - timestamp_start)/1000000;
	// dt =(float)(timestamp - timestamp_previous)/1000000;
	// timestamp_previous = timestamp;
	dt = 19/1e6;
	return 0;
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
 
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

#define PI 3.141592

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
uint counts_platform_motor = 0;							// encoder counts when the snapshot was captured in platform motor
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
uint aout_P = 3;
int errcode = S826_ERR_OK;
int steps = 0;
int steps_rot = 0;
int steps_plat = 0;

//for loop oparation
long long int i ;

static double function_exicution_time = 0.0;
static double dt = 0.0;
static double F_ref = 0.0;
static double F_ref_R = 0.0;
static double F_ref_max = 0.0;
static double F_ref_min = 0.0;
static double ramp_rate = 0.0;
static double X_ref = 0;
static double X_res = 0;
static double X_sum = 0;
static double X_error = 0;
static double f_error = 0;
static double out_PD = 0;

// motor drive oparation	
unsigned int analog_out_motor;
unsigned int analog_out_master;
unsigned int analog_out_platform;	
static double I_a_ref = 0.0;
static double I_a_ref_M = 0.0;
static double I_a_ref_P = 0.0;
static double I_motor = 0.0;
static double I_motor_M = 0.0;
static double I_motor_P = 0.0;

// motor parameters
static double K_tn_M = 24.0;								// motor force constent
static double M_n = 0.643;								// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green 											//bearing and shft connector)=600g and (whight hammer head)=55g .   Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g . Changed the hammer head mass to 43 from 57 from elctronic balance, thus the new mass =  0.655 - 55 + 43 =0.643g 
static double friction_M = 0.0;

static double K_tn = 0.135;								// motor force constent
static double J_n = 0.268/10000;
static double M_n_R = (0.268/10000)/(0.01591*0.01591);								// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green 											//bearing and shft connector)=600g and (whight hammer head)=55g .   Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g . Changed the hammer head mass to 43 from 57 from elctronic balance, thus the new mass =  0.655 - 55 + 43 =0.643g 
static double friction = 0.0;

static double K_tn_P = 47.0;								// motor force constent
static double M_n_P = 2.817;								// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green 											//bearing and shft connector)=600g and (whight hammer head)=55g .   Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g . Changed the hammer head mass to 43 from 57 from elctronic balance, thus the new mass =  0.655 - 55 + 43 =0.643g 
static double friction_p = 0.0;

// Velocity mesurment
static double velocity_sum = 0.0;
static double velocity = 0.0;										// motor velocity
static double velocity_prev = 0.0;
static double g_velocity = 40.0;									// velocity filter constent
static double position_x = 0.0;										// motor physical position
static double position_x_prevous = 0.0;
static double position_dx = 0.0;

static double velocity_sum_R = 0.0;
static double velocity_R = 0.0;										// motor velocity
static double velocity_prev_R = 0.0;
static double g_velocity_R = 400.0;									// velocity filter constent
static double position_x_R = 0.0;										// motor physical position
static double position_x_prevous_R = 0.0;
static double position_dx_R = 0.0;

static double velocity_sum_P = 0.0;
static double velocity_P = 0.0;										// motor velocity
static double velocity_prev_P = 0.0;
static double g_velocity_P = 400.0;
static double position_x_P = 0.0;										// motor physical position
static double position_x_prevous_P = 0.0;
static double position_dx_P = 0.0;


// DOb
static double dob_input = 0.0;
static double dob_filter_input = 0.0;
static double dob_filter_output = 0.0;
int const g_dis = 150.0;
static double dob_torque = 0.0;

static double dob_input_R = 0.0;
static double dob_filter_input_R = 0.0;
static double dob_filter_output_R = 0.0;
int const g_dis_R = 200.0;
static double dob_torque_R = 0.0;

// RTOB
static double rtob_filter_input = 0.0;
static double rtob_filter_output = 0.0;
static double rtob_torque = 0.0;

static double rtob_filter_input_R = 0.0;
static double rtob_filter_output_R = 0.0;
static double rtob_torque_R = 0.0;

// PID
const double K_p_m = 900.0; //1400.0;
const double K_d_m = 4000.0; //120.0;


const double K_p = 0.0;	//2.0;

const double K_p_P = 2750.0; //2000.0;	//900.0;
const double K_I_P = 150.0;		//0.0;
const double K_D_P = 450.0;	//350.0; //120.0;

const double K_p_R = 500.0;	//900.0;

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
	X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_5_5, 0));// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation
	X826( S826_DacRangeWrite(board, aout_M, S826_DAC_SPAN_0_5, 0));// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation
	X826( S826_DacRangeWrite(board, aout_P, S826_DAC_SPAN_0_5, 0));		// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation , 

	X826( S826_CounterModeWrite(board, counter_master_motor, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_master_motor, 1) ); // Start tracking encoder position.
	X826( S826_CounterModeWrite(board, counter_rotory_motor, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_rotory_motor, 1) ); // Start tracking encoder position.	
	X826( S826_CounterModeWrite(board, counter_platform_motor, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_platform_motor, 1) ); // Start tracking encoder position.	

	I_motor = 0.0;	//stop the motor input current
	I_motor_M = 0.0;
	I_motor_P = 0.0;
	MotorOutDAC(board);
	
	X826( S826_CounterSnapshot(board, counter_master_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_master_motor, &counts_master_motor, &timestamp_start, NULL, 0));// Read the snapshot:
	printf("Timestamp:%d \n",timestamp_start);
	printf("counts:%d \n",counts_master_motor);

	X826( S826_CounterSnapshot(board, counter_rotory_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_rotory_motor, &counts_rotory_motor, &timestamp, NULL, 0));// Read the snapshot:
	printf("Timestamp:%d \n",timestamp);
	printf("counts:%d \n",counts_rotory_motor);

	X826( S826_CounterSnapshot(board, counter_platform_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_platform_motor, &counts_platform_motor, &timestamp, NULL, 0));// Read the snapshot:
	printf("Timestamp:%d \n",timestamp);
	printf("counts:%d \n",counts_platform_motor);

	timestamp_previous = timestamp_start;

	dtCalculation();

	// ****file write operation
	FILE *fp = fopen("/home/linux/Documents/fyp2023-1/Data/IM_176.txt","a+");
//	FILE *fp = fopen("/home/linux/Documents/fyp2023-1/Data/t4.txt","a+");


	for(i=0;i<200000;i++)
	{	
		X_ref = X_ref + (dt * (-0.005));


		X826( S826_CounterSnapshot(board, counter_master_motor)); // Trigger snapshot on counter 0.
		X826( S826_CounterSnapshot(board, counter_rotory_motor)); // Trigger snapshot on counter 0.
		X826( S826_CounterSnapshot(board, counter_platform_motor)); // Trigger snapshot on counter 0.


		X826( S826_CounterSnapshotRead(board, counter_master_motor, &counts_master_motor, &timestamp, NULL, 0));// Read the snapshot:
		X826( S826_CounterSnapshotRead(board, counter_rotory_motor, &counts_rotory_motor, &timestamp, NULL, 0));// Read the snapshot:
		X826( S826_CounterSnapshotRead(board, counter_platform_motor, &counts_platform_motor, &timestamp, NULL, 0) 	);		// Read the snapshot:receive the snapshot info here; no need to wait for snapshot; it's already been captured


		steps_rot = counts_rotory_motor;
		steps = counts_master_motor;
		steps_plat = counts_platform_motor;

		dtCalculation();
		VelocityCalculation();
		DoB();
		RToB();

		// Rotary Motor Force Controller

		F_ref_R = 0.8;		
//		I_a_ref = K_p_R*(F_ref_R*0.01591 - rtob_torque_R)/K_tn;
		I_a_ref = F_ref_R*0.01591/K_tn;
		I_motor = I_a_ref; //+ (dob_torque_R/K_tn);

		// Linear Master Motor Impedence Controller


		X_error = 5*position_x_R*0.01591 - position_x;
		out_PD = K_p_m*X_error + K_d_m*(velocity - velocity_prev); 
		f_error = -5*rtob_torque_R/0.01591 - rtob_torque;

		I_a_ref_M = (out_PD + K_p*f_error)*(M_n)/(K_tn_M);
		I_motor_M = I_a_ref_M + (dob_torque/K_tn_M);

		
		
		
		// Linear Platform Motor Position Control
		X_res = steps_plat*0.000001;
		X_sum = X_sum + (X_ref - X_res)*dt;
		I_a_ref_P = (K_p_P*(X_ref - X_res)+ K_I_P*X_sum + K_D_P*(velocity_P-velocity_prev_P))*(M_n_P/K_tn_P);
		I_motor_P = -I_a_ref_P;

	
		
//		X_ref = steps_rot*0.000005*10;
//		X_res = steps*0.000005;
//		X_sum = X_sum + (X_ref - X_res)*dt;
//		I_a_ref_M = (K_p*(X_ref - X_res)+ K_I*X_sum+ K_D*(velocity-velocity_prev))*(M_n/K_tn_M);
//		I_motor_M = I_a_ref_M + (dob_torque/K_tn_M);
		printf("%f,%f,%f,%f\n",dt,I_motor_P,rtob_torque,I_motor_M);
		fprintf(fp,"%d,%f,%f,%f,%f,%f,%f\n",timestamp,position_x_R*0.01591,position_x,rtob_torque_R/0.01591,rtob_torque,X_res,X_ref);
//		fprintf(fp,"%d,%f,%f\n",timestamp,X_ref,X_res);

		MotorOutDAC(board);
		velocity_prev = velocity;
		velocity_prev_R = velocity_R;
		velocity_prev_P = velocity_P;
//		printf("%f\n",I_a_ref);

	}

	

	fclose(fp);

	// Injecting nevagive current, stopping cycle
	F_ref = 0.0;
	I_motor = 0.0;
	I_motor_M = 0.0;
	I_motor_P = 0.0;
	I_a_ref = 0.0;//stop the motor input current
	MotorOutDAC(board);
	X826( S826_CounterStateWrite(board, counter_master_motor, 0)); // Stop tracking encoder position.
	X826( S826_CounterStateWrite(board, counter_rotory_motor, 0)); // Stop tracking encoder position.
	X826( S826_CounterStateWrite(board, counter_platform_motor, 0)); // Stop tracking encoder position.

	return errcode;
} // end of main function
////////////////////////////////////////////////////////////////////////////////////////////////

// Motor drive input volate genarating function, using dac
static int MotorOutDAC(uint board)
{	
	if (I_motor < 0.0) {I_motor = 0.0;} else if(I_motor > 1.0)	{I_motor = 1.0;}
	if (I_motor_M < -0.59) {I_motor_M = -0.59;} else if(I_motor_M > 0.59) {I_motor = 0.59;}
	if (I_motor_P < -0.7) {I_motor_P = -0.7;} else if (I_motor_P > 0.7) {I_motor_P = 0.7;}

//	analog_out_motor = (unsigned int)(32768 + (I_motor/0.6)*32768);			// DAC senstivity 16 bit (2**16)movo zero vlaue 0 -->  2.5V out in dac
	analog_out_motor = (unsigned int)((65535*0.5) + ((-I_motor*65535)/(1.0*2)));//(I_motor*65635);
	analog_out_master = (unsigned int)((65535*0.5) + ((-I_motor_M*65535)/(0.6*2)));					// DAC senstivity 16 bit (2**16)movo zero vlaue 0 -->  0V out in dac
	analog_out_platform = (unsigned int)((65535*0.5) + ((I_motor_P*65535)/(0.6*2)));	// DAC senstivity 16 bit (2**16)movo zero vlaue 0 --> 0V out in dac (-5 - 5V)
	X826( S826_DacDataWrite(board, aout, analog_out_motor, 0) );		// Analog value out from dac
	X826( S826_DacDataWrite(board, aout_M, analog_out_master, 0) );
	X826( S826_DacDataWrite(board, aout_P, analog_out_platform, 0) );			// Analog value out from dac
	return 0;															// negativa current - negative cont, Positive current positive counter incrimet
}

// velocitycalculation fucntions with function
static float VelocityCalculation()
{
	position_x = steps*0.000005;
	velocity_sum = velocity_sum + velocity*dt;
	velocity = g_velocity * (position_x - velocity_sum);

	position_x_R = steps_rot*PI/10000.0;
	velocity_sum_R = velocity_sum_R + velocity_R*dt;
	velocity_R = g_velocity_R * (position_x_R - velocity_sum_R);

	position_x_P = steps*0.000005;
	velocity_sum_P = velocity_sum_P + velocity_P*dt;
	velocity_P = g_velocity_P * (position_x_P - velocity_sum_P);
	
	return 0;
}


// DOB
static int DoB()
{
	dob_input = velocity * g_dis * M_n;
	dob_filter_input = K_tn_M * I_motor_M + dob_input;
	dob_filter_output = dob_filter_output + g_dis*(dob_filter_input - dob_filter_output)*dt;
	dob_torque = (dob_filter_output - dob_input);

	dob_input_R = velocity_R * g_dis_R * J_n;
	dob_filter_input_R = K_tn * I_motor + dob_input_R;
	dob_filter_output_R = dob_filter_output_R + g_dis_R*(dob_filter_input_R - dob_filter_output_R)*dt;
	dob_torque_R = (dob_filter_output_R - dob_input_R);
	return 0;

	
}


// RTOB
static int RToB()
{
	rtob_filter_input = dob_filter_input - friction_M;
	rtob_filter_output = rtob_filter_output + g_dis*(rtob_filter_input - rtob_filter_output)*dt;
	rtob_torque = rtob_filter_output - dob_input;

	rtob_filter_input_R = dob_filter_input_R - friction;
	rtob_filter_output_R = rtob_filter_output_R + g_dis_R*(rtob_filter_input_R - rtob_filter_output_R)*dt;
	rtob_torque_R = rtob_filter_output_R - dob_input_R;
	return 0;
}

// dt calculation 
static int dtCalculation()
{
	function_exicution_time = (float)(timestamp - timestamp_start)/1000000;
	dt =(float)(timestamp - timestamp_previous)/1000000;
	timestamp_previous = timestamp;
//	dt = 35/1e6;
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
 
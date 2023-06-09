////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSORAY MODEL 826 PROGRAMMING EXAMPLES
// This file contains simple functions that show how to program the 826.
// Copyright (C) 2012 Sensoray
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FYP project setup - R0 
// J.L. Dantanarayana & U.G.S. Kashmira
// 22.04.2023
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
uint aout_m = 2;
uint aout_r = 0;// output duplicate waveform on this dac channel
uint aout_p = 3;
int errcode = S826_ERR_OK;
int steps_m = 0;
int steps_r = 0;
int steps_p = 0;

//for loop oparation
long long int i ;

static double function_exicution_time = 0.0;
static double dt = 0.0;


// motor drive operation
unsigned int analog_out_motor_m;
static double I_a_ref_m = 0.0;
static double I_motor_m = 0.0;

unsigned int analog_out_motor_r;
static double I_a_ref_r = 0.0;
static double I_motor_r = 0.0;

unsigned int analog_out_motor_p;	
static double I_a_ref_p = 0.0;
static double I_motor_p = 0.0;


// motor parameters
static double K_tn_m = 24.0;										// Master motor force constent
static double M_n_m = 0.643;										// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g .
static double friction_m = 0.0;

float K_tn_r = 0.135/0.01591;										// Replica motor force constent
float J_n_r = 0.268/10000;
float M_n_r = (0.268/10000)/(0.01591*0.01591);										// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g .
float friction_r = 0.0;

static double K_tn_P = 47.0;								// motor force constent
static double M_n_P = 2.817;								// Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green 											//bearing and shft connector)=600g and (whight hammer head)=55g .   Mass : (Motor shaft, 2 x linera bearings, 1 x Long green bearing and shft connector 1 x short green bearing and shft connector)=600g and (whight hammer head)=55g . Changed the hammer head mass to 43 from 57 from elctronic balance, thus the new mass =  0.655 - 55 + 43 =0.643g 
static double friction_p = 0.0;

// Velocity mesurment
float velocity_sum_m = 0.0;
float velocity_m = 0.0;										// motor velocity
float velocity_m_prev = 0.0;
float position_x_m = 0.0;										// motor physical position
float position_x_m_prev = 0.0;
float position_dx_m = 0.0;

float velocity_sum_r = 0.0;
float velocity_r = 0.0;										// motor velocity	// velocity filter constent
float velocity_r_prev = 0.0;
float position_x_r = 0.0;										// motor physical position
float position_x_r_prev = 0.0;
float position_dx_r = 0.0;

float velocity_sum_p = 0.0;
float velocity_p = 0.0;										// motor velocity	// velocity filter constent
float velocity_p_prev = 0.0;
float position_x_p = 0.0;										// motor physical position
float position_x_p_prev = 0.0;
float position_dx_p = 0.0;
float X_ref_p = 0.0;
float X_res_p = 0.0;
float X_sum_p = 0.0;

float g_velocity = 300.0;									// velocity filter constent

// DOb
static double dob_input_m = 0.0;
static double dob_filter_input_m = 0.0;
static double dob_filter_output_m = 0.0;
static double dob_force_m = 0.0;

int const g_dis_m = 200;

static double dob_input_r = 0.0;
static double dob_filter_input_r = 0.0;
static double dob_filter_output_r = 0.0;
static double dob_torque_r = 0.0;

int const g_dis_r = 200;
int const g_dis_p = 200;

// RTOB
float rtob_filter_input_m = 0.0;
float rtob_filter_output_m = 0.0;
float rtob_force_m = 0.0;

float rtob_filter_input_r = 0.0;
float rtob_filter_output_r = 0.0;
float rtob_torque_r = 0.0;

// PID
float const C_f = 10.0;

float const K_p_m = 1400;
float const K_d_m = 60;

float const K_p_r = 900;
float const K_d_r = 10;

const double K_p_P = 2750.0; //2000.0;	//900.0;
const double K_I_P = 150.0;		//0.0;
const double K_D_P = 450.0;	//350.0; //120.0;


// Bilateral Controller
float x_error_m = 0.0;
float x_error_r = 0.0;
float x_error_m_prev = 0.0;
float x_error_r_prev = 0.0;
float alpha = 1.0;
float beta = 1.0;
float out_PD_m = 0.0;
float out_PD_r = 0.0;
float f_error = 0.0;
float f_m = 0.0;
float f_r = 0.0;

// end of variable declarations

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
// Bilateral Control with Excess force Reduction

static int ControlLoop(uint board)
{	// ***Configure interfaces and start them running.
	//X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_10_10, 0)); // program dac output range: -10V to +10V
	X826( S826_DacRangeWrite(board, aout_m, S826_DAC_SPAN_5_5, 0));// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation
	X826( S826_DacRangeWrite(board, aout_r, S826_DAC_SPAN_0_5, 0));// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation
	X826( S826_DacRangeWrite(board, aout_p, S826_DAC_SPAN_0_5, 0));		// program dac output range: -0V to +5V , motor 0 value is 2.5V / for motor drive operation , 
	
	X826( S826_CounterModeWrite(board, counter_master_motor, S826_CM_K_QUADX4)); // Configure counter as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_master_motor, 1) ); // Start tracking encoder position.
	X826( S826_CounterModeWrite(board, counter_rotory_motor, S826_CM_K_QUADX4)); // Configure counter as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_rotory_motor, 1) ); // Start tracking encoder position.	
	X826( S826_CounterModeWrite(board, counter_platform_motor, S826_CM_K_QUADX4)); // Configure counter0 as incremental encoder interface. quadrature
	X826( S826_CounterStateWrite(board, counter_platform_motor, 1) ); // Start tracking encoder position.	


	I_motor_m = 0.0;	//stop the motor input current
    I_motor_r = 0.0;	//stop the motor input current
	I_motor_p = 0.0;
	MotorOutDAC(board);
	
	X826( S826_CounterSnapshot(board, counter_master_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_master_motor, &counts_master_motor, &timestamp_start, NULL, 0));// Read the snapshot:
//	printf("Timestamp:%d \n",timestamp_start);
//	printf("counts:%d \n",counts_plotform_motor);

	X826( S826_CounterSnapshot(board, counter_rotory_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_rotory_motor, &counts_rotory_motor, &timestamp, NULL, 0));// Read the snapshot:
//	printf("Timestamp:%d \n",timestamp);
//	printf("counts:%d \n",counts_plotform_motor);

	X826( S826_CounterSnapshot(board, counter_platform_motor)); // Trigger snapshot on counter 0.
	X826( S826_CounterSnapshotRead(board, counter_platform_motor, &counts_platform_motor, &timestamp, NULL, 0));// Read the snapshot:

	timestamp_previous = timestamp_start;
	dtCalculation();

	// ****file write operation
	FILE *fp;// file pointer file file operation
	fp = fopen("/home/linux/Documents/texture_haptic/BLC_001.txt", "a+");

	// 1
	for(i=0;i<400000;i++)
	{	
	    X_ref_p = X_ref_p + (dt * (-0.005));
		
		X826( S826_CounterSnapshot(board, counter_master_motor));     // Trigger snapshot on counter 0.
		X826( S826_CounterSnapshot(board, counter_rotory_motor));       // Trigger snapshot on counter 0.
		X826( S826_CounterSnapshot(board, counter_platform_motor)); // Trigger snapshot on counter 0.
		
		X826( S826_CounterSnapshotRead(board, counter_master_motor, &counts_master_motor, &timestamp, NULL, 0));// Read the snapshot:
		X826( S826_CounterSnapshotRead(board, counter_rotory_motor, &counts_rotory_motor, &timestamp, NULL, 0));// Read the snapshot:
		X826( S826_CounterSnapshotRead(board, counter_platform_motor, &counts_platform_motor, &timestamp, NULL, 0) 	);		// Read the snapshot:receive the snapshot info here; no need to wait for snapshot; it's already been captured


		steps_r = counts_rotory_motor;
		steps_m = counts_master_motor;
		steps_p = counts_platform_motor;

		dtCalculation();
        VelocityCalculation();
        DoB();
        RToB();

        x_error_m = (position_x_r*alpha*0.01591 - position_x_m);
        out_PD_m = K_p_m * x_error_m + K_d_m * (velocity_m - velocity_m_prev);
        x_error_m_prev = x_error_m;

        x_error_r = (position_x_m - position_x_r*alpha);
        out_PD_r = K_p_r * x_error_r + K_d_r * (velocity_r - velocity_r_prev);
        x_error_r_prev = x_error_r;

       

        f_error =-rtob_force_m - beta * rtob_torque_r/0.01591;

        f_m = (out_PD_m * beta / (alpha + beta) * M_n_m + f_error * alpha / (alpha + beta) * C_f);
        f_r = (out_PD_r / (alpha + beta) * J_n_r + f_error/(alpha + beta) * C_f * 0.01591);

		// Linear Platform Motor Position Control
		X_res_p = steps_p*0.000001;
		X_sum_p = X_sum_p + (X_ref_p - X_res_p)*dt;
		I_a_ref_p = (K_p_P*(X_ref_p - X_res_p)+ K_I_P*X_sum_p + K_D_P*(velocity_p-velocity_p_prev))*(M_n_P/K_tn_P);
		I_motor_p = -I_a_ref_p;
        I_motor_m = (f_m + dob_force_m) / K_tn_m;
        I_motor_r = (f_r + dob_torque_r) / K_tn_r;

		MotorOutDAC(board);	// the motor input current

		velocity_m_prev = velocity_m;
		velocity_r_prev = velocity_r;
		velocity_p_prev = velocity_p;
		
		fprintf(fp,"%f,%d,%d\n",function_exicution_time,counts_plotform_motor,counts_rotory_motor);
	}

	fclose(fp);

	// Injecting nevagive current, stopping cycle
	I_motor_m = 0.0;
    I_motor_r = 0.0;
	I_a_ref_p = 0.0;

	MotorOutDAC(board);
	X826( S826_CounterStateWrite(board, counter_master_motor, 0)); // Stop tracking encoder position.
	X826( S826_CounterStateWrite(board, counter_rotory_motor, 0)); // Stop tracking encoder position.
	X826( S826_CounterStateWrite(board, counter_platform_motor, 0)); // Stop tracking encoder position.
	return errcode;
} // end of main function

////////////////////////////////////////////////////////////////////////////////////////////////
// Analog Voltage generation for Motor Driver (in Current control mode)
static int MotorOutDAC(uint board)
{	
	if (I_motor_r < -1.0) {I_motor_r = -1.0;} else if(I_motor_r > 1.0)	{I_motor_r = 1.0;}
	if (I_motor_m < -0.59) {I_motor_m = -0.59;} else if(I_motor_m > 0.59) {I_motor_m = 0.59;}
	if (I_motor_p < -0.7) {I_motor_p = -0.7;} else if (I_motor_p > 0.7) {I_motor_p = 0.7;}

	analog_out_motor = (unsigned int)((65535*0.5) + ((-I_motor_r*65535)/(1.0*2)));//(I_motor*65635);
	analog_out_master = (unsigned int)((65535*0.5) + ((-I_motor_m*65535)/(0.6*2)));					// DAC senstivity 16 bit (2**16)movo zero vlaue 0 -->  0V out in dac
	analog_out_platform = (unsigned int)((65535*0.5) + ((I_motor_p*65535)/(0.6*2)));	// DAC senstivity 16 bit (2**16)movo zero vlaue 0 --> 0V out in dac (-5 - 5V)

    X826( S826_DacDataWrite(board, aout_m, analog_out_motor_m, 0) );		// Analog value out from dac
    X826( S826_DacDataWrite(board, aout_r, analog_out_motor_r, 0) );		// Analog value out from dac
	X826( S826_DacDataWrite(board, aout_p, analog_out_motor_p, 0) );			// Analog value out from dac


    return 0;															// negativa current - negative cont, Positive current positive counter incrimet
}

// Sampling time calculator
static int dtCalculation()
{
    function_exicution_time = (float)(timestamp - timestamp_start)/1000000;
    dt =(float)(timestamp - timestamp_previous)/1000000;
    timestamp_previous = timestamp;
    return 0;
}

// Velocity Calculation for Master and Replica
static float VelocityCalculation()
{
	position_x_m = steps_m*0.000005;
	velocity_sum_m = velocity_sum_m + velocity_m*dt;
	velocity_m = g_velocity * (position_x_m - velocity_sum_m);

    position_x_r = steps_r*PI/10000.0;
    velocity_sum_r = velocity_sum_r + velocity_r*dt;
    velocity_r = g_velocity * (position_x_r - velocity_sum_r);

	position_x_p = steps*0.000005;
	velocity_sum_p = velocity_sum_p + velocity_p*dt;
	velocity_p = g_velocity_p * (position_x_p - velocity_sum_p);

	return 0;
}

// DOB
static int DoB()
{
	dob_input_m = velocity_m * g_dis_m * M_n_m;
	dob_filter_input_m = K_tn_m * I_motor_m + dob_input_m;
	dob_filter_output_m = dob_filter_output_m + g_dis_m*(dob_filter_input_m - dob_filter_output_m)*dt;
	dob_force_m = dob_filter_output_m - dob_input_m;

    dob_input_r = velocity_r * g_dis_r * J_n_r;
    dob_filter_input_r = K_tn_r * I_motor_r + dob_input_r;
    dob_filter_output_r = dob_filter_output_r + g_dis_r*(dob_filter_input_r - dob_filter_output_r)*dt;
    dob_torque_r = dob_filter_output_r - dob_input_r;
	return 0;
}

// RTOB
static int RToB()
{
	rtob_filter_input_m = dob_filter_input_m - friction_m;
	rtob_filter_output_m = rtob_filter_output_m + g_dis_m*(rtob_filter_input_m - rtob_filter_output_m)*dt;
	rtob_force_m = rtob_filter_output_m - dob_input_m;

    rtob_filter_input_r = dob_filter_input_r - friction_r;
    rtob_filter_output_r = rtob_filter_output_r + g_dis_r*(rtob_filter_input_r - rtob_filter_output_r)*dt;
    rtob_torque_r = rtob_filter_output_r - dob_input_r;
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

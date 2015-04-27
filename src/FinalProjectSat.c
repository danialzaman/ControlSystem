/*
 ============================================================================
 Name        : FinalProject.c
 Author      : Danial Zaman
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>
#include <errno.h>

#include "dlab.h"

pthread_t Control_thread;
sem_t data_avail;
#define MAXS 5000
float theta[MAXS];
float track[MAXS];
float ref[MAXS];
char input[100];
int no_of_points = 500;
double run_time = 3.0;
double Kp = 13.5;
float Fs = 200.0;
float Ti = 0.01;
float Td = 0.005675;
int N = 20.0;
int motor_number = 11;
float magnitude = 0;
float frequency = 0;
float dutyCycle = 0;

float satblk(float v) {
	if (v >= 1.4)
		v = 1.4;
	if (v <= -1.4)
		v = -1.4;
	if (v > -1.4 && v < 1.4)
		v = v;

	return (v);
}

void *Control(void *parameters) {
	float motor_position, eK = 0;
	float uK = 0;
	float pK;
	float vK;
	float aK;
	float iK;
	float dK;
	float iK_prev = 0;
	float dK_prev = 0;
	float eK_prev = 0;
	int k = 0;
	float T = 1 / Fs;
	int no_of_samples = (int) (run_time * Fs);
	double previous_error = 0;
	double integral = 0;
	while (k < MAXS) {
		sem_wait(&data_avail);
		motor_position = EtoR(ReadEncoder());
		eK = ref[k] - motor_position;
		uK = Kp * eK; //calculate control value
		iK = iK_prev + ((Kp * eK_prev * T) / Ti);
		dK = Td / (N * T + Td) * dK_prev + ((Kp * Td * N) / (N * T + Td)) * (eK - eK_prev);
		vK = pK + iK + dK;
		uK = satblk(vK);
		aK = uK - vK;
		DtoA(VtoD(uK));
		theta[k] = motor_position;
		k++;
		if (k == 4999){
			k =0;
		}
	}
	pthread_exit(NULL);
}

int main(void) {

	double temp;
	char selection;
	int x = 0;
	int i = 0;
	int no_of_samples = (int) (run_time * Fs);
	for (i = 0; i < no_of_samples; i++) {
		ref[i] = 50 * 3.14 / 180.0;
	}

	while (1) {

		printf("Please Select From The Following List:\n");
		printf("R: Run the Control Algorithm \n");
		printf("P: Change The Value Of Kp \n");
		printf("F: Change The Value Of The Sample Frequency \n");
		printf("T: Change The Value Of The Run Time \n");
		printf("I: Change The Value Of Ti \n");
		printf("D: Change The Value Of Td \n");
		printf("N: Change The Value Of N \n");
		printf("U: Change The Type Of Input (Square or Step) \n");
		printf("H: Save The Plot Results In Post Script or Display \n");
		printf("Q: Exit \n");
		selection = getchar();

		switch (selection) {
		case 'R':
			printf("Case is R \n");
			sem_init(&data_avail, 0, 0);
			if (Initialize(DLAB_SIMULATE, Fs, motor_number) != 0) {
				printf("Error in Initialize()\n");
				exit(99);
			}
			pthread_create(&Control_thread, NULL, &Control, NULL);
			//pthread_join(Control_thread, NULL);
			Terminate();
			sem_destroy(&data_avail);
			break;
		case 'U':
			printf("Please Specify Type of Input: Square on Step? \n");
			scanf("%s", input);
			if (strcmp(input, "Step") == 0) {
				printf("Input Magnitude? \n");
				scanf("%f", &magnitude);
				Square(ref, MAXS, Fs, magnitude * PI / 180.0, 0.1, 50);
			} else if (strcmp(input, "Square") == 0) {
				printf("Input Magnitude? \n");
				scanf("%f", &magnitude);
				printf("Input Frequency? \n");
				scanf("%f", &frequency);
				printf("Input Duty Cycle? \n");
				scanf("%f", &dutyCycle);
				Square(ref, MAXS, Fs, magnitude * PI / 180.0, frequency,
						dutyCycle);
			} else {
				printf("No Correct Input");
			}
			break;
		case 'P':
			sem_wait(&data_avail);
			printf("Input New Value of Kp? \n");
			scanf("%lf", &Kp);
			break;
		case 'F':
			sem_wait(&data_avail);
			printf("Input New Value of Sample Frequency? \n");
			scanf("%f", &Fs);
			break;
		case 'T':
			sem_wait(&data_avail);
			printf("Input New Value of Run Time? \n");
			scanf("%lf", &run_time);
			break;
		case 'I':
			sem_wait(&data_avail);
			printf("Input New Value of Ti? \n");
			scanf("%lf", &Ti);
			break;
		case 'D':
			sem_wait(&data_avail);
			printf("Input New Value of Td? \n");
			scanf("%lf", &Td);
			break;
		case 'N':
			sem_wait(&data_avail);
			printf("Input New Value of N? \n");
			scanf("%lf", &N);
			break;
		case 'H':
			sem_wait(&data_avail);
			printf("Plotting The Graph. \n");
			printf("Screen or Save? \n");
			scanf("%s", input);
			if (strcmp(input, "Screen") == 0) {
				printf("Screen");
				plot(ref, theta, Fs, no_of_samples, SCREEN, "Graph Title",
						"x-axis", "y-axis");
			} else if (strcmp(input, "Save") == 0) {
				printf("Saving File");
				plot(ref, theta, Fs, no_of_samples, PS, "Graph Title", "x-axis",
						"y-axis");
			}
			break;
		case 'Q':
			printf("Case is Q \n");
			exit(0);
		default:
			printf("No correct selection made \n");
			break;
		}
	}

}

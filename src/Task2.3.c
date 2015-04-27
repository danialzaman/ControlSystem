#include "dlab.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>
#include <errno.h>
#include <math.h>


#define MAXS 5000 
float theta[MAXS];
float ref[MAXS];  
float Kp, Fs,  run_time;
sem_t data_avail;
float Kp=13.25;
float run_time=10.0;
float Fs=200.0;
float Ti=0.01;
float Td=0.00286;
float Tt=0.01;
int N=20;
int motor_number=3; 

void *Control_thread_function(void *arg) 
{

int k; 
k = 0; 
int no_of_samples; 
float motor_position; 
no_of_samples = (int)(run_time*Fs); 
int motor_number=3;
	float ek,ek_prev=0.0;
	float uk;
	float vk;
	float ak;
	float pk;
	float ik;
	float ik_prev=0.0;
	float ik2;
	float dk,dk_prev=0.0;
	float T=1/Fs;
while(k < no_of_samples){
sem_wait(&data_avail); 
motor_position = EtoR(ReadEncoder()); 
ek = ref[k] - motor_position; 
uk = Kp*ek; //calculate control value
		ik=ik_prev+((Kp*ek_prev*T)/Ti);
		
		dk=Td/(N*T+Td)*dk_prev+((Kp*Td*N)/(N*T+Td))*(ek-ek_prev);
		
		uk=pk+ik+dk;

DtoA(VtoD(uk));
theta[k] = motor_position;// Input is Square //ion;
k++;
}

}

int main(void *arg)
{
 float mag, mag1, freq, duty; 
 char select; 
 int type; 
 int no_of_samples;
 pthread_t Control; 
int i,j,k,m; 
Kp = 18; // Initialize Kp to 1.
run_time = 10.0; // Set the initial run time to 3 seconds.
Fs = 200.0; // Set the initial sampling frequency to 200 Hz
motor_number = 3;
no_of_samples = (int)(run_time*Fs); 
Square(ref, no_of_samples, Fs, 50, 0.1, 50);


 
while(1){
printf("Selection Menu:\n");
printf("r: Run the control algorithm\n");
printf("p: Change value of Kp\n"); 
printf("f: Change value of sample freq\n"); 
printf("t: Change value of run time\n"); 
printf("u: Change the type of inputs(Step or Square)\n"); 
printf("For Step input, prompt for the magnitude of the step\n"); 
printf("For Square input, prompt for the magnitude, frequency and duty cycle\n"); 
printf("g: Plot results on screen\n"); 
printf("h: Save the Plot results in Postscript\n"); 
printf("u: Change the type of inputs.\n");
printf("q: exit\n");  
printf("Choose a selection from the selection menu:\n"); 
scanf("%c",&select); 

while(select != 'r' && select != 'p' && select != 'f' && select != 't' && select != 'u' && select != 'h' && select != 'q'){
//printf("Choose a selection from the selection menu:\n"); 
scanf("%c",&select); 
}

switch(select){
case 'r':
sem_init(&data_avail,0,0);
Initialize(DLAB_SIMULATE, Fs, motor_number);// Through the Initialize() DLaB function, an interrupt is produced every Ts=1/Fs seconds. DLAB_SIMULATE sets the op mode of dlab library functions to simulation mode
if (pthread_create(&Control, NULL, &Control_thread_function, NULL) != 0) 

{
      
 printf("Error creating Control thread.\n");
      
 exit(-1);
   
}
pthread_join(Control,NULL);
Terminate();
sem_destroy(&data_avail);
break;

case'u':
printf("Choose type of input (Step or Square):\n"); 
scanf("%d", &type); 
while(type != 0 && type != 1){
printf("Choose type of input (Step (0) or Square (1)):\n"); 
scanf("%d", &type); 
}


if(type == 0){  // Input is Step 
printf("Input is a Step.\n"); 
printf("Enter the magnitude:\n"); 
scanf("%f", &mag); 
for(i = 0; i<no_of_samples; i++){
ref[i] = mag*3.14/180.0;  // Conversion from degrees to radians 
}
}

if(type == 1){ // Input is Square 
printf("Input is a Square.\n"); 
printf("Enter the magnitude, frequency, and duty cycle:\n"); 
scanf("%f %f %f", &mag1, &freq, &duty); 
Square(ref, no_of_samples, Fs, mag1*3.14/180.0, freq, duty); 
}

break; 

case 'p':
printf("Enter a new value of Kp:\n"); 
scanf("%f", &Kp); 
break; 

case 'f':
printf("Enter a new value of Fs:\n"); 
scanf("%f", &Fs); 
break; 

case 't':
printf("Enter a new value of run time:\n"); 
scanf("%f", &run_time);  
break; 

case'g':
for(i = 0; i < no_of_samples; i++){
ref[i] = ref[i]*180.0/3.14;
}
for(i = 0; i < no_of_samples ; i++){
theta[i] = theta[i]*180.0/3.14;
}
plot(ref,theta, Fs, no_of_samples, SCREEN, "Motor Position vs Time", "Time(s)", "Motor Position(Degrees)"); 
break;

case'h': 
for(i = 0; i < no_of_samples ; i++){
ref[i] = ref[i]*180.0/3.14;
}
for(i = 0; i < no_of_samples ; i++){
theta[i] = theta[i]*180.0/3.14;
}
plot(ref,theta, Fs, no_of_samples , PS, "Task2.3: Basic PID Controller", "Time(secs)", "Motor Position(deg)");
break;

case 'q':
pthread_exit(NULL); 

default:
  
printf("Selection is invalid.\n"); 
break; 

}

}

}





/*
 * Program should take in the output, calcalute the difference between output and ideal, adjust the programs own output accordinly, repeat until difference is ~0 consistently
 */

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <sys/netmgr.h>
#include <hw/inout.h>
#include <pthread.h>
#include <time.h>

#define BASE 0x280

int period =1;
int integral =0;
int derivative = 0;
/*
 * InitializeAD
 * Initializes the A/D converter for use
 */
void InitializeAD(){
	uintptr_t gain_handle; //controls the gain
	uintptr_t chan_handle; //controls which channels
	gain_handle = mmap_device_io(1,BASE+3);
	chan_handle = mmap_device_io(1,BASE+2);
	out8(chan_handle,0x11); //sets channel 1 as A/D input
	out8(gain_handle,0x0); //Sets gain as +-10V
}

/* Start
 * Starts an A/D conversion
 */
void start(){
	uintptr_t write_handle; //the address that starts the conversion
	write_handle = mmap_device_io(1,BASE);
	out8(write_handle,0x80); //writes a high bit to tell hardware to perform conversion
}


 /* Convert
 * Performs an A/D Conversion
 */
int16_t convertAD(){
	uintptr_t status_handle; //maintains the status of the converter
	uintptr_t least_handle; //Least significant bit of the conversion
	uintptr_t most_handle; //most significant bit of the conversion
	int LSB;
	int MSB;
	status_handle = mmap_device_io(1,BASE+3); //Status register of A/D converter
	least_handle = mmap_device_io(1,BASE); //Register where LSB is held
	most_handle = mmap_device_io(1,BASE+1);
	start();
	while(in8(status_handle)& 0x80){ //waits for status bit to change and conversion to finish

	}
	LSB = in8(least_handle);
	MSB = in8(most_handle);
	return MSB*256+LSB;
}

void convertDA(int16_t input){
	uintptr_t lsb_handle;
	uintptr_t msb_handle;
	lsb_handle= mmap_device_io(1,BASE+6);
	msb_handle =mmap_device_io(1,BASE+7);



	int8_t lsb = input&255;
	int8_t msb=input/256; //add value to use channel other then 0
	out8(lsb,lsb_handle);
	out8(msb,msb_handle);



}

int16_t compute(int16_t input){
	int16_t out;
	if(input != 1 || input != 0){
		out =(period+integral+derivative)*(input*input)-(period+2*derivative)*input+derivative;
		out = out/((input-1)*input);
	}
	else
	{
		out = 0;
	}
	return out;
}

void performMath(){
	int16_t analog;
	int16_t out;
	analog = convertAD();
	out = compute(analog);
	convertDA(out);
}

void * userControl(void * args){
	printf("Enter Period \n");

	printf("Enter Integral \n");

	printf("Enter Derivative \n");
}

int main(int argc, char *argv[]) {


	int pid;
	int chid;
	int pulse_id = 0;
	struct _pulse pulse;
	struct _clockperiod clkper;
	struct sigevent event;
	struct itimerspec timer;
	timer_t timer_id;
	int privity_err;

	privity_err = ThreadCtl( _NTO_TCTL_IO, NULL );
	if ( privity_err == -1 )
	{
		fprintf( stderr, "can't get root permissions\n" );
		return -1;
	}

	/*Real Time Clock Setup*/
	clkper.nsec = 1000000;
	clkper.fract = 0;

	ClockPeriod(CLOCK_REALTIME, &clkper, NULL, 0);
	chid = ChannelCreate(0);
	assert(chid != -1);

	/*Event creation and Set up*/
	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE,0,chid,0,0);
	event.sigev_priority = getprio(0);
	event.sigev_code = 1023;
	event.sigev_value.sival_ptr = (void *)pulse_id;

	/*Timer Set up and Creation*/
	timer_create( CLOCK_REALTIME, &event, &timer_id );

	timer.it_value.tv_sec = 0;
	timer.it_value.tv_nsec = 1000000;
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_nsec = 1000000;

	timer_settime( timer_id, 0, &timer, NULL );

	InitializeAD();

	while(1){
		pid = MsgReceivePulse ( chid, &pulse, sizeof( pulse ), NULL );
		performMath();
	}

	return EXIT_SUCCESS;
}

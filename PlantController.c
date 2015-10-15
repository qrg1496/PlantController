#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/mman.h>
#include <hw/inout.h>

#define BASE 0x280
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
/*IntializePort
 * Sets up the output port to send data
 */
void InitializePort(){
	uintptr_t portDir;
	portDir = mmap_device_io(1,BASE+11);
	out8(portDir,0x80); //sets Port A and B as out
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
	least_handle = mmap_device_io(1,BASE); //Register where LSB is heald
	most_handle = mmap_device_io(1,BASE+1);
	start();
	while(in8(status_handle)& 0x80){ //waits for status bit to change and conversion to finish

	}
	LSB = in8(least_handle);
	MSB = in8(most_handle);
	return MSB*256+LSB;


}

int main(int argc, char *argv[]) {
	printf("Welcome to the QNX Momentics IDE\n");
	return EXIT_SUCCESS;
}

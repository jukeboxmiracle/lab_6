/* ENGR-2350 SPRING 2020 - Lab 4
 * **** Template Code ****
 * */
 //Nik Pepmeyer

#define PRINTTOFILE     "lab_6_info.csv"
#define RIN             661961476

#include "C8051_SIM.h"
#include<stdio.h>
#include<stdlib.h>


#define WINDOW_LENGTH 20 // Specify the length of your window here


void Port_Init(void);               // Initialize all the GPIO ports (I/O only)
void PCA_Init(void);                // Initialize PCA function
void XBR_Init(void);                // Initialize Crossbar function
void Interrupt_Init(void);          //Initialize interrupts function
void SMB_Init(void);                // Initialize I2C function

void read_compass(void);            // Function to read the compass heading

void set_servoPW(void);
void set_motorPW(void);

signed int MOTOR_NEUT = 2765;       // Neutral              1.5 ms
signed int MOTOR_MAX = 3502;        // Full Forward         1.9 ms
signed int MOTOR_MIN = 2027;        // Full Reverse         1.1 ms
signed int MOTOR_PW;                // Current PW

unsigned char counts = 0;           // PCA overflow counter
unsigned char new_range = 0;        // Flag denoting if a new range value should be retrieved (set in interrupt)
unsigned char new_heading = 0;      // Flag denoting if a new heading value should be retrieved (set in interrupt)
unsigned char new_print = 0;        // Flag denoting that we can print (set in interrupt)
unsigned char r_count = 0;          // Counter to count 80 ms for ranger delay
unsigned char h_count = 0;          // Counter to count 40 ms for compass delay
unsigned char p_count = 0;          // Counter to count ?? ms for print delay
signed int heading = 0;             // Variable to hold compass heading value
unsigned int range = 64;            // Variable to hold ranger distance value

unsigned int actuator_flag = 0;
unsigned int pushed = 0;
signed int des_heading = 900;         // Desired heading
signed int curr_error = 0;       // Error calculation
signed int prev_error = 0;

signed int error_hist[WINDOW_LENGTH]; // Array to store historical samples
signed long error_sum = 0; // Variable storing sum of historical samples
unsigned char tail_index = 0; // Location of oldest sample in array

                   // Proportional Gain constant for steering
float kd = 0;                   // Proportional Gain constant for Drive
float kp = 2;

void main(void){

    Sys_Init();
    putchar(0);
    Port_Init();
    PCA_Init();
    Interrupt_Init();
    XBR_Init();
    SMB_Init();

    // Read ranger once to trigger ping.

    // Initialize the drive motor to neutral for 1 s
        // Set motor pulsewidth to neutral
     while(counts < 10){ // Wait 1 second
        Sim_Update();   // Called in all loops!
        MOTOR_PW = MOTOR_NEUT;
        PCA0CP2 = 65535 - MOTOR_PW;
    }

    // Clear the flag counter variables to prevent a double read
    r_count = h_count = p_count = 0;
    new_range = new_heading = new_print = 0;

    // Print data headers
    //printf("...");



    // Run program loop
        // while(1) loop may or may not be needed, depending on how it's implemented.
    while(1){
        Sim_Update();
        read_compass();

        if (new_heading == 1){
            des_heading = 900;

        }
        else if(new_heading == 2){
            des_heading = 1800;

        }
        else if(new_heading == 3){
            des_heading = 2700;

        }
        else if(new_heading == 0){
            des_heading = 0;

        }

        set_motorPW();

        if (new_print){
            // Print import stuff
            new_print = 0;
            printf("Motor: %d\t Heading: %d\r\n", MOTOR_PW, heading);
        }

    }
}

void Port_Init(void){
    P1MDOUT |= 0x0D; // Make P1.0,.2,.3 outputs
    P2MDOUT &= !0x10;
    P3MDOUT &= !0x01;
    P2 |= 0x10;
    P3 |= 0x01;

    // Fill in remaining
}

void XBR_Init(void){
    XBR0 = 0x27;    // 00100111, Enable SPI, I2C, and CEX0-3
}

void PCA_Init(void){
    PCA0MD |= 0x01; // SYSCLK/12, Interrupt Enable
    PCA0CPM0 |= 0xC2; // Enable 16-bit PWM, compare function
    PCA0CPM2 |= 0xC2;
    PCA0CPM3 |= 0xC2;
    CR = 1; // Same as PCA0CN |= 0x40;
}

void Interrupt_Init(void){
    EIE1 |= 0x08;       // Enable PCA interrupt
    EA = 1;             // Globally Enable interrupts

}

void SMB_Init(void){
    SMB0CR = 0x93;      // Configure the I2C Clock Rate
    ENSMB=1;            // Enable the module
}

void read_compass(void){
    unsigned char addr = 0xC0;          // the address of the sensor, 0xC0 for the compass
    unsigned char Data[2];              // Data is an array with a length of 2
    i2c_read_data(addr, 2, Data, 2);                        // read two byte, starting at reg 2
    heading =(((unsigned int)Data[0] << 8) | Data[1]);      // combine the two values
                            //heading has units of 1/10 of a degree
}



void set_motorPW(){
    curr_error = des_heading - heading;

    MOTOR_PW = MOTOR_NEUT + (kp*curr_error)+ kd*(curr_error - prev_error);


     if(MOTOR_PW > MOTOR_MAX){
        MOTOR_PW = MOTOR_MAX;
     }else if(MOTOR_PW < MOTOR_MIN){
        MOTOR_PW = MOTOR_MIN;
     }

    PCA0CP2 = 0xFFFF - MOTOR_PW;
    PCA0CP3 = 0xFFFF - (2*(MOTOR_NEUT-MOTOR_PW));
}


void PCA_ISR(void){
    if(CF){     // If a PCA overflow has occurred
        PCA0 = 28671;   // Preload the counter for 20 ms operation

        counts++;       // Increment *all* our counters

        h_count++;


        CF = 0;
        // Add flag setting, e.g.:

        if(h_count > 1000){
            new_heading = rand()%4;
            prev_error = curr_error;
            h_count = 0;
        }
    }
    PCA0CN = 0x40;      // Clear all interrupts (including CF), leave CR=1
}

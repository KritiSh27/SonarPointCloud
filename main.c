/* ----------------Include libraries--------------------- */


#include <stdio.h>             
#include <linux/i2c-dev.h>             //i2c file
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <termios.h>                    //uart file
#include <unistd.h>
#include <sys/signal.h>
#include <string.h>


/* ---------------- Registers and Variables  --------------------- */


#define MPU_ACCEL_XOUT1 0x3b            //MPU acceleration values
#define MPU_ACCEL_XOUT2 0x3c
#define MPU_ACCEL_YOUT1 0x3d
#define MPU_ACCEL_YOUT2 0x3e
#define MPU_ACCEL_ZOUT1 0x3f
#define MPU_ACCEL_ZOUT2 0x40

#define MPU_GYRO_XOUT1 0x43             //MPU Gyroscope values
#define MPU_GYRO_XOUT2 0x44
#define MPU_GYRO_YOUT1 0x45
#define MPU_GYRO_YOUT2 0x46
#define MPU_GYRO_ZOUT1 0x47
#define MPU_GYRO_ZOUT2 0x48

#define MPU_MAG_XOUT1 0x04              //MPU Magnetometer values
#define MPU_MAG_XOUT2 0x03
#define MPU_MAG_YOUT1 0x06
#define MPU_MAG_YOUT2 0x05
#define MPU_MAG_ZOUT1 0x08
#define MPU_MAG_ZOUT2 0x07

#define MPU_TEMP1 0x41
#define MPU_TEMP2 0x42

#define MPU_POWER1 0x6B             //Power registers
#define MPU_POWER2 0x6c

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

//UART   definitions
#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyO4"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
        
volatile int STOP=FALSE; 
volatile float roll,pitch, yaw;

void signal_handler_IO (int status);   /* definition of signal handler */
int wait_flag=TRUE;                    /* TRUE while no signal received */


/* ---------------- Registers& Variables end here  --------------------- */



/* ----------------Initializations begin here--------------------- */


//I2C
int fd;
char *fileName = "/dev/i2c-2";
int  address = 0x68;
int address1 = 0x0c;
//I2C ends here

//UART
int fd1,c, res;
struct termios oldtio,newtio;
struct sigaction saio;           /* definition of signal action */
char buf[255];
//UART ends here

//File inits
FILE *fp;
char buff[255];
//File ends here

/* ----------------Initializations end here--------------------- */



/* ----------------Function definitions begin here--------------------- */


void i2c_init(address)                              //I2C setup
{
    int adapter_nr = 0;
    char filename[20];

    snprintf(filename, 19, "/dev/i2c-2", adapter_nr);
    fd = open(filename, O_RDWR);
    
    if(fd < 0)
        printf("Error: Failed to open file %s\n", filename);
   
    int success= (ioctl(fd, I2C_SLAVE, address));

    if(fd < 0) 
    {
        printf("Error: IMU I2C Failed to Open\n");
    }
}


void uart_init()                                    //UART setup
{
    /* open the device to be non-blocking (read will return immediatly) */
    fd1 = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd1 <0) {perror(MODEMDEVICE); exit(-1); }
        
    /* install the signal handler before making the device asynchronous */
    saio.sa_handler = signal_handler_IO;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);
          
    // allow the process to receive SIGIO 
    fcntl(fd1, F_SETOWN, getpid());
    /* Make the file descriptor asynchronous (the manual page says only 
        O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    fcntl(fd1, F_SETFL, FASYNC);
        
    tcgetattr(fd1,&oldtio); /* save current port settings */
    /* set new port settings for canonical input processing */
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;
    newtio.c_cc[VMIN]=1;
    newtio.c_cc[VTIME]=0;
    tcflush(fd1, TCIFLUSH);
    tcsetattr(fd1,TCSANOW,&newtio);
        
    write(fd1,"$PTNLSNM,273F,01*27/r/n",21);
}


void init_file()                                                    //File setup
{
   fp = fopen("pointcloud.txt", "w");
   if (fp == NULL) 
   {
    printf("Error");
    exit(1);
   }

}


void signal_handler_IO (int status)                               //UART signal handler
 {
    wait_flag = FALSE;
 }


void write_register(uint8_t register_address, uint8_t value)    //writing to register
{
    uint8_t data[]={register_address,value};
    write(fd, data,ARRAY_SIZE(data));
}


uint8_t read_register(uint8_t register_address)                 //reading from register
{
    uint8_t value;
   
    if(write(fd, &register_address, sizeof(register_address)) !=1)
    {
        printf("%d\n",write(fd, &register_address, sizeof(register_address)));
        printf("Failed to send data\n");
    }
   
    read(fd, &value, sizeof(value));
    return value;
}



/* ----------------Function definitions end here--------------------- */



/* ---------------- Main --------------------- */


int main(int argc, char **argv)
{
    /* Initializations */
    int i=0;
    uart_init();
    init_file();
    i2c_init(address);
    int8_t power = i2c_smbus_read_byte_data(fd, MPU_POWER1);
    i2c_smbus_write_byte_data(fd, MPU_POWER1, ~(1 << 6) & power);

    /*Setup registers*/
    write_register(0x6B, 0x00);                    // exit sleep
    write_register(0x19, 109);                     // sample rate = 8kHz / 110 = 72.7Hz
    write_register(0x1B, 0x18);                    // gyro full scale = +/- 2000dps
    write_register(0x1C, 0x08);                    // accelerometer full scale = +/- 4g
    
    while (1) 
    {
        int16_t xaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT1) << 8 |
                         i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT2);
        int16_t yaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT1) << 8 |
                         i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT2);
        int16_t zaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT1) << 8 |
                         i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT2);
    
        // Normalize into range
        float accel_x = xaccel / 16384.0f;
        float accel_y = yaccel / 16384.0f;
        float accel_z = zaccel / 16384.0f;

        //IMU calculations
        float roll = atan2f(accel_y,accel_z)*(180/3.14);
        float pitch = atan2f(accel_x,sqrt(accel_y*accel_y + accel_z*accel_z))*(180/3.14);
        float yaw = atan2f(accel_x,accel_y)*(180/3.14);
        
        //Print IMU calculations
        printf(" roll :  %f \n\n",roll);
        printf(" pitch :  %f \n\n",pitch);
        printf(" yaw :  %f \n\n",yaw);   
   
        
        //Read from UART
        res = read(fd1,buf,255);
        buf[res]=0;
        printf(" ultrasonic : %s \n", buf);
        

        //Write to text file
        fprintf(fp, "%f %f %s",roll,pitch,buf );
        
        //Increment loop
        i++;
   

        //Condition to limit file length
        if (i>3000)
        {
            fclose(fp);
        }
   }

    tcsetattr(fd,TCSANOW,&oldtio);  //UART old settings
    return 0;
}

/* ----------------End of Program--------------------- */
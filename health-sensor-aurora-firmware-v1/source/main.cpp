#include "mbed.h"
#include "MPU6050.h"
#define debug(X) pc.printf(X);
#define debugf(X,Y) pc.printf(X,Y);
#define CRASH(X) {wait(X);LED=1;wait(X);LED=0;}

Serial pc(P0_10,P0_13);

#include "SDFileSystem.h"

bool write_config();
void fifo_overflow();
void update_time();
 
SDFileSystem sd(P0_23, P0_25, P0_24, P0_22, "sd"); // the pinout on the mbed Cool Components workshop board
DigitalOut LED(P0_21);
DigitalIn CARD_STATUS(P0_1);
InterruptIn MPU_INTERRUPT(P0_29);

MPU6050 mpu(MPU6050_ADDRESS_AD0_HIGH);


#define CARD_INSERTED 0x00
#define CARD_BAY_EMPTY 0x01

unsigned long timestamp = 0;
unsigned long file_entries = 0;

bool overflow = false;
volatile bool new_data=false;
Ticker ts;
 
int main() {
    wait(4);
    ts.attach(&update_time, 0.001);
    CARD_STATUS.mode(PullUp);
    wait(0.1);
    
    debug("\r\nBooting Totem - HealthPatch V1. Alpha Firmware\r\n");
    if(CARD_STATUS == CARD_BAY_EMPTY)
    {
        debug("Could not find SD-Card, halting operation");
        while(1) CRASH(0.1)
    }else debug("SD-Card found\r\n");
    bool passed = false;
    while(!passed){
    debug("Booting SD-Card\r\n");
        passed =write_config();
        LED=1;
        wait(1);
        LED=0;
    }
    
    char filename[13];
    sprintf(filename,"/sd/%d.txt",timestamp);
    debugf("Opening Data File: %s",filename); 
    
    FILE *fp = fopen(filename, "w");
    fprintf(fp, "Timestamp,X,Y,Z,Temperature\r\n");
   
    LED=1;
    debug("Succes\r\n");
    debug("Starting MPU");
    mpu.reset();
    wait(1);
    
    mpu.initialize();
    bool mpu6050TestResult = mpu.testConnection();
    if(!mpu6050TestResult)
    {
        debug("Could not start MPU");
        while(1) CRASH(0.5)
    } else debug("MPU Started\r\n");
    
    MPU_INTERRUPT.mode(PullDown);
    MPU_INTERRUPT.rise(&fifo_overflow);
    
    
    mpu.setSleepEnabled(false);          
    mpu.setTempSensorEnabled(false);
    
    mpu.setStandbyXGyroEnabled(true);
    mpu.setStandbyYGyroEnabled(true);
    mpu.setStandbyZGyroEnabled(true);

    mpu.setInterruptMode(0);
    mpu.setInterruptDrive(0);
    mpu.setInterruptLatch(true);
    mpu.setInterruptLatchClear(true);
  
  //  mpu.setIntFIFOBufferOverflowEnabled(true);  
    mpu.setWakeFrequency(3);
  //  mpu.setWakeCycleEnabled(true);
    mpu.setIntDataReadyEnabled(true);
    mpu.setFIFOEnabled(true);
    mpu.setAccelFIFOEnabled(true);


    fclose(fp);
    LED=0;
    uint16_t data_buffer[330];
    while(true)
    {
        char filename[20];
        sprintf(filename,"/sd/%d.txt",timestamp);
        debugf("Opening Data File: %s",filename);      
        FILE *fp = fopen(filename, "w");
        fprintf(fp, "Timestamp,X,Y,Z,Temperature\r\n");
        file_entries=0;
    
        while(file_entries < 1000)
        {
          if(new_data)
          {
            new_data=false;
            //    LED=1;
            int16_t x;
            int16_t y;
            int16_t z;
            
            mpu.getAcceleration(&x,&y,&z);
            data_buffer[0] = x;
            data_buffer[1] = y;
            data_buffer[2] =z;
            fprintf(fp,"%d,%d,%d,%d,23\r\n",timestamp,x,y,z);
            uint8_t sample[6];
            mpu.getFIFOBytes(sample, 6); 
            data_buffer[0] = ((sample[0]<<8)|sample[1]);
            data_buffer[1] = ((sample[2]<<8)|sample[3]);
            data_buffer[2] = ((sample[4]<<8)|sample[5]);
          //  pc.printf("%d,%d,%d,%d,23\r\n",timestamp,x,y,z);
            file_entries++;
            //    LED=0;              
          }   
        }
        fclose(fp);
        pc.printf("File Done");
    }
    
    
       
    while(file_entries<1000)
    {
        int fifo_count = mpu.getFIFOCount();
        int16_t x;
        int16_t y;
        int16_t z;
        
        mpu.getAcceleration(&x,&y,&z);
      //  debugf("x=%d\n",x);
      //  debugf("y=%d\n",y);
      //  debugf("z=%d\n",z);
        debug("n\r\n");
        wait(0.05);
        timestamp+=50;
        fprintf(fp,"%d,%d,%d,%d,23\r\n",timestamp,x,y,z);
        file_entries++;
        /*
        if(overflow)
        {
            overflow=false;
            int fifo_count = mpu.getFIFOCount();
            debugf("b:%d\n",fifo_count);
            debug("FLUSH\n");
            overflow=false;
            uint8_t measurement_store[100];
            mpu.getFIFOBytes(measurement_store, 100); 
            mpu.resetFIFO();
        }
        */
    }
    
    fclose(fp);
    LED=1;
    while(1);
   // printf("Goodbye World!\n");
  
}

void update_time()
{
    timestamp+=1;
}

bool write_config()
{
    FILE *fp = fopen("/sd/config.txt", "w");
    LED=0;
    if(fp == NULL) {
      debug("Could not open config file for write,halting operation\n");
      return false;
    }
    fprintf(fp, "Mode: Accelerometer Only\r\n");
    fprintf(fp, "Freq: 100hz\r\n");  
    fclose(fp);
    return true;
}

void fifo_overflow()
{
 new_data=true;
 //  overflow=true;
   // mpu.resetFIFO();
}


   // mkdir("/sd/mydir", 0777);
    
    
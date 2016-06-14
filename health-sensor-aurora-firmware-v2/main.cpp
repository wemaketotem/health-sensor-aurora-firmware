#include "mbed.h"
#include "BLEDevice.h"
#include "UARTService.h"
#include "MPU6050.h"
#include "SDFileSystem.h"
                                                                            //Maak een sd kaart object
SDFileSystem sd(P0_23, P0_25, P0_24, P0_22, "sd");                          //Pin verbonden aan MCU
#define CARD_INSERTED 0x00
#define CARD_BAY_EMPTY 0x01
DigitalIn CARD_STATUS(P0_1);

int sessionCounter = 0;

//--------------------------------------------------------------------------Timer init
Timer timer;
int begin, end;   

int timeStamp[13];

unsigned newTimestamp = 0;
unsigned int updateTimestamp;

//--------------------------------------------------------------------------Battery monitor init


//--------------------------------------------------------------------------Data ontvangen voor instructies
const char * incommingData[20];
int aanvulling;

char filter[20];
char timestamp[13];

int incommingSwitch;
bool startFlag = false;
bool accFlag = true;
bool gyroFlag = true;
bool tempFlag = true;
bool hzFlag = true;

//--------------------------------------------------------------------------Debug configuratie
I2C TMP102(P0_3, P0_2);                                                     //I2C verbinding
DigitalOut debugLed(P0_21);                                                 //Debug ledje knippert bij iedere loop

Serial pc(USBTX, USBRX);                                                    //RX TX voor monitoren pc

//--------------------------------------------------------------------------Initialiseren en declareren temp variabelen
const int addr = 0x90;                                                      //sensor adres 0x90 wanneer add0 laag is!!!!!!!!!!
char regByte[3];                                                            //config bytes decl                                    
char temp_read[2];                                                          //rouwe sensor waarde 2 bytes breed
float temp;                                                                 //sensor waarde

//--------------------------------------------------------------------------Initialisatie en declaratie MPU6050 vars en setup
InterruptIn MPU_INTERRUPT(P0_29);
MPU6050 mpu(MPU6050_ADDRESS_AD0_HIGH);                                      //Adres selectie: AD0 hoog is 0x69 

int16_t ax, ay, az, gx, gy, gz;
void fifo_overflow(); 
volatile bool new_data=false;

//--------------------------------------------------------------------------------------------------------------------------------------------
BLEDevice  ble;                                                             //Maak bluetooth object

UARTService *uartServicePtr;

static volatile bool  triggerSensorPolling = false;


void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)  //Geef disconnectie feedback
{
    ble.startAdvertising();
}

unsigned parse_int(const char * p)             //char array converter 
{
    unsigned result = 0;
    unsigned digit;
    while ((digit = *p++ - '0') < 10)
    {
        result = result * 10 + digit;           //begin vanaf 0 en loop door de hele array om te vermenigvuldigen met 10 + element waarde
    }    
    return result;                              //geef het resultaat terug
}

void onDataWritten(const GattCharacteristicWriteCBParams *params)                    //UART data ontvangen 
{
    if ((uartServicePtr != NULL) && (params->charHandle == uartServicePtr->getTXCharacteristicHandle())) {     //Iprint wanneer data is ontvangen
        uint16_t bytesRead = params->len;
        
        const char *incommingData = (char*)params->data;
        
        filter[0] = incommingData[0];
        filter[1] = incommingData[1];
       
        if(std::strcmp(filter, "TS") == 0){
            uint8_t lenght = 2;
            
            for(int i=0; i<=9; i++){
                timestamp[i] = incommingData[lenght];               //zet elk char array element in nieuw modifiable array
                lenght++;
            }
            
            newTimestamp = parse_int(timestamp);                    //converteer de data van const char naar unsigned int
            timer.start();                                          //start de timer voor timestamp
            
        }else if(std::strcmp(incommingData, "TSdd") == 0){

        }else if(std::strcmp(incommingData, "LDstart") == 0){
            startFlag = true;
            
        }else if(std::strcmp(incommingData, "LDstop") == 0){
            startFlag = false;
            
        }else if(std::strcmp(incommingData, "ADaan") == 0){
            accFlag = true;
            
        }else if(std::strcmp(incommingData, "ADuit") == 0){
            accFlag = false;
            
        }else if(std::strcmp(incommingData, "GDaan") == 0){
            gyroFlag = true;
            
        }else if(std::strcmp(incommingData, "GDuit") == 0){
            gyroFlag = false;
            
        }else if(std::strcmp(incommingData, "TDaan") == 0){
            tempFlag = true;
            
        }else if(std::strcmp(incommingData, "TDuit") == 0){
            tempFlag = false;
            
        }else if(std::strcmp(incommingData, "HZ") == 0){            
            
        }else if(std::strcmp(incommingData, "") != 0){

        }
        
        ble.updateCharacteristicValue(uartServicePtr->getRXCharacteristicHandle(), params->data, bytesRead);   // Terug sturen over BLE
        triggerSensorPolling = true;
    }
}

void tempInit(){
    regByte[0] = 0x01;                                                      //configureer register
    regByte[1] = 0x60;                                                      //configureer byte 1
    regByte[2] = 0xA0;                                                      //configureer byte 2
    
    TMP102.write(addr, regByte, 3);                                         //schrijf config-data naar temp sensor
    regByte[0] = 0x00;                                                      //pointer register 
    TMP102.write(addr, regByte, 1); 
}

void mpuInit(){
    mpu.reset();
    wait(0.2);
    
    mpu.initialize();                                                       //Initialisatie van de MPU6050
    wait(0.2);
    
    bool mpu6050TestResult = mpu.testConnection();
    
    if(!mpu6050TestResult)                                                  //Test resultaat. Als ID niet gevonden is start de health patch opnieuw op
    {
        while(1){
            debugLed = 1;
            wait(0.2);
            debugLed = 0;
            wait(0.2);
        }
    } else{
    
    }
}

void sdCardTest(){
    CARD_STATUS.mode(PullUp); // Pull up the control line
    wait(0.1); 
    if(CARD_STATUS == CARD_BAY_EMPTY){    
        while(CARD_STATUS == CARD_BAY_EMPTY){
            debugLed = 1;
            wait(0.5);
            debugLed = 0;
            wait(0.5);
        }
        FILE *fp = fopen("/sd/sdtest.csv", "w");
        if (fp != NULL) {
            fprintf(fp, "We're writing to an SD card! SD card can now be used");
            fclose(fp);
        } else {

        } 
    }else{            
        FILE *fp = fopen("/sd/sdtest.csv", "w");
        if (fp != NULL) {
            fprintf(fp, "We're writing to an SD card! SD card can now be used");
            fclose(fp);
        } else {

        }   
    }
    CARD_STATUS.mode(PullDown);
}

//-------------------------------------------------------------------------------------------Meting Patch setup file-------------------------------------------------------
void dataSessionConfig(){
    char filenameSessionConfig[25];

    sprintf(filenameSessionConfig,"/sd/%d-session-config.csv",sessionCounter);

    FILE *fp = fopen(filenameSessionConfig, "w");
    fprintf(fp, "See what sensors are enabled in this session\r\n");
    if(tempFlag == true){    
        fprintf(fp,"Temperature sensor = TRUE;\r\n");                     
    }else{
        fprintf(fp,"Temperature sensor = FALSE;\r\n");                    
    }

    if(accFlag == true){    
        fprintf(fp,"Accelero sensor = TRUE;\r\n");                       
    }else{
        fprintf(fp,"Accelero sensor = FALSE;\r\n");                     
    }    
    
    if(gyroFlag == true){    
        fprintf(fp,"Gyro sensor = TRUE;\r\n");                  
    }else{
        fprintf(fp,"Gyro sensor = FALSE;\r\n");   
    }

    fclose(fp);
}
//-------------------------------------------------------------------------------------------Meting Patch data file-------------------------------------------------------
void sensorRead(){
   if(startFlag == true){
        dataSessionConfig();
        
        char filenameSessionData[23];
        sprintf(filenameSessionData,"/sd/%d-session-data.csv",sessionCounter);
        FILE *fp = fopen(filenameSessionData, "w");

       while(startFlag == true){
            
            updateTimestamp = newTimestamp + timer.read_ms();
            
            pc.printf("%u us\n",updateTimestamp);
            
            fprintf(fp,"%u", updateTimestamp);
            
            if(tempFlag == true){    
                TMP102.read(addr, temp_read, 2);                                    //lees de twee bytes
                temp = 0.0625 * (((temp_read[0] << 8) + temp_read[1]) >> 4);        //convert eer data naar float
                fprintf(fp,";%.2f;", temp);                                         //schrijf float temp naar sd kaart
            }else{
                fprintf(fp,";NaN;");                         
            }
            
            if(accFlag == true){
                mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                       //haal de data op voor de acc en de gyro
                fprintf(fp,"%d;%d;%d;",ax,ay,az);                 
            }else{
                fprintf(fp,"NaN;NaN;NaN;");
            }
           
            if(gyroFlag == true){               
                fprintf(fp,"%d;%d;%d\n\r",gx,gy,gz);
            }else{
                fprintf(fp,"NaN;NaN;NaN\n\r");                     
            }
        }
        fclose(fp);
        sessionCounter++;
    }
}

int main(void){
    tempInit();
    mpuInit();
    sdCardTest();
    
    ble.init();
    ble.onDisconnection(disconnectionCallback);                                            // callback funtie voor disconnection
    ble.onDataWritten(onDataWritten);                                                      // callback funtie voor ontvangen bericht

//-------------------------------------------------------------setup adverteren----------------------------------------------------------------------------------
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED| GapAdvertisingData::LE_GENERAL_DISCOVERABLE);             // Indicate that Legacy Bluetooth in not supported
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *)"totem02", sizeof("totem02") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.setAdvertisingInterval(Gap::MSEC_TO_ADVERTISEMENT_DURATION_UNITS(1000));            //Interval adverteren 1000mS
    ble.startAdvertising();                                                                 //Begin met adverteren

    UARTService uartService(ble);                                                           //Initialiseer pointer naar UART service
                                                                                            //UART service
    uartServicePtr = &uartService;  
         
    while(1){
        if (triggerSensorPolling && ble.getGapState().connected) {
            triggerSensorPolling = true;
            sensorRead();
        } else {
            ble.waitForEvent();
        }
    }    
}



//Brandon AMS Start
/*requires the download of TimerOne-r11 from here: https://code.google.com/archive/p/arduino-timerone/downloads
and spi-can from here: http://www.seeedstudio.com/wiki/CAN-BUS_Shield */

//TO DO: SET UP WATCHDOG ON NANO. INVOLVES NEW BOOTLOADER? 

//Use functions from given library
#include "LTC68031.h"
#include "mcp_can.h"
#include <TimerOne.h>
#include <math.h>

const int TOTAL_IC = 1;     // Number of ICs in the daisy chain

bool cfg_flag = false;
bool cnt_flg = false;
bool sent = false;
int counter = 0; //counter for timer. 
int CT_value = 0;
volatile int STATE = LOW;
int error;

//----HW constants------
//digital pins
const int CAN_int = 2;
const int WD_Vis = 3;
const int WDT = 4;
const int Midpack = 5;
const int HW_Enable = 6;
const int AMS_Stat = 7;
const int CAN_CS = 9;
const int LTC6803_CS = 10;
const int Beta = 4250000;
const int rinf = 64; // in milliohms//100000*exp(-1*(Beta/298.15));
const int freq = 20000;      //in uS for timer1
const int OW_counter = 30/*seconds*/ *1000 /*ms*/ /freq * 1000; //uS. 

//----Current Transducer constants----
const int THRESHOLD = 40;       // Threshold to ignore the OV, as we're under large current draw causing false OV
//const float Gain = .004;         // for current transducer, in V/A
const int inv_Gain = 250;         // *250 = /.004
const int Offset = 512;        // 0 current offset, allows measurement in both directions. 
//float TS_current;
int TS_current;
const int ESR = 4;            // ESR of the entire pack. Need to change this. 
//analog pin
const int CT_Sense = 0;
//----LT constants----
//const float balance = 2.67;      // Balance Voltage - at this voltage send CAN message to stop regen
const int balance = 2670;         // Balance Voltage (mV) - at this voltage send CAN message to stop regen
//const float stopbalance = 2.6;    // Voltage to stop balancing at, can start regen again
const int stopbalance = 2600;    // Voltage (mV) to stop balancing at, can start regen again
//const float OV = 2.8;             // Voltage to shut down TS at
const int OV = 2800;             // Voltage (mV) to shut down TS at
byte PEC = 0x00;

uint16_t cell_codes[TOTAL_IC][12] = {2.7}; 
/*!< 
  The cell codes will be stored in the cell_codes[][12] array in the following format:
  
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

float voltages[TOTAL_IC][12] = {0};

uint16_t temp_codes[TOTAL_IC][3] = {0};
/*!<
 The Temp codes will be stored in the temp_codes[][3] array in the following format:
 
 |  temp_codes[0][0]| temp_codes[0][1]|temp_codes[0][2] | temp_codes[1][0]| temp_codes[1][1]|   .....   |
 |------------------|-----------------|-----------------|-----------------|-----------------|-----------|
 |IC1 Temp1         |IC1 Temp2        |IC1 ITemp        |IC2 Temp1        |IC2 Temp2        |  .....    |
*/
float temps[TOTAL_IC][3] = {0};

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the LTC6803 configuration data that is going to be written 
  to the LTC6803 ICs on the daisy chain. The LTC6803 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:
  
 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |
 
*/

uint8_t rx_cfg[TOTAL_IC][7];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a LTC6803-1 daisy chain. 
  The configuration data for each IC  is stored in blocks of 7 bytes. Below is an table illustrating the array organization:

|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|---------------|---------------|----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC          |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/



void init_cfg(){                       // sets initial configuration for all ICs
  for(int i = 0; i<TOTAL_IC;i++){
    tx_cfg[i][0] = 0x91;
    tx_cfg[i][1] = 0x00 ; 
    tx_cfg[i][2] = 0x00 ;
    tx_cfg[i][3] = 0x00 ; 
    tx_cfg[i][4] = 0x00 ;             //UnderVoltage value... Not applicable here. 
    tx_cfg[i][5] = 0xAB ;             //OV set
  }
}

void setup() {                         // No Serial connection at the moment. 
  //Sets Arduino pin modes
  pinMode(CAN_int, INPUT);
  pinMode(WDT, INPUT);
  pinMode(Midpack, OUTPUT);
  digitalWrite(Midpack, LOW);
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS,HIGH);
  pinMode(LTC6803_CS, OUTPUT);
  digitalWrite(LTC6803_CS,HIGH);
  
  //Initialize timer for interrupt, counts time for CAN messages and OC detect. 
  Timer1.initialize(freq);                 //timer at set freq
  Timer1.attachInterrupt(timer_ISR);
  
  // ---Turn on LTC Chips---
  pinMode(HW_Enable, OUTPUT);
  digitalWrite(HW_Enable, HIGH);
  delay(15);
  // ---Initialize LTC chips
  LTC6803_initialize();                       //Initialize LTC6803 hardware - sets SPI to 1MHz
  init_cfg();
  LTC6803_wrcfg(TOTAL_IC, tx_cfg);            //write cfg to chips
  error = LTC6803_rdcfg(TOTAL_IC, rx_cfg);    //read back, set local copies to what was read.
  errorcheck(error);                          //if PEC error, disable TS.
  cfg_check();                                //sets tx = rx, ensures local copy is same as chip copy. 

  //Initialize CAN interface
  attachInterrupt(digitalPinToInterrupt(CAN_int),readFromCAN_ISR,FALLING);
  //other initializations here

  //show ready to go
  pinMode(AMS_Stat, OUTPUT);
  digitalWrite(AMS_Stat, HIGH);
  pinMode(WD_Vis, OUTPUT);
  digitalWrite(WD_Vis, LOW);
  Timer1.start();
}

void loop() {
  //-----start conversions, if statement for regular or open wire----
  if(!cnt_flg){
    LTC6803_stcvdc();     //start cv conversion
  } 
  else {
    LTC6803_stowdc();     //START OPEN WIRE CONVERSION
    cnt_flg = false;
  }
  LTC6803_sttmpad();      //start temp conversion
  sent = true;            //sets flag to count down time while LTC chip is processing.
  // had thoughts about this, what if the ISR clears immediately?
   
  //----Read TS Current ----
  CT_value = analogRead(CT_Sense);
  TS_current = (CT_value - Offset) * inv_Gain;
  if(digitalRead(WDT) == LOW){
    digitalWrite(AMS_Stat, LOW);
    digitalWrite(WD_Vis, HIGH);
  }

  /*These work one iteration behind on data - this is to burn as little time as possible. 
  Can be placed below receiving new data for "proper" ness but it won't be as fast
  due to floating point math happening*/
  VoltageFix();
  OVCheck();
  StopBal();
  VoltToTemp();                                 // gets temperatures, stores in temps array
  //----Adjust registers if applicable---- - same as above group.
  if(cfg_flag){
    LTC6803_wrcfg(TOTAL_IC, tx_cfg);
    error = LTC6803_rdcfg(TOTAL_IC, rx_cfg);  
    errorcheck(error);
    cfg_check();                                //read back, set local copies to what was read.
    cfg_flag = false;
  }
  //report cell voltage, temp, and CT values over CAN?


  
  //burns any remaining time for conversion
  while(sent){};                                
  //actually receiving data from cell stack.
  error = LTC6803_rdcv(TOTAL_IC, cell_codes);   //----Read Cell Voltage----
  errorcheck(error);
  error = LTC6803_rdtmp(TOTAL_IC, temp_codes);  //read temps
  errorcheck(error);


  
}

void timer_ISR(){
  Timer1.stop();
  counter++;
  sent = false;
  if(counter >= OW_counter){ //every 30 seconds do Open Wire detect
    cnt_flg = true;
    counter = 0;
  }
  Timer1.start();
}

// takes voltages from temp readings, turns them into actual temps
void VoltToTemp(){
  for(int ic_counter = 0;ic_counter < TOTAL_IC;ic_counter++)
  {
    for(int cell_counter = 0;cell_counter < 3;cell_counter++)
    {
      // our setup has no thermistors on IC 2 (the middle one) and only one on IC 1 (bottom of stack) 
      if(ic_counter == 1 && cell_counter !=2){
        temps[ic_counter][cell_counter] = 0;
        continue;
      }
      if(ic_counter == 0 && cell_counter == 1){
        temps[ic_counter][cell_counter] = 0;
        continue;
      }
      if(cell_counter == 2){ //internal temps are measured differently
        temps[ic_counter][cell_counter] = ((temp_codes[ic_counter][cell_counter] - 512)*3/16) - 273;
        continue; 
      }
      //figure out how to do this with integer math?
      temps[ic_counter][cell_counter] = ((temp_codes[ic_counter][cell_counter] - 512)*15/1000);  //weird scaling to proper voltage
      temps[ic_counter][cell_counter]*=100000/(5-temp_codes[ic_counter][cell_counter]);       //voltage to resistance, Ohms
      temps[ic_counter][cell_counter] = Beta/(log(temps[ic_counter][cell_counter]/(rinf*1000)));     //resistance to temp
    }
    
  }
}

//if any PEC error shows up, kill TS
inline void errorcheck(int error){
  if(error != 0)
    {
      digitalWrite(AMS_Stat, LOW);
    }
}
void readFromCAN_ISR(){
  //enter code to turn on halfpack relay

  //obviously a work in progress
  STATE = !STATE;
  digitalWrite(Midpack,STATE);
}

//---- LT chips report with an offset, fixes that. Also takes current into consideration to fix cell voltages---- 
void VoltageFix(){
  for(int ic_counter = 0;ic_counter < TOTAL_IC;ic_counter++)
  {
    for(int cell_counter = 0;cell_counter < 12;cell_counter++)
    {
      voltages[ic_counter][cell_counter] = (cell_codes[ic_counter][cell_counter] - 512) *15/1000;       //chip reports with an offset, this gets actual voltage
      voltages[ic_counter][cell_counter] = voltages[ic_counter][cell_counter] - ESR*TS_current/36;       // removes bias from TS current and series resistance of pack
    }
  }
}

//----sets tx =rx so that we know for sure what the status is----
void cfg_check(){
  for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++) //Loop through all ICs
  {
    for(int reg_ctr = 0;reg_ctr < 6;reg_ctr++) //Loop through all registers, ignore PEC. 
    {
      tx_cfg[ic_counter][reg_ctr] = rx_cfg[ic_counter][reg_ctr];
    }
  }
}

//----helper function to get hex value from decimal. 8 bit only----
uint8_t getHEX(uint8_t input){
  switch(input)
  {
    case 0:
    return 0x00;
    case 1:
    return 0x01;
    case 2:
    return 0x02;
    case 3:
    return 0x03;
    case 4:
    return 0x04;
    case 5:
    return 0x05;
    case 6:
    return 0x06;
    case 7:
    return 0x07;
    case 8:
    return 0x08;
    case 9:
    return 0x09;
    case 10:
    return 0x0A;
    case 11:
    return 0x0B;
    case 12:
    return 0x0C;
    case 13:
    return 0x0D;
    case 14:
    return 0x0E;
    case 15:
    return 0x0F;
    default:
    return 0x00;
  }
}

//----get cell discharge bit from decimal value----
uint8_t DCC_cell(uint8_t input){
  switch(input)
  {
  case 1:
  return 0b00000001;
  case 2:
  return 0b00000010;
  case 3:
  return 0b00000100;
  case 4:
  return 0b00001000;
  case 5: 
  return 0b00010000;
  case 6:
  return 0b00100000;
  case 7:
  return 0b01000000;
  case 8:
  return 0b10000000;
  default:
  return 0b0;
  }
}

//---- Checks for OV & balance conditions----
void OVCheck(){
  for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++) //Loop through all ICs
  {
    for(int cv_counter = 0;cv_counter < 12;cv_counter++) //Loop through all 
    {
      if( voltages[ic_counter][cv_counter] > balance)
      {
        if( voltages[ic_counter][cv_counter] > OV) //if need to shutdown tractive system due to OV.
        {
          digitalWrite(AMS_Stat, LOW);
        }
        if(cv_counter < 8)
        {
          tx_cfg[ic_counter][1] = tx_cfg[ic_counter][1] | DCC_cell(cv_counter);
          cfg_flag = true;
        }else
        {
          tx_cfg[ic_counter][2] = tx_cfg[ic_counter][2] | DCC_cell(cv_counter - 8);
          cfg_flag = true;
        }
      }
    }
  }
}

//---- Checks for balance end conditions----
void StopBal(){
  for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++) //Loop through all ICs
  {
    for(int cv_counter = 0;cv_counter < 12;cv_counter++) //Loop through all 
    {
      if( voltages[ic_counter][cv_counter] < stopbalance)
      {
        if(cv_counter < 8 && ((tx_cfg[ic_counter][1] & DCC_cell(cv_counter)) == 0))
        {
          tx_cfg[ic_counter][1] = tx_cfg[ic_counter][1] ^ DCC_cell(cv_counter);
          cfg_flag = true;
        }else if(cv_counter >= 8 && ((tx_cfg[ic_counter][2] & DCC_cell(cv_counter-8))==0))
        {
          tx_cfg[ic_counter][2] = tx_cfg[ic_counter][2] ^ DCC_cell(cv_counter - 8);
          cfg_flag = true;
        }
      }
    }
  }
}


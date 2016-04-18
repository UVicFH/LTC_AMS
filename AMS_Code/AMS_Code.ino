//Brandon AMS Start
/*requires the download of spi-can from here: http://www.seeedstudio.com/wiki/CAN-BUS_Shield */

/*TO DO: 
 * WRITE NEW BOOTLOADER TO NANO TO ENABLE WATCHDOG TIMER
*/
//Use functions from given library
#include "LTC68031.h"
#include "mcp_can.h"
#include <math.h>
#include <avr/wdt.h>

#define WDT_EN 0              // For on-board watchdog. Need Optiboot bootloader though. 
#define CAN_EN 0              // Comment this out for no CAN chip, ie for testing.
#define SER_EN 0              // Comment this out for no Serial ie for in final car
#define OLD_BAL 0             // Old balance code that works, no regen re-enable
#define NEW_BAL 1             // New, more optimized balance code that should be faster and includes regen re-enable
const int TOTAL_IC = 3;       // Number of ICs in the daisy chain

bool cfg_flag = false;
bool voltflag = false;
bool statflag = false;
bool chargeflag = false;
int CT_value = 0;
int error;

//----HW constants------
//digital pins
const int CAN_int = 2;
const int WD_Vis = 3;
const int WDT = 5;
const int Midpack = 4;
const int HW_Enable = 6;
const int AMS_Stat = 7;
const int CAN_CS = 9;
const int LTC6803_CS = 10;
const int Beta = 4250;
const float rinf = 100000*exp(-1*(Beta/298.15));
unsigned long conversiontime; 
unsigned long OWTime;
uint16_t VoltMin;
uint8_t VoltMinTrans;
uint16_t VoltMax;
uint8_t VoltMaxTrans;
uint32_t PackVoltage;
uint8_t PackVoltageTrans;
uint16_t MaxTemp;
uint8_t MaxTempTrans;
uint8_t FlagTrans;

//----Current Transducer constants----
//const float Gain = .004;         // for current transducer, in V/A
const int inv_Gain = 250;         // *250 = /.004
const int Offset = 512;        // 0 current offset, allows measurement in both directions. 
uint16_t TS_current;
const int ESR = 4;            // ESR of the entire pack. Need to change this. 
//analog pin
const int CT_Sense = 0;

//----LT constants----
//const float balance = 2.67;         // Balance Voltage - at this voltage send CAN message to stop regen
const int balance = 3100;             // Balance Voltage (mV) - at this voltage send CAN message to stop regen
//const float stopbalance = 2.6;      // Voltage to stop balancing at, can start regen again
const int stopbalance = 2600;         // Voltage (mV) to stop balancing at, can start regen again
//const float OV = 2.8;               // Voltage to shut down TS at
const int OV = 3400;                  // Voltage (mV) to shut down TS at
byte PEC = 0x00;
#ifdef NEW_BAL
  bool discharging = false;
#endif

//----CAN SHIELD STUFF----
#ifdef CAN_EN
  MCP_CAN CAN(CAN_CS);
  unsigned char dataToSend[8];
  unsigned char inFromCAN[8];
  unsigned char len = 0;
  unsigned long canID;
  const unsigned long Receive_ID = 0x52;     
  const unsigned long Trans_ID = 0x51;
#endif

uint16_t cell_codes[TOTAL_IC][12] = {1}; 
/*!< 
  The cell codes will be stored in the cell_codes[][12] array in the following format:
  
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t voltages[TOTAL_IC][12] = {0};

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
    tx_cfg[i][2] = 0xF0 ;
    tx_cfg[i][3] = 0xFF ; 
    tx_cfg[i][4] = 0x00 ;             //UnderVoltage value... Not applicable here. 
    tx_cfg[i][5] = 0xAB ;             //OV set... Not usefull so set high. 
  }
}

void setup() { 
  #ifdef WDT_EN
    wdt_disable();
  #endif
  #ifdef SER_EN
    Serial.begin(250000);
  #endif
  //Sets Arduino pin modes
  pinMode(CAN_int, INPUT);
  pinMode(WDT, INPUT);
  pinMode(Midpack, OUTPUT);
  digitalWrite(Midpack, LOW);
  pinMode(CAN_CS, OUTPUT);
  //digitalWrite(CAN_CS,HIGH);
  pinMode(LTC6803_CS, OUTPUT);
  digitalWrite(LTC6803_CS,HIGH);
  
  //Initialize CAN interface
  #ifdef CAN_EN
    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
      {
        #ifdef SER_EN
          Serial.println("CAN BUS Shield init fail");
          Serial.println(" Init CAN BUS Shield again");
          delay(200);
        #endif
      }
    //Serial.println("CAN BUS Shield init ok!");
    CAN.init_Mask(0, 0, 0xfff);                         // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, 0, 0xfff);
    CAN.init_Filt(0, 0, Receive_ID);
  #endif
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

  //other initializations here

  //show ready to go
  pinMode(AMS_Stat, OUTPUT);
  statflag = true;
  digitalWrite(AMS_Stat, HIGH);
  pinMode(WD_Vis, OUTPUT);
  digitalWrite(WD_Vis, LOW);
  conversiontime = millis();
  OWTime = conversiontime;
  #ifdef WDT_EN
    wdt_enable(WDTO_4S);
  #endif
}

void loop() {
  #ifdef WDT_EN
    wdt_reset();
  #endif
  if(digitalRead(WDT) == LOW){
    digitalWrite(AMS_Stat, LOW);
    digitalWrite(WD_Vis, HIGH);
    statflag = false;
  }
  if(digitalRead(WDT) == HIGH){
    digitalWrite(WD_Vis, LOW);
    digitalWrite(AMS_Stat, HIGH);
    statflag = true;
  }
  //----OpenWire detect?----
  if(millis() - OWTime >=30000){
    LTC6803_stowdc();     //START OPEN WIRE CONVERSION
    OWTime = millis();
  } 
  else {
    LTC6803_stcvdc();     //start cv conversion
  }
  conversiontime = millis();
  //----Read TS Current ----

  /*
  CT_value = analogRead(CT_Sense);
  TS_current = (CT_value - Offset) * inv_Gain;

  */

  /*These work one iteration behind on data - this is to burn as little time as possible. 
  Can be placed below receiving new data for "proper" ness but it won't be as fast
  due to floating point math happening*/
  VoltageFix();
  #ifdef OLD_BAL
    OVCheck();
    StopBal();
  #endif
  #ifdef NEW_BAL
    Balance_Check();
  #endif
  VoltToTemp();
   
  //----Adjust registers if applicable---- - same as above group.
  if(cfg_flag){
    LTC6803_wrcfg(TOTAL_IC, tx_cfg);
    error = LTC6803_rdcfg(TOTAL_IC, rx_cfg);  
    errorcheck(error);
    cfg_check();                                //read back, set local copies to what was read.
    cfg_flag = false;
  }
  // All CAN Operations
  #ifdef CAN_EN

    //Check for CAN Messages
    len = 0;
    if(CAN_MSGAVAIL == CAN.checkReceive()){
          CAN.readMsgBuf(&len, inFromCAN);    // read data,  len: data length, inFromCAN: data
          canID = CAN.getCanId();
      }
      
    //sets midpack relay on for charging
    if(canID == Receive_ID){
      if(inFromCAN[0] == 1){
        digitalWrite(Midpack, HIGH);
        chargeflag = true;
      }
      if(inFromCAN[0] == 0){
        digitalWrite(Midpack, LOW);
        chargeflag = false;
      }
    }
    
  //Prepare data for CAN transmission
  VoltMaxTrans = map(VoltMax,0,3000,0,255);
  //Serial.print("Max Voltage: ");
  //Serial.print(VoltMax);
  //Serial.print(" ");
  VoltMinTrans = map(VoltMin,0,3000,0,255);
 // Serial.print("Min Voltage: ");
  //Serial.print(VoltMin);
  //Serial.print(" ");
  PackVoltageTrans = map(PackVoltage,0,100000,0,255);
  //Serial.print("Pack Voltage: ");
  //Serial.print(PackVoltage);
  //Serial.print(" ");
  //Serial.print("Max Temp: ");
  //Serial.print(MaxTemp);
  //Serial.print(" ");
  MaxTempTrans = uint8_t(MaxTemp);
  //Sets data for CAN transmision based on status flags
  if(statflag){
    FlagTrans |= 0x1;
  }else{
    FlagTrans = FlagTrans & ~0x1;
  }
  if(chargeflag){
    FlagTrans |= 0x2;
  }else{
    FlagTrans = FlagTrans & ~0x2;
  }
  if(voltflag){
    FlagTrans |= 0x4;
  }else{
    FlagTrans = FlagTrans & ~0x4;
  }
  //report cell voltage, temp, and CT values over CAN
    dataToSend[0] = TS_current;
    dataToSend[1] = TS_current >> 8;
    dataToSend[2] = FlagTrans;                
    dataToSend[3] = VoltMaxTrans;
    dataToSend[4] = VoltMinTrans;
    dataToSend[5] = PackVoltageTrans;
    dataToSend[6] = MaxTempTrans;

    CAN.sendMsgBuf(0x51, 0, 7, dataToSend);
  #endif
  
  //burns any remaining time for conversion
  while(millis()-conversiontime > 20){}                              
  //actually receiving data from cell stack.
  LTC6803_sttmpad();      //start temp conversion
  conversiontime = millis();
  error = LTC6803_rdcv(TOTAL_IC, cell_codes);   //----Read Cell Voltage----
  errorcheck(error);
  //burns any remaining time for conversion
  while(millis()-conversiontime > 30){}
  error = LTC6803_rdtmp(TOTAL_IC, temp_codes);  //read temps
  errorcheck(error);
}

// takes voltages from temp readings, turns them into actual temps
void VoltToTemp(){
  MaxTemp = 0;
  for(int ic_counter = 0;ic_counter < TOTAL_IC;ic_counter++){
    for(int cell_counter = 0;cell_counter < 3;cell_counter++){
      // our setup has no thermistors on IC 1 (the middle one)  
      if(ic_counter == 1 && cell_counter !=2){
        continue;
      }
      //Only 1 thermistor (Ambient) on IC 0 (bottom of stack)
      if(ic_counter == 0 && cell_counter == 1){
        continue;
      }
      //internal temps are measured differently
      if(cell_counter == 2){  
        temps[ic_counter][cell_counter] = ((temp_codes[ic_counter][cell_counter])*0.1875)-273.15;
        continue; 
      }
      //For some reason, I'm reading a ~0.34 V offset. Vendor code did it too, so I'm just adding it here. 
      temps[ic_counter][cell_counter] = temp_codes[ic_counter][cell_counter] * 0.0015 + 0.34;
      /*
      if(ic_counter == 0 && cell_counter == 0){
        Serial.println(temps[ic_counter][cell_counter],4); //Samples to volts
      }*/
      temps[ic_counter][cell_counter] = 100000 * temps[ic_counter][cell_counter]/(3.0625 - temps[ic_counter][cell_counter]);    // Voltage to resistance, Ohms
      temps[ic_counter][cell_counter] = (Beta/(log(temps[ic_counter][cell_counter]/rinf)))-273.15;                              //resistance to temp
      MaxTemp = max(MaxTemp,temps[ic_counter][cell_counter]);
    }
  }
}

//if any PEC error shows up, kill TS
inline void errorcheck(int error){
  if(error != 0){
      Serial.println("PEC ERROR");
      digitalWrite(AMS_Stat, LOW);
    }
}

//---- LT chips report with an offset, fixes that. Also takes current into consideration to fix cell voltages---- 
void VoltageFix(){
  PackVoltage = 0;
  VoltMin = 100;
  VoltMax = 0;
  for(int ic_counter = 0;ic_counter < TOTAL_IC;ic_counter++){
    //Serial.print(ic_counter);
    //Serial.print(" ");
    for(int cell_counter = 0;cell_counter < 12;cell_counter++){
      //Serial.print(cell_counter);
      //Serial.print(" ");
      voltages[ic_counter][cell_counter] = (cell_codes[ic_counter][cell_counter]) *15/10;       //chip reports with an offset, this gets actual voltage (mV)
      //Serial.print(voltages[ic_counter][cell_counter]);
      //Serial.print(" ");
      //voltages[ic_counter][cell_counter] = voltages[ic_counter][cell_counter] - ESR*TS_current/36;       // removes bias from TS current and series resistance of pack
      VoltMin = min(VoltMin, voltages[ic_counter][cell_counter]);       //calc min      
      VoltMax = max(VoltMax, voltages[ic_counter][cell_counter]);       //calc max
      PackVoltage += voltages[ic_counter][cell_counter];                //total pack
    }
    //Serial.print("\n");
  }
  //Serial.print("\n");
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
  case 0:
  return 0b00000001;
  case 1:
  return 0b00000010;
  case 2:
  return 0b00000100;
  case 3:
  return 0b00001000;
  case 4: 
  return 0b00010000;
  case 5:
  return 0b00100000;
  case 6:
  return 0b01000000;
  case 7:
  return 0b10000000;
  default:
  return 0b0;
  }
}
#ifdef OLD_BAL
  /*---- Checks for OV & balance conditions----
  *If a cell voltage is higher than OV, shutdown Tractive system
   */
  void OVCheck(){
    for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++){       //Loop through all ICs
      for(int cv_counter = 0;cv_counter < 4;cv_counter++){          //Loop through all cells
        if( voltages[ic_counter][cv_counter] > balance){
          voltflag = true;                                          // stop regen
          if( voltages[ic_counter][cv_counter] > OV) {              //if need to shutdown tractive system due to OV.
            digitalWrite(AMS_Stat, LOW);
            statflag = false;
          }
          if(cv_counter < 8){
            tx_cfg[ic_counter][1] |= DCC_cell(cv_counter);
            cfg_flag = true;
          }else{
            tx_cfg[ic_counter][2] |= DCC_cell(cv_counter - 8);
            cfg_flag = true;
          }
        }
      }
    }
  }
  
  /*---- Checks for balance end conditions----
   * If Tractive System was shut down, allow re-enable of reset circuit.
   */
  void StopBal(){
    for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++) {  //Loop through all ICs
      for(int cv_counter = 0;cv_counter < 4;cv_counter++){      //Loop through all cells
        if(voltages[ic_counter][cv_counter] <= stopbalance){ 
          if(cv_counter < 8 && (tx_cfg[ic_counter][1] & DCC_cell(cv_counter))){
            tx_cfg[ic_counter][1] ^= DCC_cell(cv_counter);
            cfg_flag = true;
          }else if(cv_counter >= 8 && ((tx_cfg[ic_counter][2] & DCC_cell(cv_counter-8))==0)){
            tx_cfg[ic_counter][2] = tx_cfg[ic_counter][2] ^ DCC_cell(cv_counter - 8);
            cfg_flag = true;
          }
          if(!statflag){
            statflag = true;
            digitalWrite(AMS_Stat, HIGH);
          }
        }
      }
    }
  }
#endif

#ifdef NEW_BAL
  void Balance_Check(){
    for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++){       //Loop through all ICs
      discharging = true;
      for(int cv_counter = 0;cv_counter < 12;cv_counter++){          //Loop through all cells
      //over voltage checks
        if( voltages[ic_counter][cv_counter] > balance){
          voltflag = true;                                          // stop regen
          if( voltages[ic_counter][cv_counter] > OV) {              //if need to shutdown tractive system due to OV.
            digitalWrite(AMS_Stat, LOW);
            statflag = false;
          }
          if(cv_counter < 8){
            tx_cfg[ic_counter][1] |= DCC_cell(cv_counter);
            cfg_flag = true;
          }else{
            tx_cfg[ic_counter][2] |= DCC_cell(cv_counter - 8);
            cfg_flag = true;
          }
        }
        
  
      //under voltage checks
        if(voltages[ic_counter][cv_counter] <= stopbalance){ 
          if(cv_counter < 8 && (tx_cfg[ic_counter][1] & DCC_cell(cv_counter))){
            tx_cfg[ic_counter][1] ^= DCC_cell(cv_counter);
            cfg_flag = true;
          }else if(cv_counter >= 8 && ((tx_cfg[ic_counter][2] & DCC_cell(cv_counter-8))==0)){
            tx_cfg[ic_counter][2] = tx_cfg[ic_counter][2] ^ DCC_cell(cv_counter - 8);
            cfg_flag = true;
          }
          if(!statflag){
            statflag = true;
            digitalWrite(AMS_Stat, HIGH);
          }
        }
      }
      if((tx_cfg[ic_counter][1] == 0x00) && ((tx_cfg[ic_counter][2] << 4) == 0x00)){
        discharging = false;
      }
      #ifdef SER_EN
      #endif
    }
  }
#endif


//Brandon AMS Start
/* Requires the download of spi-can from here: http://www.seeedstudio.com/wiki/CAN-BUS_Shield 
 * 
 * Constantly measures TS Current and sends over CAN.
 * 
 * Alternately requests cell voltages and temperatures 
 * Due to LTC chip limitations
*/

/*TO DO: 
 * WRITE NEW BOOTLOADER TO NANO TO ENABLE WATCHDOG TIMER
 * Test this code as found LTC chip problem before I got to. 
 */

//Use functions from given library
#include "LTC68031.h"
#include "mcp_can.h"
#include "pin_defines.h"      //contains pin definitions
#include "LT_consts.h"        //contains constants related to voltages and timing
#include <math.h>             // Just need exp()
#include <avr/wdt.h>

#define WDT_EN 0              // For on-board watchdog. Need Optiboot bootloader though. 
#define CAN_EN 0              // Enables CAN - set to zero for testing without CAN modules
#define SER_EN 1              // Enables Serial, set to 0 for in final car. 
#define CURR_FIX 0            // Enables fixing the cell voltages with current and pack ESR. Experimental. Fix ESR before enabling
#define TOTAL_IC 3            // Number of ICs in the daisy chain


//internal status flags
bool cfg_flag = false;
bool voltflag = false;
bool statflag = false;
bool chargeflag = false;
bool voltConvFlag = false;
bool voltReceiveFlag = false;
bool tempConvFlag = false;
bool tempReceiveFlag = false;
bool discharging = false;
uint16_t CT_value = 0;
int error;

//thermistor constants
#define BETA 4250
const float R_INF = 100000*exp(-1*(BETA/298.15));

//LT variables, also some for CAN transmission
unsigned long conversiontime_ms; 
unsigned long Open_Wire_time_ms;
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
#define INVERSE_GAIN 250     // 250A/V = 1/(.004 V/A), from Data sheet of DHAB CT
#define CT_OFFSET 512        // 0 current offset, allows measurement in both directions. 
int TS_current;
#define ESR 4            // ESR of the entire pack. Need to change this. 

//----CAN INTERFACE STUFF----
#ifdef CAN_EN
  MCP_CAN CAN(CAN_CS);
  unsigned char dataToSend[8];
  unsigned char inFromCAN[8];
  unsigned char len = 0;
  unsigned long canID;
  const unsigned long Receive_ID = 0x52;     
  const unsigned long Trans_ID = 0x51;
#endif

//function prototypes
void init_cfg();                      // initializes config registers
void CAN_Init();                      // initializes CAN interface
void LTC_Init();                      // initializes LTC chips
void CellConversionReq();             // requests cell voltage conversion from LTC chips
void ReceiveCells();                  // receives cell voltages and computes all necessary values
void TempConversionReq();             // requests temperature conversion from LTC chips
void ReceiveTemps();                  // receives temperatures and computes all necessary values
void CAN_Receive();                   // receives CAN message to enable midpack relay for charging
void CAN_Trans();                     // transmits CAN messages to master controller
void VoltToTemp();                    // fixes temp values to proper units
inline void errorcheck(int error);    // checks PEC errors, shuts TS down if issue found
void cfg_check();                     // copies LTC config to local values to ensure we know what's up
uint8_t DCC_cell(uint8_t input);      // finds correct bit for discharge. possible move to header?
uint16_t VoltageFix(uint16_t inputvoltage, int current);  // fixes voltage values from LTC chips
void Balance_Check();                 // checks for balance/stop balance/etc conditions

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


void setup() { 
  #ifdef WDT_EN
    wdt_disable();
  #endif
  #ifdef SER_EN
    Serial.begin(250000);
  #endif
  
  //Sets Arduino pin modes
  pinMode(CAN_RECEIVED_INTERRUPT, INPUT);
  pinMode(WDT_INPUT_FR_LTC_CHIPS, INPUT);
  pinMode(MIDPACK_RELAY, OUTPUT);
  digitalWrite(MIDPACK_RELAY, LOW);
  pinMode(CAN_CS, OUTPUT);
  pinMode(LTC6803_CS, OUTPUT);
  digitalWrite(LTC6803_CS,HIGH);
  
  //Initialize CAN interface
  #ifdef CAN_EN
    CAN_Init();
  #endif
  
  //Turn on LTC Chips
  LTC_Init();                             

  //show ready to go
  pinMode(AMS_STATUS_OUTPUT, OUTPUT);
  statflag = true;
  digitalWrite(AMS_STATUS_OUTPUT, HIGH);
  pinMode(WD_LED_VISUAL, OUTPUT);
  digitalWrite(WD_LED_VISUAL, LOW);
  #ifdef WDT_EN
    wdt_enable(WDTO_4S);
  #endif
  
  //Starts off the conversion chain
  CellConversionReq();
  conversiontime_ms = millis();
  Open_Wire_time_ms = conversiontime_ms;
  voltConvFlag = true;
}

void loop(){
  //reset local watchdog timer
  #ifdef WDT_EN
    wdt_reset();
  #endif
  
  //check for LTC Watchdog reset
  if(digitalRead(WDT_INPUT_FR_LTC_CHIPS) == LOW){
    digitalWrite(AMS_STATUS_OUTPUT, LOW);
    digitalWrite(WD_LED_VISUAL, HIGH);
    statflag = false;
#ifdef SER_EN
    Serial.println("LTC watchdog timer event");
#endif
  }else{
    digitalWrite(WD_LED_VISUAL, LOW);
    digitalWrite(AMS_STATUS_OUTPUT, HIGH);
    statflag = true;
  }
  
  //Read TS Current
  CT_value = analogRead(CT_SENSE_PIN);
  TS_current = (CT_value - CT_OFFSET) * INVERSE_GAIN;
#ifdef SER_EN
  Serial.print("TS Current: ");
  Serial.println(TS_current);
#endif
  //Scheduled task for receiving cell voltages
  if(voltConvFlag && (millis() - conversiontime_ms > CELL_SENSE_TIME_ms)){
    ReceiveCells();
    /* Timer is reset inside function calls
     * Due to multiple operations happening before
     * timer should be reset.
     */
  }
  
  //Scheduled task for receiving temperatures
  if(tempConvFlag && (millis() - conversiontime_ms > TEMP_SENSE_TIME_ms)){
    ReceiveTemps();
    /* Timer is reset inside function calls
     * Due to multiple operations happening before
     * timer should be reset.
     */
  }
  
  //CAN transmit/receive
  #ifdef CAN_EN
    CAN_Receive();
    CAN_Trans();
  #endif
}

void init_cfg(){                       // sets initial configuration for all ICs
  for(int i = 0; i<TOTAL_IC;i++){
    tx_cfg[i][0] = 0x91;
    tx_cfg[i][1] = 0x00 ; 
    tx_cfg[i][2] = 0xF0 ;
    tx_cfg[i][3] = 0xFF ; 
    tx_cfg[i][4] = 0x00 ;             //UnderVoltage value... Not applicable here. 
    tx_cfg[i][5] = 0xAB ;             //OVERVOLTAGE_ERROR_mV set... Not usefull so set high. 
  }
}

//Initializes CAN interface
void CAN_Init(){
  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    #ifdef SER_EN
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");
      delay(200);
    #endif
  }
  CAN.init_Mask(0, 0, 0xfff);                         
  CAN.init_Mask(1, 0, 0xfff);
  CAN.init_Filt(0, 0, Receive_ID);
}

//Initializes LTC chips
void LTC_Init(){
  pinMode(LTC_HW_ENABLE, OUTPUT);
  digitalWrite(LTC_HW_ENABLE, HIGH);
  delay(15);
  // ---Initialize LTC chips
  LTC6803_initialize();                       //Initialize LTC6803 hardware - sets SPI to 1MHz
  init_cfg();
  LTC6803_wrcfg(TOTAL_IC, tx_cfg);            //write cfg to chips
  error = LTC6803_rdcfg(TOTAL_IC, rx_cfg);    //read cfg register back
  errorcheck(error);                          //if PEC error, disable TS.
  /*set tx = rx, ensures local copy is same as chip copy. 
   * Assumes LTC chips properly received data
   * Valid assumption due to above check
   */
  cfg_check();   
}

//Request cell conversion from LTC chips 
void CellConversionReq(){
  //----OpenWire detect?----
  if(millis() - Open_Wire_time_ms >= OPEN_WIRE_TIMER_ms){
    LTC6803_stowad();     //START OPEN WIRE CONVERSION
    Open_Wire_time_ms = millis();
  } 
  else {
    LTC6803_stcvad();     //start cv conversion
  }
  voltConvFlag = true;
  conversiontime_ms = millis();
}

/* Receive cell voltages
 * Fixes values from samples to volts by calling function
 * Check for START_BALANCE_VOLTAGE_mV conditions by calling proper function depending on new/old bal
 * Requests temp conversion
 */
void ReceiveCells(){
  //Receive Cell Voltages
  error = LTC6803_rdcv(TOTAL_IC, cell_codes);   //----Read Cell Voltage----
  errorcheck(error);
  voltConvFlag = false;
   
  //Find cell voltages/Check Balance/OverVoltage/Stop Balance conditions
  Balance_Check();
  
  //----Adjust registers if applicable---- - same as above group.
  if(cfg_flag){
    LTC6803_wrcfg(TOTAL_IC, tx_cfg);
    error = LTC6803_rdcfg(TOTAL_IC, rx_cfg);  
    errorcheck(error);
    cfg_check();                                //read back, set local copies to what was read.
    cfg_flag = false;
  }
  voltReceiveFlag = true;
  TempConversionReq();
}

//Request temp conversion from LTC chips
void TempConversionReq(){
  LTC6803_sttmpad();      //start temp conversion 
  tempConvFlag = true; 
  conversiontime_ms = millis();
}

/* Receive temperatures
 * Fixes values from samples to volts to temperatures by calling function
 * Requests cell conversion
 */
void ReceiveTemps(){
  //Receive Temps
  error = LTC6803_rdtmp(TOTAL_IC, temp_codes);  //read temps
  errorcheck(error);
  tempConvFlag = false;
  //Convert to Temperature
  VoltToTemp();
  tempReceiveFlag = true;
  CellConversionReq();
}

//Check for CAN Messages
void CAN_Receive(){
  len = 0;
  if(CAN_MSGAVAIL == CAN.checkReceive()){
        CAN.readMsgBuf(&len, inFromCAN);    // read data,  len: data length, inFromCAN: data
        //canID = CAN.getCanId();
    }
    
  //sets midpack relay on for charging
  //if(canID == Receive_ID){
  if(inFromCAN[0] == 1){
    digitalWrite(MIDPACK_RELAY, HIGH);
    chargeflag = true;
  }
  if(inFromCAN[0] == 0){
    digitalWrite(MIDPACK_RELAY, LOW);
    chargeflag = false;
  }
}

// Transmit data over CAN
void CAN_Trans(){
  //Prepare data for CAN transmission
  if(voltReceiveFlag){        //update transmission values only when new data in
    VoltMaxTrans = map(VoltMax,0,3000,0,255);
    VoltMinTrans = map(VoltMin,0,3000,0,255);
    PackVoltageTrans = map(PackVoltage,0,100000,0,255);
    voltReceiveFlag = false;
  }
  if(tempReceiveFlag){      //update transmission values only when new data in
      MaxTempTrans = uint8_t(MaxTemp);
      tempReceiveFlag = false;
  }
 
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

  //Finally Send data
  CAN.sendMsgBuf(Trans_ID, 0, 7, dataToSend);
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
      temps[ic_counter][cell_counter] = (BETA/(log(temps[ic_counter][cell_counter]/R_INF)))-273.15;                              //resistance to temp
      MaxTemp = max(MaxTemp,temps[ic_counter][cell_counter]);
    }
  }
}

//if any PEC error shows up, kill TS
inline void errorcheck(int error){
  if(error != 0){
      Serial.println("PEC ERROR");
      digitalWrite(AMS_STATUS_OUTPUT, LOW);
    }
}

//---- LT chips report with an offset, fixes that. Also takes current into consideration to fix cell voltages---- 

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

/* LT chips report cell voltages with an offset, fixes that. 
 * Also takes current into consideration to fix cell voltages
 */
uint16_t VoltageFix(uint16_t inputvoltage, int current){
  uint16_t outputVoltage = inputvoltage *15/10;       //chip reports with an offset, this gets actual voltage (mV)
#ifdef CURR_FIX
  outputVoltage = outputVoltage - ESR*current/36;       // removes bias from TS current and series resistance of pack
#endif
  return outputVoltage;
}

void Balance_Check(){
  PackVoltage = 0;
  VoltMin = 100;
  VoltMax = 0;
  for(int ic_counter = 0;ic_counter<TOTAL_IC;ic_counter++){       //Loop through all ICs
    discharging = true;
    for(int cv_counter = 0;cv_counter < 12;cv_counter++){          //Loop through all cells
      
      //get cell voltage first
      voltages[ic_counter][cv_counter] = VoltageFix(cell_codes[ic_counter][cv_counter], TS_current);
      
      VoltMin = min(VoltMin, voltages[ic_counter][cv_counter]);       //calc min      
      VoltMax = max(VoltMax, voltages[ic_counter][cv_counter]);       //calc max
      PackVoltage += voltages[ic_counter][cv_counter];                //total pack

      
      //over voltage checks
      if(voltages[ic_counter][cv_counter] > START_BALANCE_VOLTAGE_mV){
        voltflag = true;                                          // stop regen
        if( voltages[ic_counter][cv_counter] > OVERVOLTAGE_ERROR_mV) {              //if need to shutdown tractive system due to OVERVOLTAGE_ERROR_mV.
          digitalWrite(AMS_STATUS_OUTPUT, LOW);
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
      if(voltages[ic_counter][cv_counter] <= STOP_BALANCE_VOLTAGE_mV){ 
        if(cv_counter < 8 && (tx_cfg[ic_counter][1] & DCC_cell(cv_counter))){
          tx_cfg[ic_counter][1] ^= DCC_cell(cv_counter);
          cfg_flag = true;
        }else if(cv_counter >= 8 && ((tx_cfg[ic_counter][2] & DCC_cell(cv_counter-8))==0)){
          tx_cfg[ic_counter][2] = tx_cfg[ic_counter][2] ^ DCC_cell(cv_counter - 8);
          cfg_flag = true;
        }
        if(!statflag){
          statflag = true;
          digitalWrite(AMS_STATUS_OUTPUT, HIGH);
        }
      }
    }
    if((tx_cfg[ic_counter][1] == 0x00) && ((tx_cfg[ic_counter][2] << 4) == 0x00)){
      discharging = false;
    }
  }
}


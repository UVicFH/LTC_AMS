/*
Constants for voltages and timing using LTC6803 chips
*/

#define START_BALANCE_VOLTAGE_mV 2675    // Balance Voltage (mV) - at this voltage send CAN message to stop regen
#define STOP_BALANCE_VOLTAGE_mV 2500     // Voltage (mV) to stop balancing at, can start regen again
#define OVERVOLTAGE_ERROR_mV  2750       // Voltage (mV) to shut down TS at
byte PEC = 0x00;
#define CELL_SENSE_TIME_ms 20            //time required for chips to complete cell voltage reading
#define TEMP_SENSE_TIME_ms 5             //time required for chips to complete temp reading
#define OPEN_WIRE_TIMER_ms 30000         //frequency at which to test for open wire condition (in ms)

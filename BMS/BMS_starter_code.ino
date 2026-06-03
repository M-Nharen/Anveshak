#include <Arduino.h>

#include <daly-bms-uart.h> //zip library downloaded from https://github.com/maland16/daly-bms-uart/tree/main

#define BMS_SERIAL Serial2

Daly_BMS_UART bms(BMS_SERIAL);

void setup()
{
  Serial.begin(9600);

  bms.Init();
}

void loop()
{
  Serial.println("Press any key and hit enter to query data from the BMS...");
  while (Serial.available() == 0)
  {
  }
  Serial.read(); // Discard the character sent
  Serial.read(); // Discard the new line

  // SOC & Voltage) you could use other public APIs, like getPackMeasurements(), which only query
  // specific values from the BMS instead of all.
  bms.update();

  Serial.println("Basic BMS Data:              " + (String)bms.get.packVoltage + "V " + (String)bms.get.packCurrent + "I " + (String)bms.get.packSOC + "\% ");
  Serial.println("Package Temperature (C):     " + (String)bms.get.tempAverage);
  Serial.println("Highest Cell Voltage:        #" + (String)bms.get.maxCellVNum + " with voltage " + (String)(bms.get.maxCellmV / 1000));
  Serial.println("Lowest Cell Voltage:         #" + (String)bms.get.minCellVNum + " with voltage " + (String)(bms.get.minCellmV / 1000));
  Serial.println("Number of Cells:             " + (String)bms.get.numberOfCells);
  Serial.println("Number of Temp Sensors:      " + (String)bms.get.numOfTempSensors);
  Serial.println("BMS Chrg / Dischrg Cycles:   " + (String)bms.get.bmsCycles);
  Serial.println("BMS Heartbeat:               " + (String)bms.get.bmsHeartBeat); // cycle 0-255
  Serial.println("Discharge MOSFet Status:     " + (String)bms.get.disChargeFetState);
  Serial.println("Charge MOSFet Status:        " + (String)bms.get.chargeFetState);
  Serial.println("Remaining Capacity mAh:      " + (String)bms.get.resCapacitymAh);

  for (size_t i = 0; i < size_t(bms.get.numberOfCells); i++)
  {
    Serial.println("Remaining Capacity mAh:      " + (String)bms.get.cellVmV[i]);
  }

  // Alarm flags
  // These are boolean flags that the BMS will set to indicate various issues.
  // For all flags see the alarm struct in daly-bms-uart.h and refer to the datasheet
  Serial.println("Level one Cell V to High:    " + (String)bms.alarm.levelOneCellVoltageTooHigh);

  /**
   * Advanced functions:
   * bms.setBmsReset(); //Reseting the BMS, after reboot the MOS Gates are enabled!
   * bms.setDischargeMOS(true); Switches on the discharge Gate
   * bms.setDischargeMOS(false); Switches off thedischarge Gate
   * bms.setChargeMOS(true); Switches on the charge Gate
   * bms.setChargeMOS(false); Switches off the charge Gate
   */
}
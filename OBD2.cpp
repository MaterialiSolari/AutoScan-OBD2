#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_pin = 10;
MCP_CAN CAN(SPI_CS_pin);

// Function to request data from ECU
void requestData(byte len, byte mode, byte pid) {
  byte RequestdataArr[] = { len, mode, pid, 0x00, 0x00, 0x00, 0x00, 0x00 };
  CAN.sendMsgBuf(0x7DF, 0, sizeof(RequestdataArr), RequestdataArr);
}

// Function to process received data based on PID
double processData(byte mode, byte pid, byte rxBuf[]) {
  double value = 0;
  
  switch (pid) {
    case 0x0C: // Engine RPM
      value = ((rxBuf[3] * 256) + rxBuf[4]) / 4;
      break;
    case 0x0D: // Speed
      value = rxBuf[3];
      break;
    case 0x04: // Engine Load
      value = rxBuf[3] * (100.0 / 255.0);
      break;
    case 0x05: // Coolant Temperature
      value = rxBuf[3] - 40;
      break;
    case 0x0B: // Intake Manifold Pressure
      value = rxBuf[3];
      break;
    case 0x0F: // Intake Air Temperature
      value = rxBuf[3] - 40;
      break;
    case 0x10: // Mass Air Flow
      value = ((rxBuf[3] * 256) + rxBuf[4]) / 100;
      break;
    case 0x1F: // Run Time Since Engine Start
      value = (rxBuf[3] * 256) + rxBuf[4];
      break;
    case 0x21: // Distance Traveled with MIL On
      value = (rxBuf[3] * 256) + rxBuf[4];
      break;
    case 0x2F: // Fuel Level
      value = (rxBuf[3] * 100.0) / 255.0;
      break;
    case 0x00: // Stored DTCs (Mode 0x03)
      unsigned int dtc = (rxBuf[3] << 8) | rxBuf[4];
      Serial.print("Stored DTC: ");
      Serial.println(dtc, HEX);
      value = dtc;
      break;
    default:
      value = -1;
      break;
  }
  
  return value;
}

// Function to clear DTCs
void clearDTS() {
  byte clearRequest[] = { 0x02, 0x04, 0x00 }; 
  CAN.sendMsgBuf(0x7DF, 0, sizeof(clearRequest), clearRequest);
  Serial.println("Clear DTCs request sent.");
}

// Function to process O2 sensor response
void process02SensorResponse(byte rxBuf[]) {
  float voltage = rxBuf[3] / 200.0;
  float fuelTrim = rxBuf[4] - 128.0;
  
  Serial.print("O2 Sensor Voltage: ");
  Serial.print(voltage);
  Serial.print(" V | Fuel Trim: ");
  Serial.print(fuelTrim, 1);
  Serial.println(" %");
}

// Function to read data from ECU
double readData(byte len, byte mode, byte pid) {
  long unsigned int rxID;
  unsigned char rxLen = 0;
  unsigned char rxBuf[8];
  unsigned long startTime = millis();

  while (millis() - startTime < 200) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      CAN.readMsgBuf(&rxID, &rxLen, rxBuf);

      if (rxID == 0x7E8 && rxBuf[1] == (mode + 0x40) && rxBuf[2] == pid) {
        // O2 Sensor PID range: 0x01 to 0x20 for Mode 0x05
        if (mode == 0x05 && pid >= 0x01 && pid <= 0x20) {
          process02SensorResponse(rxBuf);
          return 0;  
        } else {
          return processData(mode, pid, rxBuf);
        }
      } else if (rxID == 0x7E8 && rxBuf[1] == (mode + 0x04)) {
        clearDTS();
        return 0;
      }
    }
  }

  return -1;  
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting ----SET-UP----");

  byte idmodeset = MCP_ANY; // This sets the CAN controller to any mode
  byte speedset = CAN_500KBPS; // 500 Kbps CAN speed
  byte clockset = MCP_16MHZ; // Assuming you're using a 16 MHz crystal oscillator

  if (CAN.begin(idmodeset, speedset, clockset) == CAN_OK) {
    Serial.println("CAN BUS initiated");
  } else {
    Serial.println("Failed to initiate");
    while (1);  // Halt if CAN initialization fails
  }
}

void loop() {
  // Request Engine RPM (PID 0x0C) using Mode 0x01
  double rpm = readData(8, 0x01, 0x0C);
  if (rpm >= 0) {
    Serial.print("Engine RPM: ");
    Serial.println(rpm);
  } else{
    Serial.print("Engine RPM at 0");
  }

  // You can add more requests for different data by changing PID or mode here

  delay(1000);  // Delay to prevent flooding the serial output
}

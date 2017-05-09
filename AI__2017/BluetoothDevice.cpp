#include <Arduino.h>
#include "BluetoothDevice.h"

BluetoothDevice::BluetoothDevice()
{}

BluetoothDevice::~BluetoothDevice()
{}

// CHECK: Need To Call This Func In setup() or NOT.
void BluetoothDevice::SetPinBLT(unsigned char RX_PIN, unsigned char TX_PIN)
{
  m_BTSerial = new SoftwareSerial(RX_PIN, TX_PIN);
  m_BTSerial->begin(9600);
}

void BluetoothDevice::EnableATMode(unsigned char AT_MODE_PIN)
{
  m_ATModePin = AT_MODE_PIN;
}

// Get Data From Android Device (App: Bluetooth Terminal) And Send To Serial Monitor (App: Arduino.exe Win32).
void BluetoothDevice::SentToSerialMonitor()
{
  if (m_BTSerial->available())
  {
    m_GetChar = m_BTSerial->read();

    Serial.println("");

    while (m_GetChar != '.')
    {
      Serial.write(m_GetChar);
      m_GetChar = m_BTSerial->read();
    }
  }
}

// Get Data From Arduino Board And Send To Android Device (App: Bluetooth Terminal)
void BluetoothDevice::SentToBluetoothDevice(String message)
{
  // Print To Android Device.
  m_BTSerial->println(message);

  // Print To Serial Monitor.
  Serial.println(message);
}

// Get Data From Arduino Board And Send To Android Device (App: Bluetooth Terminal)
void BluetoothDevice::SentToBluetoothDevice(int message)
{
  // Print To Android Device.
  m_BTSerial->println(message);

  // Print To Serial Monitor.
  Serial.println(message);
}


#pragma once
#include <SoftwareSerial.h>

class BluetoothDevice
{
  public:
    BluetoothDevice();
    ~BluetoothDevice();

    void SetPinBLT(unsigned char RX_PIN, unsigned char TX_PIN);
    void EnableATMode(unsigned char AT_MODE_PIN);

    void SentToSerialMonitor();
    void SentToBluetoothDevice(String message);
     void SentToBluetoothDevice(int message);

  private:
    SoftwareSerial *m_BTSerial;
    unsigned char   m_ATModePin;
    char            m_GetChar;
};

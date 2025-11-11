/*
 * Remote Control Module (SBUS)
 * Handles SBUS receiver communication and mode extraction
 */

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <Arduino.h>
#include "sbus.h"

// Remote control mode definitions (from channel 6)
#define RC_MODE_MANUAL    0
#define RC_MODE_SEMI_AUTO 1
#define RC_MODE_FULL_AUTO 2

// Remote control data structure
struct RemoteControlData
{
    bool connected;           // SBUS connection status
    uint8_t drive_mode;       // Manual/Semi-auto/Full-auto mode
    int16_t ch[16];           // All 16 SBUS channels
    bool lost_frame;          // Frame loss flag
    bool failsafe;            // Failsafe flag
    uint32_t last_update_ms;  // Last update timestamp
};

class RemoteControl
{
public:
    // Constructor
    RemoteControl(HardwareSerial* serial_port);

    // Initialize SBUS communication
    void begin();

    // Update SBUS data (call this regularly in loop or task)
    // Returns true if new data was received
    bool update();

    // Get remote control data
    RemoteControlData getData() const;

    // Get connection status
    bool isConnected() const;

    // Get drive mode (MANUAL/SEMI_AUTO/FULL_AUTO)
    uint8_t getDriveMode() const;

    // Get specific channel value (0-15)
    int16_t getChannel(uint8_t channel) const;

    // Get mode as string for debugging
    const char* getDriveModeString() const;

private:
    bfs::SbusRx* sbus_rx_;
    bfs::SbusData sbus_data_;
    RemoteControlData rc_data_;
    uint32_t timeout_ms_;

    // Extract drive mode from channel 6
    void extractDriveMode();

    // Check for timeout
    void checkTimeout();
};

#endif // REMOTE_CONTROL_H

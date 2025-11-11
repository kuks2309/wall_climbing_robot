/*
 * Remote Control Module Implementation
 */

#include "remote_control.h"

// Constructor
RemoteControl::RemoteControl(HardwareSerial* serial_port)
    : timeout_ms_(500)
{
    sbus_rx_ = new bfs::SbusRx(serial_port);

    // Initialize remote control data
    rc_data_.connected = false;
    rc_data_.drive_mode = RC_MODE_MANUAL;
    rc_data_.lost_frame = false;
    rc_data_.failsafe = false;
    rc_data_.last_update_ms = 0;

    for (int i = 0; i < 16; i++)
    {
        rc_data_.ch[i] = 0;
    }
}

// Initialize SBUS communication
void RemoteControl::begin()
{
    if (sbus_rx_)
    {
        sbus_rx_->Begin();
    }
}

// Update SBUS data
bool RemoteControl::update()
{
    bool new_data = false;

    if (sbus_rx_ && sbus_rx_->Read())
    {
        // Get SBUS data
        sbus_data_ = sbus_rx_->data();

        // Update connection status
        rc_data_.connected = true;
        rc_data_.last_update_ms = millis();

        // Copy channel data
        for (int i = 0; i < 16; i++)
        {
            rc_data_.ch[i] = sbus_data_.ch[i];
        }

        // Copy flags
        rc_data_.lost_frame = sbus_data_.lost_frame;
        rc_data_.failsafe = sbus_data_.failsafe;

        // Extract drive mode from channel 6
        extractDriveMode();

        new_data = true;
    }

    // Check for timeout
    checkTimeout();

    return new_data;
}

// Extract drive mode from channel 6 (based on paste-2.txt logic)
void RemoteControl::extractDriveMode()
{
    int16_t ch6 = rc_data_.ch[6];

    if (ch6 <= 300)
    {
        rc_data_.drive_mode = RC_MODE_MANUAL;
    }
    else if ((ch6 >= 900) && (ch6 <= 1100))
    {
        rc_data_.drive_mode = RC_MODE_SEMI_AUTO;
    }
    else if ((ch6 >= 1700) && (ch6 <= 2000))
    {
        rc_data_.drive_mode = RC_MODE_FULL_AUTO;
    }
    else
    {
        rc_data_.drive_mode = RC_MODE_MANUAL;
    }
}


// Check for timeout
void RemoteControl::checkTimeout()
{
    if (millis() - rc_data_.last_update_ms > timeout_ms_)
    {
        rc_data_.connected = false;
    }
}

// Get remote control data
RemoteControlData RemoteControl::getData() const
{
    return rc_data_;
}

// Get connection status
bool RemoteControl::isConnected() const
{
    return rc_data_.connected;
}

// Get drive mode
uint8_t RemoteControl::getDriveMode() const
{
    return rc_data_.drive_mode;
}


// Get specific channel value
int16_t RemoteControl::getChannel(uint8_t channel) const
{
    if (channel < 16)
    {
        return rc_data_.ch[channel];
    }
    return 0;
}

// Get drive mode as string
const char* RemoteControl::getDriveModeString() const
{
    switch (rc_data_.drive_mode)
    {
        case RC_MODE_MANUAL:
            return "MANUAL";
        case RC_MODE_SEMI_AUTO:
            return "SEMI_AUTO";
        case RC_MODE_FULL_AUTO:
            return "FULL_AUTO";
        default:
            return "UNKNOWN";
    }
}


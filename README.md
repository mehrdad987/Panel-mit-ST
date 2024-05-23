Instead of using individual Serial_Data_Reset() and Serial_Clk_Reset() calls, you can group them together and send the data in a single block.

You can also try to minimize the number of times the display is updated by updating only the changed values instead of refreshing the entire display on each iteration.

###In this modified version, the functions start the serial data transfer, send the row address, send the segment data, and then end the serial data transfer in a single block. This should reduce the number of individual Serial_Data_Reset() and Serial_Clk_Reset() calls, potentially improving the display update performance.

Additionally, you can consider implementing a more efficient display update mechanism, where you only update the segments that have changed since the last update, instead of refreshing the entire display on each iteration. This can further optimize the display update process.
###

// Start 
void SendToSegment5Digit(unsigned char SegSTR[5], unsigned char ROW) {
    unsigned char i;
    unsigned char data;

    // Start the serial data transfer
    Serial_Data_Set();
    Serial_Clk_Set();

    // Send the row address
    data = (ROW << 4) | 0x0F;
    for (i = 0; i < 8; i++) {
        if (data & 0x80)
            Serial_Data_Set();
        else
            Serial_Data_Reset();
        Serial_Clk_Reset();
        Serial_Clk_Set();
        data <<= 1;
    }

    // Send the segment data
    for (i = 0; i < 5; i++) {
        data = SegSTR[i];
        for (int j = 0; j < 8; j++) {
            if (data & 0x80)
                Serial_Data_Set();
            else
                Serial_Data_Reset();
            Serial_Clk_Reset();
            Serial_Clk_Set();
            data <<= 1;
        }
    }

    // End the serial data transfer
    Serial_Data_Reset();
    Serial_Clk_Reset();
}
void SendToSegment4Digit(unsigned char SegSTR[4], unsigned char ROW) {
    unsigned char i;
    unsigned char data;

    // Start the serial data transfer
    Serial2_Data_Set();
    Serial2_Clk_Set();

    // Send the row address
    data = (ROW << 4) | 0x0F;
    for (i = 0; i < 8; i++) {
        if (data & 0x80)
            Serial2_Data_Set();
        else
            Serial2_Data_Reset();
        Serial2_Clk_Reset();
        Serial2_Clk_Set();
        data <<= 1;
    }

    // Send the segment data
    for (i = 0; i < 4; i++) {
        data = SegSTR[i];
        for (int j = 0; j < 8; j++) {
            if (data & 0x80)
                Serial2_Data_Set();
            else
                Serial2_Data_Reset();
            Serial2_Clk_Reset();
            Serial2_Clk_Set();
            data <<= 1;
        }
    }

    // End the serial data transfer
    Serial2_Data_Reset();
    Serial2_Clk_Reset();
}

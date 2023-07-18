#include <Arduino.h>

const int fifoSize = 128;
uint8_t bufferIndex = 0;
char buffer[fifoSize];

void readFromOtherCore()
{
    // Read from FIFO
    u_int32_t data;
    boolean gotData = rp2040.fifo.pop_nb(&data);

    if (gotData)
    {
        // Split data into chars and add to buffer
        for (int i = 0; i < 4; i++)
        {
            if (bufferIndex >= fifoSize)
            {
                Serial.println("Buffer overflow!");

                // Reset buffer
                memset(buffer, 0, fifoSize);
                bufferIndex = 0;
                break;
            }

            char value = (char)(data >> (i * 8));

            if (value == '\n')
            {
                // End of line
                // Do something with buffer
                Serial.println(buffer);
                memset(buffer, 0, fifoSize);
                bufferIndex = 0;
                break;
            }

            buffer[bufferIndex] = value;
            bufferIndex++;
        }
    }
}

void writeToOtherCore(String message)
{
    // message is ascii
    // convert to uint32_t by splitting into 4 bytes
    // send to other core
    
    for(int i = 0; i < message.length(); i+=4)
    {
        uint32_t data = 0;

        for(int j = 0; j < 4; j++)
        {
            if(i + j >= message.length())
            {
                break;
            }

            data |= (message[i + j] << (j * 8));
        }

        rp2040.fifo.push_nb(data);
    }
}
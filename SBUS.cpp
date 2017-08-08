/*
SBUS.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-13

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SBUS.h"

SBUS::SBUS(std::string tty)
{
    _tty = tty;
}

SBUS::~SBUS()
{
    close(_fd);
}

/* starts the serial communication */
void SBUS::begin()
{
    int r;

    // open file descriptor
    _fd = open(_tty, O_RDWR| O_NOCTTY);

    if(_fd < 0 )
        cout << "Error " << errno << " opening " << _tty << ": " << strerror(errno) << endl;

    // other configuration
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if(tcgetattr(_fd, &tty ) != 0 )
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;

    tty.c_cflag         &=  ~PARENB;                        // Make 8n1
    tty.c_cflag         &=  ~CSTOPB;
    tty.c_cflag         &=  ~CSIZE;
    tty.c_cflag         |=  CS8;
    tty.c_cflag         &=  ~CRTSCTS;                       // no flow control
    tty.c_lflag         =   0;                              // no signaling chars, no echo, no canonical processing
    tty.c_oflag         =   0;                              // no remapping, no delays
    tty.c_cc[VMIN]      =   0;                              // read doesn't block
    tty.c_cc[VTIME]     =   5;                              // 0.5 seconds read timeout

    tty.c_cflag     |=  CREAD | CLOCAL;                     // turn on READ & ignore ctrl lines
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);            // turn off s/w flow ctrl
    tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG);    // make raw
    tty.c_oflag     &=  ~OPOST;                             // make raw

    // flush port then apply attributes
    tcflush(USB, TCIFLUSH);

    if(tcsetattr(USB, TCSANOW, &tty) != 0)
        cout << "Error " << errno << " from tcsetattr" << endl;

    // set custom baudrate
    struct termios2 tio;
    r = ioctl(_fd, TCGETS2, &tio);
    if(r)
    {
        perror("TCGETS2");
        return -1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = tio.c_ospeed = 100000;

    r = ioctl(_fd, TCSETS2, &tio);
    if(r)
    {
        perror("TCSETS2");
        return -1;
    }
}

  // initialize parsing state
  /*_fpos = 0;
  #if defined(__MK20DX128__) || defined(__MK20DX256__)  // Teensy 3.0 || Teensy 3.1/3.2
    // begin the serial port for SBUS
    _bus->begin(100000,SERIAL_8E1_RXINV_TXINV);
    SERIALPORT = _bus;
  #endif

  #if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC
    // begin the serial port for SBUS
    _bus->begin(100000,SERIAL_8E2_RXINV_TXINV);
  #endif*/
}

/* read the SBUS data and calibrate it to +/- 1 */
bool SBUS::readCal(float* calChannels, uint8_t* failsafe, uint16_t* lostFrames)
{
    uint16_t channels[16];

    // read the SBUS data
    if(read(&channels[0],failsafe,lostFrames))
    {
        // linear calibration
        for(uint8_t i = 0; i < 16; i++)
        {
            calChannels[i] = channels[i] * _sbusScale + _sbusBias;
        }

        // return true on receiving a full packet
        return true;
    }
    else
    {
        // return false if a full packet is not received
        return false;
    }
}

/* read the SBUS data */
bool SBUS::read(uint16_t* channels, uint8_t* failsafe, uint16_t* lostFrames)
{
    // parse the SBUS packet
    if(parse())
    {
        // 16 channels of 11 bit data
        channels[0]  =(int16_t)((_payload[0]    |_payload[1]<<8)                          & 0x07FF);
        channels[1]  =(int16_t)((_payload[1]>>3 |_payload[2]<<5)                          & 0x07FF);
        channels[2]  =(int16_t)((_payload[2]>>6 |_payload[3]<<2 |_payload[4]<<10)         & 0x07FF);
        channels[3]  =(int16_t)((_payload[4]>>1 |_payload[5]<<7)                          & 0x07FF);
        channels[4]  =(int16_t)((_payload[5]>>4 |_payload[6]<<4)                          & 0x07FF);
        channels[5]  =(int16_t)((_payload[6]>>7 |_payload[7]<<1 |_payload[8]<<9)          & 0x07FF);
        channels[6]  =(int16_t)((_payload[8]>>2 |_payload[9]<<6)                          & 0x07FF);
        channels[7]  =(int16_t)((_payload[9]>>5 |_payload[10]<<3)                         & 0x07FF);
        channels[8]  =(int16_t)((_payload[11]   |_payload[12]<<8)                         & 0x07FF);
        channels[9]  =(int16_t)((_payload[12]>>3|_payload[13]<<5)                         & 0x07FF);
        channels[10] =(int16_t)((_payload[13]>>6|_payload[14]<<2|_payload[15]<<10)        & 0x07FF);
        channels[11] =(int16_t)((_payload[15]>>1|_payload[16]<<7)                         & 0x07FF);
        channels[12] =(int16_t)((_payload[16]>>4|_payload[17]<<4)                         & 0x07FF);
        channels[13] =(int16_t)((_payload[17]>>7|_payload[18]<<1|_payload[19]<<9)         & 0x07FF);
        channels[14] =(int16_t)((_payload[19]>>2|_payload[20]<<6)                         & 0x07FF);
        channels[15] =(int16_t)((_payload[20]>>5|_payload[21]<<3)                         & 0x07FF);

        // count lost frames
        if(_payload[22] & _sbusLostFrame)
        {
            *lostFrames = *lostFrames + 1;
        }

        // failsafe state
        if(_payload[22] & _sbusFailSafe)
        {
            *failsafe = 1;
        }
        else
        {
            *failsafe = 0;
        }

        // return true on receiving a full packet
        return true;
    }
    else
    {
        // return false if a full packet is not received
        return false;
    }
}

int SBUS::bytesAvalaible()
{
    int bytes_avail;
    ioctl(_fd, FIONREAD, &bytes_avail);
    return bytes_avail;
}

/* parse the SBUS data */
bool SBUS::parse()
{
    static elapsedMicros sbusTime = 0;

    if(sbusTime > SBUS_TIMEOUT)
        {_fpos = 0;}

    // see if serial data is available
    while(bytesAvalaible() > 0)
    {
        sbusTime = 0;
        uint8_t c;
        read(_fd, &c, 1);

        // find the header
        if(_fpos == 0)
        {
            if(c == _sbusHeader)
            {
                _fpos++;
            }
            else
            {
                _fpos = 0;
            }
        }
        else
        {
            // strip off the data
            if((_fpos-1) < _payloadSize)
            {
                _payload[_fpos-1] = c;
                _fpos++;
            }

            // check the end byte
            if((_fpos-1) == _payloadSize)
            {
                if((c == _sbusFooter)||((c & 0x0F) == _sbus2Footer))
                {
                    _fpos = 0;
                    return true;
                }
                else
                {
                    _fpos = 0;
                    return false;
                }
            }
        }
    }
    // return false if a partial packet
    return false;
}

/* write SBUS packets */
void SBUS::write(uint16_t* channels)
{
    uint8_t packet[25];

    /* assemble the SBUS packet */

    // SBUS header
    packet[0] = _sbusHeader;

    // 16 channels of 11 bit data
    packet[1] =(uint8_t)((channels[0] & 0x07FF));
    packet[2] =(uint8_t)((channels[0] & 0x07FF)>>8 |(channels[1] & 0x07FF)<<3);
    packet[3] =(uint8_t)((channels[1] & 0x07FF)>>5 |(channels[2] & 0x07FF)<<6);
    packet[4] =(uint8_t)((channels[2] & 0x07FF)>>2);
    packet[5] =(uint8_t)((channels[2] & 0x07FF)>>10 |(channels[3] & 0x07FF)<<1);
    packet[6] =(uint8_t)((channels[3] & 0x07FF)>>7 |(channels[4] & 0x07FF)<<4);
    packet[7] =(uint8_t)((channels[4] & 0x07FF)>>4 |(channels[5] & 0x07FF)<<7);
    packet[8] =(uint8_t)((channels[5] & 0x07FF)>>1);
    packet[9] =(uint8_t)((channels[5] & 0x07FF)>>9 |(channels[6] & 0x07FF)<<2);
    packet[10] =(uint8_t)((channels[6] & 0x07FF)>>6 |(channels[7] & 0x07FF)<<5);
    packet[11] =(uint8_t)((channels[7] & 0x07FF)>>3);
    packet[12] =(uint8_t)((channels[8] & 0x07FF));
    packet[13] =(uint8_t)((channels[8] & 0x07FF)>>8 |(channels[9] & 0x07FF)<<3);
    packet[14] =(uint8_t)((channels[9] & 0x07FF)>>5 |(channels[10] & 0x07FF)<<6);
    packet[15] =(uint8_t)((channels[10] & 0x07FF)>>2);
    packet[16] =(uint8_t)((channels[10] & 0x07FF)>>10 |(channels[11] & 0x07FF)<<1);
    packet[17] =(uint8_t)((channels[11] & 0x07FF)>>7 |(channels[12] & 0x07FF)<<4);
    packet[18] =(uint8_t)((channels[12] & 0x07FF)>>4 |(channels[13] & 0x07FF)<<7);
    packet[19] =(uint8_t)((channels[13] & 0x07FF)>>1);
    packet[20] =(uint8_t)((channels[13] & 0x07FF)>>9 |(channels[14] & 0x07FF)<<2);
    packet[21] =(uint8_t)((channels[14] & 0x07FF)>>6 |(channels[15] & 0x07FF)<<5);
    packet[22] =(uint8_t)((channels[15] & 0x07FF)>>3);

    // flags
    packet[23] = 0x00;

    // footer
    packet[24] = _sbusFooter;

    int n_written = write(_fd, packet, sizeof(packet) -1)

    cout << n_written << " bytes written" << endl;
}

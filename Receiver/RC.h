#define CHANNELS 8
#define IN_PIN 2 //GPIO2, D4 on wemos D1 mini



typedef struct 
{
uint16_t Ch[CHANNELS];
//  uint16_t Ch1     : 11; 
//  uint16_t Ch2     : 11;
//  uint16_t Ch3     : 11;
//  uint16_t Ch4     : 11;
//  uint16_t Ch5     : 11;
//  uint16_t Ch6     : 11;
//  uint16_t Ch7     : 11;
//  uint16_t Ch8     : 11;
//  uint8_t spare    : 8;
}RCdataTY;  //Payload

uint8_t pin[8];


#define RCdataSize 12
//typedef union
//{
//  Payload chans;
//  uint8_t data[RCdataSize];
//} RCdataTY;

RCdataTY RCdata;

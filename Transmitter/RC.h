#define RCCHANNELS 8
#define IN_PIN 2 //GPIO2, D4 on wemos D1 mini



typedef struct 
{
uint16_t Ch[RCCHANNELS];
}RCdataTY;  //Payload

uint8_t pin[8];


#define RCdataSize 12
//typedef union
//{
//  Payload chans;
//  uint8_t data[RCdataSize];
//} RCdataTY;

RCdataTY RCdata;

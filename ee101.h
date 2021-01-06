#define GRAY    0 // x-y ch:0
#define BROWN   1 // x-y ch:1
#define RED     2 // x-y ch:2
#define ORANGE  3 // x-y ch:3
#define YELLOW  4 // x-y ch:4
#define GREEN   5 // x-y ch:5
#define BLUE    6 // x-y ch:6
#define PURPLE  7 // x-y ch:7

#define euint8   unsigned char    // unsigned 8 bit value
#define eint8    signed char      // signed 8 bit value
#define eint32   signed long      // signed 32 bit value
#define echar    char             // bytes within a string

void EE101Value( euint8 channel, eint32 value );            // Output a Value for this channel
void EE101Text( euint8 channel, echar *string );            // Output Text for this channel
void EE101ValueLabel( euint8 channel, echar *string );      // Set the label for this Value Channel (sent every 256 times)
void EE101TextLabel( euint8 channel, echar *string );       // Set the label for this Text Channel (sent every 256 times)
void EE101printf( euint8 channel, echar *format, ... );     // printf-like function with variable argument list

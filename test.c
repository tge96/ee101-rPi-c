/*pi@raspberrypi:~/ee101 $ gpio readall
 +-----+-----+---------+------+---+---Pi 3B+-+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
 |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
 |   4 |   7 | GPIO. 7 |  OUT | 0 |  7 || 8  | 1 | ALT0 | TxD     | 15  | 14  |
 |     |     |      0v |      |   |  9 || 10 | 1 | ALT0 | RxD     | 16  | 15  |
 |  17 |   0 | GPIO. 0 |  OUT | 0 | 11 || 12 | 0 | OUT  | GPIO. 1 | 1   | 18  |
 |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 |  22 |   3 | GPIO. 3 |  OUT | 0 | 15 || 16 | 0 | OUT  | GPIO. 4 | 4   | 23  |
 |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
 |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
 |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 |   6 |  22 | GPIO.22 |  OUT | 0 | 31 || 32 | 0 | OUT  | GPIO.26 | 26  | 12  |
 |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
 |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | OUT  | GPIO.27 | 27  | 16  |
 |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
 |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+---Pi 3B+-+---+------+---------+-----+-----+*/
//https://pinout.xyz/pinout/1_wire - disable 1-wire, otherwise wPi 7 is INput

  /* Worth a mention, here are some "old" GPIO notes from the author of wiringPi:
   * WiringPi defines 17 pins, (21 on a Rev. 2 board) but some of them 
  and the functions we can use may potentially cause problems with other 
  parts of the Raspberry Pi Linux system. 
  * Pins 0 through 6 (BCM_GPIO 17, 18,  21, 22, 23, 24, 25 respectively, 
  substitute 27 for 21 on a Rev. 2 board): These are safe to use at any 
  time and can be set to input or output with or without the internal 
  pull-up or pull-down resistors enabled. 
  * Pin 7 (BCM_GPIO 4) This is normally OK to use, however it is used by 
  the 1-Wire kernel driver and it can optionally be connected to a GPIO 
  clock source. 
  * PWM: You can change the function of pin 1 (BCM_GPIPO 18) to be PWM 
  output, however if you are currently playing music or using the audio 
  system via the 3.5mm jack socket, then you’ll find one channel of audio 
  PWM coming through the pin! If you are not using the audio at all, (or 
  the audio is going via the HDMI cable), then this pin is free to be 
  used in PWM mode. 
  * Pins 8 and 9 * (BCM GPIO 0 and 1 on a Rev. 1 board, 2 and 3 on a Rev. 
  2 board): These are the I2C pins. You may use them for digital IO if 
  you are not using any I2C drivers which use these pins, however note 
  that they have on-board 1.8KΩ resistors pulling the signals to the 3v3 
  supply. This feature does make them handy for switch inputs where the 
  switch simply shorts the pin to ground without having to enable the 
  internal pull-up resistors 
  * Pins 10, 11, 12, 13 and 14 (GPIO 8, 7, 10, 9 and 11 respectively): 
  These are used for the SPI interface. Like the I2C interface, if you 
  are not using it, then you can freely use them for your own purposes. 
  Unlike I2C, these pins do not have any external pull up (or pull down) 
  resistors. 
  * Pins 15 and 16 (GPIO 14 and 15): These are used by the UART for Tx 
  and Rx respectively. If you want to use these pins as general purpose 
  I/O pins then you need to make sure that you reboot your Pi with the 
  serial console disabled. See the file /boot/cmdline.txt and edit it 
  appropriately. 
  * Pins 17, 18, 19 and 20: (BCM_GPIO 28, 29, 30 and 31) These are 
  additional GPIO pins on the Rev. 2 board. 
  * Remember: The Raspberry Pi is a 3.3 volt device! Attempting to 
  directly connect to any 5V logic system will very likely result in 
  tears… */

//pi@raspberrypi:~/ee101 $ gcc -c test.c -l wiringPi -l ncurses
//pi@raspberrypi:~/ee101 $ gcc -c ee101.c -l wiringPi -l ncurses
//pi@raspberrypi:~/ee101 $ gcc -o mytest test.o ee101.o -l wiringPi -l ncurses
//pi@raspberrypi:~/ee101 $ ./mytest 

//#include <stdio.h>
//#include <string.h>
#include <ncurses.h>      // $ apt-get install libncurses-dev   # add -l ncurses to linker command
#include "ee101.h"        // EE101 Embedded Firmware Debugger, www.ee101.com # add ee101.c and ee101.h files to your project
#include <wiringPi.h>     // Rpi GPIO library, www.wiringpi.com # add -l wiringPi to linker command.
#include <wiringSerial.h> // included when adding -l wiringPi to linker command
#include <wiringPiI2C.h>  // included when adding -l wiringPi to linker command 
#include <wiringPiSPI.h>  // included when adding -l wiringPi to linker command
#include <wiringShift.h>  // included when adding -l wiringPi to linker command
#include <softPwm.h>      // add -l pthread to linker command
#include <softServo.h>    // add -l pthread to linker command
#include <softTone.h>     // add -l pthread to linker command

int fd;
int main(int argc, char *argv[]) {
  int i, x, ch;
  
  if(wiringPiSetup() == -1) return 1;                                               // setup wiringPi lib, using wPi pins notation
  pinMode(7, OUTPUT); pinMode(0, OUTPUT); pinMode(3, OUTPUT); pinMode(22, OUTPUT);  // set some GPIOs as outputs, see comment below about wPi 7 usage
  pinMode(1, OUTPUT); pinMode(4, OUTPUT); pinMode(26, OUTPUT); pinMode(27, OUTPUT); // LEDS   
  pinMode(25, INPUT); pullUpDnControl(25, PUD_UP);                                  // wPi 25 as input, w/ ~50k-ohm pull-up enabled
     
  if((fd = serialOpen("/dev/serial0", 9600)) < 0) return 1;                   // open handle to /dev/serial0 using wiringSerial, used for EE101 1-wire mode, must disable Rpi's serial console, $ sudo raspi-config
  initscr(); cbreak(); noecho(); keypad(stdscr, TRUE); nodelay(stdscr, TRUE); // configure terminal using ncurses, mainly for its non-blocking getch(); implementation
  printw("Press any key to exit program...\n");
  
  x = 115;

  for (;;) {
    x = x + 20;
    if(x > 315 ) x = 115;
    EE101Text(GRAY,"11.29.20");                                               // continuously print some debug messages to the EE101 firmware debugger
    EE101Text(BROWN,"Raspberry Pi 3B+ Ultimate Kit, www.canakit.com");        // ...until user presses any key, on their keyboard, via terminal window
    EE101Text(RED,"EE101 Firmware Debugger, www.ee101.com");
    EE101Text(ORANGE,"...using wiringPi library, www.wiringpi.com");
    EE101Text(YELLOW,"...using EE101 Firmware Debugger 1-wire mode @ /dev/serial0");
    EE101Text(GREEN,"...using wiringPi/wiringSerial/ncurses libraries");
    EE101Text(BLUE,"...using Geany IDE, with manual command line build process");
    EE101Text(PURPLE,"...with only standard edits to EE101 Firmware Debugger source files");
    
    i = digitalRead(25);
    
    EE101Value(GRAY, millis());
    EE101Value(BROWN, millis());
    EE101Value(RED, micros());
    EE101Value(ORANGE, micros());
    EE101Value(YELLOW, i);
    EE101Value(GREEN, wpiPinToGpio(7));
    EE101Value(BLUE, x /*physPinToGpio(7)*/);
    EE101Value(PURPLE, getAlt(7));
    
    digitalWrite(1, HIGH); delay(x); digitalWrite(1, LOW); delay(x);     // toggle wPi 1 LED
    digitalWrite(4, HIGH); delay(x); digitalWrite(4, LOW); delay(x);     // toggle wPi 4 LED
    digitalWrite(26, HIGH); delay(x); digitalWrite(26, LOW); delay(x);   // toggle wPi 26 LED
    digitalWrite(27, HIGH); delay(x); digitalWrite(27, LOW); delay(x);   // toggle wPi 27 LED
    
    digitalWrite(7, HIGH); delayMicroseconds(50); digitalWrite(7, LOW); delayMicroseconds(50); // disable Rpi's 1-wire feature for wPi 7 OUT to work, $ sudo raspi-config
    digitalWrite(0, HIGH); delayMicroseconds(100); digitalWrite(0, LOW); delayMicroseconds(100);
    digitalWrite(3, HIGH); delayMicroseconds(150); digitalWrite(3, LOW); delayMicroseconds(150);
    digitalWrite(22, HIGH); delayMicroseconds(200); digitalWrite(22, LOW); delayMicroseconds(200);
    
    if(ch = getch() != ERR) break;                                            // see printw("Press any key to exit program...\n"); above
  }
  serialClose(fd);                                                            // close handle to /dev/serial0 using wiringSerial
  endwin();                                                                   // close ncurses, returning terminal to normal
  return 0;
}

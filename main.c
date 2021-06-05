#include <stdint.h>
#include "D:/College/Embedded 2/EE319Kware/inc/tm4c123gh6pm.h"
#define LCD_RS  (*((volatile unsigned long *)0x40004200))    //PA.7 for register select pin

#define LCD_EN  (*((volatile unsigned long *)0x40004100))   //PA.6 for enable pin

#define LCD_RW  (*((volatile unsigned long *)0x40004080))   //PA.5 for rw pin
void SystemInit(){}
/* Milli seconds delay function */
void mdelay(int n)
{
 int i,j;
 for(i=0;i<n;i++)
 for(j=0;j<3180;j++)
 {}
}

/* Micro seconds delay function */
void udelay(int n)
{
 int i,j;
 for(i=0;i<n;i++)
 for(j=0;j<3;j++)
 {}

}
void LCD_INIT(void) {
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0X00000002;   // allow the clock for portB
    delay = SYSCTL_RCGC2_R;     // short delay for clock
    GPIO_PORTB_AFSEL_R &= ~0xFF;    //disable alternative functions for portB
    GPIO_PORTB_AMSEL_R &= ~0XFF;    //disable analogue function
    GPIO_PORTB_PCTL_R &= ~0XFF;     //regular digital pins
    GPIO_PORTB_DIR_R  |= 0XFF;      //set the direction of PB0-7 as     output
    GPIO_PORTB_DEN_R  |= 0XFF;      //enable digital portB

    SYSCTL_RCGC2_R |= 0X00000001;   // allow the clock for PA5,6,7
    delay = SYSCTL_RCGC2_R;     // short delay for clock
    GPIO_PORTA_AFSEL_R &= ~0xE0;    //disable alternative functions for PA5,6,7
    GPIO_PORTA_AMSEL_R &= ~0XE0;    //disable analogue function for PA5,6,7
    GPIO_PORTA_PCTL_R &= ~0XE0;     //regular digital pins
    GPIO_PORTA_DIR_R |= 0XE0;       //set the direction of PA5,6,7 as output
    GPIO_PORTA_DEN_R |= 0XE0;       //enable digital PA5,6,7
}
//this function passes the command to the LCD
void LCD_command (unsigned char cmd) {
  LCD_RS = 0x00;  //set PA7 register select pin to command
  LCD_RW = 0x00;  //set PA5 r/w pin to write
  GPIO_PORTB_DATA_R = cmd;    //set PB7-0 as the passed command to the function

   LCD_EN = 0x40;  //set enable pin to high
   mdelay(50);
   LCD_EN = 0x00;  //set enable pin to low
}
//this function passes the data to the LCD
void LCD_Data (unsigned char data) {
    LCD_RS = 0x80;  //set PA7 to data
    LCD_RW = 0x00;  //set pA5 to
    GPIO_PORTB_DATA_R = data;   //write the data to PB7-0
    LCD_EN = 0x40;  //set the enable pin high
    mdelay(50);
    LCD_EN = 0x00;  //set the enable pin to low
}

int main(void)
{
     LCD_INIT();
     LCD_command(0X01);  //clear display
          mdelay(50);
    while(1) {
        LCD_command(0X30);  //wake up call
        mdelay(50);
        LCD_command(0X38);  //8-bit bus mode, 2 line display mode, 5x8 dots display mode
        mdelay(50);
       // LCD_command(0X01);  //clear display
       // mdelay(50);
        LCD_command(0X0F);  //diplay is on
        mdelay(50);
        LCD_command(0X80);  //First line

        LCD_Data('H');
        mdelay(500);
        LCD_Data('O');
        mdelay(500);
        LCD_Data('S');
        mdelay(500);







        LCD_command(0X06);  //cursor display shift
        mdelay(500);
        LCD_Data('B');
        mdelay(500);
    }

}

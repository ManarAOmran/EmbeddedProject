
#include "D:/College/Embedded 2/EE319Kware/inc/tm4c123gh6pm.h"
#include <stdint.h>
#include <string.h>
#include "string.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define ConversionFactor 111194.93  // = pi/180 *radius of earth:6371* 10^3 m --> to convert from degrees to meters
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board

#define LCD_RS  (*((volatile unsigned long *)0x40004200))    //PA.7 for register select pin

#define LCD_EN  (*((volatile unsigned long *)0x40004100))   //PA.6 for enable pin

#define LCD_RW  (*((volatile unsigned long *)0x40004080))   //PA.5 for rw pin
char lon_c [15];
char lat_c[15];
char  in;
float lon[1000];
float lat [1000];
long int counter = 0;
float distance =0;
char input;
int i = 0;
int j = 0;
float lon1,lat1,lon2,lat2;
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
void InitPortF(){

SYSCTL_RCGCGPIO_R |= 0X20;
while((SYSCTL_PRGPIO_R & 0x20)==0);
GPIO_PORTF_LOCK_R = 0x4C4F434B;
GPIO_PORTF_CR_R = 0x02;
GPIO_PORTF_DEN_R = 0x02;
GPIO_PORTF_AMSEL_R = 0x00;
GPIO_PORTF_DIR_R = 0x02;
GPIO_PORTF_PUR_R = 0x00;
GPIO_PORTF_AFSEL_R = 0x00;
GPIO_PORTF_PCTL_R = 0x00000000;
}
uint32_t PortF_Input(void){
  return (GPIO_PORTF_DATA_R&0x11);
}
void LCD_INIT(void) {
    SYSCTL_RCGCGPIO_R |= 0x02 ; // Initialize all clocks for ports A,B,D, and F
    while( (SYSCTL_PRGPIO_R & 0x02) == 0 ) {}
    //volatile unsigned long delay;
    //SYSCTL_RCGC2_R |= 0X00000002;   // allow the clock for portB
    //delay = SYSCTL_RCGC2_R;     // short delay for clock
    GPIO_PORTB_AFSEL_R &= ~0xFF;    //disable alternative functions for portB
    GPIO_PORTB_AMSEL_R &= ~0XFF;    //disable analogue function
    GPIO_PORTB_PCTL_R &= ~0XFF;     //regular digital pins
    GPIO_PORTB_DIR_R  |= 0XFF;      //set the direction of PB0-7 as     output
    GPIO_PORTB_DEN_R  |= 0XFF;      //enable digital portB


    SYSCTL_RCGCGPIO_R |= 0x01 ; // Initialize all clocks for ports A,B,D, and F
    while( (SYSCTL_PRGPIO_R & 0x01) == 0 ) {}
    //SYSCTL_RCGC2_R |= 0X00000001;   // allow the clock for PA5,6,7
    //delay = SYSCTL_RCGC2_R;     // short delay for clock
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
   //mdelay(50);
   LCD_EN = 0x00;  //set enable pin to low
}
//this function passes the data to the LCD
void LCD_Data (unsigned char data) {
    LCD_RS = 0x80;  //set PA7 to data
    LCD_RW = 0x00;  //set pA5 to
    GPIO_PORTB_DATA_R = data;   //write the data to PB7-0
    LCD_EN = 0x40;  //set the enable pin high
    //mdelay(50);
    LCD_EN = 0x00;  //set the enable pin to low
}

void LCD_display(unsigned char *str)
{
    int i;
    for(i = 0; str[i] != '\0'; i++)
    {
        LCD_Data(str[i]);
        //mdelay(500);
    }
}



void UART0_Init(void){
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;

    UART0_CTL_R &= ~UART_CTL_UARTEN;
    //set buad rate devider
    UART0_IBRD_R = 104;
    UART0_FBRD_R = 11;
        UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART0_CTL_R |= (UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE);

    GPIO_PORTA_AFSEL_R |= 0x03;
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & ~0xFF) | (GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX);
    GPIO_PORTA_DEN_R |= 0x03;
}

char UART0_read(void){
    while((UART0_FR_R&0x10) == 0x10);
    return UART0_DR_R & 0xFF;
}

void UART0_write(char c){
    while((UART0_FR_R & UART_FR_TXFF) != 0);
    UART0_DR_R = c;
}

void UART2_Init(void){
   SYSCTL_RCGCUART_R |= 0x04 ; //enable UART2 Clock
    SYSCTL_RCGCGPIO_R |= 0x08 ;
    while( (SYSCTL_PRGPIO_R & 0x08) == 0 ) {}

    UART2_CTL_R  &= 0xFFFE; //Enable = 0
    UART2_IBRD_R = 104 ; //setting baud rate tp 9600
    UART2_FBRD_R = 11 ;
    UART2_LCRH_R = 0x70 ; //fifo enabled, 8 bit word length , one stop bit , no  parity check
    UART2_CTL_R  |= 0x0201; //Enable = 1 , RXE = 1 , Receive only

    GPIO_PORTD_AMSEL_R  &= ~0xC0 ;
    GPIO_PORTD_DEN_R |= 0xC0 ;
    GPIO_PORTD_AFSEL_R |= 0xC0 ;
    GPIO_PORTD_PCTL_R = 0x11000000 ;
}

char UART2_read(void){
    while((UART2_FR_R & 0x10) == 0x10);
    return UART2_DR_R & 0xFF;
}
double Extract_degrees(char* lon) {
    double dl = atof(lon);
    double dl2 = dl / 100;
    int dint = (int)dl2;
    double dpoint = (dl2 - dint) * 100;
    return dint + (dpoint / 60);
}

char uart_rx() {
  in = UART2_read();
  return in;
}
void get_input()
{
  while (1) {
      //LCD_display("distance ");

    input = uart_rx();
    if(input!= '$')
      continue;
      input = uart_rx();
    if(input!= 'G')
      continue;
        input = uart_rx();
    if(input!= 'P')
        continue;
        input = uart_rx();
        if(input!= 'G')
          continue;
          input = uart_rx();
          if(input!= 'G')
            continue;
            input = uart_rx();
            if(input!= 'A')
              continue;
              input = uart_rx();
              if(input!= ',')
                continue;
                input = uart_rx();
                while(input!= ','){
                  input = uart_rx();
                }
                  input = uart_rx();
                  for(i=0;input!=',';i++) {
                    lat_c[i]=input;
                    input = uart_rx();
                  }
                  input = uart_rx();
                  while(input!= ',') {
                    input = uart_rx();
                  }
                  input = uart_rx();
                  for(j=0; input!=','; j++){
                    lon_c[j] = input;
                    input = uart_rx();

                  }
                  char status = PortF_Input();
                  if (status == 0x01){  // SW1 pressed
                      lon1 = Extract_degrees(lon_c);
                             lat1 = Extract_degrees(lat_c);
                  }
                  else if (status== 0x10){  // SW2 pressed
                      lon2 = Extract_degrees(lon_c);
                               lat2 = Extract_degrees(lat_c);
                               break;

                  }
  }
        lat1 = lat1 * ConversionFactor;
        lon1 = lon1 * ConversionFactor;
        lat2 = lat2 * ConversionFactor;
        lon2 = lon2 * ConversionFactor;

     distance = fabs(lat1 - lat2) +fabs(lon1 - lon2);
     if (distance >100)
         {
         GPIO_PORTF_DATA_R |= 0X02;
         }
     char vout = (char)distance;
     LCD_display("distance");

     LCD_display(vout);


 // printf("lon = %f", lon);
  //printf("lat = %f", lat);


}

int main() {
    InitPortF();
             LCD_INIT();
             //UART0_Init();
             UART2_Init();
             //LCD_command(0X01);  //clear display
                  //mdelay(50);
            while(1) {
                LCD_command(0X30);  //wake up call
               // mdelay(50);
                LCD_command(0X38);  //8-bit bus mode, 2 line display mode, 5x8 dots display mode
                //mdelay(50);
                LCD_command(0X01);  //clear display
               // mdelay(50);
                LCD_command(0X0F);  //diplay is on
                //mdelay(50);
                LCD_command(0X80);  //First line
                //LCD_display(UART2_read());
                //mdelay(500);
                get_input();
               /* LCD_Data('H');
                mdelay(500);
                LCD_Data('O');
                mdelay(500);
                LCD_Data('S');
                mdelay(500);







                LCD_command(0X06);  //cursor display shift
                mdelay(500);
                LCD_Data('B');
                mdelay(500); */
            }

}

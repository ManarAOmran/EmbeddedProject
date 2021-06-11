#include "c:/Keil/EE319Kware/inc/tm4c123gh6pm.h"
#include <stdint.h>

// UART configuration function
void UART_Init(void){
SYSCTL_RCGCUART_R |= 0x20;  //activate UART5
SYSCTL_RCGCGPIO_R |= 0x10;  // activate portE
while((SYSCTL_PRGIO_R & 0x10) == 0);  // DELAY(can be eliminated)

 UART5_CTL_R = 0;         // disable UART (can't be programmed if enabled)
 UART5_IBRD_R = 104;      // int(16MHz/(16*9600))=int( 104.1667)
 UART5_FBRD_R = 11;       // round(0.1667 * 64) = 11
 UART5_LCRH_R = 0x70;     //  8-bit data, 1 stop bit, not parity bit, FIFO
 UART5_CTL_R = 0x0301;     // Enable UART5 , Rx , Tx


GPIO_PORTE_DEN_R |= 0x30;      // Use PE4, PE5 as digital
GPIO_PORTE_AMSEL_R &= ~ 0x30;    // Turn off analog function
GPIO_PORTE_AFSEL_R | = 0x30;    // Use PE4,PE5 as alternate function
GPIO_PORTE_PCTL_R = 0x00110000;     // Use PE4, PE5 as UART
                                   //(i used direct assignment since i know the others pins aren't used)
}






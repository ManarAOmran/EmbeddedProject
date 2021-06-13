
// transmitting function
void UART5_Transmitter(char data)
{
    while((UART5_FR_R & 0x20) != 0);  // check if FIFO is full
    UART5_DR_R = data;
}

// receiving function
char UART5_Receiver(void)
{
    char data;
     while((UART5_FR_R & 0x10) != 0); // check if FIFO is not empty
     data = UART5_DR_R ;
    return (unsigned char) data;
}

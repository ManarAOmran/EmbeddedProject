#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdlib.h>
char lon_c [15];
char lat_c[15];
char  in;
float lon[1000];
float lat [1000];
long int counter = 0;
char input;
int i = 0;
int j = 0;
char uart_rx() {
  in = UART2_read()
}
void get_input()
{
  while (1) {
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
                  break;
  }

  //printf("lon = %f", lon);
  //printf("lat = %f", lat);


}

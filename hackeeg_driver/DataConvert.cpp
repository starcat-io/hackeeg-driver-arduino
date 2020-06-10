//#include "DataConvert.h"
//#include <iostream.h>
#include <stdio.h>
#include <string.h>
//encode_multi_plot(output,input + TIMESTAMP_SIZE_IN_BYTES + SAMPLE_NUMBER_SIZE_IN_BYTES, int max_channels)
void encode_multi_plot(char *output, char *input, int channels){
    register int count = 0;
    //char * FrameSign = "A5A5";
    //strcpy(output,"A5A5");
    output[count++] = 0xA5;
    output[count++] = 0xA5;
    //count = sizeof(FrameSign);
    for (register int i = 0; i < channels; i++) {
        if(input[i*3+0]&0x80)
            output[count++] = 0xff;//MSB
        else
            output[count++] = 0;//MSB

        output[count++] = input[i*3+0];
        output[count++] = input[i*3+1];
        output[count++] = input[i*3+2];//LSB
    }
    output[count] = 0;
	
}

#include "mbed.h"

RawSerial pc(USBTX, USBRX);

RawSerial a(PA_9, PA_10);

int main() {
    a.baud(115200);
    pc.baud(115200);
    while(true) {
        if(a.readable()) 
            pc.putc(a.getc());
        if(pc.readable())
            a.putc(pc.getc());
    }
}

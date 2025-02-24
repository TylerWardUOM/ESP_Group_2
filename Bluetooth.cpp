#include "mbed.h" 

Serial pc(USBTX, USBRX);  // PC serial port (for PuTTY)
Serial hm10(PA_11, PA_12); // HM-10 serial port

int main() {
    pc.baud(9600);  // PC serial port baud rate
    hm10.baud(9600);  // HM-10 baud rate
    
    //pc.printf("HM-10 AT \r\n");

    while (1) {
        // Read AT commands from PC and forward them to HM-10
        if (pc.readable()) {
            char c = pc.getc();
            hm10.putc(c);  // Send to HM-10
        }

        // Read response from HM-10 and send it back to PC
        if (hm10.readable()) {
            char d = hm10.getc();
            pc.putc(d);  // Display HM-10 response in PuTTY
        }
    }
}
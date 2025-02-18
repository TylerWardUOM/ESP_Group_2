#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "LED.h"



//bipolar 1 jp1 pin 1 digital out 
//jp1 pin 2
//jp1 pin 3
//enable jp1 pin 7

Motor motor1 = Motor(PB_7,PA_13,PB_14,PA_14);
Motor motor2 = Motor(PB_15,PB_2,PB_13,PA_14);

bool flag = true;
int main(){
    motor2.stop();
    if (flag==true){
        wait(1);
        motor1.setSpeed(0.5);
        wait(1);
        motor1.setSpeed(0.1);
        wait(1);
        motor1.stop();
        flag=false;
    }
    if (flag==false){
        motor1.stop();
    }
    return 0;
}

// #include <stdint.h> // maybe need to include?

void PixyControl(uint16_t x){
    // variables
    uint16_t r = 158;
    double p = 1;
    double i = 0;
    double d = 0;

    // calculate error
    double error = x - r;
    
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    analogWrite(PWM1,255);
    analogWrite(PWM2,-255);

}

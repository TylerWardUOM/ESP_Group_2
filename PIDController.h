#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float scaling_multiplier = 1);
    float update(float error, float dt);
    void reset();

private:
    float _scaling_multiplier;
    float kp, ki, kd;
    float integral;
    float prevError;
};

#endif // PID_CONTROLLER_H
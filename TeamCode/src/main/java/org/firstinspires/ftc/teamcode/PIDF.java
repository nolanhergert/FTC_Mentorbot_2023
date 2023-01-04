package org.firstinspires.ftc.teamcode;

public class PIDF {
    public double setpoint;
    public double kp;
    public double ki;
    public double kd;
    public double kf;

    private double integral;
    private double last_sensor;
    private double last_time;

    private boolean is_limited = false;
    private double max_actuator = 0;

    public double integral_max = 10;

    public PIDF(){
        integral = 0;
        last_sensor = 0;
        last_time = 0;
    }

    public void set_max(double max_act){
        max_actuator = max_act;
        is_limited = true;
    }

    public double next(double sensor, double time){
        double ds = sensor - last_sensor;
        double dt = time - last_time;

        if(dt > 1){
            dt = 0.03; // probable
        }

        double e = sensor - setpoint;
        integral += e*dt;

        double base_power = kf*setpoint - kp*e;
        if(is_limited && Math.abs(ki*integral + base_power) > max_actuator && Math.abs(base_power) < max_actuator){
            // back-calculate a permissible accumulator value
            integral = (max_actuator * Math.signum(base_power) - base_power) / ki;
        }

        if(integral > integral_max){
            integral = integral_max;
        }
        integral *= 0.99;

        last_sensor = sensor;
        last_time = time;
        return base_power + -kd*ds/dt + -ki*integral;
    }
}

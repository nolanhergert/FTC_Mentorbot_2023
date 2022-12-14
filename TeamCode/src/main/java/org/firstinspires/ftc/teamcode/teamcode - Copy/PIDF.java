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

    public PIDF(){
        integral = 0;
        last_sensor = 0;
        last_time = 0;
    }

    public double next(double sensor, double time){
        double ds = sensor - last_sensor;
        double dt = time - last_time;
        double e = sensor - setpoint;
        integral += e/dt;
        last_sensor = sensor;
        last_time = time;
        return -kp*e + -kd*ds/dt + -ki*integral + kf*setpoint;
    }
}

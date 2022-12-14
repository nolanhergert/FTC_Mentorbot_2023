package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Elevator {
    PIDF ctrl_lift;

    private double stall_power = 0.0;

    private DcMotor lift; // POSITIVE is UP
    private Servo drop;
    private TouchSensor limit;

    private double SERVO_UP = 0.0;
    private double SERVO_DOWN = 0.3;
    private double LOW_LIMIT = -0.5;

    private double prev_t;
    private boolean prev_limit;

    private double inch_offset = 0;

    // (in/tick) = (in/mm) * (mm/teeth) * (teeth/rev) * (rev/ticks)
    // in/tick = (tooth/rev) * (mm/tooth) / (mm/in) / (ticks/rev)
    private double ticks_per_rev = 288;
    private double teeth_per_rev = 12;
    private double mm_per_tooth = 3;
    private double mm_per_in = 25.4;
    private double in_per_tick = teeth_per_rev * mm_per_tooth / mm_per_in / ticks_per_rev;

    public double lift_position(){
        return lift.getCurrentPosition() * in_per_tick - inch_offset;
    }

    public Elevator(DcMotor lift, Servo drop, TouchSensor limit){
        this.lift = lift;
        this.drop = drop;
        this.limit = limit;

        // more guesses
        ctrl_lift = new PIDF();
        ctrl_lift.kf = 0;
        ctrl_lift.kp = 1;

        prev_t = System.currentTimeMillis() / 1000.0;
    }

    private void reset_position(){
        this.inch_offset = 0;
        this.inch_offset = lift_position();
    }

    public void set_posn(double val){
        ctrl_lift.setpoint = val;
    }

    public void go_manual(double power){
        if(limit.isPressed() && !prev_limit) {
            this.reset_position();
        }

        if(this.lift_position() < this.LOW_LIMIT && power < 0){
            power = 0;
        }
        prev_limit = limit.isPressed();
        lift.setPower(power);
    }

    public void go(){
        double time = System.currentTimeMillis() / 1000.0;
        double dt = time - prev_t;
        prev_t = time;
        double x = this.lift_position();
        go_manual(ctrl_lift.next(x, dt) + this.stall_power);
    }

    public void drop(){
        this.drop.setPosition(SERVO_DOWN);
    }

    public void undrop(){
        this.drop.setPosition(SERVO_UP);
    }
}

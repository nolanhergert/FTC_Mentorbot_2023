package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Elevator {
    public PIDF ctrl_lift;

    private double stall_power = 0.0;

    private DcMotor lift; // POSITIVE is UP
    private Servo drop;
    private TouchSensor limit;

    private double SERVO_UP = 0.15;
    private double SERVO_LOCK = 0.0;
    private double SERVO_DOWN = 0.4;

    private double LOW_LIMIT = -0.1;
    private double HIGH_LIMIT = 13.7;
    private double dynamic_idle_posn = 3.0;

    public boolean auto = true;

    private double prev_t;
    private boolean prev_limit;
    
    private boolean homed = false;
    private double inch_offset = 0;

    // (in/tick) = (in/mm) * (mm/teeth) * (teeth/rev) * (rev/ticks)
    // in/tick = (tooth/rev) * (mm/tooth) / (mm/in) / (ticks/rev)
    private double ticks_per_rev = 288;
    private double teeth_per_rev = 12;
    private double mm_per_tooth = 3;
    private double mm_per_in = 25.4;
    private double extra_ratio = 72.0/30.0;
    private double in_per_tick = teeth_per_rev * extra_ratio * mm_per_tooth / mm_per_in / ticks_per_rev;

    // Roughly the position of the bottom of the cone
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
        ctrl_lift.kp = 0.5;
        ctrl_lift.ki = 0.00;
        ctrl_lift.set_max(1);

        prev_t = System.currentTimeMillis() / 1000.0;
    }

    // typical sequence:
    // 1. idle (servo unlocked, elevator above floor cone height)
    // 2. pickup (servo unlocked, elevator at bottom)
    // Then either 3 or 1
    // 3. lift (servo locked, elevator above floor cone height)
    // 4. drop (servo drop, elevator doesn't move
    // Back to step 1

    // shortcuts:
    // go to top (for score on low thing)
    // go to low 3" (for score on unscored terminal)
    // go to medium 7"-8"(for score on scored terminal)

    private void reset_position(){
        this.inch_offset = 0.31;
        this.inch_offset = lift_position();
    }

    public void set_posn(double val){
        ctrl_lift.setpoint = val;
    }

    public void pickup(){
        this.set_posn(LOW_LIMIT);
        this.drop.setPosition(SERVO_UP);
    }

    public void lift(){
        this.set_posn(this.lift_position() + 6);
        this.drop.setPosition(SERVO_LOCK);
        dynamic_idle_posn = this.lift_position() + 4;
    }

    public void idle(){
        this.set_posn(dynamic_idle_posn);
        this.drop.setPosition(SERVO_UP);
    }

    public void go_manual(double power){
        if(limit.isPressed() && !prev_limit) {
            if(!homed){
                power = 0;
            }
            homed = true;
            this.reset_position();
        }

        if(this.lift_position() < this.LOW_LIMIT && power < 0){
            power = 0;
        }

        if(this.lift_position() > this.HIGH_LIMIT && power > 0){
            power = 0;
        }

        if(!homed){
            if(this.limit.isPressed()){
                power = 0.5;
                if(!prev_limit){
                    homed = true;
                    this.reset_position();
                }
            }
            else {
                power = -1;
                if(prev_limit){
                    homed = true;
                    this.reset_position();
                }
            }
        }
        prev_limit = limit.isPressed();
        lift.setPower(power);
    }

    public void go(){
        if(!auto){
            return;
        }
        double time = System.currentTimeMillis() / 1000.0;
        prev_t = time;
        double x = this.lift_position();
        go_manual(ctrl_lift.next(x, time) + this.stall_power);
    }

    public void drop(){
        this.drop.setPosition(SERVO_DOWN);
    }

    public void undrop(){
        this.drop.setPosition(SERVO_UP);
    }
}

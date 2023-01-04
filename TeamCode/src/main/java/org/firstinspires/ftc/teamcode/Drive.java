package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

public class Drive {
    PIDF ctrl_fl, ctrl_fr, ctrl_bl, ctrl_br;
    PIDF ctrl_turn;

    LowPassFilter v_fl, v_fr, v_bl, v_br;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private boolean heading_lock = false;
    private double heading_target = 0;
    private BNO055IMU imu;

    public double odom_normal;
    public double odom_tangential;

    private double prev_fl, prev_fr, prev_bl, prev_br;
    private double prev_t;

    // parameters
    private double ticks_per_rev = 537.7;
    private double motor_diameter = 3.75; // inches

    private double to_inches(int encoder){
        return encoder / ticks_per_rev * motor_diameter * Math.PI;
    }

    public Drive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
                 BNO055IMU imu){
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;

        this.imu = imu;

        ctrl_bl = new PIDF();
        ctrl_fl = new PIDF();
        ctrl_br = new PIDF();
        ctrl_fr = new PIDF();

        double kf = 1.0 / (6*12);
        double kp = 0.01; // random guess again
//        double kp = 0.0; // temp
        double ki = 0.0; // 0.001;
        ctrl_bl.kf = kf;
        ctrl_bl.kp = kp;
        ctrl_bl.ki = ki;
        ctrl_fl.kf = kf;
        ctrl_fl.kp = kp;
        ctrl_fl.ki = ki;
        ctrl_fr.kf = kf;
        ctrl_fr.kp = kp;
        ctrl_fr.ki = ki;
        ctrl_br.kf = kf;
        ctrl_br.kp = kp;
        ctrl_br.ki = ki;

        ctrl_bl.setpoint = 0;
        ctrl_br.setpoint = 0;
        ctrl_fl.setpoint = 0;
        ctrl_fr.setpoint = 0;

        ctrl_turn = new PIDF();
        ctrl_turn.kf = 0;
        ctrl_turn.kp = 48.0/Math.PI; // wild guess
//        ctrl_turn.kd = 10;
        ctrl_turn.setpoint = 0;
        ctrl_turn.ki = 1; // also guess

        double lpf_alpha = 0.7;
        v_fl = new LowPassFilter(lpf_alpha);
        v_fr = new LowPassFilter(lpf_alpha);
        v_bl = new LowPassFilter(lpf_alpha);
        v_br = new LowPassFilter(lpf_alpha);

        prev_t = System.currentTimeMillis() / 1000.0;
    }

    public void setHeadingTarget(double t){
        ctrl_turn.setpoint = t;
        this.heading_target = t;
    }

    public void go(double vt_f, double vn_f, double vh){
        double time = System.currentTimeMillis() / 1000.0;

        double heading = -this.imu.getAngularOrientation().firstAngle * Math.PI / 180;
        if(heading_lock) {
            vh = ctrl_turn.next(heading, time);
        }

        // tangential/normal speed in the robot frame
        // after field-centric frame rotation
        double vt_r = vt_f * Math.cos(-heading) - vn_f * Math.sin(-heading);
        double vn_r = vt_f * Math.sin(-heading) + vn_f * Math.cos(-heading);

        double v_fl = vt_r + vn_r - vh;
        double v_fr = vt_r - vn_r + vh;
        double v_bl = vt_r - vn_r - vh;
        double v_br = vt_r + vn_r + vh;

        ctrl_fl.setpoint = v_fl;
        ctrl_fr.setpoint = v_fr;
        ctrl_bl.setpoint = v_bl;
        ctrl_br.setpoint = v_br;

        double dt = time - prev_t;
        prev_t = time;

        double fl = to_inches(frontLeft.getCurrentPosition());
        double fr = to_inches(frontRight.getCurrentPosition());
        double bl = to_inches(backLeft.getCurrentPosition());
        double br = to_inches(backRight.getCurrentPosition());

        double dfl = fl - prev_fl;
        double dfr = fr - prev_fr;
        double dbl = bl - prev_bl;
        double dbr = br - prev_br;

        prev_fl = fl;
        prev_fr = fr;
        prev_bl = bl;
        prev_br = br;

        double ve_fl = this.v_fl.next(dfl/dt);
        double ve_fr = this.v_fr.next(dfr/dt);
        double ve_bl = this.v_bl.next(dbl/dt);
        double ve_br = this.v_br.next(dbr/dt);

        double vh_fl = ctrl_fl.next(ve_fl, time);
        double vh_fr = ctrl_fr.next(ve_fr, time);
        double vh_bl = ctrl_bl.next(ve_bl, time);
        double vh_br = ctrl_br.next(ve_br, time);

        odom_tangential = (dfl + dfr + dbl + dbr) / 4;
        odom_normal = (dfl + dbr - dbl - dbr) / 2; // don't ask

        frontLeft.setPower(vh_fl);
        frontRight.setPower(vh_fr);
        backLeft.setPower(vh_bl);
        backRight.setPower(vh_br);
    }
}

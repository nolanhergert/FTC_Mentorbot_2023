package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

public class Drive {
    PIDF ctrl_fl, ctrl_fr, ctrl_bl, ctrl_br;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // TODO: heading lock
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

    public Drive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;

        // guesses (overestimated)
//        speed_control_ang.kf = 1/(4*360); // degrees
//        speed_control_tan.kf = 1/(8*12); // in
//        speed_control_norm.kf = 1/(5*12); // in

        ctrl_bl = new PIDF();
        ctrl_fl = new PIDF();
        ctrl_br = new PIDF();
        ctrl_fr = new PIDF();

        double kf = 1.0 / (5*12);
        double kp = 0.00; // random guess again
        ctrl_bl.kf = kf;
        ctrl_bl.kp = kp;
        ctrl_fl.kf = kf;
        ctrl_fl.kp = kp;
        ctrl_fr.kf = kf;
        ctrl_fr.kp = kp;
        ctrl_br.kf = kf;
        ctrl_br.kp = kp;

        prev_t = System.currentTimeMillis() / 1000.0;
    }

    public void go(double vt, double vn, double vh){
        double time = System.currentTimeMillis() / 1000.0;

        double v_fl = vt + vn + vh;
        double v_fr = vt - vn - vh;
        double v_bl = vt - vn + vh;
        double v_br = vt + vn - vh;

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

        double vh_fl = ctrl_fl.next(dfl/dt, time);
        double vh_fr = ctrl_fr.next(dfr/dt, time);
        double vh_bl = ctrl_bl.next(dbl/dt, time);
        double vh_br = ctrl_br.next(dbr/dt, time);

        odom_tangential = (dfl + dfr + dbl + dbr) / 4;
        odom_normal = (dfl + dbr - dbl - dbr) / 2; // don't ask

        frontLeft.setPower(vh_fl);
        frontRight.setPower(vh_fr);
        backLeft.setPower(vh_bl);
        backRight.setPower(vh_br);
    }
}

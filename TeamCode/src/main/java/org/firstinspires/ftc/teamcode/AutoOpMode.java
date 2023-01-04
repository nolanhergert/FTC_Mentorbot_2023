package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto drive test", group="dev")
public class AutoOpMode extends OpModeBase {
    private double drive_target_x = 0;
    private double drive_target_y = 0;
    private double drive_target_h = 0;

    private PIDF posn_x;
    private PIDF posn_y;
    private PIDF posn_h;

    private int path_step = 0;

    double[] xs = {36, 72, 108, 76, 32, 0, 0};
    double[] ys = {0, -12, -12, 8, 0, 36.0, 0};

    @Override
    public void init(){
        super.init();

        // target in/sec per inch
        double linear_kp = 1.4;
        double linear_kd = 0.1;
        double linear_ki = 0.005;
        // target in/sec per rad
        double angular_kp = 12;
        posn_x = new PIDF();
        posn_y = new PIDF();
        posn_h = new PIDF();

        posn_x.kp = linear_kp;
        posn_y.kp = linear_kp;
        posn_h.kp = angular_kp;

        posn_x.kd = linear_kd;
        posn_y.kd = linear_kd;

        posn_x.ki = linear_ki;
        posn_y.ki = linear_ki;

        posn_x.set_max(24);
        posn_y.set_max(24);
        posn_h.set_max(24);
    }

    private void drive_to(double x, double y){
        drive_to(x, y, posn_h.setpoint);
    }

    private void drive_to(double x, double y, double h){
        posn_x.setpoint = x;
        posn_y.setpoint = y;
        posn_h.setpoint = h;
    }

    private boolean is_within_range(double dist, double heading){
        double dh = Math.abs(this.orientation() - heading);
        if(dh > Math.PI){
            dh = 2*Math.PI - dh;
        }
        return this.is_within_range(dist) && dh < heading;
    }

    private boolean is_within_range(double dist){
        double dx = odom.x() - posn_x.setpoint;
        double dy = odom.y() - posn_y.setpoint;

        return (dx * dx + dy * dy) < dist*dist;
    }

    @Override
    public void loop(){
        super.loop();

        double x = 0;
        double y = 0;
        if(this.path_step < xs.length) {
            x = xs[path_step];
            y = ys[path_step];
        } else {
            x = xs[xs.length - 1];
            y = ys[ys.length - 1];
        }

        this.drive_to(x, y);

        double time = System.currentTimeMillis() / 1000.0;

        double vx = posn_x.next(odom.x(), time);
        double vy = posn_y.next(odom.y(), time);
        double vh = posn_h.next(orientation(), time);

        this.drive.go(vx, vy, vh);

        if(is_within_range(1)){
            this.path_step++;
        }
    }
}

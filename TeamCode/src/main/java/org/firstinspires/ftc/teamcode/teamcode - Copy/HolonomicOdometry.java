package org.firstinspires.ftc.teamcode;

public class HolonomicOdometry {
    // 0 heading is along x axis
    private double posn_x, posn_y, prev_heading;

    public HolonomicOdometry(){
        posn_x = posn_y = prev_heading = 0;
    }

    public void update(double tangential, double normal, double heading){
        double dh = heading - prev_heading;
        if(dh == 0) {
            double ch = Math.cos(heading);
            double sh = Math.sin(heading);
            posn_x += ch * tangential - sh * normal;
            posn_y += sh * tangential + ch * normal;
        }
        else{
            double dsin = Math.sin(heading) - Math.sin(prev_heading);
            double dcos = Math.cos(heading) - Math.cos(prev_heading);
            posn_x += (tangential * dsin + normal * dcos) / dh;
            posn_y += (normal * dsin - tangential * dcos) / dh;
        }
    }

    public double x(){
        return posn_x;
    }

    public double y(){
        return posn_y;
    }
}

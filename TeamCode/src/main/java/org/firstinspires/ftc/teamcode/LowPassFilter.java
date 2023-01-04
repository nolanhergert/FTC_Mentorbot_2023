package org.firstinspires.ftc.teamcode;

public class LowPassFilter {
    private double val;
    private double alpha;

    public LowPassFilter(double alpha){
        this.alpha = alpha;
        this.val = 0;
    }

    public double next(double val){
        this.val = (this.val * alpha) + (val * (1-alpha));
        return this.val;
    }

    public double get(){
        return this.val;
    }
}

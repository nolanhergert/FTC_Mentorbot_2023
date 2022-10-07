/*
Sourced from AEMBOT (classic) 2022's repo:
https://github.com/AEMBOT/FTC_2022_AEM_Classic/blob/aeea3260d0c67252abf30b473015427f26f6c361/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TankDrive.java

@author Will or James?
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 *
 * controls a basic tank drive chassis
 *
 */

public class TankDrive {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor[] driveMotors;

    private double speed = 1;

    private double leftPower = 0;
    private double rightPower = 0;

    private double invert = 1;

    private boolean invertEnabled = false;

    public void InvertToggle(){
        invertEnabled = !invertEnabled;
    }

    public void Periodic(){

        if(invertEnabled){
            invert = -1;
        }
        else{
            invert = 1;
        }

    }

    public TankDrive(HardwareMap hardwareMap){

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveMotors = new DcMotor[4];
        driveMotors[0] = frontLeft;
        driveMotors[1] = frontRight;
        driveMotors[2] = backLeft;
        driveMotors[3] = backRight;

    }

    public void BasicTank(double FWDl, double FWDr, Telemetry telemetry){

        /*
        telemetry.addData(String.valueOf(frontLeft.getPower()), "frontLeft");
        telemetry.addData(String.valueOf(frontRight.getPower()), "frontRight");
        telemetry.addData(String.valueOf(backLeft.getPower()), "backLeft");
        telemetry.addData(String.valueOf(backRight.getPower()), "backRight");
        */

        if(invertEnabled){
            leftPower = FWDl;
            rightPower = FWDr;
        }
        else{
            leftPower = FWDr;
            rightPower = FWDl;
        }

        double[] motorValues = normalizeWheelSpeeds(leftPower, rightPower);

        for (int i = 0; i < motorValues.length; i++) {
            driveMotors[i].setPower(motorValues[i] * speed);
        }

    }

    public double[] normalizeWheelSpeeds(double FWDl, double FWDr) {
        double front_left = FWDl*invert;
        double front_right = FWDr*invert;
        double back_left = FWDl*invert;
        double back_right = FWDr*invert;

        double max = Math.abs(front_left);
        if (Math.abs(front_right) > max) max = Math.abs(front_right);
        if (Math.abs(back_left) > max) max = Math.abs(back_left);
        if (Math.abs(back_right) > max) max = Math.abs(back_right);

        if (max > 1) {
            front_left /= max;
            front_right /= max;
            back_left /= max;
            back_right /= max;
        }

        return new double[]{front_left, front_right, back_left, back_right};
    }

    public void slowModeToggle() {
        if (speed == 1)
            speed = 0.5;
        else
            speed = 1;
    }

}
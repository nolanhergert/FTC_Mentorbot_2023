package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.FieldNavigationSystem;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;


@Autonomous(name = "AutoOpMode")
public class AutoOpMode extends LinearOpMode {

    FieldNavigationSystem m_navigation;
    TankDrive m_drive;

    @Override
    public void runOpMode() throws InterruptedException {
        m_navigation = new FieldNavigationSystem(hardwareMap, "Webcam 1");
        m_drive = new TankDrive(hardwareMap);

        // Try to travel one foot forward (robot-oriented drive)
        //m_drive.travelDistanceInch(x,y,rot);

        // When there, update nav

        // TODO: Handle if we don't see the target...

        m_navigation.update();
        OpenGLMatrix location = m_navigation.getLastLocation();

        boolean hasVisibleTarget = m_navigation.isTargetVisible();

        // update telemetry
        telemetry.addData("Has Visible Target", hasVisibleTarget);

        if (hasVisibleTarget) {
            // express position (translation) of robot in inches.
            VectorF translation = location.getTranslation();
            telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }

        // TODO: Correct for drift using vuforia tag
    }
}
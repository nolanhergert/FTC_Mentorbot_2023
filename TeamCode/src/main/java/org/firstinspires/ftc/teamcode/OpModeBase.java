package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class OpModeBase extends OpMode {
    // Declare OpMode members.
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor lift = null;
    private Servo lift_servo = null;
    private TouchSensor lift_limit = null;

    private long millis;

    protected Drive drive;
    protected Elevator elevator;
    protected HolonomicOdometry odom;
    protected BNO055IMU imu;

    private double orientation;

    private boolean log = true;

    public double orientation(){
        return this.orientation;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        lift = hardwareMap.get(DcMotor.class, "lift_motor");
        lift_limit = hardwareMap.get(TouchSensor.class, "lift_limit");
        lift_servo = hardwareMap.get(Servo.class, "lift_dropper");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.elevator = new Elevator(lift, lift_servo, lift_limit);
        this.elevator.idle();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        drive = new Drive(frontLeft, frontRight, backLeft, backRight, imu);
        odom = new HolonomicOdometry();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        Orientation angles = imu.getAngularOrientation();

        this.orientation = -angles.firstAngle * Math.PI / 180;

        this.odom.update(
                this.drive.odom_tangential,
                this.drive.odom_normal,
                this.orientation
        );
        if(this.log) {
            telemetry.addLine()
                    .addData("orientation",
                            "%.2f ->.2f",
                            this.orientation,
                            drive.ctrl_turn.setpoint
                    );

            telemetry.addLine()
                    .addData(
                            "Lift Encoder",
                            "%.2f -> %.2f @ %.2f",
                            elevator.lift_position(),
                            elevator.ctrl_lift.setpoint,
                            lift.getPower()
                    );

            telemetry.addData(
                    "Lift Limit Switch",
                    this.lift_limit.isPressed() ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Lift Servo Posn",
                    "%.2f",
                    this.lift_servo.getPosition()
            );

            telemetry.addData(
                    "Odometry",
                    "x: %.2f, y: %.2f (in)",
                    this.odom.x(),
                    this.odom.y()
            );

            long ms = System.currentTimeMillis();
            telemetry.addData("Loop period", "(%d) ms", ms - this.millis);
            millis = ms;
        }
    }

}

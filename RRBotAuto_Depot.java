package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Autonomous opmode that descends off the lander, drops team marker, and drives into the crater on depot side
 * @author John Brereton
 * @since 12-9-2018
 */

@Autonomous(name="RRBotAuto_Depot")
public class RRBotAuto_Depot extends LinearOpMode {

    /* Declare OpMode members. */
    RRBotHardware         robot   = new RRBotHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.2;
    static final double     LIFT_SPEED              = 1;

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initGyro();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.rearRightDrive.getCurrentPosition(),
                          robot.rearLeftDrive.getCurrentPosition(),
                          robot.frontRightDrive.getCurrentPosition(),
                          robot.frontLeftDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Step 1:  Go down for 7 seconds
        robot.liftArm.setPower(LIFT_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Stop Lowering
        robot.liftArm.setPower(0);

        // Step 3:  Release the servo
        robot.liftPin.setPosition(1);

        // Wait one second before moving
        sleep(1000);

        // Step 4: Drive forward 58 inches
        encoderDrive(DRIVE_SPEED,  58,  58, 5.0);

        // Step 5: drop marker

        // Step 6: turn Left 42 degrees
        TurnByGyro(TURN_SPEED, "left", 42);

        // Step 7: Drive Backward 110 inches
        encoderDrive(DRIVE_SPEED, -110, -110, 10.0);

        /**
        // Step 5: Turn left 90 degrees
        TurnByGyro(TURN_SPEED, "left", 90);

        // Step 6: Drive forward 45 inches
        encoderDrive(DRIVE_SPEED, 68, 68, 10.0);

        // Step 7: turn Left 45 degrees
        TurnByGyro(TURN_SPEED, "left", 42);

        // Step 8: Drive forward 56 inches
        encoderDrive(DRIVE_SPEED, 98, 98, 10.0);

        // Step 9: Drop marker


        // Step 10: Drive Backward 86 inches
        encoderDrive(DRIVE_SPEED, -110, -110, 10.0);
         **/

        //sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.rearRightDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.rearRightDrive.setTargetPosition(newRightTarget);
            robot.rearLeftDrive.setTargetPosition(newLeftTarget);
            robot.frontRightDrive.setTargetPosition(newRightTarget);
            robot.frontLeftDrive.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rearRightDrive.setPower(Math.abs(speed));
            robot.rearLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.frontLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.rearRightDrive.isBusy() && robot.rearLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.frontLeftDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.rearRightDrive.getCurrentPosition(),
                                            robot.rearLeftDrive.getCurrentPosition(),
                                            robot.frontRightDrive.getCurrentPosition(),
                                            robot.frontLeftDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.rearRightDrive.setPower(0);
            robot.rearLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.frontLeftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void TurnByGyro(double speed, String direction, int angle) {
        //get angle values from the gyro
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float startHeading = angles.firstAngle; //define the starting angle

        //turn on motors to turn the robot in the specified direction
        if(direction.equals("left"))
        {
            robot.rearRightDrive.setPower(speed);
            robot.rearLeftDrive.setPower(-speed);
            robot.frontRightDrive.setPower(speed);
            robot.frontLeftDrive.setPower(-speed);
        }
        else if(direction.equals("right"))
        {
            robot.rearRightDrive.setPower(-speed);
            robot.rearLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(-speed);
            robot.frontLeftDrive.setPower(speed);
        }

        //keep looping until the difference between the current heading and the start heading equals the specified angle
        while(opModeIsActive() && Math.abs(angles.firstAngle - startHeading) < angle)
        {
            //get values from the gyro
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //report the current heading to the driver station
            telemetry.addData("heading", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
            telemetry.update();
        }

        TurnOffMotors();
    }

    /**
     * Initialize the BNO055IMU gyro
     */
    public void initGyro()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //gyro angle formatting methods
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void TurnOffMotors()
    {
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Teleop Opmode class, contains separate methods that update each mechanism of the robot which are called by loop()
 * @author John Brereton
 * @since 12-9-2018
 */

@TeleOp(name="RRbotTeleop", group="Iterative Opmode")
// @Disabled
public class RRBotTeleop extends OpMode
{
    // Declare OpMode members.
    RRBotHardware robot = new RRBotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    boolean liftPinInit;
    boolean markerInit;
    boolean slowDrive;
    boolean prevSlowDrive;
    boolean prevLiftArm;
    boolean prevMarker;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the servoInit to true in the beginning of the match
        liftPinInit = true;

        // Set slowDrive to false in the beginning of the match
        slowDrive = false;

        telemetry.addData("Status", "Initialized");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        driveUpdate();

        liftArmUpdate();

        markerUpdate();

        telemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * Updates the drive system with manual and automatic movements
     */
    public void driveUpdate() {

        // Uses left stick to move forward and to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        if (gamepad1.b && !prevSlowDrive) {
            if (slowDrive) {
                slowDrive = false;
            } else {
                slowDrive = true;
            }
            prevSlowDrive = true;
        }
        if (!gamepad1.b) {
            prevSlowDrive = false;
        }

        if (slowDrive) {
            leftPower = Range.clip(drive + turn, -1.0, 1.0)/2;
            rightPower = Range.clip(drive - turn, -1.0, 1.0)/2;
        } else {
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
        }

        // Send calculated power to wheels
        robot.rearRightDrive.setPower(rightPower);
        robot.rearLeftDrive.setPower(leftPower);
        robot.frontRightDrive.setPower(rightPower);
        robot.frontLeftDrive.setPower(leftPower);
    }

    /**
     * Updates the lifting arm. Reads button inputs to move the arm up and down and move the pin in and out
     */
    public void liftArmUpdate() {

        robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Use right stick on operator controller to control lift arm
        robot.liftArm.setPower(gamepad2.left_stick_y);

        telemetry.addData("encoder", "encoder value: " + robot.liftArm.getCurrentPosition());

        // Use "a" button on operator controller to move pin in and out
        if (gamepad2.b && !prevLiftArm) {
            if (liftPinInit) {
                robot.liftPin.setPosition(1);
                liftPinInit = false;
            } else if (!liftPinInit) {
                robot.liftPin.setPosition(0);
                liftPinInit = true;
            }
            prevLiftArm = true;
        }
        if (!gamepad2.b){
            prevLiftArm = false;
        }
    }

    public void markerUpdate() {
        if (gamepad2.y && !prevMarker) {
            if (markerInit) {
                robot.markerDropper.setPosition(1);
                markerInit = false;
            } else if (!markerInit) {
                robot.markerDropper.setPosition(0);
                markerInit = true;
            }
            prevMarker = true;
        }
        if (!gamepad2.y) {
            prevMarker = false;
        }
    }

    /**
     * Shows debug values to be on the driver station
     */
    public void telemetry() {
        // Show the robots name
        telemetry.addData("Name", "Name: Tim" );

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // Show the wheel power
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        // Show the state of the pin
        telemetry.addData("Pin", "Initiated: " + liftPinInit);

        // Show the state of  the marker dropper
        telemetry.addData("Marker", "Initiated: " + markerInit);
    }
}

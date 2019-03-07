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

    // front arm variables
    boolean frontArmExtended;
    boolean prevFrontArmExtended;
    boolean intakeFolded;
    boolean prevIntakeFolded;

    // rear arm variables
    boolean rearArmExtended;
    boolean prevRearArmExtended;

    // lift arm variables
    boolean liftPinInit;
    boolean prevLiftPinInit;

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

        // Set the servoInit to true in the beginning of the match
        liftPinInit = true;

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

        frontArmUpdate();

        liftArmUpdate();

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


        leftPower = Range.clip(drive + turn, -1.0, 1.0)/2;
        rightPower = Range.clip(drive - turn, -1.0, 1.0)/2;

        leftPower = leftPower + gamepad1.right_trigger;
        rightPower = rightPower + gamepad1.right_trigger;

        // Send calculated power to wheels
        robot.rearRightDrive.setPower(rightPower);
        robot.rearLeftDrive.setPower(leftPower);
        robot.frontRightDrive.setPower(rightPower);
        robot.frontLeftDrive.setPower(leftPower);
    }

    /**
     * Updates the front arm. Reads button inputs to move the arm in and out and control the intake
     */
    public void frontArmUpdate() {
        // Use right stick left and right on operator controller to control front arm
        //robot.frontArm.setPower(gamepad2.right_stick_x);
        if (gamepad2.dpad_right && frontArmExtended) {
            robot.frontArm.setTargetPosition(0);
            robot.frontArm.setPower(1);
            frontArmExtended = false;
        } else if (gamepad2.dpad_left && !frontArmExtended) {
            robot.frontArm.setTargetPosition(-1);
            robot.frontArm.setPower(1);
            frontArmExtended = true;
        }
        /*
        // Use left and right d-pad buttons on operator controller to move arm in and out based on preset positions
        if (!prevFrontArmExtended) {
            if (gamepad2.dpad_right && frontArmExtended) {
                robot.frontArm.setTargetPosition(0);
                robot.frontArm.setPower(1);
                frontArmExtended = false;
            } else if (gamepad2.dpad_left && !frontArmExtended) {
                robot.frontArm.setTargetPosition(1000);
                robot.frontArm.setPower(1);
                frontArmExtended = true;
            }
            prevFrontArmExtended = true;
        }
        if (!gamepad2.dpad_right || !gamepad2.dpad_left){
            prevFrontArmExtended = false;
        }

        // Use "a" button on operator controller to move intake up and down
        if (gamepad2.a && !prevIntakeFolded) {
            if (intakeFolded) {
                robot.intakeServo.setPosition(1);
                intakeFolded = false;
            } else if (!intakeFolded) {
                robot.intakeServo.setPosition(0);
                intakeFolded = true;
            }
            prevIntakeFolded = true;
        }
        if (!gamepad2.a){
            prevIntakeFolded = false;
        }
        */
    }

    /**
     * Updates the rear arm. Reads button inputs to move the arm up and down and control the output
     */
    public void rearArmUpdate() {
        // Use right stick up and down on operator controller to control rear arm
        robot.frontArm.setPower(gamepad2.right_stick_y);
        /*
        // Use up and down d-pad buttons on operator controller to move arm in and out based on preset positions
        if (!prevRearArmExtended) {
            if (gamepad2.dpad_down && rearArmExtended) {
                robot.rearArm.setTargetPosition(0);
                robot.rearArm.setPower(1);
                rearArmExtended = false;
            } else if (gamepad2.dpad_up && !rearArmExtended) {
                robot.rearArm.setTargetPosition(10);
                robot.rearArm.setPower(1);
                frontArmExtended = true;
            }
            prevFrontArmExtended = true;
        }
        if (!gamepad2.dpad_down || !gamepad2.dpad_up) {
            prevRearArmExtended = false;
        }
        */

    }

    /**
     * Updates the lifting arm. Reads button inputs to move the arm up and down and move the pin in and out
     */
    public void liftArmUpdate() {
        // Use left stick on operator controller to control lift arm
        robot.liftArm.setPower(gamepad2.left_stick_y);

        // Use "b" button on operator controller to move pin in and out
        if (gamepad2.b && !prevLiftPinInit) {
            if (liftPinInit) {
                robot.liftPin.setPosition(1);
                liftPinInit = false;
            } else if (!liftPinInit) {
                robot.liftPin.setPosition(0);
                liftPinInit = true;
            }
            prevLiftPinInit = true;
        }
        if (!gamepad2.b){
            prevLiftPinInit = false;
        }
    }

    /**
     * Shows debug values to be on the driver station
     */
    public void telemetry() {
        // Show the robots name
        telemetry.addData("Name", "Name: Tim" );

        telemetry.addData("Encoder", "Count: " + robot.frontArm.getCurrentPosition());

        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // Show the wheel power
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        // Show the front arm status
        telemetry.addData("Front-Arm", "Extended: " + frontArmExtended);

        // Show the state of the pin
        telemetry.addData("Pin", "Initiated: " + liftPinInit);
    }
}

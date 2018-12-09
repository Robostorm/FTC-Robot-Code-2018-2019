/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Teleop Opmode class, contains separate methods that update each mechanism of the robot which are called by loop()
 * @author John Brereton
 * @since 12-2-2018
 */

@TeleOp(name="RRbotTeleop", group="Iterative Opmode")
public class RRBotTeleop extends OpMode
{
    // Declare OpMode members.
    RRBotHardware robot = new RRBotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    boolean liftPinInit;
    boolean plowInit;
    boolean prevLiftArm;
    boolean prevPlow;

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

        // Set plowInit to false in the beginning of the match
        plowInit = false;

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

        plowUpdate();

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
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

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
        // Use right stick on operator controller to control lift arm
        robot.liftArm.setPower(gamepad2.left_stick_y);

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

    /**
     * Updates the plow.  Reads button inputs to move the plow up and down
     */
    public void plowUpdate() {
        // Use "a" button on operator controller to control plow
        if(gamepad2.a && !prevPlow) {
            if (plowInit) {
                robot.plow.setPosition(0);
                plowInit = false;
            } else if (!plowInit) {
                robot.plow.setPosition(1);
                plowInit = true;
            }
            prevPlow = true;
        }
        if(!gamepad2.a){
            prevPlow = false;
        }
    }

    /**
     * Shows debug values to be on the driver station
     */
    public void telemetry() {
        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // Show the wheel power
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        // Show the state of the Pin
        telemetry.addData("Pin", "Initiated: " + liftPinInit);

        // Show the state of the Plow
        telemetry.addData("Pin", "Initiated: " + plowInit);
    }
}

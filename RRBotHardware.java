package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Hardware class, defines the robot's hardware and initializes it
 * @author John Brereton
 * @since 12-2-2018
 */

public class RRBotHardware
{
    /* Public OpMode members. */
    public DcMotor rearRightDrive = null;
    public DcMotor rearLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor frontLeftDrive = null;
    public DcMotor liftArm = null;
    public Servo liftPin = null;
    public Servo plow = null;
    public Servo markerDropper = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RRBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        rearRightDrive  = hwMap.get(DcMotor.class, "rear-right");
        rearLeftDrive = hwMap.get(DcMotor.class, "rear-left");
        frontRightDrive = hwMap.get(DcMotor.class, "front-right");
        frontLeftDrive = hwMap.get(DcMotor.class, "front-left");
        liftArm = hwMap.get(DcMotor.class, "lift-arm");

        //set motors to drive forwards
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        liftArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        rearRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        liftArm.setPower(0);

        //set drive motors to run using encoder
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set all motors to run in brake mode
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        liftPin = hwMap.get(Servo.class, "lift-pin");
        plow = hwMap.get(Servo.class, "plow");
        markerDropper = hwMap.get(Servo.class, "marker-dropper");
    }
 }


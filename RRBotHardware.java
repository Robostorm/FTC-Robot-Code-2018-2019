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
    public DcMotor frontArm = null;
    public DcMotor rearArm = null;
    public DcMotor liftArm = null;
    public Servo intakeServo = null;
    public Servo liftPin = null;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_DRIVE         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     SPOOL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_ARM         = (COUNTS_PER_MOTOR_REV) / (SPOOL_DIAMETER_INCHES * 3.1415);


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RRBotHardware(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        rearRightDrive  = hwMap.get(DcMotor.class, "rear-right");
        rearLeftDrive = hwMap.get(DcMotor.class, "rear-left");
        frontRightDrive = hwMap.get(DcMotor.class, "front-right");
        frontLeftDrive = hwMap.get(DcMotor.class, "front-left");
        frontArm = hwMap.get(DcMotor.class, "front-arm");
        rearArm = hwMap.get(DcMotor.class, "rear-arm");
        liftArm = hwMap.get(DcMotor.class, "lift-arm");

        //set motors to drive forwards
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontArm.setDirection(DcMotor.Direction.FORWARD);
        rearArm.setDirection(DcMotor.Direction.FORWARD);
        liftArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        rearRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontArm.setPower(0);
        rearArm.setPower(0);
        liftArm.setPower(0);

        //set motors to run using encoder
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all motors to run in brake mode
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        liftPin = hwMap.get(Servo.class, "lift-pin");
    }
 }


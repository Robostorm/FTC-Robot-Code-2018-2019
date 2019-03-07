package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Autonomous opmode that descends off the lander, drops team marker, and drives into the crater on crater side
 * @author John Brereton
 * @since 12-9-2018
 */

@Autonomous(name="RRBotAuto_HomeDepot")
// @Disabled
public class RRBotAuto_HomeDepot extends LinearOpMode {

    /* Declare OpMode members. */
    RRBotHardware robot = new RRBotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.4;
    static final double     LIFT_SPEED              = 1;

    //gyro variables
    private BNO055IMU imu;
    private Orientation angles;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AaNbWrj/////AAABGcyKoOzHikLauD3MYblKvsAUshkJepPvidwtWLGLGCf+fwkT6rjzjnENqh0RoDV0YEsT/tBYChIqyGjfp23Myf6n9QJcBr3lGU6P+Kl0119XaSck9fERcIubHlBZtpw0xrNpeTupwTNH91lQpaZf7pQVWI+OU2Qdj8RI33VMKDJueTMmPpO1wl058bkhzE6teVz3o+k4w/SdX5AEeJHYdOBuSPPjRVmcwHL3cbo8rDQ9jxHezKmfewWg2ZWPevGlAUvv+VtHobPqmY/+owlVZs4VNTJdyiGGh2vHxVIYzJoqqMU45Cvrh++84Tz7BTU2dPOMhOSBS+Z7YAnbSlSJDHgGnD4td2nJagJXP+U5KwZs";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private String goldPos = "";

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initGyro();

        telemetry.addData("Name", "Name: Andrew" );

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.rearRightDrive.getCurrentPosition(),
                          robot.rearLeftDrive.getCurrentPosition(),
                          robot.frontRightDrive.getCurrentPosition(),
                          robot.frontLeftDrive.getCurrentPosition());
        telemetry.update();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        if (!opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            //visionTime.reset();
            while (!opModeIsActive() && !isStopRequested()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {

                        // Now sort by address instead of name (default).
                        Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                            public int compare(Recognition one, Recognition other) {
                                if (one.getTop() > other.getTop()) {
                                    return -1;
                                } else if (one.getTop() < other.getTop()) {
                                    return 1;
                                } else {
                                    return 0;
                                }
                            }
                        });

                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() > 0) {
                            telemetry.addData("Lowest object", updatedRecognitions.get(0).getTop());
                        }

                        if (updatedRecognitions.size() > 1) {
                            telemetry.addData("2nd Lowest object", updatedRecognitions.get(1).getTop());
                        }

                        if (updatedRecognitions.size() > 2) {
                            telemetry.addData("3rd Lowest object", updatedRecognitions.get(2).getTop());
                        }

                        if (updatedRecognitions.size() >= 2) {
                            Recognition left;
                            Recognition center;
                            if (updatedRecognitions.get(0).getLeft() < updatedRecognitions.get(1).getLeft()) {
                                left = updatedRecognitions.get(0);
                                center = updatedRecognitions.get(1);
                            } else {
                                left = updatedRecognitions.get(1);
                                center = updatedRecognitions.get(0);
                            }
                            if (left.getLabel().equals(LABEL_SILVER_MINERAL) && center.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldPos = "right";
                            } else if (left.getLabel().equals(LABEL_SILVER_MINERAL) && center.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldPos = "center";
                            } else if (left.getLabel().equals(LABEL_GOLD_MINERAL) && center.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldPos = "left";
                            } else {
                                telemetry.addData("Gold Mineral Position", "Andrew is confused");
                                goldPos = "center";
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        // Step 1:  Go down for 6 seconds
        robot.liftArm.setPower(-LIFT_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Stop the lift motor
        robot.liftArm.setPower(0);

        // Step 2: Run Vuforia code
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.


        // Step 3: Release the servo
        robot.liftPin.setPosition(1);

        // Wait one second before moving
        sleep(2000);

        // Run based on gold mineral position
        Drive.encoder(DRIVE_SPEED,  12, 5.0, this, robot);

        robot.liftPin.setPosition(0);

        if(goldPos.equals("left")) {
            TurnByGyro(TURN_SPEED, "left", 40);
            Drive.encoder(DRIVE_SPEED, 20, 5, this, robot);
            TurnByGyro(TURN_SPEED, "right", 18);


            Drive.encoder(DRIVE_SPEED, 14, 10.0, this, robot);

            Drive.encoder(DRIVE_SPEED,-2,10.0, this, robot);

            TurnByGyro(TURN_SPEED, "right", 65);

            Drive.encoder(DRIVE_SPEED, 24, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 35);

            // Step 9: Drop marker


            TurnByGyro(TURN_SPEED, "right", 35);

            Drive.encoder(DRIVE_SPEED, -77, 10.0, this, robot); //-78
        }
        else if(goldPos.equals("right")) {
            TurnByGyro(TURN_SPEED, "right", 40);
            Drive.encoder(DRIVE_SPEED, 20, 5, this, robot);
            TurnByGyro(TURN_SPEED, "left", 18);


            Drive.encoder(DRIVE_SPEED, 13, 10.0, this, robot);

            Drive.encoder(DRIVE_SPEED,-3, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 65);

            Drive.encoder(DRIVE_SPEED, 26, 10.0, this, robot);

            // Step 9: Drop marker


            Drive.encoder(DRIVE_SPEED, -4, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 56);

            Drive.encoder(DRIVE_SPEED, 12, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 13);

            Drive.encoder(DRIVE_SPEED, 6, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 10);

            Drive.encoder(DRIVE_SPEED, 50, 10.0, this, robot);


        }
        else { //center
            Drive.encoder(DRIVE_SPEED, 38, 5, this, robot);

            // Step 9: Drop marker


            Drive.encoder(DRIVE_SPEED, -3, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 88);

            Drive.encoder(DRIVE_SPEED, 8, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 10);

            Drive.encoder(DRIVE_SPEED, 4, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 11);

            Drive.encoder(DRIVE_SPEED, 2, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 5);

            Drive.encoder(DRIVE_SPEED, 2, 10.0, this, robot);

            TurnByGyro(TURN_SPEED, "left", 4);

            Drive.encoder(DRIVE_SPEED, 47, 10.0, this, robot);

            //TurnByGyro(TURN_SPEED, "left", 5);

            // TurnByGyro(TURN_SPEED, "left", 2);

            // Step 10: Drive Backward 60 inches
            //encoderDrive(DRIVE_SPEED, -66, -66, 10.0);
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
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}

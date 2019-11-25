package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import android.app.Activity;
import android.content.res.AssetFileDescriptor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

import java.lang.Math;


//motor.setZeroPowerBehavior (if you want it float or brake)
//opModeIsActive() //if you running within 30 seconds
//robot is a hardware object so you can use hardware methods
//idle() -

//@TeleOp(name = "Testing Autonomous", group = "Linear Opmode")

public class Autonomous extends LinearOpMode {

    Hardware robot;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final double COUNTS_PER_INCH_HD_MECANUM = 1120 / Math.PI / 4;
    private static final int COUNTS_PER_REV_CORE = 288;
    private static final double TURN_DISTANCE_PER_DEGREE = Math.sqrt(1560.49) * Math.PI / 360 / 2;

    //imute
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.30, correction;
    double baseAngle;

    protected CameraName cameraName = null;

    private ElapsedTime runtime = new ElapsedTime();
    public Interpreter tflite;

    //Vuforia stuff
    private static final String VUFORIA_KEY = "ATKKdVf/////AAABmb9SxtpqfUvxqCFmSowoT10see3Vz9mze+DVTbtqieMNjFxZverOpqc4OYMhAkuv9rnJMQZyuaweuLOXioXqVuYJ2P2yRohAKL//zPiF1drlPCUbzdhh3pFV8X4rnBILwoF9C3gWvpQfB//IJdZXNBkWYOZAp+UXGBW2WGdt2rQFHw4Y23GrGb2XCmPEHynO8tiNb6IzR6vOh/KOZ8GyTVES7+GyMVhFWNqgL969+ra6Ev5mgfDqaIt4DAqOoiMomDF9mm+Ixx7m6R2pwJC69XVvqAE6+fuotOs8fvA2XRtU+NNaD2ALR247keSC3qK0RnH8JGjYbSmiOHuRqHW9p9J/JrG1OPOxKnKuGEhhcgA7";
    protected VuforiaLocalizer vuforia;
    //TensorFlow Object detector
    protected TFObjectDetector tfod;
    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        robot = new Hardware(hardwareMap);
        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        leftBack = robot.leftBack;
        rightBack = robot.rightBack;

        //runtime = new ElapsedTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //TODO Have to properly name the camera
        cameraName = hardwareMap.get(CameraName.class, "Webcam");

        //Set directions for motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        initTfod();
        initVuforia();

        while (!imu.isSystemCalibrated()) idle();

        // Wait for the game to start (driver presses PLAY)
        tfod.activate();
        waitForStart();
        if (!opModeIsActive()) return;
        //TODO Write function to run TFOD object on image and determine important position info.

        tfod.deactivate();
        runtime.reset();

        //TODO Write code to move motors and perform the task based on result of TFOD classification.


        //OLD: Need an Activity object here. Which file do we get this from?
        //tflite = new Interpreter(loadModelFile());

        //To run the model, we have to do tflite.run(imgData, labelProbArray);
        //I am thinking that, from the video we can take 10 fps and and analyze each image
        // in the frame to make the prediction.
    }

    public void strafe(double vertical, double horizontal, double power) {
        double rotation = 0;
        double magnitude = power;
        double direction = Math.atan2(-vertical, horizontal);

        double lf = magnitude * Math.sin(direction + Math.PI / 4) - rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) + rotation;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        //INFO Load the model and the classes.
        //TODO need to load the classes. I don't know the class names.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, "");
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    protected void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the com.vuforia.Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = cameraName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    //README commented out below function for loading the Model's file.
    /*private MappedByteBuffer loadModelFile(Activity activity) throws IOException {
        //in openFd(), input the a String that points to the file in assets folder where .tflite file is
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd();
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());

        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }*/

    void waitAbsolute(double seconds) {
        /*
         * TODO Keep the robot waiting until a certain time is reached.
         * */
        while (opModeIsActive() && runtime.seconds() <= seconds) {
            if(!opModeIsActive()){
                StopDriveMotors();
                break;
            }
            telemetry.addData("Time Remaining ", Math.ceil(seconds - runtime.seconds()));
            telemetry.update();
            telemetry.addData("Current Time ", runtime.seconds());
            telemetry.update();
            idle();
        }
        if(!opModeIsActive())
            stop();
    } //wait to move on to next step

    void waitFor(double seconds) {
        //adds the seconds to the current time
        waitAbsolute(getNewTime(seconds));
    }

    public void timeDrive(double angle, double time, double power) {
        angle = Math.toRadians(angle) - Math.PI / 4;

        //if opModeIsActive(), move the motors
        //for a certain time
        //stop motors

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        waitFor(time);

        StopDriveMotors();
    }

    double getNewTime(double addedSeconds) {
        return runtime.seconds() + addedSeconds;
    }

    void timeTurn(double speed, double time){

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);

        waitFor(time);

        StopDriveMotors();

    }
    void AbsoluteTurn(double speed, double targetAngle){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        if (currentAngle < targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetAngle) {

                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
            }


        }else if (currentAngle > targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetAngle) {

                leftFront.setPower(speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(-speed);
            }
        }

        StopDriveMotors();

    }

    private double correctAngle(double angle) { // [-180, 180] → [0, 360]
        return angle + 180;
    }
    private double getCorrectedAngle() {
        return correctAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }
    void AbsoluteTurnCorrected(double speed, double targetAngle) {
        double currentAngle = getCorrectedAngle();
        targetAngle = correctAngle(targetAngle);
        int rollovers = Math.abs((int) (targetAngle / 360));
        double targetAfterRollover = targetAngle % 360;
        if (targetAngle < 0) {
            rollovers++;
            targetAfterRollover += 360;
        }
        if (targetAngle > currentAngle) {
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftBack.setPower(-speed);
            rightBack.setPower(speed);

            for (int i = 0; i < rollovers; i++) {
                while (opModeIsActive() && getCorrectedAngle() <= 180) {
                    // do nothing
                }
                while (opModeIsActive() && getCorrectedAngle() >= 180) {
                    // do nothing
                }
                // this constitutes 1 rollover
            }
            while (opModeIsActive() && getCorrectedAngle() < targetAfterRollover) {
                // do nothing
            }
        }
        else if (targetAngle < currentAngle) {
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftBack.setPower(speed);
            rightBack.setPower(-speed);

            for (int i = 0; i < rollovers; i++) {
                while (opModeIsActive() && getCorrectedAngle() >= 180) {
                    // do nothing
                }
                while (opModeIsActive() && getCorrectedAngle() <= 180) {
                    // do nothing
                }
                // this constitutes 1 rollover
            }
            while (opModeIsActive() && getCorrectedAngle() > targetAfterRollover) {
                // do nothing
            }
        }
        StopDriveMotors();
    }
    //encoder constants
    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // change for mecanum
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    void EncoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int) ((leftInches * COUNTS_PER_INCH));
            newRightTarget = rightFront.getCurrentPosition() + (int) ((rightInches * COUNTS_PER_INCH));
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newLeftTarget);
            leftBack.setTargetPosition(newRightTarget);
            rightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            StopDriveMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    void StrafeEncoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int) ((leftInches * COUNTS_PER_INCH));
            newRightTarget = rightBack.getCurrentPosition() + (int) ((rightInches * COUNTS_PER_INCH));
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(-newLeftTarget);
            leftBack.setTargetPosition(-newRightTarget);
            rightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            StopDriveMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private void StopDriveMotors(){

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

}
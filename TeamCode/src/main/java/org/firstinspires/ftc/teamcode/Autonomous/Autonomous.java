package org.firstinspires.ftc.teamcode.Autonomous;

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

@TeleOp(name = "Testing Autonomous", group = "Linear Opmode")

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

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    protected CameraName cameraName = null;

    private ElapsedTime runtime = new ElapsedTime();
    public Interpreter tflite;

    //Vuforia stuff
    private static final String VUFORIA_KEY = "ATKKdVf/////AAABmb9SxtpqfUvxqCFmSowoT10see3Vz9mze+DVTbtqieMNjFxZverOpqc4OYMhAkuv9rnJMQZyuaweuLOXioXqVuYJ2P2yRohAKL//zPiF1drlPCUbzdhh3pFV8X4rnBILwoF9C3gWvpQfB//IJdZXNBkWYOZAp+UXGBW2WGdt2rQFHw4Y23GrGb2XCmPEHynO8tiNb6IzR6vOh/KOZ8GyTVES7+GyMVhFWNqgL969+ra6Ev5mgfDqaIt4DAqOoiMomDF9mm+Ixx7m6R2pwJC69XVvqAE6+fuotOs8fvA2XRtU+NNaD2ALR247keSC3qK0RnH8JGjYbSmiOHuRqHW9p9J/JrG1OPOxKnKuGEhhcgA7";
    protected VuforiaLocalizer vuforia;
    //TensorFlow Object detector
    protected TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot = new Hardware(hardwareMap);
        //runtime = new ElapsedTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //TODO Have to properly name the motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfdrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfdrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftbdrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightbdrive");

        //TODO Have to properly name the camera
        cameraName = hardwareMap.get(CameraName.class, "Webcam");

        //Set directions for motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
    } //wait to move on to next step

    void waitFor(double seconds) {
        //WaitAbsolute(getNewTime(seconds));
        //adds the seconds to the current time
    }

    public void timeDrive(double angle, double time, double power) {
        angle = Math.toRadians(angle) - Math.PI / 4;

        //if opModeIsActive(), move the motors
        //for a certain time
        //stop motors

    }


}
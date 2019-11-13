package org.firstinspires.ftc.teamcode.Autonomous;

import android.app.Activity;
import android.content.res.AssetFileDescriptor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;


//motor.setZeroPowerBehavior (if you want it float or brake)
//opModeIsActive() //if you running within 30 seconds
//robot is a hardware object so you can use hardware methods
//idle() -

public class Autonomous extends LinearOpMode{

    Hardware robot;

    //imute
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.30, correction;
    double baseAngle;

    public Interpreter tflite;

    @Override
    public void runOpMode(){
        robot = new Hardware(hardwareMap);
        //runtime = new ElapsedTime();

        //Need an Activity object here. Which file do we get this from?
        tflite = new Interpreter(loadModelFile());

        //To run the model, we have to do tflite.run(imgData, labelProbArray);
        //I am thinking that, from the video we can take 10 fps and and analyze each image
        // in the frame to make the prediction.
    }

    private MappedByteBuffer loadModelFile(Activity activity) throws IOException {
        //in openFd(), input the a String that points to the file in assets folder where .tflite file is
        AssetFileDescriptor fileDescriptor = activity.getAssets().openFd();
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());

        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    void waitAbsolute(double seconds)
    {
        /*
        *It keeps the robot waiting until a certain time is reached
        * */
    } //wait to move on to next step

    void waitFor (double seconds){
        //WaitAbsolute(getNewTime(seconds));
        //adds the seconds to the current time
    }

    public void timeDrive(double angle, double time, double power){
        angle = Math.toRadians(angle) - Math.PI/4;

        //if opModeIsActive(), move the motors
        //for a certain time
        //stop motors

    }


}
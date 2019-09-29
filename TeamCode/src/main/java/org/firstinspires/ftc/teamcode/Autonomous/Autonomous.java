package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware;




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

    @Override
    public void runOpMode(){
        robot = new Hardware(hardwareMap);
        //runtime = new ElapsedTime();


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
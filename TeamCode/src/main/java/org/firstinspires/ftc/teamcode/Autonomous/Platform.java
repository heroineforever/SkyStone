package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Platform Auto", group = "Autonomous")

public class Platform extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {


    @Override
    public void runOpMode() {

        super.runOpMode();

        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);
        waitForStart();

        /*//Strafe right 2 seconds
        strafe(1, 0, 1, 2.5);
        waitFor(0.5);
        rotate(true, 0.05, 1);
        //Lower the platform servos
        platformR.setPosition(Servo.MAX_POSITION);
        platformL.setPosition(Servo.MIN_POSITION);
        waitFor(2);
        //Go back and sideways
        strafe(-1, -0.2, 0.7, 2);
        waitFor(0.5);
        //Turn
        rotate(true, 0.5, .3);
        waitFor(0.5);
        //Strafe left for 1 seconds.
        strafe(0, -1, 1, 1);
        waitFor(0.5);
        //Let go of platform
        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);
        waitFor(0.5);
        strafe(-1, 0, 0.2, 0.8);
        waitFor(0.5);
        stop();*/

        strafe(1, 0, 1, 2.5);
        waitFor(0.5);
        platformR.setPosition(Servo.MAX_POSITION);
        platformL.setPosition(Servo.MIN_POSITION);
        waitFor(0.5);
        strafe(-1, 0, 1, 2.4);
        waitFor(0.5);
        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);
        waitFor(0.5);
        strafe(0, 1, 1, 1.5);
        stop();

    }
}

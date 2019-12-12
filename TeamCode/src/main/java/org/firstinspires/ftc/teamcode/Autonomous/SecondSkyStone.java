package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Testing Two", group = "Autonomous")

public class SecondSkyStone extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {

    @Override
    public void runOpMode() {

        super.runOpMode();

        waitForStart();

        ////strafe right 1 second.
        strafe(0, 1, 1, 1);
        //Move forward 1/2 second
        strafe(1, 0, 1, .5);
        //Lower the arm
        arm.setPosition(Servo.MAX_POSITION);
        ////strafe left for .2 seconds.
        strafe(0, -1, 1, .2);
        //Go back for 3 seconds
        strafe(-1, 0, 1, 3);
        //Let go of arm
        arm.setPosition(Servo.MIN_POSITION);
        //Forward 1.5 seconds.
        strafe(1, 0, 1, 1.5);
        ////strafe right .2
        strafe(0, 1, 1, .2);
        //Clamp
        arm.setPosition(Servo.MAX_POSITION);
        ////strafe left .2
        strafe(0, -1, 1, .2);
        //Go back 1.5
        strafe(-1, 0, 1, 1.5);
        //Forward .5
        strafe(1, 0, 1, .5);
        stop();
    }
}
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Platform Auto", group = "Autonomous")

public class Platform extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {

    @Override
    public void runOpMode() {

        waitForStart();

        //Strafe right 1 second.
        strafe(1, 0, 1, 0.9);
        //Lower the platform servos
        rotate(true, 0.2);
        strafe(1, 0.2, 0.2, 0.5);
        platformR.setPosition(Servo.MAX_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);
        //Turn
        //AbsoluteTurn(1, 0);
        //Strafe left for 1 seconds.
        strafe(0, -1, 1, 1);
        //Let go of platform
        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MIN_POSITION);
        strafe(-1, 0, 0.2, 0.8);
        stop();
    }
}

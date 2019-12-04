package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Platform Auto", group = "Autonomous")

public class Platform extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {

    @Override
    public void runOpMode() {

        waitForStart();

        //Strafe right 1 second.
        strafe(0, 1, 1, 1);
        //Lower the platform servos
        platformR.setPosition(Servo.MAX_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);
        //Strafe left for 1 seconds.
        strafe(0, -1, 1, 1);
        //Let go of platform
        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MIN_POSITION);
        stop();
    }
}

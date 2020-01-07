package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Left", group = "Autonomous")

public class Left extends org.firstinspires.ftc.teamcode.Autonomous.Autonomous {


    @Override
    public void runOpMode() {

        super.runOpMode();

        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);
        arm.setPosition(Servo.MAX_POSITION);
        waitForStart();
        strafe(0, -1, 1, 1.5);
        stop();

    }
}
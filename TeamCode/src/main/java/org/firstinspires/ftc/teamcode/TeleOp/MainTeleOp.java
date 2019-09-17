package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.teamcode.Testers.MecanumTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;


@TeleOp(name = "Main TeleOp", group = "Linear Opmode");

//robot.motor.setPower(numerical value);
//encoders are doubles
//.getCurrentPosition(), retrieves encoder values, getPosition() for servo
//gamepad1.left_trigger and such are scaled 0-1
//.getMode()


public class MainTeleOp extends OpMode {


    //make a robot
    Hardware robot;
    //private ElaspedTime runTime; for if you need to drive by time


    @Override
    //initialize
    public void init() {
        //map hardware
        robot = new Hardware(hardwareMap);

        //runtime = new ElapsedTime();
    }

    //Code that runs repeatedly
    @Override
    public void loop() {
        DriveControl();
        HorizontalLiftControl();
        VerticalLiftControl();
    }

    //driving hub
    public void DriveControl() {
        //got the direction from the controller then told the motors what to do
        double movement = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        //joystick ranges -1 to 1
        robot.rightDrive.setPower(movement * (turn * -1));
        robot.leftDrive.setPower(movement * turn);

    }

    //Function for handling horizontal lift
    public void HorizontalLiftControl() {
        if (gamepad1.dpad_left)
            robot.horizontalLift.setPower(-.7);
        else if (gamepad1.dpad_right)
            robot.horizontalLift.setPower(.7);
        else
            robot.horizontalLift.setPower(0);
    }

    //Function for handling vertical lift
    public void VerticalLiftControl(){
        if (gamepad1.a)
            robot.verticalLift.setPower(-.7);
        else if (gamepad1.b)
            robot.verticalLift.setPower(.7);
        else
            robot.verticalLift.setPower(0);
    }

}
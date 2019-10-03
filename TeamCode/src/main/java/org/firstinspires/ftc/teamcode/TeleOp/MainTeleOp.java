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


@TeleOp(name = "Main TeleOp", group = "Linear Opmode")
//@Disabled
//robot.motor.setPower(numerical value);
//encoders are doubles
//.getCurrentPosition(), retrieves encoder values, getPosition() for servo
//gamepad1.left_trigger and such are scaled 0-1
//.getMode()


public class MainTeleOp extends OpMode {


    //make a robot
    Hardware robot;
    //private ElaspedTime runTime; for if you need to drive by time
    double movement;
    double turn;


    @Override
    //initialize
    public void init() {
        //map hardware
        robot = new Hardware(hardwareMap);

        //ElapsedTime runtime = new ElapsedTime();
    }

    //Code that runs repeatedly
    @Override
    public void loop() {
        DriveControl();
        //HorizontalLiftControl();
        //VerticalLiftControl();

        telemetry.addData("Left Drive Position", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Right Drive Position", robot.rightDrive.getCurrentPosition());
        //telemetry.addData("Left Drive Power", robot.leftDrive.getPower());
        //telemetry.addData("Right Drive Power", robot.rightDrive.getPower());

        //telemetry.addData("bucket position", robot.scoopServo.getPosition());

        telemetry.update();
    }

    //Driving Control function
    public void DriveControl() {
        //got the direction from the controller then told the motors what to do
        movement = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        //joystick ranges -1 to 1
        if (turn == 0) {
            robot.rightDrive.setPower(movement);
            robot.leftDrive.setPower(movement);
        } else if (movement == 0) {
            robot.leftDrive.setPower(turn);
            robot.rightDrive.setPower(turn);
        } else if (turn < 0) {
            robot.rightDrive.setPower(turn * movement);
            robot.leftDrive.setPower(turn * -1 * movement);
        } else {
            robot.rightDrive.setPower(turn * -1 * movement);
            robot.leftDrive.setPower(turn * movement);
        }


    }


    //Function for handling horizontal lift
    /*public void HorizontalLiftControl() {
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
    }*/

}
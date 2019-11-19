package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.teamcode.Testers.MecanumTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

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
        //telemetry.addData("Left Drive Position", robot.leftBack.getCurrentPosition());
        //telemetry.addData("Right Drive Position", robot.rightBack.getCurrentPosition());

        telemetry.addData("Left Back Power", robot.leftBack.getPower());
        telemetry.addData("Left Front Power", robot.leftFront.getPower());
        telemetry.addData("Right Back Power", robot.rightBack.getPower());
        telemetry.addData("Right Front Power", robot.rightFront.getPower());

        telemetry.addData("Left JoyStick Y", gamepad1.left_stick_y);
        telemetry.addData("Right JoyStick X", gamepad1.right_stick_x);

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
            robot.rightBack.setPower(movement);
            robot.leftBack.setPower(movement);
        } else if (movement == 0) {
            if (turn > 0) {
                robot.leftBack.setPower(turn * -1);
                robot.rightBack.setPower(0);
            } else {
                robot.leftBack.setPower(0);
                robot.rightBack.setPower(turn);
            }
            //  robot.leftBack.setPower(turn * -1);
            // robot.rightBack.setPower(turn);
        } else if (turn < 0) {
            if (movement > 0) {
                robot.rightBack.setPower(Math.abs(turn) / 4);
                robot.leftBack.setPower(turn * movement);
            } else {
                robot.rightBack.setPower(turn * -1 * movement);
                robot.leftBack.setPower(Math.abs(turn) / 4);
            }
        } else {
            if (movement > 0) {
                robot.rightBack.setPower(turn * -1 * movement);
                robot.leftBack.setPower(Math.abs(turn) / 4);
            } else {
                robot.rightBack.setPower(Math.abs(turn) / 4);
                robot.leftBack.setPower(turn * movement);
            }
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
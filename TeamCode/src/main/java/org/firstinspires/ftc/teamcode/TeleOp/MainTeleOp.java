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
    double rotation;
    double strafe;

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
        ArmControl();
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

        telemetry.addData("Arm Position", robot.arm.getPosition());

        telemetry.update();
    }

    //Driving Control function
    public void DriveControl() {
        //got the direction from the controller then told the motors what to do
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        //INFO joystick ranges -1 to 1


        double x1 = gamepad1.left_stick_x;
        double y1 = -gamepad1.left_stick_y;
        double x2 = gamepad1.right_stick_x;

        //trig implementation
        double power = Math.hypot(x1, y1);
        double angle = Math.atan2(y1, x1) - Math.PI/4;

        robot.leftBack.setPower(power * Math.cos(angle) + x2));
        robot.leftFront.setPower(power * Math.sin(angle) - x2));
        robot.rightFront.setPower(power * Math.sin(angle) + x2));
        robot.rightBack.setPower(power * Math.cos(angle) - x2));



        /*if (rotation == 0) {
            robot.rightBack.setPower(movement);
            robot.leftBack.setPower(movement);
        } else if (movement == 0) {
            if (rotation > 0) {
                robot.leftBack.setPower(rotation * -1);
                robot.rightBack.setPower(0);
            } else {
                robot.leftBack.setPower(0);
                robot.rightBack.setPower(rotation);
            }
            //  robot.leftBack.setPower(rotation * -1);
            // robot.rightBack.setPower(rotation);
        } else if (rotation < 0) {
            if (movement > 0) {
                robot.rightBack.setPower(Math.abs(rotation) / 4);
                robot.leftBack.setPower(rotation * movement);
            } else {
                robot.rightBack.setPower(rotation * -1 * movement);
                robot.leftBack.setPower(Math.abs(rotation) / 4);
            }
        } else {
            if (movement > 0) {
                robot.rightBack.setPower(rotation * -1 * movement);
                robot.leftBack.setPower(Math.abs(rotation) / 4);
            } else {
                robot.rightBack.setPower(Math.abs(rotation) / 4);
                robot.leftBack.setPower(rotation * movement);
            }
        }*/
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

    public void ArmControl()
    {
        if (gamepad1.x){
            robot.arm.setPosition(Servo.MAX_POSITION);

        }

        if (gamepad1.y){

            robot.arm.setPosition(Servo.MIN_POSITION);
/*
        }
    }


}
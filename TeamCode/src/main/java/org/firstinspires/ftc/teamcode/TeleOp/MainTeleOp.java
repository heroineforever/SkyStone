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


@TeleOp(name = "Testing TeleOp", group = "Linear Opmode")
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
    DcMotor leftFront, rightFront, leftBack, rightBack;
    Servo arm;


    @Override
    //initialize
    public void init() {
        //map hardware
        robot = new Hardware(hardwareMap);
        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        rightBack = robot.rightBack;
        leftBack = robot.leftBack;
        arm = robot.arm;

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

        //telemetry.addData("Arm Position", robot.arm.getPosition());

        telemetry.update();
    }

    //Driving Control function
    public void DriveControl() {
        //got the direction from the controller then told the motors what to do
        /*movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;*/
        //INFO joystick ranges -1 to 1

        approachOne();
        //IJApproach();

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

    //INFO This works too, but rotation is slow.
    private void IJApproach() {
        //INFO IJ Approach works, but everything is rotated 90 degrees.
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double x = strafe;
        double y = movement;
        double hypot = Math.hypot(x, y);

        double max = Math.max(Math.abs(x), Math.abs(y));
        double ratio = hypot / max;
        //README Scale it up to make one side 1 or -1.
        x *= ratio;
        y *= ratio;

        //README a and b are the values for the motors eventually.
        //  a is for the right top. b is for top left.
        double b = 0, a = 0;
        b = (x + y) / 2;
        a = x - b;

        double secondRatio = hypot / (Math.max(Math.abs(a), Math.abs(b)));
        a *= secondRatio;
        b *= secondRatio;
        //README right now a is opposite of what it should be.
        a *= -1;
        if (rotation > 0) {
            // If rotation is positive, then have to move top left forward and bottom right backward.
            double topLeft = b, bottomRight = b;
            topLeft = Math.max(b * rotation, b * rotation + 0.15f);
            bottomRight = Math.max(b * rotation, b * rotation + 0.15f);

            leftFront.setPower(Math.sin(topLeft + Math.PI / 4));
            rightBack.setPower(Math.cos(bottomRight * -1 + Math.PI / 4));
            rightFront.setPower(Math.sin(a + Math.PI / 4));
            leftBack.setPower(Math.cos(a + Math.PI / 4));

        } else if (rotation < 0) {
            // If rotation is negative, then have to move top right forward and bottom left backward.
            double topRight = a, bottomLeft = a;
            rotation *= -1;
            topRight = Math.max(a * rotation, a * rotation + 0.15f);
            bottomLeft = Math.max(a * rotation, a * rotation + 0.15f);

            leftFront.setPower(b);
            rightBack.setPower(b);
            rightFront.setPower(topRight);
            leftBack.setPower(bottomLeft * -1);
        } else {
            leftFront.setPower(b);
            rightBack.setPower(b);
            rightFront.setPower(a);
            leftBack.setPower(a);
        }

    }

    private void approachOne() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rotation = -gamepad1.right_stick_x;

        //trig implementation
        //double power = Math.hypot(x1, y1);
        //double angle = Math.atan2(y1, x1) - Math.PI/4;

        //INFO Increasing speed to maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) - rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);

        /*leftFront.setPower((magnitude * Math.sin(direction + Math.PI / 4) + rotation)*-1);
        leftBack.setPower((magnitude * Math.cos(direction + Math.PI / 4) + rotation)*-1);
        rightFront.setPower((magnitude * Math.cos(direction + Math.PI / 4) - rotation)*-1);
        rightBack.setPower((magnitude * Math.sin(direction + Math.PI / 4) - rotation)*-1);*/
    }


    //Function for handling horizontal lift
    /*public void HorizontalLiftControl() {

        if (gamepad2.dpad_left)
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



    public void ArmControl() {
        double armPosition = arm.getPosition();
        if(gamepad1.x && arm.getPosition() == Servo.MIN_POSITION)
            arm.setPosition(Servo.MAX_POSITION);
        else if(gamepad1.x && arm.getPosition() ==  Servo.MAX_POSITION)
            arm.setPosition(Servo.MIN_POSITION);

        /*if (gamepad1.x) {
            robot.arm.setPosition(Servo.MAX_POSITION);
            robot.arm.setPosition(Servo.MAX_POSITION);
        }
        if (gamepad1.y) {
            robot.arm.setPosition(Servo.MIN_POSITION);
            robot.arm.setPosition(Servo.MIN_POSITION);
        }*/
    }
}
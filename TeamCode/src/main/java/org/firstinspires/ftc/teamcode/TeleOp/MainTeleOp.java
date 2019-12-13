package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

/*
  Notes:
    Encoders are doubles
    .getCurrentPosition() retrieves encoder values
    left and right triggers on controllers are scaled 0-1
    .getMode() exists
 */
@TeleOp(name = "Testing TeleOp", group = "Linear Opmode")
//@Disabled

/**
 * MainTeleOp is the class responsible for all of the TeleOp methods. It has a robot, movement. rotation, strafe, four motors, and a servo
 */
public class MainTeleOp extends OpMode {

    //Create a robot---responsible for connecting hardware of Hardware class to methods
    Hardware robot;

    //private ElaspedTime runTime; for if you need to drive by time

    //Directions
    double movement;
    double rotation;
    double strafe;

    //Define the Motors and Servos here to not rely on referencing the robot variable to access the motors and servos
    DcMotor leftFront, rightFront, leftBack, rightBack, greenWheelLeft, greenWheelRight, horizontalLift, verticalLift;
    Servo arm, platformL, platformR, constrictL, gate; //extrusionL, extrusionR;


    @Override
    //initialize
    public void init() {
        //map hardware
        robot = new Hardware(hardwareMap);
        //Assign the motors and servos to the ones on the robot to not require
        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        rightBack = robot.rightBack;
        leftBack = robot.leftBack;
        arm = robot.arm;
        greenWheelLeft = robot.greenWheelLeft;
        greenWheelRight = robot.greenWheelRight;
        horizontalLift = robot.horizontalLift;
        verticalLift = robot.verticalLift;
        platformL = robot.platformL;
        platformR = robot.platformR;
        constrictL = robot.constrictL;
        //constrictR = robot.constrictR;
        gate = robot.gate;
        //extrusionL = robot.extrusionL;
        //extrusionR = robot.extrusionR;

        robot.resetDriveEncoders();

        //Set starting position for arm servo
        arm.setPosition(Servo.MAX_POSITION);
        platformR.setPosition(Servo.MIN_POSITION);
        platformL.setPosition(Servo.MAX_POSITION);


        //ElapsedTime runtime = new ElapsedTime();
    }

    //Code that runs repeatedly
    @Override
    public void loop() {
        DriveControl();
        ArmControl();
        LiftControl();
        PlatformControl();
        //
        // Test();
        //Intake();
        //VerticalLiftControl();
        //telemetry.addData("Left Drive Position", robot.leftBack.getCurrentPosition());
        //telemetry.addData("Right Drive Position", robot.rightBack.getCurrentPosition());

        greenWheelRight.setPower(0.9);
        greenWheelLeft.setPower(-0.9);
        //horizontalLift.setPower(0.2);

        telemetry.addData("Rotation Times", verticalLift.getCurrentPosition());
        telemetry.addData("Left Back Power", robot.leftBack.getPower());
        telemetry.addData("Left Front Power", robot.leftFront.getPower());
        telemetry.addData("Right Back Power", robot.rightBack.getPower());
        telemetry.addData("Right Front Power", robot.rightFront.getPower());

        telemetry.addData("Lift Encoder", robot.verticalLift.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", robot.horizontalLift.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", robot.horizontalLift.getPower());

        telemetry.addData("Left JoyStick Y", gamepad1.left_stick_y);
        telemetry.addData("Right JoyStick X", gamepad1.right_stick_x);

        //telemetry.addData("Arm Position", robot.arm.getPosition());

        telemetry.update();
    }

    //Driving Control function

    public void Test(){
        rightBack.setPower(.5);
        rightFront.setPower(.5);
        leftBack.setPower(.5);
        leftFront.setPower(.5);
    }

    public void DriveControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rotation = gamepad1.right_stick_x;

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

        ////////////////////////////////////////////////////////////////////////////////////////////

        //got the direction from the controller then told the motors what to do
        /*movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;*/
        //INFO joystick ranges -1 to 1

        //approachOne();
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

    /*//INFO This works too, but rotation is slow.
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

        //README a and b are the values for the motors eventuall.
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
        double rotation = gamepad1.right_stick_x;

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
        rightBack.setPower((magnitude * Math.sin(direction + Math.PI / 4) - rotation)*-1);
    }*/


    //Function for handling horizontal lift
    public void LiftControl() {

        double vertical = gamepad2.left_stick_y;
        double horizontal = gamepad2.right_stick_y;

        //README intakes
        robot.horizontalLift.setPower(horizontal);
        robot.verticalLift.setPower((verticalLift.getCurrentPosition() < -300
                || verticalLift.getCurrentPosition() > 2260) ? 0 : vertical);

        //README Suction wheels
        /*robot.greenWheelLeft.setPower(1);
        robot.greenWheelRight.setPower(1);*/
        /*robot.greenWheelLeft.setPower((gamepad2.y) ? -1 : 0);
        robot.greenWheelRight.setPower((gamepad2.y) ? -1 : 0);*/
//aa
        //README Gate open/close using triggers and constriction.
        double rt = gamepad2.right_trigger;
        double lt = gamepad2.left_trigger;

        //INFO Do constriction and close gate.=
        if (rt > 0.2) {
            robot.gate.setPosition(1);
            //Delay 0.8 second
            Handler h = new Handler();
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    robot.constrictL.setPosition(1);
                }
            };
            h.postDelayed(r, 800);
        } //INFO Undo constriction and open gate.
        else if (lt > 0.2) {
        //else if (gamepad2.x) {
            robot.gate.setPosition(0);
            //Delay 0.8 second
            Handler h = new Handler();
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    robot.constrictL.setPosition(0);
                }
            };
            h.postDelayed(r, 800);
        }


        if (gamepad2.dpad_left)
            robot.horizontalLift.setPower(-.7);
        else if (gamepad1.dpad_right)
            robot.horizontalLift.setPower(.7);
        else
            robot.horizontalLift.setPower(0);
    }

    //Function for handling vertical lift
    /*public void VerticalLiftControl(){
        if (gamepad1.a)
            robot.verticalLift.setPower(-.7);
        else if (gamepad1.b)
            robot.verticalLift.setPower(.7);
        else
            robot.verticalLift.setPower(0);
    }*/

    boolean up = true;

    public void ArmControl() {
        if(gamepad1.x)
            arm.setPosition(Servo.MAX_POSITION);
        if(gamepad1.y)
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

    public void PlatformControl()
    {
        if(gamepad1.a) {
            platformR.setPosition(Servo.MAX_POSITION);
            platformL.setPosition(Servo.MIN_POSITION);
        }
        if(gamepad1.b) {
            platformR.setPosition(Servo.MIN_POSITION);
            platformL.setPosition(Servo.MAX_POSITION);
        }
    }

}

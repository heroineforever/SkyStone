package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

public class Hardware{

    public DcMotor leftFront, rightFront, rightBack, leftBack;
    public Servo arm;
    public CameraName cameraName;
    //public DcMotor verticalIntake, horizontalIntake;
    //public DcMotor greenWheelLeft, greenWheelRight;
    //public Servo gate, constrictL, constrictR, extrusion;

    //public DcMotor horizontalLift, verticalLift;


    //Constructor
    public Hardware(HardwareMap hwmp) {
        //Assign all motors/servos to the spots on the phone

        //INFO. Just change the name of the motor.
        //Motors
        leftFront = hwmp.dcMotor.get("Left Front");
        rightFront = hwmp.dcMotor.get("Right Front");
        rightBack = hwmp.dcMotor.get("Right Back");
        leftBack = hwmp.dcMotor.get("Left Back");

       /* greenWheelLeft = hwmp.dcMotor.get("Green Wheel Left");
        greenWheelRight = hwmp.dcMotor.get("Green Wheel Right");
        horizontalIntake = hwmp.dcMotor.get("Horizontal Intake");
        verticalIntake = hwmp.dcMotor.get("Vertical Intake");

        gate = hwmp.servo.get("Gate");
        constrictL = hwmp.servo.get("Constriction Left");
        constrictR = hwmp.servo.get("Constriction Right");
        extrusion = hwmp.servo.get("Extrusion Servo");*/

        //INFO If you want to put the motor flipped
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //leftFront.setDirection(DcMotor.Direction.REVERSE);
        //leftBack.setDirection(DcMotor.Direction.REVERSE);

        //INFO Reverse the robot.
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        //liftServo = hwmp.servo.get("Lift Servo");
        arm = hwmp.servo.get("Arm");

        cameraName = hwmp.get(CameraName.class, "Webcam");

    }

    //General Methods
    public void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(Servo.Direction.FORWARD);
        //horizontalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //verticalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //get ready for rerun
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //horizontalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //verticalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopAllMotors() {
        leftFront.setPower(0); //this method to stop any motors, range -1 to 1
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        arm.setPosition(0);
        //verticalLift.setPower(0);
        //horizontalLift.setPower(0);
    }



}
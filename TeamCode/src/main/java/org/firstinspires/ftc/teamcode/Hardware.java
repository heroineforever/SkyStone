package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware{

    public DcMotor leftDrive, rightDrive;

    public DcMotor frontRight, frontLeft, backRight, backLeft;

    public Servo latch;

    //Constructor
    public Hardware(HardwareMap hwmp) {
        //Assign all motors/servos to the spots on the phone

        //Motors
        leftDrive = hwmp.dcMotor.get("Left Drive");
        rightDrive = hwmp.dcMotor.get("Right Drive");

        frontRight = hwmp.dcMotor.get("Front Right");
        frontLeft = hwmp.dcMotor.get("Front Left");
        backRight = hwmp.dcMotor.get("Back Right");
        backLeft = hwmp.dcMotor.get("Back Left");

        //If you want to put the motor flipped
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        latch = hwmp.servo.get("Latch Servo");


    }

    //General Methods
    public void resetDriveEncoders() {

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //get ready for rerun
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void stopAllMotors() {
        rightDrive.setPower(0); //this method to stop any motors, range -1 to 1
        leftDrive.setPower(0);
        //verticalLift.setPower(0);
        //horizontalLift.setPower(0);
    }



}
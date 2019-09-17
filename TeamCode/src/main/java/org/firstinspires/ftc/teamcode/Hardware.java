package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareRobot {

    public DcMotor leftDrive, rightDrive;

    public DcMotor horizontalLift, verticalLift;


    //Constructor
    public Hardware(HardwareMap hwmp) {
        //Assign all motors/servos to the spots on the phone

        //Motors
        leftDrive = hwmp.dcMotor.get("Left Drive");
        rightDrive = hwmp.dcMotor.get("Right Drive");
        horizontalLift = hwmp.dcMotor.get("Horizontal Lift");
        verticalLift = hwmp.dcMotor.get("Vertical Left");

        //If you want to put the motor flipped
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        //liftServo = hwmp.servo.get("Lift Servo");


    }

    //General Methods
    public void resetDriveEncoders() {

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //get ready for rerun
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopAllMotors() {
        rightDrive.setPower(0); //this method to stop any motors, range -1 to 1
        leftDrive.setPower(0);
        verticalLift.setPower(0);
        horizontalLift.setPower(0);
    }



}
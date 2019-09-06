public class HardwareRobot {

    public DcMotor leftDrive, rightDrive;

    public Servo liftServo;


    //Constructor
    public HardwareRobot(HardWareMap hwmp) {
        //Assign all motors/servos to the spots on the phone

        //Motors
        leftDrive = hwmp.dcMotor.get("Left Drive");
        rightDrive = hwmp.dcMotor.get("Right Drive");
        //If you want to put the motor flipped
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        liftServo = hwmp.servo.get("Lift Servo");


    }

    //General Methods
    public void resetDriveEncoders() {

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //get ready for rerun
    }

    public void stopAllMotors() {
        rightDrive.setPower(0); //this method to stop any motors, range -1 to 1
    }



}
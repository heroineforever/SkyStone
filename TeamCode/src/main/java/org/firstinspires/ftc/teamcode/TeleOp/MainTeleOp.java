


@TeleOp(name="Main TeleOp", group = "Linear Opmode");

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
    public void init(){
        //map hardware
        robot = new Hardware(hardwareMap);

        //runtime = new ElapsedTime();
    }
    //start code
    @Override
    public void loop(){

    }

    //driving hub
    public void DriveControl(){
        //got the direction from the controller then told the motors what to do

        //joystick ranges -1 to 1
    }

    public void aRandomServoControl(){
        //servo ranges 0 to 1, 0 is min
        robot.liftServo.setPostion(Servo.MIN_POSITION);
    }

}



@Autonomous(name = "An Autonomous Test", group = "Autonomous")

public class AutoRun extends Autonomous{



    @Override
    public void runOpMode() {
        super.runOpMode();

        //use methods from autonomous to command the robot

        waitForStart();
    }
}
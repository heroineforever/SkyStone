package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;
import org.firstinspires.ftc.teamcode.Hardware;


@TeleOp(name = "OpMode (Blocks to Java)", group = "")
@Disabled
public class OpMode extends LinearOpMode {

    private VuforiaRoverRuckus vuforiaRoverRuckus;
    private TfodRoverRuckus tfodRoverRuckus;
    VuforiaLocalizer vuforia;
    Hardware robot;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        robot = new Hardware(hardwareMap);
        List recognitions;
        double goldMineralX;
        double silverMineral1X;
        double silverMineral2X;
        final String LABEL_SPECIAL_SKYSTONE = "Special Skystone";

        vuforiaRoverRuckus = new VuforiaRoverRuckus();
        tfodRoverRuckus = new TfodRoverRuckus();


        //Vuforia stuff
        final String VUFORIA_KEY = "ATKKdVf/////AAABmb9SxtpqfUvxqCFmSowoT10see3Vz9mze+DVTbtqieMNjFxZverOpqc4OYMhAkuv9rnJMQZyuaweuLOXioXqVuYJ2P2yRohAKL//zPiF1drlPCUbzdhh3pFV8X4rnBILwoF9C3gWvpQfB//IJdZXNBkWYOZAp+UXGBW2WGdt2rQFHw4Y23GrGb2XCmPEHynO8tiNb6IzR6vOh/KOZ8GyTVES7+GyMVhFWNqgL969+ra6Ev5mgfDqaIt4DAqOoiMomDF9mm+Ixx7m6R2pwJC69XVvqAE6+fuotOs8fvA2XRtU+NNaD2ALR247keSC3qK0RnH8JGjYbSmiOHuRqHW9p9J/JrG1OPOxKnKuGEhhcgA7";

        //TensorFlow Object detector
        TFObjectDetector tfod;

        // Put initialization blocks here.
        //vuforiaRoverRuckus.initialize(VUFORIA_KEY, DEFAULT, hardwareMap.get(WebcamName.class, "Webcam"),true, true, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                //0, 0, 0, 0, 0, 0, true);tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float) 0.4, true, true);

                telemetry.addData(">", "Press Play to start");
        telemetry.update();

        initVuforia();
        waitForStart();
        if (opModeIsActive()) {
            tfodRoverRuckus.activate();
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                recognitions = tfodRoverRuckus.getRecognitions();
                telemetry.addData("# Objects Recognized", recognitions.size());
                if (recognitions.size() == 3) {
                    goldMineralX = -1;
                    silverMineral1X = -1;
                    silverMineral2X = -1;
                    // TODO: Enter the type for variable named recognition
                    for (Object recognition : recognitions) {
                        Recognition recognitio = (Recognition) recognition;
                        if (recognitio.getLabel().equals("Gold Mineral")) {
                            goldMineralX = recognitio.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = recognitio.getLeft();
                        } else {
                            silverMineral2X = recognitio.getLeft();
                        }
                    }
                    // Make sure we found one gold mineral and two silver minerals.
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                        }
                    }
                }
                telemetry.update();
                telemetry.update();
            }
            tfodRoverRuckus.deactivate();
        }

        vuforiaRoverRuckus.close();
        tfodRoverRuckus.close();
    }
    void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ATKKdVf/////AAABmb9SxtpqfUvxqCFmSowoT10see3Vz9mze+DVTbtqieMNjFxZverOpqc4OYMhAkuv9rnJMQZyuaweuLOXioXqVuYJ2P2yRohAKL//zPiF1drlPCUbzdhh3pFV8X4rnBILwoF9C3gWvpQfB//IJdZXNBkWYOZAp+UXGBW2WGdt2rQFHw4Y23GrGb2XCmPEHynO8tiNb6IzR6vOh/KOZ8GyTVES7+GyMVhFWNqgL969+ra6Ev5mgfDqaIt4DAqOoiMomDF9mm+Ixx7m6R2pwJC69XVvqAE6+fuotOs8fvA2XRtU+NNaD2ALR247keSC3qK0RnH8JGjYbSmiOHuRqHW9p9J/JrG1OPOxKnKuGEhhcgA7";;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = robot.cameraName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}

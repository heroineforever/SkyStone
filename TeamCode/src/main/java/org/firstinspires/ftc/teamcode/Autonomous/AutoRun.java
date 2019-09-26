package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.annotation.Annotation;

@Autonomous(name = "An Autonomous Test", group = "Autonomous")

public class AutoRun implements Autonomous {

    @Override
    public String name() {
        return null;
    }

    @Override
    public String group() {
        return null;
    }

    @Override
    public Class<? extends Annotation> annotationType() {
        return null;
    }
}
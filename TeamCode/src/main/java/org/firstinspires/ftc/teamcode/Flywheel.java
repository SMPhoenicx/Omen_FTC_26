package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private MotorEx flywheel = new MotorEx("fly");


    private Flywheel() { }
    public void setPower(double power) {
        flywheel.setPower(power);
    }

    public void stop() {
        flywheel.setPower(0);
    }
}
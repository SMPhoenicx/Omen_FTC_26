package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private MotorEx intake = new MotorEx("in").reversed();


    private Intake() { }
    public void setPower(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }
}
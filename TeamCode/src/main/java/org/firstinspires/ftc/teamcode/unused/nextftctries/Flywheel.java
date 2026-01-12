package org.firstinspires.ftc.teamcode.unused.nextftctries;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

// ts is so weird bruh
public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() {}

    private MotorEx fly1 = new MotorEx("fly1");
    private MotorEx fly2 = new MotorEx("fly2").reversed();
    //region FLYWHEEL SYSTEM
    // PID Constants
    double flyKp = 11.82;
    double flyKi = 0.53;
    double flyKd = 6.1;
    double flyKiOffset = 0.0;
    double flyKpOffset = 0.0;
    //endregion

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(flyKp + flyKpOffset, flyKi + flyKiOffset, flyKd)
            .build();

    @Override
    public void periodic() {
        fly1.setPower(controlSystem.calculate(fly1.getState()));
        fly2.setPower(controlSystem.calculate(fly2.getState()));
    }

    public void setPower(double power) {
        fly1.setPower(power);
        fly2.setPower(power);
    }

    public void stop() {
        fly1.setPower(0);
        fly2.setPower(0);
    }

    public double getVelocityOne() {
        return fly1.getVelocity();
    }

    public double getVelocityTwo() {
        return fly2.getVelocity();
    }
}
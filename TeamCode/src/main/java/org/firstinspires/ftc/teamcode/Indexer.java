package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Indexer implements Subsystem {
    public static final Indexer INSTANCE = new Indexer();
    private MotorEx indexer1 = new MotorEx("t1");
    private MotorEx indexer2 = new MotorEx("t2");


    private Indexer() { }
    public void start() {
        indexer1.setPower(1);
        indexer2.setPower(1);
    }

    public void stop() {
        indexer1.setPower(0);
        indexer2.setPower(0);
    }
}
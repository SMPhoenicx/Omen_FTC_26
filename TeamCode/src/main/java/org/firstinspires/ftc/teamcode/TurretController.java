package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;

public class TurretController {
    // Hardware
    private final CRServo turret1, turret2;
    private final AnalogInput turretEncoder;

    // Tuning constants
    public static double Kp = 0.005, Ki = 0.0006, Kd = 0.00014, Kf = 0.005;

    private static double tuKv = 0.00;


    public static double integralLimit = 500.0;
    public static double positionToleranceDeg = 2.0;
    public static double outputDeadband = 0.02;

    public double globalOffset = 0.0;

    private double integral = 0.0;
    private double lastError = 0.0;

    public volatile double currentAngle = 0;
    public volatile double currentIntegral = 0;

    private volatile double targetAngle = 0;
    private static final double turretZeroDeg = 7;
    private static final double TURRET_LIMIT_DEG = 150.0;


    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;
    public TurretController(CRServo s1, CRServo s2, AnalogInput enc) {
        this.turret1 = s1;
        this.turret2 = s2;
        this.turretEncoder = enc;
    }

    public void setTarget(double angle, double velocityFF) {
    }    public void update(double dt) {

}}



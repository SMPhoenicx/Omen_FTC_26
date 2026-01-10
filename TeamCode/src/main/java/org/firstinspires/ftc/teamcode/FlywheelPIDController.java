package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FlywheelPIDController {
    // Motors
    private final DcMotorEx fly1;
    private final DcMotorEx fly2;

    // makes the initial spinup faster, boosts ff
    public double SPINUP_BOOST = 0.07;

    // Variables to tune
    public double Kp = 0.0008;
    public double Ki = 0.0;
    public double Kd = 0.000062;
    public double Kv = 0.00042;
    public double Ka = 1e-7;

    // integral stuff
    private double integral = 0.0;
    public double integralLimit = 0.25;
    public double integralActiveError = 1000.0;

    public double maxAccel = 500.0;

    // voltage compensation
    public double voltageFiltered = 12.7;
    public double voltageAlpha = 0.02; //voltage smoothing
    private long lastFUpdateMs = 0;
    public long fUpdateIntervalMs = 250;

    // derivative stuff
    private double lastError = 0.0;
    public double lastMeasuredVelocity = 0.0;
    private double dFiltered = 0.0;
    public double dFilterAlpha = 0.2;

    // state stuff
    public double commandedVelocity = 0.0;
    public double outputDeadband = 0.01;
    public double velocityTolerance = 50.0;

    public FlywheelPIDController(DcMotorEx fly1, DcMotorEx fly2) {
        this.fly1 = fly1;
        this.fly2 = fly2;

        this.fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void updateFlywheelPID(double targetTicksPerSec, double dt, double batteryVoltage) {
        if (targetTicksPerSec <= 1e-6) {
            // reset state
            integral = 0.0;
            lastError = 0.0;
            //apply 0 power, not negative to reduce unnecessary motor strain
            fly1.setPower(0.0);
            fly2.setPower(0.0);

            lastMeasuredVelocity =
                    (fly1.getVelocity() + fly2.getVelocity()) * 0.5;

            return;
        }

        if (dt <= 0) return;

        voltageFiltered =
                voltageFiltered * (1.0 - voltageAlpha)
                        + batteryVoltage * voltageAlpha;

        double maxStep = maxAccel * dt;
        double diff = targetTicksPerSec - commandedVelocity;
        if (Math.abs(diff) > maxStep) commandedVelocity += Math.signum(diff) * maxStep;
        else commandedVelocity = targetTicksPerSec;

        double measured = (fly1.getVelocity() + fly2.getVelocity()) * 0.5;

        double commandedAccel = (commandedVelocity - lastMeasuredVelocity) / Math.max(dt, 1e-6);
        double ff = Kv * commandedVelocity + Ka * commandedAccel;

        if (commandedVelocity > 0 &&
                lastMeasuredVelocity < 0.8 * commandedVelocity) {
            ff += SPINUP_BOOST;
        }

        if (System.currentTimeMillis() - lastFUpdateMs > fUpdateIntervalMs) {
            double scale = 12.7 / Math.max(8.0, voltageFiltered);
            ff *= scale;
            lastFUpdateMs = System.currentTimeMillis();
        }

        double error = commandedVelocity - measured;

        //derivative
        double dRaw = (error - lastError) / Math.max(dt, 1e-6);
        dFiltered = dFilterAlpha * dFiltered + (1.0 - dFilterAlpha) * dRaw;

        // integral
        if (Math.abs(error) < integralActiveError) {
            integral += error * dt;
            double integralPower = Ki * integral;
            if (integralPower > integralLimit) integral = integralLimit / Math.max(Ki, 1e-12);
            if (integralPower < -integralLimit) integral = -integralLimit / Math.max(Ki, 1e-12);
        } else {
            integral *= 0.9;
        }

        double pidOut = Kp * error + Ki * integral + Kd * dFiltered;

        double power = ff + pidOut;

        //clamp
        power = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(power) < outputDeadband) power = 0.0;

        if (Math.abs(error) <= velocityTolerance) {
            power = ff;
            integral *= 0.5;
        }

        fly1.setPower(power);
        fly2.setPower(power);

        lastMeasuredVelocity = measured;
        lastError = error;
    }
}

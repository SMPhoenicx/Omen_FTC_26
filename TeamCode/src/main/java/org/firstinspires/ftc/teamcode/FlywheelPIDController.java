package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// FlywheelPIDController.java
public class FlywheelPIDController {
    // Motors
    private final DcMotorEx fly1;
    private final DcMotorEx fly2;

    public double SPINUP_BOOST = 0.085;

    // PID
    public double Kp = 0.0008;
    public double Ki = 0.0;
    public double Kd = 0.000055;
    // Feedforward
    public double Kv = 0.00042;  // initial guess (power per ticks/sec) — measure this
    public double Ka = 1e-7;     // small accel FF (power per ticks/sec^2)

    // integral stuff
    private double integral = 0.0;
    public double integralLimit = 0.25;    // clamp integral contribution (power)
    public double integralActiveError = 1000.0; // only integrate if |error| < this

    public double maxAccel = 500.0; // ticks/sec^2 (tune)

    // voltage compensation
    public double voltageFiltered = 12.5;
    public double voltageAlpha = 0.02; // EMA alpha for voltage (0.01 - 0.1)
    private long lastFUpdateMs = 0;
    public long fUpdateIntervalMs = 250;

    // derivative stuff
    private double lastError = 0.0;
    public double lastMeasuredVelocity = 0.0;
    private double dFiltered = 0.0;
    public double dFilterAlpha = 0.2;

    // state stuff
    public double commandedVelocity = 0.0; // ticks/sec (slewed target)
    public double outputDeadband = 0.01; // power deadband
    public double velocityTolerance = 50.0; // ticks/sec tolerance to consider "at speed"

    public FlywheelPIDController(DcMotorEx fly1, DcMotorEx fly2) {
        this.fly1 = fly1;
        this.fly2 = fly2;

        this.fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void updateFlywheelPID(double targetTicksPerSec, double dt, double batteryVoltage) {
        if (targetTicksPerSec <= 1e-6) {
            // Reset controller state
            integral = 0.0;
            lastError = 0.0;

            // Do NOT apply negative power
            fly1.setPower(0.0);
            fly2.setPower(0.0);

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
                lastMeasuredVelocity < 0.85 * commandedVelocity) {
            ff += SPINUP_BOOST;
        }

        if (System.currentTimeMillis() - lastFUpdateMs > fUpdateIntervalMs) {
            double scale = 12.7 / Math.max(8.0, voltageFiltered);
            ff *= scale;
            lastFUpdateMs = System.currentTimeMillis();
        }

        // --- 5) PID calculations (error in ticks/sec => pidOut in power units) ---
        double error = commandedVelocity - measured;

        // derivative (filtered)
        double dRaw = (error - lastError) / Math.max(dt, 1e-6);
        dFiltered = dFilterAlpha * dFiltered + (1.0 - dFilterAlpha) * dRaw;

        // integral (conditional)
        if (Math.abs(error) < integralActiveError) {
            integral += error * dt;
            // clamp integral *before* applying Ki so we limit its power contribution
            double integralPower = Ki * integral;
            if (integralPower > integralLimit) integral = integralLimit / Math.max(Ki, 1e-12);
            if (integralPower < -integralLimit) integral = -integralLimit / Math.max(Ki, 1e-12);
        } else {
            // decay integral if far away to avoid long wind-up
            integral *= 0.9;
        }

        double pidOut = Kp * error + Ki * integral + Kd * dFiltered;

        // --- 6) total power and clipping ---
        double power = ff + pidOut;

        // clamp to allowable motor power -1..1
        power = Math.max(-1.0, Math.min(1.0, power));
        if (Math.abs(power) < outputDeadband) power = 0.0;

        // if within velocity tolerance, optionally zero output and damp integral to avoid bumping
        if (Math.abs(error) <= velocityTolerance) {
            power = ff; // keep steady feedforward only; or set to 0.0 if you prefer
            // slight integrator decay
            integral *= 0.5;
        }

        fly1.setPower(power);
        fly2.setPower(power);

        lastMeasuredVelocity = measured;
        lastError = error;
    }
}

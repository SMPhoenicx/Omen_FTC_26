package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FlywheelPIDController {
    // Motors
    private final DcMotorEx fly1;
    private final DcMotorEx fly2;

    // makes the initial spinup faster, boosts ff
    public double SPINUP_BOOST = 0.0;

    // Variables to tune
    public double Kp = 0.0022;
    public double Ki = 0.0001;
    public double Kd = 0.0008;
    public double Kv = 0.000446;
    public double Ka = 1e-7;

    // integral stuff
    private double integral = 0.0;
    public double integralLimit = 0.25;
    public double integralActiveError = 100.0;

    public double maxAccel = 50000.0;

    // voltage compensation
    public double voltageFiltered = 12.47;
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

    public double teleopMultiplier = 1.0;

    public FlywheelPIDController(DcMotorEx fly1, DcMotorEx fly2) {
        this.fly1 = fly1;
        this.fly2 = fly2;

        this.fly1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fly2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            double scale = 12.47 / Math.max(8.0, voltageFiltered);
            ff *= scale;
            lastFUpdateMs = System.currentTimeMillis();
        }

        // REPLACE the existing PID calculation and clamping logic with this:

        double error = commandedVelocity - measured;

        double power;

        if (commandedVelocity <= 10) {
            // 1. OFF STATE
            power = 0;
            integral = 0; // Reset integral when stopped
        }
        else if (error > 300) {
            // 2. TURBO STATE (Bang-Bang)
            if (batteryVoltage < 11) {
                power = 0.77 * teleopMultiplier;
            } else {
                power = 0.98 * teleopMultiplier;
            }
        }
        //TODO check this
//        else if (error < -200) {
//            power = -0.1; // Safe active braking to stop the "1200 spike"
//        }
        else {
            // 3. STABILIZATION STATE (PID + FF)
            // Derivative
            double dRaw = (error - lastError) / Math.max(dt, 1e-6);
            dFiltered = dFilterAlpha * dFiltered + (1.0 - dFilterAlpha) * dRaw;

            // Integral with strict window
            // Integral with strict window
            if (Math.abs(error) < integralActiveError) {
                integral += error * dt;

                double maxI = 0.25;
                // Safer implementation:
                double potentialIPower = integral * Ki;
                if (potentialIPower > maxI) integral = maxI / Ki;
                if (potentialIPower < -maxI) integral = -maxI / Ki;

            } else {
                integral = 0;
            }

            // Total Output
            double pidOut = (Kp * error) + (Ki * integral) + (Kd * dFiltered);
            power = ff + pidOut;
        }

        power = Math.max(-1.0, Math.min(1.0, power));
        fly1.setPower(power);
        fly2.setPower(power);

        lastMeasuredVelocity = measured;
        lastError = error;
    }
}

package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GlobalOffsets;

import java.util.Arrays;

public class SpindexerController {
    // Hardware
    private final CRServo spin1, spin2;
    private final AnalogInput spinEncoder;

    // Tuning constants
    public static double Kp = 0.0067, Ki = 0.0003, Kd = 0.00073, Kf = 0.01;
    public static double integralLimit = 4.0;
    public static double positionToleranceDeg = 0.2;
    public static double outputDeadband = 0.02;
    public static double tau = 0.03;

    public double globalOffset = 30.0;

    //removed in between positions
    private final double[] SPINDEXER_POSITIONS = {49.75, 79.75, 109.75, 139.75, 169.75, 19.75};

    private int targetIndex = 0;
    private boolean isRapidFire = false;

    private double integral = 0.0;
    public double lastError = 0.0;
    private double lastFilteredD = 0.0;
    public boolean isArmed = false;

    public char[] savedBalls = {'n', 'n', 'n'}; // n = none, g = green, etc.
    public boolean[] presentBalls = {false, false, false};
    // Telemetry getters
    public volatile double currentAngle = 0;
    public volatile double currentIntegral = 0;

    public int prevSpindexerIndex = 0;
    public SpindexerController(CRServo s1, CRServo s2, AnalogInput enc) {
        this.spin1 = s1;
        this.spin2 = s2;
        this.spinEncoder = enc;
    }

    /**
     * The Main Control Loop - Run this in your Background Thread
     * @param dt The delta time in seconds (e.g., 0.02)
     */
    public void update(double dt) {
        if (isRapidFire) {
            spin1.setPower(0.93);
            spin2.setPower(0.93);
            return;
        }

        if (!isArmed) {
            // Reset PID state on first loop
            lastError = 0;
            lastFilteredD = 0;
            integral = 0;
            spin1.setPower(0);
            spin2.setPower(0);
            isArmed = true;
            return;
        }

        // get encoder position
        double voltage = spinEncoder.getVoltage();
        currentAngle = mapVoltageToAngle360(voltage, 0.0, 3.3);

        // find angle
        double baseTarget = SPINDEXER_POSITIONS[targetIndex] + globalOffset;

        // twin target shortest path
        double targetB = (baseTarget + 180.0) % 360.0;
        double errorA = -angleError(baseTarget, currentAngle);
        double errorB = -angleError(targetB, currentAngle);

        // choose path to position
        double error = (Math.abs(errorA) <= Math.abs(errorB)) ? errorA : errorB;

        // integral
        integral += error * dt;
        integral = Range.clip(integral, -integralLimit, integralLimit);
        currentIntegral = integral;

        // deriv with low pass filter
        double alpha = Math.exp(-dt / tau);

        double rawD = (error - lastError) / Math.max(dt, 1e-6);
        double d = alpha * lastFilteredD + (1 - alpha) * rawD;

        lastFilteredD = d;

        // output
        double out = Kp * error + Ki * integral + Kd * d;

        // ff
        if (Math.abs(error) > 1.0) out += Kf * Math.signum(error);

        // limits
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // avoid jitter
        if (Math.abs(error) <= positionToleranceDeg) {
            out *= 0.1;
            integral *= 0.05;
        }

        spin1.setPower(out);
        spin2.setPower(out);

        lastError = error;
    }

   //region HELPER METHODS
    public void setRapidFire(boolean active) {
        this.isRapidFire = active;
        if (!active) {
            Arrays.fill(savedBalls, 'n');
            calculateNearestIndex();
        }
    }

    public boolean getRapidFire() {
        return isRapidFire;
    }

    public void spinClockwise() {
        prevSpindexerIndex = targetIndex;
        targetIndex += (targetIndex % 2 != 0) ? 1 : 0;
        targetIndex = (targetIndex - 2 + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
    }

    public void spinCounterClockwise() {
        int newIndex = targetIndex;
        newIndex += (newIndex % 2 != 0) ? 1 : 0;
        newIndex = (newIndex + 2) % SPINDEXER_POSITIONS.length;
        targetIndex = newIndex;
    }

    public void updateBallState(char detectedColor, boolean present) {
        // Only update if settled
        if (Math.abs(lastError) < 10) {
            setBallAtIndex(targetIndex, detectedColor, present);
        }
    }

    public void autoIndexForIntake() {
        int currentSlot = -1;
        for (int i = 0; i < savedBalls.length; i++) {
            if (savedBalls[i] == 'n') {
                currentSlot = i;
                break;
            }
        }
        if (currentSlot != -1) {
            int desiredIndex = currentSlot * 2;
            if (targetIndex != desiredIndex) {
                prevSpindexerIndex = targetIndex;
                targetIndex = desiredIndex;
            }
        }
    }

    public void sortGreen(int greenPos) {
        int greenIn=-1;
        for(int i=0;i<3;i++) if(savedBalls[i]=='g') greenIn=i;


        if(greenIn==-1) for(int i=0;i<3;i++) if(savedBalls[i]=='n') greenIn=i;

        if(greenIn==-1) greenIn=0;

        int diff = (greenIn + greenPos) % 3;
        if(diff==0) targetIndex=4;
        else if(diff==1) targetIndex=0;
        else targetIndex=2;
    }

    public void calculateNearestIndex() {
        double currentAngle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        int bestIndex = targetIndex;
        double minAbsError = 360.0;

        for (int i = 0; i < SPINDEXER_POSITIONS.length; i+=2) {
            double baseTarget = SPINDEXER_POSITIONS[i] + GlobalOffsets.spindexerOffset;

            double errorNormal = Math.abs(angleError(baseTarget, currentAngle));

            double ghostTarget = baseTarget + 180.0;
            double errorGhost = Math.abs(angleError(ghostTarget, currentAngle));

            double localMinError = Math.min(errorNormal, errorGhost);

            if (localMinError < minAbsError) {
                minAbsError = localMinError;
                bestIndex = i;
            }
        }

        targetIndex = bestIndex;
    }

    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        return angle;
    }

    private double angleError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    private void setBallAtIndex(int idx, char color, boolean isPresent) {
        if (idx == 0) {
            savedBalls[0] = color;
            presentBalls[0] = isPresent;
        }
        else if (idx == 2) {
            savedBalls[1] = color;
            presentBalls[1] = isPresent;
        }
        else if (idx == 4) {
            savedBalls[2] = color;
            presentBalls[2] = isPresent;
        }
    }

    public boolean hasEmptySlot() {
        for (char c : savedBalls) {
            if (c == 'n') return true;
        }
        return false;
    }
}
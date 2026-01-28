package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class CachingDcMotor implements DcMotor {
    public final DcMotor dcMotor;
    public double cachingTolerance = 0.005;
    private double cachedPower = Double.NaN;


    public CachingDcMotor(DcMotor dcMotor) {
        this(dcMotor, 0.005);
    }

    public CachingDcMotor(DcMotor dcMotor, double cachingTolerance) {
        this.dcMotor = dcMotor;
        this.cachingTolerance = cachingTolerance;
    }

    public void setPower(double power) {
        double corrected = Math.min(1.0, Math.max(-1.0, power));
        synchronized (this) {
            if (Math.abs(corrected - cachedPower) >= cachingTolerance
                    || (corrected == 0.0 && cachedPower != 0.0)
                    || (corrected >= 1.0 && !(cachedPower >= 1.0))
                    || (corrected <= -1.0 && !(cachedPower <= -1.0))
                    || Double.isNaN(cachedPower)) {
                cachedPower = corrected;
                dcMotor.setPower(corrected);
            }
        }
    }

    public boolean setPowerResult(double power) {
        double corrected = Math.min(1.0, Math.max(-1.0, power));
        synchronized (this) {
            if (Math.abs(corrected - cachedPower) >= cachingTolerance
                    || (corrected == 0.0 && cachedPower != 0.0)
                    || (corrected >= 1.0 && !(cachedPower >= 1.0))
                    || (corrected <= -1.0 && !(cachedPower <= -1.0))
                    || Double.isNaN(cachedPower)) {
                cachedPower = corrected;
                dcMotor.setPower(corrected);
                return true;
            }
        }
        return false;
    }

    public boolean setPowerRaw(double power) {
        synchronized (this) {
            double originalCachingTolerance = this.cachingTolerance;
            this.cachingTolerance = 0.0;
            boolean res = setPowerResult(power);
            this.cachingTolerance = originalCachingTolerance;
            return res;
        }
    }

    // Delegate all other DcMotor methods to dcMotor

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public double getPower() {
        return dcMotor.getPower();
    }

    @Override
    public void setMode(DcMotor.RunMode mode) {
        dcMotor.setMode(mode);
    }

    @Override
    public DcMotor.RunMode getMode() {
        return dcMotor.getMode();
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        dcMotor.setZeroPowerBehavior(behavior);
    }

    @Override
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    @Override
    public void setDirection(DcMotor.Direction direction) {
        dcMotor.setDirection(direction);
    }

    @Override
    public DcMotor.Direction getDirection() {
        return dcMotor.getDirection();
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return dcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        dcMotor.close();
    }
}
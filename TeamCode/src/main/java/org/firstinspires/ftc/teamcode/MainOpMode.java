// (keeps your original header & imports)
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import dev.nextftc.control.feedback.PIDController;

@TeleOp(name="MainOpMode", group = "Concept")
public class MainOpMode extends LinearOpMode
{

    final double TURN_GAIN   =  0.02  ;
    final double MAX_AUTO_TURN  = 0.4;

    // MARK:- MOTORS AND SERVOS
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor fly1 = null;
    private DcMotor fly2 = null;
    private DcMotor intake = null;
    private CRServo rspin = null;
    private CRServo lspin = null;
    private Servo feeder = null;
    private Servo led = null;

    // MARK:- CAMERA
    private static final int DESIRED_TAG_ID = 20;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;

    // MARK:- ENCODERS / pots
    private AnalogInput spinRight;
    private AnalogInput spinLeft;

    // --- Carousel preset positions (6 presets, every 60 degrees) ---
    private final double[] CAROUSEL_POSITIONS = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};
    private int carouselIndex = 0;

    // --- PIDF gains (tweak on robot) ---
    private  double pidKp = 0.0001;    // start small, increase until responsive
    private  double pidKi = 0.0;  // tiny integral (if needed)
    private  double pidKd = 0.0;  // derivative to damp oscillation
    private  double pidKf = 0.05;    // small directional feedforward to overcome stiction

    private double integralR = 0.0;
    private double integralL = 0.0;
    private double lastErrorR = 0.0;
    private double lastErrorL = 0.0;
    private double integralLimit = 500.0; // clamp integral

    private double pidLastTimeMs = 0.0; // ms timestamp for PID dt

    // tolerance and deadband
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;
    int prevFlyPosition1 = 0;
    int prevFlyPosition2 = 0;
    double[] prevFlySpeeds1 = new double[100];
    double[] prevFlySpeeds2 = new double[100];
    ElapsedTime flyTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // --- existing variables from your code (fly state toggles, button edge trackers, etc) ---
    @Override public void runOpMode()
    {
        boolean targetFound     = false;
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;
        double flySpeed = 0;
        boolean flyOn = false;
        boolean flyAtSpeed = false;
        double lastTime = 0;
        boolean feederUp = false;

        boolean lb1Pressed = false;
        boolean rb1Pressed = false;
        boolean b1Pressed = false;
        boolean a1Pressed = false;
        boolean x1Pressed = false;
        boolean y1Pressed = false;
        boolean down1Pressed = false;
        boolean up1Pressed = false;
        boolean right1Pressed = false;
        boolean left1Pressed = false;

        boolean lb2Pressed = false;
        boolean rb2Pressed = false;
        boolean b2Pressed = false;
        boolean a2Pressed = false;
        boolean x2Pressed = false;
        boolean y2Pressed = false;
        boolean down2Pressed = false;
        boolean up2Pressed = false;
        boolean right2Pressed = false;
        boolean left2Pressed = false;
        // Initialize the Apriltag Detection process
        initAprilTag();

        // Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotor.class, "fly1");
        fly2 = hardwareMap.get(DcMotor.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        //Servos
        rspin = hardwareMap.get(CRServo.class, "rspin");
        lspin = hardwareMap.get(CRServo.class, "lspin");
        feeder = hardwareMap.get(Servo.class, "gate");
        led = hardwareMap.get(Servo.class,"led");
        //encoders/pots
        spinRight = hardwareMap.get(AnalogInput.class, "respin");
        spinLeft = hardwareMap.get(AnalogInput.class, "lespin");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rspin.setDirection(CRServo.Direction.FORWARD);
        lspin.setDirection(CRServo.Direction.FORWARD);
        feeder.setDirection(Servo.Direction.FORWARD);

        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        feeder.setPosition(1);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        // initialize pid timestamp after runtime was reset
        runtime.reset();
        pidLastTimeMs = runtime.milliseconds();

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            // ----- PID: compute dt -----
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0/50.0; // fallback
            pidLastTimeMs = nowMs;

            // ENCODING FOR SERVOS
            double voltR = spinRight.getVoltage();
            double voltL = spinLeft.getVoltage();

            // === PIDF tuning via Gamepad2 ===
            double adjustStepP = 0.00002;
            double adjustStepI = 0.00001;
            double adjustStepD = 0.0002;
            double debounceTime = 250; // milliseconds

            if (runtime.milliseconds() - lastPAdjustTime > debounceTime) {
                if (gamepad2.a) { pidKp += adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
                if (gamepad2.b) { pidKp -= adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastIAdjustTime > debounceTime) {
                if (gamepad2.x) { pidKi += adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
                if (gamepad2.y) { pidKi -= adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastDAdjustTime > debounceTime) {
                if (gamepad2.dpad_up) { pidKd += adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
                if (gamepad2.dpad_down) { pidKd -= adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
            }


// Safety clamp
            pidKp = Math.max(0, pidKp);
            pidKi = Math.max(0, pidKi);
            pidKd = Math.max(0, pidKd);

// Display PID constants on telemetry
            telemetry.addData("PID Tuning", "Press A/B=P+,P- | X/Y=I+,I- | Dpad Up/Down=D+,D-");
            telemetry.addData("kP", "%.4f", pidKp);
            telemetry.addData("kI", "%.4f", pidKi);
            telemetry.addData("kD", "%.4f", pidKd);

            // --- Flywheel controls ---
            if(gamepad1.a && !a1Pressed)  {
                flyOn = !flyOn;
                flySpeed = 0.5;
            }
            if(flyOn) {
                fly1.setPower(flySpeed);
                fly2.setPower(flySpeed);
            }
            else {
                fly1.setPower(0);
                fly2.setPower(0);
            }
            if(gamepad1.right_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                flySpeed += (flySpeed < 1)? 0.05:0;
                lastTime = runtime.milliseconds();
            }
            if(gamepad1.left_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                flySpeed -= (flySpeed > 0)? 0.05:0;
                lastTime = runtime.milliseconds();
            }
            //flywheel encoder stuff
            //wheel 1
            for(int i=prevFlySpeeds1.length-1;i>0;i--){
                prevFlySpeeds1[i]=prevFlySpeeds1[i-1];
            }
            prevFlySpeeds1[0] = (double) (fly1.getCurrentPosition() - prevFlyPosition1) / flyTimer.time();
            //idk stdev
            double mean = 0;
            for (double n : prevFlySpeeds1) mean += n;
            mean /= prevFlySpeeds1.length;
            double sum = 0;
            for (double n : prevFlySpeeds1) sum += Math.pow(n - mean, 2);
            flyAtSpeed = (Math.sqrt(sum / prevFlySpeeds1.length)) < 0.2;
            prevFlyPosition1 = fly1.getCurrentPosition();
            //wheel 2
            //flywheel encoder stuff
            for(int i=prevFlySpeeds2.length-1;i>0;i--){
                prevFlySpeeds2[i]=prevFlySpeeds2[i-1];
            }
            prevFlySpeeds2[0] = (double) (fly2.getCurrentPosition() - prevFlyPosition2) / flyTimer.time();
            //idk stdev
            mean = 0;
            for (double n : prevFlySpeeds2) mean += n;
            mean /= prevFlySpeeds2.length;
            sum = 0;
            for (double n : prevFlySpeeds2) sum += Math.pow(n - mean, 2);
            flyAtSpeed = ((Math.sqrt(sum / prevFlySpeeds2.length)) < 0.2)&&(flyAtSpeed);
            prevFlyPosition2 = fly2.getCurrentPosition();
            flyTimer.reset();

            // ----- CAROUSEL: dpad edge-detect to cycle presets -----
            if (gamepad1.dpad_right && !right1Pressed) {
                carouselIndex = (carouselIndex + 1) % CAROUSEL_POSITIONS.length;
            }
            if (gamepad1.dpad_left && !left1Pressed) {
                carouselIndex = (carouselIndex - 1 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
            }

            // always run PID towards the current selected preset while opMode active
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            updateCarouselPID(targetAngle, dtSec);

            // gate toggle
            if(gamepad1.dpad_up && !up1Pressed){
                feederUp = !feederUp;
                if(feederUp){
                    feeder.setPosition(0);
                }
                else{
                    feeder.setPosition(1);
                }
            }

            // Intake
            if(gamepad1.right_bumper && !rb1Pressed) {
                if(intake.getPower() <= 0) intake.setPower(1);
                else intake.setPower(0);
            }
            // Outtake
            if(gamepad1.left_bumper && !lb1Pressed) {
                intake.setPower(-0.6);
            }

            // AprilTag detection (unchanged)
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        desiredTag = detection;
                        targetFound = true;
                        break;
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Turn to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            }

            // Auto-turn to tag (unchanged)
            if (gamepad1.dpad_down && targetFound) {
                double headingError = desiredTag.ftcPose.bearing;
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                drive = 0;
                strafe = 0;
                telemetry.addData("Auto", "Turn %5.2f (headingErr %3.0f°)", turn, headingError);
            } else {
                drive  = -gamepad1.left_stick_y;
                strafe = -gamepad1.left_stick_x;
                turn   = -gamepad1.right_stick_x;
            }

            //led
            if(!flyOn){
                led.setPosition(1);//white
            }
            else if(flyAtSpeed){
                led.setPosition(0.5);//green
            }
            else{
                led.setPosition(0.3);//red
            }

            // Apply desired axes motions to the drivetrain (unchanged).
            moveRobot(drive, strafe, turn);

            // update edge-tracking booleans at end of loop (so edge-detect works next loop)
            b1Pressed = gamepad1.b;
            a1Pressed = gamepad1.a;
            x1Pressed = gamepad1.x;
            y1Pressed = gamepad1.y;
            down1Pressed = gamepad1.dpad_down;
            up1Pressed = gamepad1.dpad_up;
            left1Pressed = gamepad1.dpad_left;
            right1Pressed = gamepad1.dpad_right;
            lb1Pressed = gamepad1.left_bumper;
            rb1Pressed = gamepad1.right_bumper;

            b2Pressed = gamepad2.b;
            a2Pressed = gamepad2.a;
            x2Pressed = gamepad2.x;
            y2Pressed = gamepad2.y;
            down2Pressed = gamepad2.dpad_down;
            up2Pressed = gamepad2.dpad_up;
            left2Pressed = gamepad2.dpad_left;
            right2Pressed = gamepad2.dpad_right;
            lb2Pressed = gamepad2.left_bumper;
            rb2Pressed = gamepad2.right_bumper;

            // telemetry: include PID status & selected preset
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SelectedPresetIdx", carouselIndex + " -> " + targetAngle + "°");
            telemetry.addData("Fly state", flyOn);
            telemetry.addData("Fly power", flySpeed);
            telemetry.addData("Actual fly speed","Wheel 1: %7.1 Wheel 2: %7.1", prevFlySpeeds1[0],prevFlySpeeds2[0]);
            telemetry.addData("Fly at correct power", flyAtSpeed);
            telemetry.addData("Feeder Up",feederUp);
            telemetry.addData("Angle? RIGHT", mapVoltageToAngle360(voltR, 0.01, 3.29));
            telemetry.addData("Angle? LEFT", mapVoltageToAngle360(voltL, 0.01, 3.29));
            telemetry.addData("VOLT? RIGHT", voltR);
            telemetry.addData("VOLT? LEFT", voltL);

            telemetry.update();

            sleep(10);
        } // end while
    } // end runOpMode()

    // --- PID update that runs once per loop (non-blocking) ---

    private void updateCarouselPID(double targetAngle, double dt) {
        // read angles 0..360
        double angleR = mapVoltageToAngle360(spinRight.getVoltage(), 0.01, 3.29);
        double angleL = mapVoltageToAngle360(spinLeft.getVoltage(), 0.01, 3.29);

        // compute shortest signed error [-180,180]
        double errorR = angleError(targetAngle, angleR);
        double errorL = angleError(targetAngle, angleL);

        // integral with anti-windup
        integralR += errorR * dt;
        integralL += errorL * dt;
        integralR = clamp(integralR, -integralLimit, integralLimit);
        integralL = clamp(integralL, -integralLimit, integralLimit);

        // derivative
        double dR = (errorR - lastErrorR) / Math.max(dt, 1e-6);
        double dL = (errorL - lastErrorL) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double outR = pidKp * errorR + pidKi * integralR + pidKd * dR;
        double outL = pidKp * errorL + pidKi * integralL + pidKd * dL;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(errorR) > 1.0) outR += pidKf * Math.signum(errorR);
        if (Math.abs(errorL) > 1.0) outL += pidKf * Math.signum(errorL);

        // clamp to [-1,1] and apply deadband
        outR = Range.clip(outR, -1.0, 1.0);
        outL = Range.clip(outL, -1.0, 1.0);
        if (Math.abs(outR) < outputDeadband) outR = 0.0;
        if (Math.abs(outL) < outputDeadband) outL = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(errorR) <= positionToleranceDeg) {
            outR = 0.0;
            integralR *= 0.2;
        }
        if (Math.abs(errorL) <= positionToleranceDeg) {
            outL = 0.0;
            integralL *= 0.2;
        }

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        rspin.setPower(outR);
        lspin.setPower(outL);

        // store errors for next derivative calculation
        lastErrorR = errorR;
        lastErrorL = errorL;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Carousel Target", "%.1f°", targetAngle);
        telemetry.addData("R", "angle=%.2f, err=%.2f, pwr=%.2f", angleR, errorR, outR);
        telemetry.addData("L", "angle=%.2f, err=%.2f, pwr=%.2f", angleL, errorL, outL);

    }

    // reuse your mapping helpers (keeps same math)
    private double mapVoltToAngle(double v)
    {
        return 180.0*(v-0.01)/(3.29);
    }

    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        return angle;
    }

    // Compute shortest signed difference between two angles
    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    // small clamp utility
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // rest of your class unchanged: moveRobot, initAprilTag, setManualExposure etc...
    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // (unchanged) your existing initAprilTag content copied here...
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(904.848699568, 904.848699568, 658.131998572, 340.91602987)
                .build();
        aprilTag.setDecimation(3);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

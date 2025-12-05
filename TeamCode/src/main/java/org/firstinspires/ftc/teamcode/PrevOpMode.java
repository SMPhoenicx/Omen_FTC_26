/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="PrevOpMode", group = "A")
public class PrevOpMode extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //region HARDWARE DECLARATIONS
    // Drive Motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    // Mechanism Motors
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private DcMotor trans = null;

    // Servos
    private Servo led = null;
    private CRServo hood = null;
    private CRServo spin1 = null;
    private CRServo spin2 = null;
    private CRServo turret1 = null;
    private CRServo turret2 = null;

    // Sensors
    private AnalogInput spinEncoder;
    private AnalogInput hoodEncoder;
    private AnalogInput turretEncoder;
    private NormalizedColorSensor color = null;

    // Vision Hardware
    private Limelight3A limelight;
    //endregion

    //region VISION SYSTEM
    // AprilTag Configuration
    private static final int DESIRED_TAG_ID = 20; //blue=20, red=24
    private AprilTagDetection desiredTag;

    // FTC Vision Portal
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Tracking State
    private boolean facingGoal = false;
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 500;
    private double txOffset = 0;
    private double distCamOffset = 0;

    // Heading PID
    private double lastHeadingError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN = 0.02;
    final double MAX_AUTO_TURN = 0.4;
    //endregion

    //region FLYWHEEL SYSTEM
    // PID Constants
    double flyKp = 9.0;
    double flyKi = 0.6;
    double flyKd = 3.6;
    double flyKiOffset = 0.0;
    //endregion

    //region HOOD SYSTEM
    // Hood PIDF Constants
    private double hoodKp = 0.0048;
    private double hoodKi = 0.00014;
    private double hoodKd = 0.00;
    private double hoodKf = 0.0;

    // Hood PID State
    private double hoodIntegral = 0.0;
    private double hoodLastError = 0.0;
    private double hoodIntegralLimit = 50.0;
    private double hoodOutputDeadband = 0.05;
    private double hoodToleranceDeg = 2.0;

    // Tuning Adjustments
    private final double hoodKpUp = 0.005;
    private final double hoodKiUp = 0.00001;
    private final double hoodKdUp = 0.005;

    // Hood Positions
    private double hoodAngle = 0;
    private double hoodOffset = 0;
    //endregion

    //region SPINDEXER SYSTEM
    // Spindexer PIDF Constants
    private double pidKp = 0.0075;
    private double pidKi = 0.0006;
    private double pidKd = 0.000106;
    private double pidKf = 0.0001;


    // Spindexer PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;

    // Spindexer Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;

    // Spindexer Positions (6 presets, every 60 degrees)
    // 57, 177, and 297 face the intake; others face the transfer
    private final double[] SPINDEXER_POSITIONS = {40.0, 100.0, 160.0, 220.0, 280.0, 340.0};
    private int spindexerIndex = 0;
    private int prevSpindexerIndex = 0;

    // Ball Storage Tracking
    // 'n' = none (empty), 'p' = purple, 'g' = green
    private char[] savedBalls = {'n', 'n', 'n'};
    //endregion

    //region TURRET SYSTEM
    // PIDF Constants
    private double tuKp = 0.0055;
    private double tuKi = 0.000;
    private double tuKd = 0.00012;
    private double tuKf = 0.0;


    // PID State
    private double tuIntegral = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Control Parameters
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.03;

    // Turret Position
    private double tuPos = 0;

    // Simple low-pass on the camera error to kill jitter
    private double filteredHeadingError = 0.0;
    //endregion

    //region PEDROPATHING SYSTEM
    private Follower follower;
    PathChain endgame = null;
    Pose endgamePose = new Pose(103, 37.5, Math.toRadians(90));
    //endregion

    //region SHOOTING VARS
    private static final double[] RANGE_SAMPLES = {31, 36, 41, 46, 51, 71.5, 78, 85, 90, 93};
    private static final double[] FLY_SPEEDS = {1220, 1255, 1300, 1340, 1390, 1560, 1590, 1630, 1670, 1700};
    private static final double[] HOOD_ANGLES = {-6, -33, -40, -50, -80, -140, -180, -200, -187, -177};

    private double smoothedRange = 0;
    private boolean isInitialized = false;
    private static final double ALPHA = 0.9;

    private boolean flyHoodLock = false;

    private int autoShootNum = 3;
    private double autoShootTime = 0;
    private boolean autoShot = false;
    //endregion

    @Override
    public void runOpMode() {
        //region OPERATIONAL VARIABLES
        // Camera State
        boolean targetFound = false;

        // Mechanism States
        boolean tranOn = false;
        boolean intakeOn = false;
        double intakePower = 0;
        boolean flyOn = false;
        boolean flyAtSpeed = false;

        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Flywheel Control
        double flySpeed = 1160;
        double flyOffset = 0;
        double lastTime = 0;
        double lastPidTime = 0;

        // Localization
        boolean localizeApril = true;
        double aprilLocalizationTimeout = 0;

        //endregion

        //region HARDWARE INITIALIZATION
        // Initialize Drive Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        // Initialize Mechanism Motors
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        trans = hardwareMap.get(DcMotor.class, "trans");

        // Initialize Servos
        spin1 = hardwareMap.get(CRServo.class, "spin1");
        spin2 = hardwareMap.get(CRServo.class, "spin2");
        led = hardwareMap.get(Servo.class, "led");
        hood = hardwareMap.get(CRServo.class, "hood");
        turret1 = hardwareMap.get(CRServo.class, "tu1");
        turret2 = hardwareMap.get(CRServo.class, "tu2");

        // Initialize Encoders
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin1");
        hoodEncoder = hardwareMap.get(AnalogInput.class, "hooden");
        turretEncoder = hardwareMap.get(AnalogInput.class, "tuen");

        // Initialize Sensors
        color = hardwareMap.get(NormalizedColorSensor.class, "Color 1");

        // Configure Motor Modes
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure Motor Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        trans.setDirection(DcMotor.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.FORWARD);
        spin2.setDirection(CRServo.Direction.FORWARD);
        turret1.setDirection(CRServo.Direction.FORWARD);
        turret2.setDirection(CRServo.Direction.FORWARD);
        //endregion

        //region SUBSYSTEM INITIALIZATION
        // Initialize Path Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23, 120, Math.toRadians(90)));
        //endregion

        //region PRE-START
        initAprilTag();
        setManualExposure(4, 200);

        telemetry.update();
        waitForStart();
        runtime.reset();
        //endregion

        while (opModeIsActive()) {
            double turnInput = -gamepad1.right_stick_x;
            follower.update();
            pidLastTimeMs = runtime.milliseconds();

            //region VISION PROCESSING
            targetFound = false;
            desiredTag = null;

            //Webcam april tag processing
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == DESIRED_TAG_ID) {
                        desiredTag = detection;
                        targetFound = true;
                        break;
                    } else {
                        telemetry.addData("Camera: ", "Tag ID %d is not desired", detection.id);
                    }
                }
                else {
                    telemetry.addData("Camera: ", "Tag Not Found");
                }
            }

            // Process Target Data
            if (targetFound) {
                // get info and adjust decimation
                adjustDecimation(desiredTag.ftcPose.range);
                double range = desiredTag.ftcPose.range;

                // smooth ranges to prevent too much fluctuation
                if (!isInitialized) {
                    smoothedRange = range;
                    isInitialized = true;
                } else {
                    // Smooth the readings
                    smoothedRange = smooth(range, smoothedRange);
                }

                //pid integer offsets //TODO
                if (smoothedRange > 70) {
                    flyKiOffset = 0.45;
                }
                if (smoothedRange < 40) {
                    flyKiOffset = -0.2;
                }

                // auto speed and hood interpolation
                if (!flyHoodLock) {
                    flySpeed = interpolate(smoothedRange, RANGE_SAMPLES, FLY_SPEEDS);
                    hoodAngle = interpolate(smoothedRange, RANGE_SAMPLES, HOOD_ANGLES);
                }

                hoodAngle = hoodAngle < -190 ? -190:hoodAngle;
                telemetry.addData("Target ID", desiredTag.id);
                telemetry.addData("Distance", "%.1f inches", smoothedRange);
                telemetry.addData("Range", "%.1f inches", range);
            }
            //endregion

            //region COLOR SENSOR AND BALL TRACKING
            char detectedColor = getDetectedColor();

            // Update saved ball positions based on spindexer position
            if (spindexerIndex == 0) savedBalls[0] = detectedColor;
            if (spindexerIndex == 2) savedBalls[1] = detectedColor;
            if (spindexerIndex == 4) savedBalls[2] = detectedColor;

            // Clear ball positions when transferring
            if (tranOn && flyOn) {
                if (spindexerIndex == 1) savedBalls[2] = 0;
                if (spindexerIndex == 3) savedBalls[0] = 0;
                if (spindexerIndex == 5) savedBalls[1] = 0;
            }

            telemetry.addData("Saved Balls", "0: %1c, 1: %1c, 2: %1c", savedBalls[0], savedBalls[1], savedBalls[2]);
            //endregion

            //region FLYWHEEL CONTROL
            // Manual Speed Adjustment
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyOffset += 10;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyOffset -= 10;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3) {
                flyOffset = 0;
                hoodOffset = 0;
            }


            // Flywheel Toggle
            if (gamepad2.crossWasPressed()) {
                flyOn = !flyOn;
            }

            // Voltage Compensation
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0 / 2450.0;
            double compensatedF = baseF * (13.0 / voltage);
            //Set custom PID values
            fly1.setVelocityPIDFCoefficients(flyKp, flyKi+flyKiOffset, flyKd, compensatedF);
            fly2.setVelocityPIDFCoefficients(flyKp, flyKi+flyKiOffset, flyKd, compensatedF);

            // Set Flywheel Velocity
            if (flyOn) {
                fly1.setVelocity(flySpeed + flyOffset);
                fly2.setVelocity(flySpeed + flyOffset);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            // Check if Flywheel at Speed
            flyAtSpeed = (flySpeed - fly1.getVelocity() < 50) && (flySpeed - fly1.getVelocity() > -50) &&
                    (flySpeed - fly2.getVelocity() < 50) && (flySpeed - fly2.getVelocity() > -50);

            // Update LED Indicator
            if (!flyOn) {
                led.setPosition(1); // white
            } else if (flyAtSpeed) {
                led.setPosition(0.5); // blue
            } else {
                led.setPosition(0.3); // red (ish)
            }
            //endregion

            //region HOOD CONTROL
            // Hood Position Selection
            if (gamepad1.dpadUpWasPressed()) {
                hoodOffset += 5;
            }
            if (gamepad1.dpadDownWasPressed()) {
                hoodOffset -= 5;

            }

            // Update Hood PID
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0 / 50.0; // fallback
            //(angles must be negative for our direction)
            updateHoodPID(hoodAngle + hoodOffset, dtSec);
            //endregion

            //region INTAKE CONTROL
            if (gamepad1.rightBumperWasPressed()) {
                intakePower = 1;
                intakeOn = !intakeOn;
            }

            // Outtake
            if (gamepad1.leftBumperWasPressed()) {
                intakePower = -0.6;
            }

            if (intakeOn) {
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0);
            }
            //endregion

            //region TRANSFER CONTROL
            if (gamepad2.triangleWasPressed()) {
                tranOn = !tranOn;
            }
            if (tranOn && flyOn) {
                trans.setPower(1);
            } else {
                trans.setPower(0);
            }
            //endregion

            //region SPINDEXER CONTROL
            // Spindexer Navigation
            //Left and Right go to intake positions, aka the odd numbered indices on the pos array
            if (gamepad2.dpadLeftWasPressed()) {
                spinCounterClock();
            }
            if (gamepad2.dpadRightWasPressed()) {
                spinClock();
            }
            //intake positions are the even ones
            if (gamepad2.dpadUpWasPressed()) {
                autoShot = true;
                autoShootNum = 3;
            }

            if (autoShot && autoShootNum > 0 && (runtime.milliseconds() - autoShootTime > 350)) {
                spinClock();
                autoShootNum--;
                autoShootTime = runtime.milliseconds();
            }

            if (autoShootNum <= 0) {
                autoShot = false;
            }
            if (gamepad2.dpadDownWasPressed()) {
                prevSpindexerIndex = spindexerIndex;
                spindexerIndex += spindexerIndex % 2 == 0 ? 1 : 0;
                spindexerIndex = (spindexerIndex + 2) % SPINDEXER_POSITIONS.length;
                spinBallLED();
            }

            // Update Spindexer PID
            double targetAngle = SPINDEXER_POSITIONS[spindexerIndex];
            updateSpindexerPID(targetAngle, dtSec);
            //endregion

            //region GOAL TRACKING
            double dt = Math.max(pidTimer.seconds(), 1e-3);
            pidTimer.reset();

// How aggressively the turret target chases camera error (deg/sec per deg error)
            final double K_TRACK = 6.0;     // main tuning knob
            final double MAX_TRACK_RATE = 240.0;  // absolute max deg/sec

            if (targetFound) {

                // Camera is upside down, so reverse bearing from AprilTag
                double rawBearing = -desiredTag.ftcPose.bearing;

                lastKnownBearing = rawBearing;
                lastDetectionTime = System.currentTimeMillis();

                // --- Single low-pass filter to smooth AprilTag noise ---
                double alpha = 0.3;  // 0–1, higher = faster but noisier
                filteredHeadingError += alpha * (rawBearing - filteredHeadingError);

                // Use the filtered error
                // Use the filtered error
                double trackingError = filteredHeadingError;

                if (Math.abs(trackingError) < 1.5) {
                    trackingError = 0.0;
                }

// -------- TURN FEEDFORWARD --------
// How fast the robot can spin at full stick (guess)
                final double ROBOT_MAX_YAW_RATE = 180.0; // deg/sec at full right stick

// How strongly turret should react to that spin (tune this)
                final double kTurnFF = 0.8; // start around 0.5–1.0

// Positive turnInput = robot turning one way -> turret must compensate opposite.
// The minus sign here is a guess; we’ll test & flip if needed.
                double ffRate = -kTurnFF * turnInput * ROBOT_MAX_YAW_RATE;  // deg/sec
// ----------------------------------

// Desired turret target *angular speed* based on camera + driver turn
                double desiredRate = K_TRACK * trackingError + ffRate;

                desiredRate = Range.clip(desiredRate, -MAX_TRACK_RATE, MAX_TRACK_RATE);

                double step = desiredRate * dt;
                tuPos += step;


                // Clamp turret target to allowed range
                tuPos = Range.clip(tuPos, -180, 180);

                telemetry.addData("Tracking", "LIVE err=%.1f° rate=%.1f°/s target=%.1f°",
                        trackingError, desiredRate, tuPos);
            }
            else {
                // No tag: hold last target, don't keep walking and don't fight the PID
                telemetry.addData("Tracking", "LOST");
            }
// Manual control
            turn = turnInput;

//endregion

            //region ENDGAME NAVIGATION
            if (gamepad1.dpadLeftWasPressed() && gamepad1.circleWasPressed() && !follower.isBusy() && localizeApril) {
                endgame = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), endgamePose))
                        .setLinearHeadingInterpolation(follower.getHeading(), endgamePose.getHeading())
                        .build();
                follower.followPath(endgame, true);
            }
            if (gamepad1.circleWasPressed() && !follower.isBusy() && !localizeApril) {
                follower.breakFollowing();
                localizeApril = true;
            }
            if (endgame != null && !follower.isBusy()) {
                localizeApril = false;
            }
            //endregion

            //region TURRET CONTROl

            if (gamepad1.right_trigger > 0.5 && runtime.milliseconds() - lastPidTime > 200) {
                tuKi += 0.0002;
                lastPidTime = runtime.milliseconds();
            }
            if (gamepad1.left_trigger > 0.5 && runtime.milliseconds() - lastPidTime > 200) {
                tuKi -= 0.0002;
                lastPidTime = runtime.milliseconds();
            }

            updateTurretPID(tuPos, dtSec);
            //endregion

            //region DRIVE CONTROL
            // Get manual drive inputs
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;

            // Apply drive commands when not path following
            if (!follower.isBusy()) {
                moveRobot(drive, strafe, turn);
            }
            //endregion

            telemetry.addData("Flywheel Speed", "%.0f", flySpeed + flyOffset);
            telemetry.update();
        }
    }


    //region HELPER METHODS
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    public void spinClock() {
        prevSpindexerIndex = spindexerIndex;
        spindexerIndex += spindexerIndex % 2 != 0 ? 1 : 0;
        spindexerIndex = (spindexerIndex - 2 + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
        spinBallLED();
    }
    public void spinCounterClock() {
        prevSpindexerIndex = spindexerIndex;
        spindexerIndex += spindexerIndex % 2 != 0 ? 1 : 0;
        spindexerIndex = (spindexerIndex + 2) % SPINDEXER_POSITIONS.length;
        spinBallLED();
    }

    private void updateSpindexerPID(double targetAngle, double dt) {
        double ccwOffset = -6.0;
        // read angles 0..360
        double angle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        //raw error
        double rawError = -angleError(targetAngle, angle);

        //adds a constant term if it's in a certain direction.
        // we either do this or we change the pid values for each direction.
        // gonna try and see if simpler method works tho
        double compensatedTarget = targetAngle;
        if (rawError < 0) { // moving CCW
            compensatedTarget = (targetAngle + ccwOffset) % 360.0;
        }
        // compute shortest signed error [-180,180]
        double error = -angleError(compensatedTarget, angle);

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double d = (error - lastError) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double out = pidKp * error + pidKi * integral + pidKd * d;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += pidKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= positionToleranceDeg) {
            out = 0.0;
            integral *= 0.2;
        }

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        spin1.setPower(out);
        spin2.setPower(out);

        // store errors for next derivative calculation
        lastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Spindexer Target", "%.1f°", targetAngle);

    }

    private void updateTurretPID(double targetAngle, double dt) {
        // read angles 0..360
        double angle = mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29);

        //raw error
        double rawError = -angleError(targetAngle, angle);

        //adds a constant term if it's in a certain direction.
        // we either do this or we change the pid values for each direction.
        // gonna try and see if simpler method works tho
        double compensatedTarget = targetAngle;
        if (rawError < 0) { // moving CCW
            compensatedTarget = (targetAngle) % 360.0;
        }
        // compute shortest signed error [-180,180]
        double error = -angleError(compensatedTarget, angle);

        // integral with anti-windup
        tuIntegral += error * dt;
        tuIntegral = clamp(tuIntegral, -tuIntegralLimit, tuIntegralLimit);

        // derivative
        double d = (error - tuLastError) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double out = tuKp * error + tuKi * tuIntegral + tuKd * d;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += tuKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < tuDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= tuToleranceDeg) {
            out = 0.0;
            tuIntegral *= 0.2;
        }

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        turret1.setPower(out);
        turret2.setPower(out);

        // store errors for next derivative calculation
        tuLastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Turret Target", "%.1f°", targetAngle);

    }

    private void updateHoodPID(double targetAngle, double dt) {
        // Read hood angle from encoder (0..360)
        double angle = mapVoltageToAngle360(hoodEncoder.getVoltage(), 0.01, 3.29);

        // Compute shortest signed error [-180,180]
        double error = -angleError(targetAngle, angle);

        // Integral with anti-windup
        hoodIntegral += error * dt;
        hoodIntegral = clamp(hoodIntegral, -hoodIntegralLimit, hoodIntegralLimit);

        // Derivative
        double d = (error - hoodLastError) / Math.max(dt, 1e-6);

        // PIDF output
        double out = hoodKp * error + hoodKi * hoodIntegral + hoodKd * d;

        // Feedforward to overcome stiction
        if (Math.abs(error) > 1.0) {
            out += hoodKf * Math.signum(error);
        }

        // Clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < hoodOutputDeadband) out = 0.0;

        // If within tolerance, zero output and decay integrator
        if (Math.abs(error) <= hoodToleranceDeg) {
            out = 0.0;
            hoodIntegral *= 0.2;
        }

        // Apply power to hood servo
        hood.setPower(out);

        // Store error for next iteration
        hoodLastError = error;

        // Telemetry
        telemetry.addData("Hood Target", "%.1f°", targetAngle);
        telemetry.addData("Hood Actual", "%.1f°", angle);
        telemetry.addData("Hood Error", "%.1f°", error);
        telemetry.addData("Hood Power", "%.3f", out);
    }

    private void spinBallLED(){
        if(spindexerIndex % 2 == 0){
            int avgIndex = (spindexerIndex + prevSpindexerIndex)/ 2;
            if (avgIndex == 2) avgIndex = 5;
            int ballIndex = 0;

            if(avgIndex==1) ballIndex=2;
//            else if(avgIndex==3) ballIndex=0;
            else if(avgIndex==5) ballIndex=1;

            ballIndex = (ballIndex - 1 + 3) % 3;

            if(savedBalls[ballIndex]=='p') {
                gamepad1.setLedColor(128,0,128,2000); //purple
                gamepad2.setLedColor(128,0,128,2000); //purple
            }
            else if(savedBalls[ballIndex]=='g') {
                gamepad1.setLedColor(0,128,0,2000); //green
                gamepad2.setLedColor(0,128,0,2000); //green
            }
            else {
                gamepad1.setLedColor(255,255,255,2000); //white
                gamepad2.setLedColor(255,255,255,2000); //white
            }
        }
    }

    private char getDetectedColor(){
        NormalizedRGBA colors = color.getNormalizedColors();
        float nRed = colors.red/colors.alpha;
        float nGreen = colors.green/colors.alpha;
        float nBlue = colors.blue/colors.alpha;

        if (colors.alpha < 0.15) {
            // If proximity is too low, the slot is empty or the sensor is moving between slots.
            return 'n';
        }

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
    }

    private boolean isBallPresent() {
        NormalizedRGBA colors = color.getNormalizedColors();

        return colors.alpha > 0.15;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
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

    // Linear interpolation helper method
    private double interpolate(double x, double[] xValues, double[] yValues) {
        // Clamp to table bounds
        if (x <= xValues[0]) return yValues[0];
        if (x >= xValues[xValues.length - 1]) return yValues[yValues.length - 1];

        // Find surrounding points
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {
                // Linear interpolation formula
                double t = (x - xValues[i]) / (xValues[i + 1] - xValues[i]);
                return yValues[i] + t * (yValues[i + 1] - yValues[i]);
            }
        }
        return yValues[yValues.length - 1]; // fallback
    }

    private double smooth(double newValue, double previousValue) {
        return ALPHA * newValue + (1 - ALPHA) * previousValue;
    }

    //Webcam methods
    private void initAprilTag() {

        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                0, //x, right +, left -
                4, //y, forward +, back -
                12.5, //z up + down -
                0
        );

        YawPitchRollAngles orientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                0, //yaw, left and right
                -70, //pitch, forward, back
                180, //roll, orientation
                0
        );
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, orientation)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(904.848699568, 904.848699568, 658.131998572, 340.91602987)

                .build();

        aprilTag.setDecimation(4);

        // Create the vision portal by using a builder.
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

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
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

    private void adjustDecimation(double range) {
        int newDecimation;

        if (range > 90) {
            newDecimation = 3;
        } else if (range > 50) {
            newDecimation = 3;
        } else {
            newDecimation = 4;
        }

        aprilTag.setDecimation(newDecimation);
        telemetry.addData("Decimation: ", "%d", newDecimation);

    }
    //endregion
}
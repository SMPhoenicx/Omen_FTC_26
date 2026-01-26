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

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MainBlueOpMode", group = "A")
public class MainBlueOpMode extends LinearOpMode
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
    private Servo hood = null;
    private CRServo spin1 = null;
    private CRServo spin2 = null;
    private CRServo turret1 = null;
    private CRServo turret2 = null;

    // Sensors
    private AnalogInput spinEncoder;
    private AnalogInput turretEncoder;
    private NormalizedColorSensor color1 = null;
    private NormalizedColorSensor color2 = null;
    private GoBildaPinpointDriver pinpoint = null;
    //endregion

    //region VISION SYSTEM
    // AprilTag Configuration
    private AprilTagDetection desiredTag;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Vision-based turret correction & Tuning
    private double visionCorrectionDeg = 0.0;
    private static final double VISION_CORRECTION_GAIN = 0.1;
    private static final double VISION_P_GAIN = 0.3;
    private static final double MAX_VISION_CORRECTION_DEG = 10.0;
    private static final double VISION_MIN_RANGE = 20.0;
    private static final double VISION_MAX_RANGE = 120.0;
    //endregion

    //region HOOD SYSTEM
    // Hood Positions
    private double hoodAngle = 0;
    private double hoodOffset = 0;
    //endregion

    //region SPINDEXER SYSTEM
    // Spindexer PIDF Constants
    private double pidKp = 0.006;
    private double pidKi = 0.0;
    private double pidKd = 0.000425;
    private double pidKf = 0.006;

    // Spindexer PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 300.0;
    private double pidLastTimeMs = 0.0;
    private double lastFilteredD = 0.0;

    // Spindexer Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;
    private boolean spindexerOverride = false;
    private double overrideTime = 0.0;

    // Spindexer Positions
    private final double[] SPINDEXER_POSITIONS = {49.75, 79.75, 109.75, 139.75, 169.75, 19.75};
    private int spindexerIndex = 0;
    private int prevSpindexerIndex = 0;
    private int greenPos = 0;

    // Ball Storage Tracking
    private char[] savedBalls = {'n', 'n', 'n'};
    private boolean[] presentBalls = {false, false, false};

    private boolean spindexerPidArmed = false;

    //endregion

    //region TURRET SYSTEM
    // PIDF Constants
    private double tuKp = 0.0050;
    private double tuKi = 0.0006;
    private double tuKd = 0.00014;
    private double tuKf = 0.005;
    private static final double tuKv = 0.00;

    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;

    // PID State
    private double tuIntegral = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Control Parameters
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.02;

    // Turret Position
    private double tuPos = 0.0;
    private static final double turretZeroDeg = 7;
    private static final double TURRET_LIMIT_DEG = 150.0;
    private double tuOffset = 0.0;
    //endregion

    //region SHOOTING SYSTEM
    private FlywheelPIDController flywheel;
    private double flyTargetTicksPerSec = 0.0;

    private static final double[] CAM_RANGE_SAMPLES =   {25, 31.8, 37, 39.2, 44.2,  52.6, 53.1, 56.9, 61.5, 65.6, 70.3, 73.4, 77.5, 84.3, 91.8, 100.4, 110.0, 118.4};
    private static final double[] ODOM_RANGE_SAMPLES =  {45.2, 50.2, 55.3, 60.9, 66.5, 72.2, 76.7, 81.1, 86.3, 90.9, 96.2, 99.7, 104.3, 109.9, 118.1, 128.5, 139.6, 148.7};
    private static final double[] FLY_SPEEDS =          {1005, 1026, 1059, 1083, 1129, 1143, 1155, 1162, 1219, 1251, 1261, 1267, 1256, 1283, 1297, 1370, 1393, 1420};
    private static final double[] HOOD_ANGLES = GlobalOffsets.globalHoodAngles;
    private double smoothedRange = 0;
    private static final double ALPHA = 0.8;
    private boolean flyHoodLock = false;

    // Auto Shooting State
    private int autoShootNum = 3;
    private double autoShootTime = 0;
    private boolean autoShot = false;
    private double lastTriggered = 0;
    boolean isRapidFire = false;
    double rapidFireStartTime = 0;
    //endregion

    //region NAVIGATION & LOCALIZATION
    private Follower follower;
    private PathChain endgame = null;

    private boolean trackingOn = false;
    private boolean hasManuallyLocalized = true;
    private boolean isInitialized = false;
    private double localizeTime = 0;
    //endregion

    //region VARIANT VARS (Alliance Specific)
    private static final double goalX = 0;
    private static final double goalY = 143;
    private static final int DESIRED_TAG_ID = 20; //blue=20, red=24
    private static final Pose LOCALIZE_POSE = new Pose(135, 8.9, Math.toRadians(0));
    private Pose endgamePose = new Pose(40, 33, Math.toRadians(90));
    private static final double TAG_X = 14.4;
    private static final double TAG_Y = 129.0;
    //endregion

    @Override
    public void runOpMode() {
        //region OPERATIONAL VARIABLES
        // Camera State
        boolean targetFound = false;

        // Mechanism States
        boolean tranOn = false;
        boolean intakeOn = false;
        boolean flyOn = false;
        boolean flyAtSpeed = false;
        boolean prevflyState = false;

        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Flywheel Control
        double flySpeed = 1100;
        double flyOffset = 0;
        double lastTime = 0;

        // Localization
        boolean localizeApril = true;
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
        hood = hardwareMap.get(Servo.class, "hood");
        turret1 = hardwareMap.get(CRServo.class, "tu1");
        turret2 = hardwareMap.get(CRServo.class, "tu2");

        // Initialize Encoders
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin1");
        turretEncoder = hardwareMap.get(AnalogInput.class, "tuen");

        // Initialize Sensors
        color1 = hardwareMap.get(NormalizedColorSensor.class, "Color 1");
        color2 = hardwareMap.get(NormalizedColorSensor.class, "Color 2");

        // Hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Set Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        trans.setDirection(DcMotor.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.FORWARD);
        spin2.setDirection(CRServo.Direction.FORWARD);
        turret1.setDirection(CRServo.Direction.FORWARD);
        turret2.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.REVERSE);
        //endregion
        flywheel = new FlywheelPIDController(
              hardwareMap.get(DcMotorEx.class, "fly1"),
             hardwareMap.get(DcMotorEx.class, "fly2")
        );
        flywheel.teleopMultiplier = 0.88;


        //region PRE-START
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StateVars.lastPose);

        initAprilTag();
        setManualExposure(4, 200);

        //TODO check if pattern works
        int patternTag = StateVars.patternTagID;
        if (patternTag == 21) {
            greenPos = 0;
        } else if (patternTag == 22) {
            greenPos = 1;
        } else if (patternTag == 23) {
            greenPos = 2;
        }


        telemetry.update();
        waitForStart();
        runtime.reset();
        //endregion

        while (opModeIsActive()) {
            //region IMPORTANT VARS
            //needed at beginning of loop, don't change location
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            pidLastTimeMs = nowMs;

            if (dtSec <= 0.0) dtSec = 1.0 / 50.0;

            double turnInput = -gamepad1.right_stick_x;

            follower.update();
            Pose robotPose = follower.getPose();
            //endregion

            //region TRACKING AND LOCALIZATION
            if (gamepad1.squareWasPressed()) {
                // Reset localization
                hasManuallyLocalized = false;
                localizeTime = runtime.milliseconds();

                tuPos = turretZeroDeg;
                tuIntegral = 0.0;
                tuLastError = 0.0;
                lastTuTargetInit = false;
                visionCorrectionDeg = 0.0;
            }

            if (gamepad1.leftBumperWasPressed()) {
                trackingOn = !trackingOn;

                tuIntegral = 0.0;
                tuLastError = 0.0;
                lastTuTargetInit = false;
                visionCorrectionDeg = 0.0;
            }

            if (!hasManuallyLocalized &&
                    runtime.milliseconds() - localizeTime > 300) {

                follower.setPose(LOCALIZE_POSE);
                hasManuallyLocalized = true;

                tuIntegral = 0.0;
                tuLastError = 0.0;
                lastTuTargetInit = false;
                isInitialized = false;
                visionCorrectionDeg = 0.0;
            }

            //endregion

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
                    }
                }
            }
            // Process Target Data
            if (targetFound) {
                double tagRange = desiredTag.ftcPose.range;
                adjustDecimation(tagRange);

                telemetry.addData("Target ID", desiredTag.id);
                telemetry.addData("Tag Range", "%.1f inches", tagRange);
            }
            //endregion

            //region VISION-BASED TURRET CORRECTION
            if (trackingOn && targetFound && desiredTag != null) {

                double range = desiredTag.ftcPose.range;

                // Gate by range to ensure measurements are credible
                if (range > VISION_MIN_RANGE && range < VISION_MAX_RANGE) {
                    double rx = robotPose.getX();
                    double ry = robotPose.getY();

                    double angleToTagRad  = Math.atan2(TAG_Y - ry, TAG_X - rx);
                    double angleToGoalRad = Math.atan2(goalY - ry, goalX - rx);

                    double angleToTagDeg  = Math.toDegrees(angleToTagRad);
                    double angleToGoalDeg = Math.toDegrees(angleToGoalRad);

                    double angleDiffDeg = normalizeDeg180(angleToGoalDeg - angleToTagDeg);
                    double measuredTagBearingDeg = -desiredTag.ftcPose.bearing;

                    double correctedBearingToAimDeg = measuredTagBearingDeg + angleDiffDeg;

                    //    This amplifies correction if robot is far from the diagonal.
                    double signedDist = (rx + ry) - 144.0; // >0 means y > 144 - x (your "right" side)
                    double maxDistForScale = 120.0; // max expected magnitude (tune)
                    double kScale = 0.5; // scaling aggressiveness (tune 0.0..2.0). 0 = no extra scaling
                    double lateralScale = 1.0 + kScale * (Math.max(-maxDistForScale, Math.min(maxDistForScale, signedDist)) / maxDistForScale);
                    // Clamp so scale stays reasonable:
                    lateralScale = Math.max(0.5, Math.min(2.0, lateralScale));

                   double scaledCorrectedBearingDeg = correctedBearingToAimDeg * lateralScale;

                    // Proportional assist
                    double pAssist = Math.max(
                            -2.5,
                            Math.min(2.5, VISION_P_GAIN * scaledCorrectedBearingDeg)
                    );

                    // Integral correction
                    visionCorrectionDeg += VISION_CORRECTION_GAIN * scaledCorrectedBearingDeg;

                    visionCorrectionDeg += pAssist;

                    // Hard clamp to avoid overcorrection
                    visionCorrectionDeg = Math.max(-MAX_VISION_CORRECTION_DEG, Math.min(MAX_VISION_CORRECTION_DEG, visionCorrectionDeg));
                }
            } else {
                // Slow decay when tag not visible so stale corrections fade
                visionCorrectionDeg *= 0.995;
            }
            //endregion

            //region AUTO FLYSPEED/ANGLE
            //dist calc from goal to bot
            double dx = goalX - robotPose.getX();
            double dy = goalY - robotPose.getY();
            double odomRange = Math.hypot(dx, dy);

            //smooth range so values rnt erratic
            if (!isInitialized) {
                smoothedRange = odomRange;
                isInitialized = true;
            } else {
                smoothedRange = smooth(odomRange, smoothedRange);
            }

            // interpolate between measured values
            if (!flyHoodLock) {
                flySpeed = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, FLY_SPEEDS);
                hoodAngle = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, HOOD_ANGLES);
                hoodAngle = Math.max(hoodAngle, -140); //clamp to prevent it going too high
            }

            telemetry.addData("Odom Range", "%.1f inches", smoothedRange);
            //endregion

            //region COLOR SENSOR AND BALL TRACKING
            char detectedColor = getRealColor();
            boolean present = isBallPresent();

            //start detection when spindexer has reached rest position
            if (Math.abs(lastError) < 12) {
                setBallAtIndex(spindexerIndex, detectedColor, present);
            }

            telemetry.addData("Saved Balls", "0: %1c, 1: %1c, 2: %1c", savedBalls[0], savedBalls[1], savedBalls[2]);
            telemetry.addData("PRESNT Balls", "0: %b, 1: %b, 2: %b", presentBalls[0], presentBalls[1], presentBalls[2]);
            //endregion

            //region FLYWHEEL CONTROL
            // manual speed adjust and reset all adjustmentsss
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyOffset += 3;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyOffset -= 3;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3) {
                flyOffset = 0;
                hoodOffset = 0;
            }

            if (gamepad1.crossWasPressed()) {
                flyOn = !flyOn;
            }

            if (flyOn) {
                flyTargetTicksPerSec = flySpeed + flyOffset;
            } else {
                flyTargetTicksPerSec = 0.0;
            }

            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            flywheel.updateFlywheelPID(
                    flyTargetTicksPerSec,
                    dtSec,
                    voltage
            );

            // check if flywheel is at speed
            flyAtSpeed = Math.abs(flyTargetTicksPerSec - flywheel.lastMeasuredVelocity) < 50;

            // update LED & rumble
            if (intakeOn && !hasEmptySlot()) {
                led.setPosition(0.6);
            }
            else if (!flyOn) {
                led.setPosition(1); // white
            } else if (flyAtSpeed) {
                if (prevflyState != flyAtSpeed) {
                    gamepad1.rumble(300);
                }
                led.setPosition(0.5); // green
            } else {
                led.setPosition(0.3); // red
            }
            prevflyState = flyAtSpeed;
            //endregion

            //region HOOD CONTROL
            // Hood Position Selection
            if (gamepad2.dpadUpWasPressed()) {
                hoodOffset -= 5;
            }
            if (gamepad2.dpadDownWasPressed()) {
                hoodOffset += 5;
            }

            double recoilOffset = 0.0;

            double flyDiff = flyTargetTicksPerSec - flywheel.lastMeasuredVelocity;

            if ((flyOn && isRapidFire) && (Math.abs(flyDiff) > 10) && (Math.abs(flyDiff) < 50)) {
                recoilOffset = flyDiff*1.2;
            }

            double finalHoodAngle = clamp(hoodAngle + hoodOffset + recoilOffset, 26, 292.6);

            // Update Hood PID
            hood.setPosition((finalHoodAngle)/355.0);
            //endregion

            //region INTAKE CONTROL
            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
                tranOn = false;
            }

            if (intakeOn) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }
            //endregion

            //region SPINDEXER AND TRANSFER CONTROL
            // Spindexer Navigation
            //Left and Right go to intake positions, aka the odd numbered indices on the pos array
            if (gamepad1.dpadLeftWasPressed()) {
                spindexerOverride = true;
                overrideTime = runtime.milliseconds();
                spinCounterClock();
            }
            if (gamepad1.dpadRightWasPressed()) {
                spindexerOverride = true;
                overrideTime = runtime.milliseconds();
                spinClock();
            }
            if (spindexerOverride && runtime.milliseconds() - overrideTime > 1000) {
                spindexerOverride = false;
            }

            //Rapid Fire stuff
            if (gamepad1.dpadUpWasPressed()) {
                isRapidFire = true;
                rapidFireStartTime = runtime.milliseconds();
                tranOn = true;
            }

            if (isRapidFire) {
                boolean timeUp = (runtime.milliseconds() - rapidFireStartTime > 1200);

                if (timeUp || intakeOn || spindexerOverride) {
                    isRapidFire = false;
                    tranOn = false;
                    Arrays.fill(savedBalls, 'n');
                    Arrays.fill(presentBalls, false);
                    calculateNearestIndex();
                }
            }

            //Pattern sorting
            if (gamepad1.dpadDownWasPressed()) {
                tranOn = false;
                int greenIn=-1;
                for(int i=0;i<3;i++){
                    if(savedBalls[i]=='g'){
                        greenIn=i;
                    }
                }
                if(greenIn==-1){
                    for(int i=0;i<3;i++){
                        if(savedBalls[i]=='n'){
                            greenIn=i;
                        }
                    }
                }
                if(greenIn==-1) greenIn=0;

                int diff = (greenIn + greenPos) % 3;
                if(diff==0) spindexerIndex=4;
                else if(diff==1) spindexerIndex=0;
                else spindexerIndex=2;
            }

            //region TRANSFER CONTROL
            if (gamepad1.triangleWasPressed()) {
                tranOn = !tranOn;
            }
            if (tranOn && flyOn) {
                trans.setPower(1);
            } else {
                trans.setPower(0);
            }
            //endregion

            //spin to empty slot while intaking.
            if (intakeOn && !spindexerOverride) {
                int currentSlot = -1;
                // look for first empty slot in savedBalls
                for (int i = 0; i < savedBalls.length; i++) {
                    if (savedBalls[i] == 'n') {
                        currentSlot = i;
                        break;
                    }
                }
                if (currentSlot != -1) {
                    int targetIndex = currentSlot * 2;
                    if (spindexerIndex != targetIndex) {
                        prevSpindexerIndex = spindexerIndex;
                        spindexerIndex = targetIndex;
                    }
                }
            }
            if (isRapidFire) {
                spin1.setPower(0.93);
                spin2.setPower(0.93);

            } else {
                double targetAngle = SPINDEXER_POSITIONS[spindexerIndex];
                updateSpindexerPID(targetAngle + GlobalOffsets.spindexerOffset, dtSec);
            }
           //endregion

            //region GOAL TRACKING
            if (trackingOn) {
                if (!hasManuallyLocalized) {
                    tuPos = turretZeroDeg;
                }
                else {
                    tuPos = calcTuTarget(
                            robotPose.getX(),
                            robotPose.getY(),
                            robotPose.getHeading()
                    )
                            + tuOffset
                            + visionCorrectionDeg;

                }
            }
            //endregion

            //region ENDGAME NAVIGATION
            if (gamepad2.dpadLeftWasPressed() && gamepad2.circleWasPressed() && !follower.isBusy()) {
                endgame = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), endgamePose))
                        .setLinearHeadingInterpolation(follower.getHeading(), endgamePose.getHeading())
                        .build();
                follower.followPath(endgame, true);
            }
            if (gamepad2.circleWasPressed()) {
                follower.breakFollowing();
            }
            //endregion

            //region TURRET CONTROl
            if (gamepad2.dpadLeftWasPressed()) {
                tuOffset -= 5;
            }
            if (gamepad2.dpadRightWasPressed()) {
                tuOffset += 5;
            }

            //needs to stay right above the final calculations, otherwise will get overwritten
            if (!trackingOn) {
                //zeros position
                tuPos = normalizeDeg180(turretZeroDeg);
            }

            double rawTurretTargetDeg = tuPos;
            //wraps position
            double safeTurretTargetDeg = applyTurretLimitWithWrap(rawTurretTargetDeg);
            tuPos = safeTurretTargetDeg;

            double targetVelDegPerSec = 0.0;

            //feedforward
            if (!lastTuTargetInit) {
                lastTuTarget = safeTurretTargetDeg;
                lastTuTargetInit = true;
            } else if (trackingOn) {
                double dTarget = normalizeDeg180(safeTurretTargetDeg - lastTuTarget);
                targetVelDegPerSec = dTarget / Math.max(dtSec, 1e-3);
                if (voltage >= 12.8) {
                    targetVelDegPerSec += -turnInput * 300;
                }
                if (voltage < 12.8) {
                    targetVelDegPerSec += -turnInput * 260;
                }
                lastTuTarget = safeTurretTargetDeg;
            } else {
                // no FF when not tracking
                targetVelDegPerSec = 0.0;
                lastTuTarget = safeTurretTargetDeg;
            }

            updateTurretPIDWithTargetFF(tuPos, targetVelDegPerSec, dtSec);
            //endregion

            //region DRIVE CONTROL
            // Get manual drive inputs
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = turnInput;

            // drive when not following paths
            if (!follower.isBusy()) {
                moveRobot(drive, strafe, turn);
            }

            if (gamepad1.right_trigger > 0.5&& runtime.milliseconds() - lastTriggered > 150) {
                tuKf += 0.002;
                lastTriggered = runtime.milliseconds();
//                flyHoodLock = !flyHoodLock;
            }
            if (gamepad1.left_trigger > 0.5 && runtime.milliseconds() - lastTriggered > 150) {
                tuKf -= 0.002;
                lastTriggered = runtime.milliseconds();
                //flyHoodLock = !flyHoodLock;
            }
            //endregion

            telemetry.addData("Flywheel Speed", "%.0f", flySpeed + flyOffset);
            telemetry.addData("last position", StateVars.lastPose);
            telemetry.addData("pidKf", "%.7f", tuKf);

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

    //region SPINDEXER HELPERS
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
    private void updateSpindexerPID(double targetBaseAngle, double dt) {
        if (!spindexerPidArmed) {
            lastError = 0;
            lastFilteredD = 0;
            integral = 0;
            spin1.setPower(0);
            spin2.setPower(0);
            spindexerPidArmed = true;
            return;
        }
        double currentAngle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        // 1. Twin Target Logic
        double targetB = (targetBaseAngle + 180.0) % 360.0;

        double errorA = -angleError(targetBaseAngle, currentAngle);
        double errorB = -angleError(targetB, currentAngle);

        double finalError = (Math.abs(errorA) <= Math.abs(errorB)) ? errorA : errorB;

        // 2. Integral Zone (Prevents windup, fixes "stopping short")
        if (Math.abs(finalError) < 10.0) { // Only accumulate when close
            integral += finalError * dt;
        } else {
            integral = 0;
        }
        integral = Range.clip(integral, -integralLimit, integralLimit);

        // 3. Filtered Derivative (The Jitter Fixer)
        double rawD = (finalError - lastError) / Math.max(dt, 1e-6);
        // Low-pass filter: 80% old value, 20% new value.
        // This smooths out the analog encoder "flicker".
        double filteredD = (lastFilteredD * 0.8) + (rawD * 0.2);
        lastFilteredD = filteredD;

        // 4. Calculate PID Output
        double pOut = pidKp * finalError;
        double iOut = pidKi * integral;
        double dOut = pidKd * filteredD;

        // 5. Static Feedforward (The Friction Killer)
        // Applies a constant "kick" to break friction whenever we aren't at the target
        double fOut = 0;
        if (Math.abs(finalError) > positionToleranceDeg) {
            fOut = Math.signum(finalError) * pidKf;
        }

        double out = pOut + iOut + dOut + fOut;

        // 6. Output Management
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // Stop and decay integral when inside tolerance
        if (Math.abs(finalError) <= positionToleranceDeg) {
            out = 0.0;
            integral *= 0.2;
        }

        spin1.setPower(out);
        spin2.setPower(out);

        lastError = finalError;
        telemetry.addData("LAST ERROR", lastError);
    }

    public void calculateNearestIndex() {
        double currentAngle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        int bestIndex = spindexerIndex;
        double minAbsError = 360.0;

        for (int i = 0; i < SPINDEXER_POSITIONS.length; i++) {
            double baseTarget = SPINDEXER_POSITIONS[i] + GlobalOffsets.spindexerOffset;

            // 1. Check the Normal Target
            double errorNormal = Math.abs(angleError(baseTarget, currentAngle));

            // 2. Check the "Ghost" Target (180 degrees away)
            // Since gearing is 1:2, this is the same physical position
            double ghostTarget = baseTarget + 180.0;
            double errorGhost = Math.abs(angleError(ghostTarget, currentAngle));

            // Find which one is closer
            double localMinError = Math.min(errorNormal, errorGhost);

            // Compare against the global best found so far
            if (localMinError < minAbsError) {
                minAbsError = localMinError;
                bestIndex = i;
            }
        }

        // Update the index.
        // The PID loop will automatically handle choosing between
        // the Normal or Ghost target again in the next frame.
        spindexerIndex = bestIndex;
    }
    //endregion

    //region COLOR AND ARTIFACT RELATED
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

    private char getRealColor(){
        char c1 = getDetectedColor1(color1);
        char c2 = getDetectedColor2(color2);

        if(c1=='p'||c2=='p'){
            return 'p';
        }
        if(c1=='g'||c2=='g'){
            return 'g';
        }
        return 'n';
    }

    private char getDetectedColor1(NormalizedColorSensor sensor){
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        telemetry.addData("Distance X", dist);
        if (Double.isNaN(dist) || dist > GlobalOffsets.colorSensorDist1) {
            return 'n';
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors.alpha == 0) return 'n';
        float nRed = colors.red/colors.alpha;
        float nGreen = colors.green/colors.alpha;
        float nBlue = colors.blue/colors.alpha;

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
    }

    private char getDetectedColor2(NormalizedColorSensor sensor){
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        telemetry.addData("Distance Y", dist);
        if (Double.isNaN(dist) || dist > GlobalOffsets.colorSensorDist2) {
            return 'n';
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors.alpha == 0) return 'n';
        float nRed = colors.red/colors.alpha;
        float nGreen = colors.green/colors.alpha;
        float nBlue = colors.blue/colors.alpha;

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
    }

    private boolean isBallPresent() {
        double dist1 = ((DistanceSensor) color1).getDistance(DistanceUnit.CM);
        double dist2 = ((DistanceSensor) color2).getDistance(DistanceUnit.CM);

        NormalizedRGBA colors1 = color1.getNormalizedColors();
        NormalizedRGBA colors2 = color2.getNormalizedColors();

        boolean s1Detected = !Double.isNaN(dist1) && dist1 < GlobalOffsets.colorSensorDist1;
        boolean s2Detected = !Double.isNaN(dist2) && dist2 < GlobalOffsets.colorSensorDist2;

        if (colors1.alpha == 0) {
            s1Detected = false;
        }
        if (colors2.alpha == 0) {
            s2Detected = false;
        }
        return s1Detected || s2Detected;
    }
    //endregion

    //region GENERAL MATH METHODS
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
        return yValues[yValues.length - 1];
    }

    private double smooth(double newValue, double previousValue) {
        return ALPHA * newValue + (1 - ALPHA) * previousValue;
    }
    //endregion

    //region CAM HELPERS
    private void initAprilTag() {

        Position cameraPosition = new Position(
                DistanceUnit.INCH,
                0, //x, right +, left -
                5.5, //y, forward +, back -
                12.5, //z up + down -
                0
        );

        YawPitchRollAngles orientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                0, //yaw, left and right
                70, //pitch, forward, back
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
    private double normalizeDeg180(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }
    //endregion

    //region STORED ARTIFACTS
    private int slotFromIndex(int idx) {
        switch (idx) {
            case 1: return 2;
            case 3: return 0;
            case 5: return 1;
            default: return -1;
        }
    }

    private void clearBallAtIndex(int idx) {
        int slot = slotFromIndex(idx);
        if (slot != -1) {
            savedBalls[slot] = 'n';
            presentBalls[slot] = false;
        }
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
    //endregion

    //region TURRET AND LOCALIZATION

    private double calcTuTarget(double robotX, double robotY, double robotHeadingRad) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double headingToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading  = Math.toDegrees(robotHeadingRad);

        //actual turret angle needed
        double turretAngleReal = headingToGoal - robotHeading;

        //converts to angle servos need to turn to to achieve turret angle
        double servoAngle = turretZeroDeg + (2 * turretAngleReal);

        return normalizeDeg180(servoAngle);
    }

    //TODO check ff and calculations for this, make as fast as possible
    private void updateTurretPIDWithTargetFF(double targetAngle, double targetVelDegPerSec, double dt) {
        double angle = getTurretAngleDeg();

        double error = -angleError(targetAngle, angle);

        tuIntegral += error * dt;
        tuIntegral = clamp(tuIntegral, -tuIntegralLimit, tuIntegralLimit);

        double d = (error - tuLastError) / Math.max(dt, 1e-6);

        double out = tuKp * error + tuKi * tuIntegral + tuKd * d;

        // stiction FF
        if (Math.abs(error) > 1.0) out += tuKf * Math.signum(error);

        // target-rate FF (helps match d(turret)/d(target))
        out += tuKv * targetVelDegPerSec;

        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < tuDeadband) out = 0.0;

        if (Math.abs(error) <= tuToleranceDeg) {
            out = 0.0;
            tuIntegral *= 0.2;
        }

        turret1.setPower(out);
        turret2.setPower(out);

        tuLastError = error;

    }

    private double getTurretAngleDeg() {
        return normalizeDeg180(mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29));
    }

    //TODO make this wrap better
    private double applyTurretLimitWithWrap(double desiredDeg) {
        // Always reason in [-180, 180]
        desiredDeg = normalizeDeg180(desiredDeg);

        // Where the turret actually is right now (also [-180, 180])
        double currentDeg = getTurretAngleDeg();

        // Shortest signed rotation from current to desired (e.g. +20, -30, etc.)
        double errorToDesired = normalizeDeg180(desiredDeg - currentDeg);

        // "Ideal" next target if we perfectly matched desired in one step
        double candidateDeg = currentDeg + errorToDesired;

        // Hard safety clamp to keep off the wires
        return clamp(candidateDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
    }
    //endregion

    boolean hasEmptySlot() {
        for (char c : savedBalls) {
            if (c == 'n') return true;
        }
        return false;
    }
    //endregion
}
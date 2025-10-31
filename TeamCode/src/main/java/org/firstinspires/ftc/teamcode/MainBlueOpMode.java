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

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MainBlueOpMode", group = "A")
public class MainBlueOpMode extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    PathChain endgame = null;
    Pose endgamePose = new Pose(103,37.5,Math.toRadians(90));

    //region HARDWARE DECLARATIONS
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private Servo led = null;
    private Servo hood = null;
    private Servo trans = null;
    private CRServo spin = null;
    // MARK:- ENCODERS / pots
    private AnalogInput spinAnalog;

    // CAMERA VARS
    private static final int DESIRED_TAG_ID = 20;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;
    private boolean facingGoal = false;

    //CAM PREDICTIONS
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 500;

    private double lastHeadingError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN   =  0.02  ;
    final double MAX_AUTO_TURN  = 0.4;
    //endregion

    //region CAROUSEL PIDF STUFF
    private  double pidKp = 0.0001;    // start small, increase until responsive
    private  double pidKi = 0.0;  // tiny integral (if needed)
    private  double pidKd = 0.0;  // derivative to damp oscillation
    private  double pidKf = 0.05;    // small directional feedforward to overcome stiction

    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0; // clamp integral

    private double pidLastTimeMs = 0.0; // ms timestamp for PID dt
    // tolerance and deadband
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;
    // --- Carousel preset positions (6 presets, every 60 degrees) ---
    private final double[] CAROUSEL_POSITIONS = {0.0, 60.0, 120.0, 180.0, 240.0, 300.0};
    private int carouselIndex = 0;
    //endregion

    @Override public void runOpMode()
    {
        //region MAIN VARS
        //CAMERA VARS
        boolean targetFound     = false;
        boolean tranOn = false;

        //DRIVE VARS
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;

        //FLYWHEEL VARS
        double flySpeed = 0;
        boolean flyOn = false;
        boolean flyAtSpeed = false;
        double lastTime = 0;
        double transTime = 0;

        //APRIL TAG LOCALIZE
        boolean localizeApril = true;
        double aprilLocalizationTimeout=0;
        //endregion

        //region CONTROL VARS
        //GAMEPAD 1
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
        //GAMEPAD 2
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
        //endregion

        initAprilTag();

        //region HARDWARE INFO
        // HARDWARE MAPS
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");

        //SERVOS
        spin = hardwareMap.get(CRServo.class, "spin");
        led = hardwareMap.get(Servo.class,"led");
        hood = hardwareMap.get(Servo.class,"hood");
        trans =  hardwareMap.get(Servo.class,"t1");

        //ENCODERS
        spinAnalog = hardwareMap.get(AnalogInput.class, "espin");


        //TOGGLESERVO
        ToggleServo hoodt = new ToggleServo(hood,  new int[]{240, 255, 270, 285, 300}, Servo.Direction.FORWARD, 270);
//40, 1150, 270
        //50, 1200, 270
        //60, 1250, 270
        //70, 1350, 285
        //100 1450, 0.591

        //MODES
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //DIRECTIONS
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        trans.setDirection(Servo.Direction.REVERSE);
        spin.setDirection(CRServo.Direction.FORWARD);

        //endregion

        //FOLLOWER SHIT
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23,120,Math.toRadians(90)));//lowkey this pos doesnt matter

        //INIT ACTIONS
        setManualExposure(4, 200);  // Use low exposure time to reduce motion blur

        //INIT TELEMETRY
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        hood.setPosition(0);
        //WAIT
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            follower.update();
            pidLastTimeMs = runtime.milliseconds();

            double lastPAdjustTime = 0;
            double lastIAdjustTime = 0;
            double lastDAdjustTime = 0;

            //region CAMERA
            targetFound = false;
            desiredTag  = null;

            //like so localization averages if both goals are in view
            Pose cameraLocalize = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                        targetFound = true;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                    //LOCALIZATION STUFF
                    if (!detection.metadata.name.contains("Obelisk")&&localizeApril&&runtime.milliseconds()-aprilLocalizationTimeout>50) {
                        if(cameraLocalize==null) {
                            cameraLocalize = new Pose(detection.robotPose.getPosition().y + 72, -detection.robotPose.getPosition().x + 72, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                        }else{
                            cameraLocalize = new Pose((cameraLocalize.getX()+detection.robotPose.getPosition().y + 72)/2,(cameraLocalize.getY()+ -detection.robotPose.getPosition().x + 72)/2,(cameraLocalize.getHeading()+detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS))/2);
                        }
                        follower.setPose(cameraLocalize);
                        aprilLocalizationTimeout=runtime.milliseconds();
                    }
                }else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // DO WHEN CAMERA TRACKING
            //MAYBE MAKE THIS WHEN facingGoal BOOL IS TRUE?
            if (targetFound) {
                adjustDecimation(desiredTag.ftcPose.range);
                double range = desiredTag.ftcPose.range;

                if(range >= 67 ) {
                    hoodt.setIndex(3);
                }
                else {
                    hoodt.setIndex(2);
                }

                flySpeed = 5.47 * range + 933.0;

                telemetry.addData("\n>","HOLD Left-Bumper to Turn to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            }
            else {
                if(gamepad1.right_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                    flySpeed += 50;
                    lastTime = runtime.milliseconds();
                }
                if(gamepad1.left_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                    flySpeed -= (flySpeed > 0)? 50:0;
                    lastTime = runtime.milliseconds();
                }

                if(gamepad1.dpad_down && !down1Pressed) {
                    hoodt.toggleLeft();
                }

                if(gamepad1.dpad_up && !up1Pressed) {
                    hoodt.toggleRight();
                }
            }
            //endregion

            //region FLYWHEEL AND LIGHTS
            //FLYWHEEL CONTROLS
            if(gamepad1.a && !a1Pressed)  {
                flyOn = !flyOn;
            }

//            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//            double compensatedF = 12.0 * (13.0 / voltage);
//            fly1.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);
//            fly2.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);

            if(flyOn) {
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);
            }
            else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            //FLYWHEEL LED
            flyAtSpeed = (flySpeed - fly1.getVelocity() < 50)||(flySpeed - fly1.getVelocity() > -50)&&(flySpeed - fly2.getVelocity() < 50)&&(flySpeed - fly2.getVelocity() > -50);

            //INDICATOR LIGHT
            if(!flyOn){
                led.setPosition(1);//white
            }
            else if(flyAtSpeed){
                led.setPosition(0.5);//green
            }
            else{
                led.setPosition(0.3);//red (ish)
            }
            //endregion

            //region INTAKE
            if(gamepad1.right_bumper && !rb1Pressed) {
                if(intake.getPower() <= 0) intake.setPower(1);
                else intake.setPower(0);
            }
            //OUTTAKE
            if(gamepad1.left_bumper && !lb1Pressed) {
                intake.setPower(-0.6);
            }
            //endregion

            //region TRANSFER
            if(gamepad1.y && !y1Pressed) {
                trans.setPosition(1);
                transTime = runtime.milliseconds();
            }
            double timeChange = runtime.milliseconds() - transTime;
            if(timeChange >= 250) {
                trans.setPosition(0);
            }
            //endregion

            //region ADJUST CAROUSEL PIDF
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0/50.0; // fallback
            pidLastTimeMs = nowMs;

            // ENCODING FOR SERVOS
            double volt = spinAnalog.getVoltage();

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
            //endregion

            //region CAROUSEL
            if (gamepad1.dpad_right && !right1Pressed) {
                carouselIndex = (carouselIndex + 1) % CAROUSEL_POSITIONS.length;
            }
            if (gamepad1.dpad_left && !left1Pressed) {
                carouselIndex = (carouselIndex - 1 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
            }

            // always run PID towards the current selected preset while opMode active
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            updateCarouselPID(targetAngle, dtSec);
            //endregion

            //region FACE GOAL
            if (gamepad1.x && !x1Pressed){
                facingGoal = !facingGoal;
            }

            if (facingGoal) {
                if (targetFound) {
                    lastKnownBearing = desiredTag.ftcPose.bearing;
                    lastKnownRange = desiredTag.ftcPose.range;
                    lastDetectionTime = System.currentTimeMillis();

                    double headingError = desiredTag.ftcPose.bearing;

                    double deltaTime = pidTimer.seconds();
                    double derivative = (headingError - lastHeadingError) / deltaTime;
                    pidTimer.reset();

                    if (Math.abs(headingError) < 2.0) {
                        turn = 0;
                    } else {
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    }

                    lastHeadingError = headingError;

                    telemetry.addData("Tracking", "LIVE (err: %.1f°, deriv: %.2f)", headingError, derivative);
                }
                else {
                    //TRYING TO PREVENT A LOT OF TRACKING LOSS
                    long timeSinceLost = System.currentTimeMillis() - lastDetectionTime;

                    if (timeSinceLost < PREDICTION_TIMEOUT) {
                        // Continue tracking last known bearing
                        double headingError = lastKnownBearing;

                        double deltaTime = pidTimer.seconds();
                        double derivative = (headingError - lastHeadingError) / deltaTime;
                        pidTimer.reset();

                        if (Math.abs(headingError) < 2.0) {
                            turn = 0;
                        } else {
                            turn = (TURN_P * headingError) + (TURN_D * derivative);
                            turn = Range.clip(turn * -1, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        }

                        lastHeadingError = headingError;

                        telemetry.addData("Tracking", "PREDICTED (lost %dms ago)", timeSinceLost);
                    } else {
                        turn = 0;
                        lastHeadingError = 0;
                        pidTimer.reset();
                        telemetry.addData("Tracking", "LOST");
                    }
                }
            }
            else{
                turn   = -gamepad1.right_stick_x;
                lastHeadingError = 0;
                pidTimer.reset();
            }
            //endregion

            //region ENDGAME
            if(gamepad1.b&&!b1Pressed&&!follower.isBusy()&&localizeApril){
                endgame = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),endgamePose))
                        .setLinearHeadingInterpolation(follower.getHeading(),endgamePose.getHeading())
                        .build();
                follower.followPath(endgame,true);
            }
            if(gamepad1.b&&!b1Pressed&&!follower.isBusy()&&!localizeApril){
                follower.breakFollowing();
                localizeApril=true;
            }
            if(endgame!=null&&!follower.isBusy()){
                localizeApril = false;
            }
            //endregion

            //MANUAL
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;

            //DRIVE
            if(!follower.isBusy()){
                moveRobot(drive, strafe, turn);
            }

            //region CONTROL RESETS
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
            //endregion

            //region TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Fly state", flyOn);
            telemetry.addData("Fly power", flySpeed);
            telemetry.addData("Encoder fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.addData("Flying at correct power", flyAtSpeed);
            telemetry.addData("Hood angle:", "%.3f", hoodt.getServo().getPosition());
            telemetry.addData("Camera Localized Pos","x: %.2f y: %.2f heading: %.2f",follower.getPose().getX(),follower.getPose().getY(),Math.toDegrees(follower.getHeading()));
            telemetry.update();
            //endregion

            //hmmmm
            sleep(10);
        }
    }

    //HELPER METHODS
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

    private void updateCarouselPID(double targetAngle, double dt) {
        // read angles 0..360
        double angle = mapVoltageToAngle360(spinAnalog.getVoltage(), 0.01, 3.29);

        // compute shortest signed error [-180,180]
        double error = angleError(targetAngle, angle);

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
        spin.setPower(out);

        // store errors for next derivative calculation
        lastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Carousel Target", "%.1f°", targetAngle);
        telemetry.addData("spin", "angle=%.2f, err=%.2f, pwr=%.2f", angle, error, out);

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(904.848699568, 904.848699568, 658.131998572, 340.91602987)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
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
        // Wait for the camera to be open, then use the controls

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
}
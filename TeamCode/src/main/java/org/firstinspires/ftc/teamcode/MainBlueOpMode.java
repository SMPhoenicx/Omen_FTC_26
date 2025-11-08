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

    //region HARDWARE DECLARATIONS
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private DcMotor trans = null;
    private Servo led = null;
    private Servo hood = null;
    private CRServo spin1 = null;
    private CRServo spin2 = null;
    // ENCODERS
    private AnalogInput spinEncoder1;
    private AnalogInput spinEncoder2;
    //COLOR
    private NormalizedColorSensor color = null;

    //LIMELIGHT
    private Limelight3A limelight;
    //endregion

    //region CAMERA VARS
    private LLResultTypes.FiducialResult desiredTag;
    private static final int DESIRED_TAG_ID = 24;
    private boolean facingGoal = false;

    //CAM PREDICTIONS
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 150;

    private double lastHeadingError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN   =  0.02  ;
    final double MAX_AUTO_TURN  = 0.4;
    //endregion

    double flyKp = 10.0;
    double flyKi = 3.0;
    double flyKd = 0.5;

    double kpUp = 0.5;
    double kiUp = 0.005;
    double kdUp = 0.005;

    //region PEDROPATHING STUFF
    private Follower follower;
    PathChain endgame = null;
    Pose endgamePose = new Pose(103,37.5,Math.toRadians(90));
    //endregion

    //region CAROUSEL PIDF STUFF
    private  double pidKp = 0.0057;    // start small, increase until responsive
    private  double pidKi = 0.00166;  // tiny integral (if needed)
    private  double pidKd = 0.00002;  // derivative to damp oscillation
    private  double pidKf = 0.0;    // small directional feedforward to overcome stiction

    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0; // clamp integral

    private double pidLastTimeMs = 0.0; // ms timestamp for PID dt
    // tolerance and deadband
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;
    // --- Carousel preset positions (6 presets, every 60 degrees) ---
    private final double[] CAROUSEL_POSITIONS = {57.0, 117.0, 177.0, 237.0, 297.0, 357.0};
    //57, 177, and 297 are facing the intake, the others face the transfer
    private int carouselIndex = 0;
    private char[] savedBalls = {'n','n','n'};//n is none (empty), p is purple, g is green  --  side note i did not realize this says nnn when the carousels empty lmao
    //endregion

    @Override public void runOpMode()
    {
        //region MAIN VARS
        //CAMERA VARS
        boolean targetFound     = false;
        boolean tranOn = false;
        boolean intakeOn = false;

        //DRIVE VARS
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;

        //FLYWHEEL VARS
        double flySpeed = 1160;
        boolean flyOn = false;
        boolean flyAtSpeed = false;
        double lastTime = 0;
        double transTime = 0;

        //APRIL TAG LOCALIZE
        boolean localizeApril = true;
        double aprilLocalizationTimeout=0;

        //PIDF spin idk
        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;
        //endregion

//TEMPORARY
        double hoodPos = 245;
        //region HARDWARE INFO
        // HARDWARE MAPS
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class,"fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        trans = hardwareMap.get(DcMotor.class,"trans");

        //SERVOS
        spin1 = hardwareMap.get(CRServo.class, "spin1");
        spin2 = hardwareMap.get(CRServo.class, "spin2");
        led = hardwareMap.get(Servo.class,"led");
        hood = hardwareMap.get(Servo.class,"hood");
//        trans =  hardwareMap.get(Servo.class,"t1");

        //ENCODERS
        spinEncoder1 = hardwareMap.get(AnalogInput.class, "espin1");
        spinEncoder2 = hardwareMap.get(AnalogInput.class, "espin2");

        //COLOR SENSOR
        color = hardwareMap.get(NormalizedColorSensor.class,"Color 1");

        //TOGGLESERVO
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
        intake.setDirection(DcMotor.Direction.FORWARD);
        trans.setDirection(DcMotor.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.FORWARD);
        spin2.setDirection(CRServo.Direction.FORWARD);

        //endregion

        //FOLLOWER SHIT
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23,120,Math.toRadians(90)));//lowkey this pos doesnt matter

        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        //INIT TELEMETRY
        telemetry.update();

        hood.setPosition(0);
        //WAIT
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            follower.update();
            pidLastTimeMs = runtime.milliseconds();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                targetFound = false;
                desiredTag = null;

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == DESIRED_TAG_ID) {
                        desiredTag = fiducial;
                        targetFound = true;
                        break;
                    }
                }

            } else {
                telemetry.addData("Limelight", "No Targets");
            }


            // DO WHEN CAMERA TRACKING
            //MAYBE MAKE THIS WHEN facingGoal BOOL IS TRUE?
            if (targetFound && desiredTag != null)
            {
                //get all distances and angles
                double tx = desiredTag.getTargetXDegrees();      // horizontal offset in degrees
                double ty = desiredTag.getTargetYDegrees();      // vertical offset in degrees
                double ta = desiredTag.getTargetArea();          //area (0-100%)

                Pose3D targetPose = desiredTag.getTargetPoseRobotSpace();

                double x = targetPose.getPosition().x;
                double y = targetPose.getPosition().y;
                double z = targetPose.getPosition().z;

                //idk what's better to use
                double distMeters = Math.sqrt(x*x + y*y + z*z);
                double slantRange = distMeters * 39.3701;
                double range = x * 39.3701; //in inches

                //def have to change these
//                if(slantRange >= 67 ) {
//                    hoodt.setIndex(3);
//                }
//                else {
//                    hoodt.setIndex(2);
//                }
//
//                flySpeed = 5.47 * slantRange + 933.0;

                telemetry.addData("Target ID", desiredTag.getFiducialId());
                telemetry.addData("Distance", "%.1f inches", slantRange);
                telemetry.addData("TX (bearing)", "%.1f degrees", tx);
                telemetry.addData("Flywheel Speed", "%.0f", flySpeed);
            }

            if(gamepad1.right_trigger > 0 && (runtime.milliseconds() - lastTime > 100)) {
                flySpeed += 20;
                lastTime = runtime.milliseconds();
            }
            if(gamepad1.left_trigger > 0 && (runtime.milliseconds() - lastTime > 100)) {
                flySpeed -= (flySpeed > 0)? 20:0;
                lastTime = runtime.milliseconds();
            }

            if(gamepad2.dpadDownWasPressed()) {
                hoodPos += 5;
            }

            if(gamepad2.dpadUpWasPressed()) {
                hoodPos -= 5;
            }

            hood.setPosition(hoodPos/ 355.0);
            telemetry.addData("HOOD POSITION", hoodPos);

            //endregion

            //region FLYWHEEL AND LIGHTS
            //FLYWHEEL CONTROLS
            if(gamepad1.crossWasPressed())  {
                flyOn = !flyOn;
            }

            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double compensatedF = 12.0 * (13.0 / voltage);

            if(gamepad2.crossWasPressed()) {
                if (gamepad2.leftBumperWasPressed()) {
                    flyKp -= kpUp;
                }
                else flyKp += kpUp;
            }
            if(gamepad2.right_trigger > 0.2 || gamepad2.circleWasPressed()) {
                if (gamepad2.leftBumperWasPressed()) {
                    flyKi -= kiUp;
                }
                else flyKi += kiUp;
            }
            if(gamepad2.left_trigger > 0.2 || gamepad2.triangleWasPressed()) {
                if (gamepad2.leftBumperWasPressed()) {
                    flyKd -= kdUp;
                }
                else flyKd += kdUp;
            }

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0/2427.0;
            compensatedF = baseF * (13.0/voltage);
            fly1.setVelocityPIDFCoefficients(flyKp, flyKi, flyKd, compensatedF);
            fly2.setVelocityPIDFCoefficients(flyKp, flyKi, flyKd, compensatedF);

            telemetry.addData("FLYKP", "%.5f", flyKp);
            telemetry.addData("FLYKI", "%.5f", flyKi);
            telemetry.addData("FLYKD", "%.5f", flyKd);

            if(flyOn) {
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);
            }
            else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            //FLYWHEEL LED
            flyAtSpeed = (flySpeed - fly1.getVelocity() < 50)&&(flySpeed - fly1.getVelocity() > -50)&&(flySpeed - fly2.getVelocity() < 50)&&(flySpeed - fly2.getVelocity() > -50);

            //INDICATOR LIGHT
            if(!flyOn){
                led.setPosition(1);//white
            }
            else if(flyAtSpeed){
                led.setPosition(0.5);//blue
            }
            else{
                led.setPosition(0.3);//red (ish)
            }
            //endregion

            //region INTAKE
            if(gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
                if(intake.getPower() <= 0) intake.setPower(1);
                else intake.setPower(0);
            }
            //OUTTAKE
            if(gamepad1.leftBumperWasPressed()) {
                intake.setPower(-0.6);
            }
            //endregion

            //region TRANSFER
            if(gamepad1.triangleWasPressed()) {
                tranOn = !tranOn;
            }
            if(tranOn && flyOn){
                trans.setPower(1);
            }else{
                trans.setPower(0);
            }
            //endregion

            //region ADJUST CAROUSEL PID
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0/50.0; // fallback
            pidLastTimeMs = nowMs;




            //region CAROUSEL
            if (gamepad1.dpadLeftWasPressed()) {
                carouselIndex += carouselIndex % 2 != 0? 1:0;
                carouselIndex = (carouselIndex + 2) % CAROUSEL_POSITIONS.length;
                spinBallLED();
            }
            if (gamepad1.dpadRightWasPressed()) {
                carouselIndex += carouselIndex % 2 != 0? 1:0;
                carouselIndex = (carouselIndex - 2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                spinBallLED();
            }
            if (gamepad1.dpadUpWasPressed()) {
                carouselIndex += carouselIndex % 2 == 0? 1:0;
                carouselIndex = (carouselIndex - 2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                spinBallLED();
            }
            if (gamepad1.dpadDownWasPressed()) {
                carouselIndex += carouselIndex % 2 == 0? 1:0;
                carouselIndex = (carouselIndex + 2) % CAROUSEL_POSITIONS.length;
                spinBallLED();
            }

            // always run PID towards the current selected preset while opMode active
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            updateCarouselPID(targetAngle, dtSec);
            //endregion

            //region FACE GOAL
            if (gamepad1.squareWasPressed()){
                facingGoal = !facingGoal;
            }

            if (facingGoal) {

                //NOTE: MIGHT NOT NEED SMOOTHING OR AS MUCH SMOOTHING FOR LIMELIGHT
                // REDUCED TIMEOUT TO 150 ms
                if (targetFound && desiredTag != null) {
                    double tx = desiredTag.getTargetXDegrees();
                    lastKnownBearing = tx;
                    lastDetectionTime = System.currentTimeMillis();

                    double headingError = -tx;

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

            //region COLOR SENSOR AND SAVED BALL POSITIONS
            char detectedColor = getDetectedColor();

            if(carouselIndex==0) savedBalls[0] = detectedColor;
            if(carouselIndex==2) savedBalls[1] = detectedColor;
            if(carouselIndex==4) savedBalls[2] = detectedColor;

            if(tranOn){
                if(carouselIndex==1) savedBalls[2] = 0;
                if(carouselIndex==3) savedBalls[0] = 0;
                if(carouselIndex==5) savedBalls[1] = 0;
            }

            telemetry.addData("Saved Balls","0: %1c, 1: %1c, 2: %1c",savedBalls[0],savedBalls[1],savedBalls[2]);
            //endregion

            //region ENDGAME
            if(gamepad1.circleWasPressed() && !follower.isBusy() && localizeApril){
                endgame = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(),endgamePose))
                        .setLinearHeadingInterpolation(follower.getHeading(),endgamePose.getHeading())
                        .build();
                follower.followPath(endgame,true);
            }
            if(gamepad1.circleWasPressed()&&!follower.isBusy()&&!localizeApril){
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


            //region TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Fly state", flyOn);
            telemetry.addData("Fly power", flySpeed);
            telemetry.addData("Encoder fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.addData("Flying at correct power", flyAtSpeed);
            telemetry.addData("Camera Localized Pos","x: %.2f y: %.2f heading: %.2f",follower.getPose().getX(),follower.getPose().getY(),Math.toDegrees(follower.getHeading()));
            telemetry.update();
            //endregion
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

    private void updateCarouselPID(double targetAngle, double dt) {
        double ccwOffset = -6.0;
        // read angles 0..360
        double angle = mapVoltageToAngle360(spinEncoder1.getVoltage(), 0.01, 3.29);

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
        telemetry.addData("Carousel Target", "%.1f°", targetAngle);

    }

    private void spinBallLED(){
        if(carouselIndex % 2 == 1){
            int ballIndex=0;
//                int ballIndex = 0.375*(carouselIndex*carouselIndex)-2.5*carouselIndex+4.125;
            if(carouselIndex==1) ballIndex=2;
            else if(carouselIndex==3) ballIndex=0;
            else if(carouselIndex==5) ballIndex=1;

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

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
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
    //endregion
}
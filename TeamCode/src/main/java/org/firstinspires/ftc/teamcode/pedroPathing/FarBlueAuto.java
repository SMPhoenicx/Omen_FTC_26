package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Disabled
@Autonomous(name="Far Blue Auto", group="Robot")
public class FarBlueAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double timeout = 0;

    //region PEDRO VARS
    private Follower follower;
    private Pose startPose, shoot1, movePoint;
    private Pose[] pickup1 = new Pose[3];
    private Pose[] pickup2 = new Pose[3];
    private Pose[] pickup3 = new Pose[3];
    private PathChain scorePath0, scorePath1, scorePath2, scorePath3, moveScore;
    private PathChain[] pickupPath1 = new PathChain[2];
    private PathChain[] pickupPath2 = new PathChain[2];
    private PathChain[] pickupPath3 = new PathChain[2];
    //endregion

    //region HARDWARE DECLARATIONS
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private DcMotor trans = null;

    //LIMELIGHT
    private Limelight3A limelight;

    //SERVOS
    private CRServo spin1 = null;
    private CRServo spin2 = null;
    private Servo led = null;
    private CRServo hood = null;

    // ENCODERS
    private AnalogInput spinEncoder;
    private AnalogInput hoodEncoder;

    //COLOR
    private NormalizedColorSensor color = null;
    //endregion

    //region FLYWHEEL SYSTEM
    // Flywheel PID Constants
    double flyKp = 9.0;
    double flyKi = 1.15;
    double flyKd = 2.9;
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

    // Hood Tuning Adjustments
    private final double hoodKpUp = 0.005;
    private final double hoodKiUp = 0.00001;
    private final double hoodKdUp = 0.005;

    // Hood Positions
    private double hoodAngle = -154;
    private double hoodOffset = 0;
    //endregion

    //region CAROUSEL SYSTEM
    // Carousel PIDF Constants
    private double pidKp = 0.0057;
    private double pidKi = 0.00166;
    private double pidKd = 0.00002/1000;
    private double pidKf = 0.0;

    // Carousel PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;

    // Carousel Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;

    // Carousel Positions (6 presets, every 60 degrees)
    // 57, 177, and 297 face the intake; others face the transfer
    private final double[] CAROUSEL_POSITIONS = {57.0, 117.0, 177.0, 237.0, 297.0, 357.0};
    private int carouselIndex = 0;

    // Ball Storage Tracking
    // 'n' = none (empty), 'p' = purple, 'g' = green
    private char[] savedBalls = {'g', 'p', 'p'};
    int greenPos=0;
    //endregion

    public void createPoses(){
        startPose = new Pose(144-87,8,Math.toRadians(90));

        pickup1[0] = new Pose(144-95,82.5,Math.toRadians(180));
        pickup1[1] = new Pose(144-124,82.5,Math.toRadians(180));
        pickup1[2] = new Pose(144-121,82.5,Math.toRadians(180-45));

        pickup2[0] = new Pose(144-96,60.5,Math.toRadians(180));
        pickup2[1] = new Pose(144-128,60.5,Math.toRadians(180));
        pickup2[2] = new Pose(144-118,58.5,Math.toRadians(180-45));

        pickup3[0] = new Pose(144-96,36,Math.toRadians(180));
        pickup3[1] = new Pose(144-128,36,Math.toRadians(180));
        pickup3[2] = new Pose(144-118,34,Math.toRadians(180-45));

        shoot1 = new Pose(144-82,21,Math.toRadians(180-45));
        movePoint = new Pose(144-114,77,Math.toRadians(180-45));
    }

    public void createPaths(){
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0,144))
                .build();
        pickupPath1[0] = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup1[0]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup1[0].getHeading())
//                .setBrakingStrength(0.5)
                .build();
        pickupPath1[1] = follower.pathBuilder()
                .addPath(new BezierLine(pickup1[0],pickup1[1]))
                .setLinearHeadingInterpolation(pickup1[0].getHeading(),pickup1[1].getHeading())
//                .setBrakingStrength(0.5)
                .setTimeoutConstraint(1000)
                .build();
        pickupPath2[0] = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup2[0]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup2[0].getHeading())
//                .setBrakingStrength(0.5)
                .build();
        pickupPath2[1] = follower.pathBuilder()
                .addPath(new BezierLine(pickup2[0],pickup2[1]))
                .setLinearHeadingInterpolation(pickup2[0].getHeading(),pickup2[1].getHeading())
//                .setBrakingStrength(0.5)
                .setTimeoutConstraint(1000)
                .build();
        pickupPath3[0] = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup3[0]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup3[0].getHeading())
//                .setBrakingStrength(0.5)
                .build();
        pickupPath3[1] = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[0],pickup3[1]))
                .setLinearHeadingInterpolation(pickup3[0].getHeading(),pickup3[1].getHeading())
//                .setBrakingStrength(0.5)
                .setTimeoutConstraint(1000)
                .build();
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1[1],pickup1[2]))
                .setLinearHeadingInterpolation(pickup1[1].getHeading(),pickup1[2].getHeading())
                .addPath(new BezierLine(pickup1[2],shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0,144))
                .build();
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2[1],pickup2[2]))
                .setLinearHeadingInterpolation(pickup2[1].getHeading(),pickup2[2].getHeading())
                .addPath(new BezierLine(pickup2[2],shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144-136,144))
                .build();
        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],pickup3[2]))
                .setLinearHeadingInterpolation(pickup3[1].getHeading(),pickup3[2].getHeading())
                .addPath(new BezierLine(pickup3[2],shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144-140,144))
                .build();
        moveScore = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,movePoint))
                .setConstantHeadingInterpolation(movePoint.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //region MAIN VARS
        int pathState = 0;
        int shootingState = 4;
        int flySpeed = 0;
        boolean running = true;
        boolean transOn = false;
        int flySpeedTarget = 1595;
        //endregion

        //region HARDWARE INFO
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class,"fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        trans = hardwareMap.get(DcMotor.class,"trans");

        //SERVOS
        spin1 = hardwareMap.get(CRServo.class, "spin1");
        spin2 = hardwareMap.get(CRServo.class, "spin2");
        led = hardwareMap.get(Servo.class,"led");
        hood = hardwareMap.get(CRServo.class,"hood");
//        trans =  hardwareMap.get(Servo.class,"t1");

        //ENCODERS
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin1");
        hoodEncoder = hardwareMap.get(AnalogInput.class, "hooden");

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
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        trans.setDirection(DcMotor.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.FORWARD);
        spin2.setDirection(CRServo.Direction.FORWARD);

        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        //endregion

        //region INITIALIZE STUFF
        createPoses();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        createPaths();
        //endregion
        hoodOffset=0;

        //WAIT
        waitForStart();
        runtime.reset();
        pidLastTimeMs = runtime.milliseconds();

        while(opModeIsActive()){
            follower.update();
            pidLastTimeMs = runtime.milliseconds();

//            if(pathState>=9) pathState=500;

            //region PATH STUFF
            if(!follower.isBusy()&&runtime.milliseconds()>timeout){
                switch(pathState){
                    //region CYCLE ZERO (READ MOTIF)
                    case 0:
                        pathState++;
                        flySpeed = flySpeedTarget;
                        timeout = runtime.milliseconds()+1000;
                        break;
                    //CASE 1 is reading motif
                    case 2:
                        follower.followPath(scorePath0,true);
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 3 is shooting
                    //endregion

                    //region CYCLE ONE
                    case 4:
                        flySpeed = flySpeedTarget;
                        transOn=false;
                        intake.setPower(1);
                        follower.followPath(pickupPath3[0],true);
                        pathState++;
                        break;
                    case 5:
                        follower.followPath(pickupPath3[1],0.2,true);
                        pathState++;
                        break;
                    //CASE 6 is intaking
                    case 7:
                        follower.followPath(scorePath3,true);
                        transOn = true;
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 8 is shooting
                    //endregion

                    //region CYCLE TWO
                    case 9:
                        flySpeed = 0;
                        transOn=false;
                        intake.setPower(1);
                        follower.followPath(pickupPath2[0],true);
                        pathState++;
                        break;
                    case 10:
                        follower.followPath(pickupPath2[1],0.2,true);
                        pathState++;
                        break;
                    //CASE 11 is intaking
                    case 12:
                        follower.followPath(scorePath2,true);
                        flySpeed = flySpeedTarget;
                        transOn = true;
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 13 is shooting
                    //endregion

                    //region CYCLE THREE
                    case 14:
                        flySpeed = 0;
                        transOn=false;
                        intake.setPower(1);
                        follower.followPath(pickupPath1[0],true);
                        pathState++;
                        break;
                    case 15:
                        follower.followPath(pickupPath1[1],0.2,true);
                        pathState++;
                        break;
                    //CASE 16 is intaking
                    case 17:
                        follower.followPath(scorePath1,true);
                        flySpeed = flySpeedTarget;
                        transOn = true;
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 18 is shooting
                    //endregion

                    case 3:
                    case 8:
                    case 13:
                    case 18:
                        //region SHOOTING
                        intake.setPower(0);
                        if(shootingState==0){
                            double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;
                            if(avgSpeed < flySpeedTarget * 0.94) break;
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
                            if(diff==0) carouselIndex=4;
                            else if(diff==1) carouselIndex=0;
                            else carouselIndex=2;
                            timeout=runtime.milliseconds()+500;
                            shootingState++;
                        }
                        else if(shootingState==1){
                            transOn = true;
                            carouselIndex = (carouselIndex-2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                            timeout=runtime.milliseconds()+500;
                            shootingState++;
                        }
                        else if(shootingState==2){
                            carouselIndex = (carouselIndex-2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                            timeout=runtime.milliseconds()+500;
                            shootingState++;
                        }
                        else if(shootingState==3){
                            carouselIndex = (carouselIndex-2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                            shootingState++;
                            pathState++;
                            timeout=runtime.milliseconds()+500;
                            savedBalls[0]='n'; savedBalls[1]='n'; savedBalls[2]='n';
                        }
                        //endregion
                        break;

                    //end of auto
                    case 20:
                        transOn=false;
                        follower.followPath(moveScore);
                        pathState++;
                        break;
                    default:
                        running=false;
                        break;
                }


            }
            //endregion

            //region INTAKE
            if((pathState==6||pathState==11||pathState==16)&&runtime.milliseconds()>timeout){
                if(getDetectedColor()!='n'&&savedBalls[carouselIndex/2]=='n'){//detect one ball intake
                    savedBalls[carouselIndex/2]=getDetectedColor();
                    carouselIndex = (carouselIndex-2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                    timeout = runtime.milliseconds()+500;
                }
                //if spindexer is full
                boolean full = true;
                for(int i=0;i<3;i++){
                    if(savedBalls[i]=='n'){
                        full=false;
                        break;
                    }
                }
                if(full||!follower.isBusy()){
                    follower.breakFollowing();
                    pathState++;
//                    intake.setPower(0);
                }
            }
            //endregion

            //region HOOD CONTROL
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0 / 50.0; // fallback
            //(angles must be negative for our direction)
            updateHoodPID(hoodAngle + hoodOffset, dtSec);
            //endregion

            //region CAROUSEL
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            updateCarouselPID(targetAngle, dtSec);
            //endregion

            //region READ MOTIF
            if(pathState==1){
                int april = readMotif();
                if(april!=-1) {
                    if (april == 21) {
                        greenPos = 0;
                    } else if (april == 22) {
                        greenPos = 1;
                    } else if (april == 23) {
                        greenPos = 2;
                    }
                    pathState++;
                }
                else if(timeout<runtime.milliseconds()){
                    pathState++;
                }
            }
            //endregion

            //region FLYWHEEL
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0/2450.0;
            double compensatedF = baseF * (13.0/voltage);
            fly1.setVelocityPIDFCoefficients(flyKp, flyKi, flyKd, compensatedF);
            fly2.setVelocityPIDFCoefficients(flyKp, flyKi, flyKd, compensatedF);

            fly1.setVelocity(flySpeed);
            fly2.setVelocity(flySpeed);
            //endregion

            //region TRANSFER
            if(transOn){
                trans.setPower(1);
            }
            else{
                trans.setPower(0);
            }
            //endregion

            //region TELEMETRY
            if(!running) telemetry.addLine("Done!");
            telemetry.addData("path state", pathState);
            telemetry.addData("shooting state",shootingState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Green Position",greenPos);
            telemetry.addData("actual fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.addData("carousel pos",carouselIndex);
            telemetry.update();
            //endregion
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

    private void updateCarouselPID(double targetAngle, double dt) {
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
        telemetry.addData("Carousel Target", "%.1f°", targetAngle);

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

    private int readMotif(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 21||fiducial.getFiducialId() == 22||fiducial.getFiducialId() == 23) {
                    return fiducial.getFiducialId();
                }
            }

        }
        return -1;
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
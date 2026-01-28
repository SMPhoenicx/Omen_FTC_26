package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.FlywheelPIDController;
import org.firstinspires.ftc.teamcode.GlobalOffsets;
import org.firstinspires.ftc.teamcode.StateVars;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Partner Far Blue 12 Ball", group="Robot")
public class FarBlue12BallPartner extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double timeout = 0;

    //region PEDRO VARS
    private Follower follower;
    private Pose startPose, shoot1, movePoint;
    private Pose[] pickup1 = new Pose[2];
    private Pose[] pickup2 = new Pose[2];
    private Pose[] pickup3 = new Pose[3];
    private Pose[] gatePose = new Pose[2];
    private PathChain scorePath0, scorePath1, scorePath2, scorePath3, moveScore,limelightPath,gatePath, pickupPath1, pickupPath2, pickupPath3;
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
    private Servo hood = null;
    private CRServo turret1 = null;
    private CRServo turret2 = null;
    //TODO temp servo for tuning flywheel pid
//    private Servo tempServo = null;

    // ENCODERS
    private GoBildaPinpointDriver pinpoint = null;
    private AnalogInput spinEncoder;
    private AnalogInput turretEncoder;

    //COLOR
    private NormalizedColorSensor color1 = null;
    private NormalizedColorSensor color2 = null;
    //endregion

    //region VISION SYSTEM
    // AprilTag Configuration
    private boolean createLimelightPathOn = false;
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
    private double KballAngle = 0.67287181376;
    private double Kball = 0.6846;
    private double ballYoffset = 35;
    //limelight path
    private double ballX;
    private double ballY;
    private double ballHeading;
    double limelightWallPos;
    //endregion

    //region FLYWHEEL SYSTEM
    // Flywheel PID Constants
    private FlywheelPIDController flywheel;
    private boolean shootReady = false;
    //endregion

    //region HOOD SYSTEM

    // Hood Positions
    private double hoodAngle = 34.2;
    private double hoodOffset = 0;
    //endregion

    //region SPINDEXER SYSTEM
    // Spindexer PIDF Constants
    private double pidKp = 0.006;
    private double pidKi = 0.0;
    private double pidKd = 0.00095;//0.00065
    private double pidKf = 0.007;

    // Spindexer PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 4.0;
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
    private char[] savedBalls = {'g', 'p', 'p'};
    private boolean[] presentBalls = {false, false, false};

    private boolean spindexerPidArmed = false;
    private boolean spindexerAtTarget = false;
    private boolean intakeOn = false;
    //endregion

    //region TURRET SYSTEM
    // PIDF Constants
    private double tuKp = 0.0050;
    private double tuKi = 0.0006;
    private double tuKd = 0.00014;
    private double tuKf = 0.02;


    // PID State
    private double tuIntegral = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Control Parameters
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.03;
    private boolean turretAtTarget = false;

    // Turret Position
    private double tuPos = 0;
    //endregion

    private final PathConstraints shootConstraints = new PathConstraints(0.99, 100, 0.85, 1);

    public void createPoses(){
        startPose = new Pose(56.8,8,Math.toRadians(180));

        //0 is control point, 1 is endpoint
        pickup1[0] = new Pose(60.86,40.52,Math.toRadians(180));
        pickup1[1] = new Pose(10,35.68,Math.toRadians(180));

        pickup2[0] = new Pose(64.0,68.6,Math.toRadians(180));
        pickup2[1] = new Pose(10,60.55,Math.toRadians(180));

        pickup3[0] = new Pose(58.25,7.76,Math.toRadians(180));
        pickup3[1] = new Pose(10.4,8.53,Math.toRadians(180));

        shoot1 = new Pose(58,19,Math.toRadians(180));
        movePoint = new Pose(35.5,18.5,Math.toRadians(90));
    }

    public void createPaths(){
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shoot1))
                .setConstraints(shootConstraints)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.87,()-> {
                    shootReady=true;
                    timeout = 400;
                })
                .build();
        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup1[0],pickup1[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.2,()->{
                    follower.setMaxPower(0.33);
                    intakeOn = true;
                    pidKp -= 0.0015;
                })
                .setTimeoutConstraint(500)
                .build();
        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup2[0],pickup2[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.35,()->{
                    follower.setMaxPower(0.36);
                    intakeOn = true;
                    pidKp -= 0.0015;
                })
                .setTimeoutConstraint(500)
                .build();
        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup3[0],pickup3[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.33,()->{
                    follower.setMaxPower(0.26);
                    intakeOn = true;
                    pidKp -= 0.0015;
                })
                .setTimeoutConstraint(500)
                .build();
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1[1],shoot1))
                .setConstraints(shootConstraints)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.5,()-> {
                    follower.setMaxPower(0.7);
                })
                .addParametricCallback(0.983,()-> shootReady=true)
                .build();
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2[1],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.5,()-> {
                    follower.setMaxPower(0.7);
                })
                .addParametricCallback(0.99,()-> shootReady=true)
                .build();
        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.4,()-> {
                    follower.setMaxPower(0.5);
                })
                .addParametricCallback(0.99,()-> shootReady=true)
                .build();
        moveScore = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,movePoint))
                .setLinearHeadingInterpolation(shoot1.getHeading(), movePoint.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //region MAIN VARS
        int pathState = 0;
        int subState = 0;

        // Camera State
        boolean targetFound = false;

        int shootingState = 0;
        boolean running = true;
        int flySpeed = 1380;
        int shoot0change = -95;
        double spindexerSavedPos = 0;

        //Ball tracking
        double ballTx=0;
        double ballTy=0;

        //BOOLEANS like stuff on or off
        boolean motifOn = false;
        boolean transOn = false;
        boolean autoShootOn = false;
        boolean intakeLimelightOn = false;
        boolean gateCutoff = false;
        boolean cutoffSpinPID = false;
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
        hood = hardwareMap.get(Servo.class,"hood");
        turret1 = hardwareMap.get(CRServo.class, "tu1");
        turret2 = hardwareMap.get(CRServo.class, "tu2");
//        tempServo = hardwareMap.get(Servo.class,"speedometer");

        //ENCODERS
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin1");
        turretEncoder = hardwareMap.get(AnalogInput.class, "tuen");

        //COLOR SENSOR
        color1 = hardwareMap.get(NormalizedColorSensor.class,"Color 1");
        color2 = hardwareMap.get(NormalizedColorSensor.class,"Color 2");

        //DIRECTIONS
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        trans.setDirection(DcMotor.Direction.REVERSE);
        spin1.setDirection(CRServo.Direction.FORWARD);
        spin2.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.REVERSE);

        // Hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        flywheel = new FlywheelPIDController(
                hardwareMap.get(DcMotorEx.class, "fly1"),
                hardwareMap.get(DcMotorEx.class, "fly2")
        );
        flywheel.teleopMultiplier = 1.0;
        //endregion

        //region CAMERA INIT
        //LIMELIGHT
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
//        limelight.start();
//        limelight.pipelineSwitch(1);

        initAprilTag();
        setManualExposure(4, 200);
        //endregion

        //region INITIALIZE PEDRO
        createPoses();

        Pose2D ftcStartPose = PoseConverter.poseToPose2D(
                startPose,
                InvertedFTCCoordinates.INSTANCE
        );

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(ftcStartPose);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        createPaths();

        StateVars.lastPose = startPose;
        limelightWallPos = pickup1[1].getX();
        //endregion
        hoodOffset=0;
        tuPos = -130;
        flySpeed -= shoot0change;

        //WAIT
        waitForStart();
        runtime.reset();
        pidLastTimeMs = runtime.milliseconds();

        while(opModeIsActive()){
            follower.update();
            StateVars.lastPose = follower.getPose();

            //region IMPORTANT VARS
            //needed at beginning of loop, don't change location
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            pidLastTimeMs = nowMs;

            if (dtSec <= 0.0) dtSec = 1.0 / 50.0;

            double turnInput = -gamepad1.right_stick_x;

            follower.update();
            Pose robotPose = follower.getPose();
            //endregion

            if (runtime.milliseconds() > 27000) {
                pathState = 4;
            }
            //region PATH STUFF
            if(!follower.isBusy()&&runtime.milliseconds()>timeout){
                switch(pathState){
                    //region CYCLE ZERO (READ MOTIF)
                    case 0:
                        if(subState==0){
                            follower.followPath(scorePath0,true);
                            motifOn = true;
                            flywheel.Kp += 0.0004;

                            timeout = runtime.milliseconds()+3000;
                            subState++;
                        }
                        //READ MOTIF is subState 1
                        else if(subState==2){
                            tuPos = -130.5;
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE ONE
                    case 1:
                        if(subState==0){
                            follower.followPath(pickupPath1,false);
                            flywheel.Kp -= 0.0005;

                            flySpeed += shoot0change;
                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            follower.setMaxPower(1);
                            follower.followPath(scorePath1,true);
                            tuPos = -133;
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 4, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE TWO
                    case 2:
//                        if(subState==0){
//                            follower.followPath(pickupPath2,false);
//
//                            subState++;
//                        }
//                        //INTAKE is subState 1
//                        else if(subState==2){
//                            follower.setMaxPower(1);
//                            follower.followPath(scorePath2,true);
//                            tuPos -= 1.2;
//                            autoShootOn = true;
//                            shootingState=0;
//
//                            subState++;
//                        }
                        pathState++;
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE THREE
                    case 3:
                        if(subState==0){
                            follower.followPath(pickupPath3,false);

                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            timeout = runtime.milliseconds() + 500;
                            follower.setMaxPower(1);
                            follower.followPath(scorePath3,true);
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    case 4:
//                        transOn=false;
                        //TODO Check if this works
                        PathChain tempPath = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(),movePoint))
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), movePoint.getHeading())
                                .build();
                        follower.followPath(tempPath);
                        pathState++;
//                        flySpeed=0;
                        running=false;
                        break;
                }


            }
            //endregion

            //region COLOR SENSOR
//            char detectedColor = getRealColor();
//
//            if (spindexerAtTarget && runtime.milliseconds() - lastColorRead > 40) {
//                int slot = indexToSlot(spindexerIndex);
//                if (slot != -1) {
//                    savedBalls[slot] = detectedColor;  // 'n', 'g', or 'p'
//                    lastColorRead = runtime.milliseconds();
//                }
//            }
            //endregion

            //region LIMELIGHT VISION PROCESSING
//            if(createLimelightPathOn){
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//
//                    List <LLResultTypes.DetectorResult> detector = result.getDetectorResults();
//                    double maxArea = 0;
//                    int maxAreaIndex=0;
//
//                    for (int i=0;i<detector.size();i++) { //checks all results of detector
//                        LLResultTypes.DetectorResult detected = detector.get(i);
//
//                        if(detected.getTargetArea()>maxArea) {
//                            maxArea=detected.getTargetArea();
//                            maxAreaIndex=i;
//                        }
//                    }
//                    LLResultTypes.DetectorResult maxDetected = detector.get(maxAreaIndex);
//                    ballTx = maxDetected.getTargetXDegrees();
//                    ballTy = maxDetected.getTargetYDegrees()+ballYoffset;
//
//                    telemetry.addData("# of balls detected",detector.size());
//                    telemetry.addData("Closest ID",maxDetected.getClassId());
//                    telemetry.addData("Closest ball",maxDetected.getClassName());
//                    telemetry.addData("Ball offset from camera","X: %.3f Y: %.3f",ballTx,ballTy);
//                    pathToBall(ballTx,ballTy);
//                    createLimelightPathOn=false;
//                } else {
//                    telemetry.addData("# of balls detected",0);
//                    ballTx=0;
//                    ballTy=0;
//                }
//            }
            //endregion

            //region READ MOTIF
            if(motifOn&&timeout<runtime.milliseconds()){
                int april = readMotif();
                if(april!=-1) {
                    if (april == 21) {
                        greenPos = 0;
                    } else if (april == 22) {
                        greenPos = 1;
                    } else if (april == 23) {
                        greenPos = 2;
                    }
                    motifOn=false;
                    StateVars.patternTagID = april;
                    subState++;
                }
                else if(!follower.isBusy()){
                    motifOn=false;
                    subState++;
                }
            }
            //endregion

            //region INTAKE
            char detectedColor = getRealColor();
            boolean present = isBallPresent();
            int currentSlot = indexToSlot(spindexerIndex);

            if(intakeOn&&runtime.milliseconds()>timeout){
                intake.setPower(1);
                transOn=false;
                if (present && savedBalls[currentSlot] == 'n' && spindexerAtTarget) {

                    if (detectedColor != 'n') {
                        savedBalls[currentSlot] = detectedColor;
                        spinClock();
                    }
                }

                if(spindexerFull()||!follower.isBusy()){
                    if(spindexerFull()){
                        intake.setPower(0);
                    }
                    follower.breakFollowing();
                    intakeOn = false;
                    pidKp += 0.0015;

                    subState++;
                }
            }else{
                getRealColor();
            }
            //endregion

            //region HOOD CONTROL
            //(angles must be negative for our direction)
            hood.setPosition((hoodAngle + hoodOffset)/355.0);
            //endregion

            //region SPINDEXER
            double targetAngle = SPINDEXER_POSITIONS[spindexerIndex];
            if(!cutoffSpinPID){
                updateSpindexerPID(targetAngle+ GlobalOffsets.spindexerOffset, dtSec);
            }
            //endregion

            //region SHOOT PREP
            if(autoShootOn&&shootingState==0&&!motifOn){
                int greenIn=-1;
                for(int i=0;i<3;i++){
                    if(savedBalls[i]=='g'){
                        greenIn=i;
                    }
                }
                if(greenIn==-1){
                    for(int i=0;i<3;i++){
                        if(savedBalls[i]=='n' || savedBalls[i]=='b'){
                            greenIn=i;
                        }
                    }
                }
                if(greenIn==-1) greenIn=0;

                int diff = (greenIn + greenPos) % 3;
                if(diff==0) spindexerIndex=4;
                else if(diff==1) spindexerIndex=0;
                else spindexerIndex=2;
                spindexerAtTarget=false;
                timeout = runtime.milliseconds() + 300;

                shootingState++;
            }
            //endregion

            //region AUTO SHOOTING
            //prevent ball not firing
//            if(autoShootOn&&shootingState==1&&spindexerAtTarget) transOn = true;

            if(autoShootOn&&runtime.milliseconds()>timeout&&(shootReady||!follower.isBusy())){
                intake.setPower(0);
//                double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;
//                if(shootingState==1&&spindexerAtTarget&&avgSpeed > flySpeed * 0.94 && avgSpeed < flySpeed * 1.08){
                if(shootingState==1&&spindexerAtTarget){
                    transOn = true;
                    if(turretAtTarget){
                        spin1.setPower(0.93);
                        spin2.setPower(0.93);
                        cutoffSpinPID = true;

                        timeout=runtime.milliseconds()+900;
                        shootingState++;
                    }
                }
                else if(shootingState==2){
                    savedBalls[0]='n'; savedBalls[1]='n'; savedBalls[2]='n';

                    cutoffSpinPID = false;
                    shootReady = false;
                    autoShootOn = false;
                    shootingState++;
                    subState=0;
                    pathState++;
                }
            }
            //endregion

            //region FLYWHEEL
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            flywheel.updateFlywheelPID(
                    flySpeed,
                    dtSec,
                    voltage
            );

            double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;

//            if(avgSpeed >= flySpeed){
//                flyKd = 3;
//            }
//            tempServo.setPosition(avgSpeed/(flySpeed*2));
            //endregion

            //region TURRET
            updateTurretPID(tuPos, dtSec);
            //endregion

            //region TRANSFER
            if(transOn){
                trans.setPower(1);
            }
            else{
                trans.setPower(0);
            }
            //endregion

            //region GATE CUTOFF
            if(runtime.milliseconds()>timeout&&gateCutoff){
                gateCutoff = false;
                follower.breakFollowing();
            }
            //endregion

            //region TELEMETRY
            if(!running) telemetry.addLine("Done!");

            telemetry.addData("Encoder Fly Speed",avgSpeed);
            telemetry.addData("path state", pathState);
            telemetry.addData("sub state",subState);
            telemetry.addData("shooting state",shootingState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Green Position",greenPos);
            telemetry.addData("actual fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.addData("spindexer pos",spindexerIndex);
            telemetry.addData("spindexer at target",spindexerAtTarget);
            telemetry.update();
            //endregion
        }
    }

    //region HELPER METHODS
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
        telemetry.addData("Distance X", dist);
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

    private int indexToSlot(int index) {
        switch (index) {
            case 0: return 0;
            case 2: return 1;
            case 4: return 2;
            default: return -1;
        }
    }

    private int slotToIndex(int slot) {
        switch (slot) {
            case 0: return 0;
            case 1: return 2;
            case 2: return 4;
            default: return -1;
        }
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
    //region SPINDEXER HELPERS
    public void spinClock() {
        prevSpindexerIndex = spindexerIndex;
        spindexerIndex += spindexerIndex % 2 != 0 ? 1 : 0;
        spindexerIndex = (spindexerIndex - 2 + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
    }

    public void spinCounterClock() {
        prevSpindexerIndex = spindexerIndex;
        spindexerIndex += spindexerIndex % 2 != 0 ? 1 : 0;
        spindexerIndex = (spindexerIndex + 2) % SPINDEXER_POSITIONS.length;
    }

    private void updateSpindexerPID(double targetAngle, double dt) {
        if (!spindexerPidArmed) {
            lastError = 0;
            lastFilteredD = 0;
            integral = 0;
            spin1.setPower(0);
            spin2.setPower(0);
            spindexerPidArmed = true;
            return;
        }
        double angle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        double targetB = (targetAngle + 180.0) % 360.0;

        double errorA = -angleError(targetAngle, angle);
        double errorB = -angleError(targetB, angle);

        // compute shortest signed error [-180,180]
        double error = (Math.abs(errorA) <= Math.abs(errorB)) ? errorA : errorB;

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double rawD = (error - lastError) / Math.max(dt, 1e-6);

        double d = (lastFilteredD * 0.8) + (rawD * 0.2);
        lastFilteredD = d;

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

        spindexerAtTarget = (Math.abs(error) <= positionToleranceDeg+10);

        spin1.setPower(out);
        spin2.setPower(out);

        lastError = error;

        telemetry.addData("ENCODER VOLTAGE", spinEncoder.getVoltage());
        telemetry.addData("ANGLE", targetAngle);
        telemetry.addData("INTEGRAL", integral);
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

        //to know its set
        turretAtTarget = (Math.abs(error) <= tuToleranceDeg + 5);

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        turret1.setPower(out);
        turret2.setPower(out);

        // store errors for next derivative calculation
        tuLastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Turret Target", "%.1f°", targetAngle);

    }

    private int readMotif(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int numTags=0;
        int lastTagNum = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 21||detection.id == 22||detection.id == 23) {
                    numTags++;
                    lastTagNum=detection.id;
                }
            }
        }
        if(numTags==1) return lastTagNum;
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

    private void pathToBall(double tx,double ty){
        double hypotenuse = Math.sqrt((tx*tx) + (ty*ty));
        telemetry.addData("hypotenuse",hypotenuse);
        double angle = Math.atan(tx/(ty-5));

        double distX = (Math.cos(follower.getHeading()-angle)*hypotenuse*Kball);
        double distY = (Math.sin(follower.getHeading()-angle)*hypotenuse*Kball);
        if(ty>50){
            distX = 0;
            distY = 0;
        }else if(hypotenuse>40){
            distX *= 1.7;
            distY *= 1.7;
        }else if(hypotenuse>30){
            distX *= 1.2;
            distY *= 1.2;
        }

        ballX = follower.getPose().getX() + distX;
        ballY = follower.getPose().getY() + distY;

        ballHeading = follower.getHeading()+(-angle*KballAngle);//in radians

        //prevent slamming into wall
        if(ballX<26){//144-118 = 26
            double distXchange = ballX-26;//negative
            double proportion = Math.abs(distXchange/distX);//positive
            double distYchange = distY * proportion;
            ballX -= distXchange;
            ballY -= distYchange;
        }

        Pose ballPose = new Pose(ballX,ballY,ballHeading);

        limelightPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),ballPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),ballPose.getHeading())
                .build();
    }
    private Pose limelightPose(){
        return new Pose();
    }

    private boolean spindexerFull(){
        for(int i=0;i<3;i++){
            if(savedBalls[i]=='n') return false;
        }
        return true;
    }
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
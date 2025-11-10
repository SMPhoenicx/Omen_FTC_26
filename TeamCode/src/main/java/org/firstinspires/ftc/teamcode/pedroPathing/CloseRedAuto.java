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

import org.firstinspires.ftc.teamcode.ToggleServo;

import java.util.List;

@Autonomous(name="Close Red 1 (-41)", group="Robot")
public class CloseRedAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double timeout = 0;

    //region PEDRO VARS
    private Follower follower;
    private Pose startPose, obelisk, shoot1, movePoint;
    private Pose[] pickup1 = new Pose[2];
    private Pose[] pickup2 = new Pose[2];
    private Pose[] pickup3 = new Pose[2];
    private PathChain obeliskPath, scorePath0, pickupPath1, pickupPath2, pickupPath3, scorePath1, scorePath2, scorePath3, moveScore;
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
    private char[] savedBalls = {'g','p','p'};//n is none (empty), p is purple, g is green  --  side note i did not realize this says nnn when the carousels empty lmao

    //MOTIF (POSITION OF GREEN BALL 0-2)
    int greenPos = 0;
    //endregion

    double flyKp = 9.0;
    double flyKi = 0.945;
    double flyKd = 3.0;

    public void createPoses(){
        startPose = new Pose(121,121,Math.toRadians(45));
        obelisk = new Pose(85,85,Math.toRadians(135));
        pickup1[0] = new Pose(101,84,Math.toRadians(0));
        pickup1[1] = new Pose(133,84,Math.toRadians(0));
        pickup2[0] = new Pose(101,60,Math.toRadians(0));
        pickup2[1] = new Pose(133,60,Math.toRadians(0));
        pickup3[0] = new Pose(101,35,Math.toRadians(0));
        pickup3[1] = new Pose(133,35,Math.toRadians(0));
        shoot1 = new Pose(106,106,Math.toRadians(45));
        movePoint = new Pose(114,77,Math.toRadians(45));
    }

    public void createPaths(){
        obeliskPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose,obelisk))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72,144))
                .build();
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(obelisk,shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,144))
                .build();
        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup1[0]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup1[0].getHeading())
                .addPath(new BezierLine(pickup1[0],pickup1[1]))
                .setLinearHeadingInterpolation(pickup1[0].getHeading(),pickup1[1].getHeading())
                .setBrakingStrength(0.5)
                .setTimeoutConstraint(500)
                .build();
        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup2[0]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup2[0].getHeading())
                .addPath(new BezierLine(pickup2[0],pickup2[1]))
                .setLinearHeadingInterpolation(pickup2[0].getHeading(),pickup2[1].getHeading())
                .setBrakingStrength(0.5)
                .setTimeoutConstraint(500)
                .build();
        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup3[0]))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup3[0].getHeading())
                .addPath(new BezierLine(pickup3[0],pickup3[1]))
                .setLinearHeadingInterpolation(pickup3[0].getHeading(),pickup3[1].getHeading())
                .setBrakingStrength(0.5)
                .setTimeoutConstraint(500)
                .build();
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1[1],shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,144))
                .build();
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2[1],shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,144))
                .build();
        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,144))
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
        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.FORWARD);
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
        hood.setPower(0);

        //WAIT
        waitForStart();
        runtime.reset();
        pidLastTimeMs = runtime.milliseconds();

        while(opModeIsActive()){
            follower.update();

            if(pathState>=4) pathState=500;

            //region PATH STUFF
            if(!follower.isBusy()&&runtime.milliseconds()>timeout){
                switch(pathState){
                    //region CYCLE ZERO (READ MOTIF)
                    case 0:
                        follower.followPath(obeliskPath,false);
                        pathState++;
                        flySpeed = 1460;
                        timeout = runtime.milliseconds()+2000;
                        break;
                    //CASE 1 is reading motif
                    case 2:
                        follower.followPath(scorePath0,true);
                        pathState++;
                        shootingState=0;
                        transOn = true;
                        break;
                    //CASE 3 is shooting
                    //endregion

                    //region CYCLE ONE
                    case 4:
                        flySpeed = 0;
                        transOn=false;
                        intake.setPower(1);
                        follower.followPath(pickupPath1,true);
                        pathState++;
                        break;
                    //CASE 5 is intaking
                    case 6:
                        follower.followPath(scorePath1,true);
                        flySpeed = 1260;
                        transOn = true;
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 7 is shooting
                    //endregion

                    //region CYCLE TWO
                    case 8:
                        flySpeed = 0;
                        transOn=false;
                        intake.setPower(1);
                        follower.followPath(pickupPath1,true);
                        pathState++;
                        break;
                    //CASE 9 is intaking
                    case 10:
                        follower.followPath(scorePath1,true);
                        flySpeed = 1260;
                        transOn = true;
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 11 is shooting
                    //endregion

                    //region CYCLE THREE
                    case 12:
                        flySpeed = 0;
                        transOn=false;
                        intake.setPower(1);
                        follower.followPath(pickupPath1,true);
                        pathState++;
                        break;
                    //CASE 13 is intaking
                    case 14:
                        follower.followPath(scorePath1,true);
                        flySpeed = 1260;
                        transOn = true;
                        pathState++;
                        shootingState=0;
                        break;
                    //CASE 15 is shooting
                    //endregion

                    case 3:
                    case 7:
                    case 11:
                    case 15:
                        //region SHOOTING
                        if(shootingState==0){
                            int greenIn=1;
                            for(int i=0;i<3;i++){
                                if(savedBalls[i]=='g'){
                                    if(i==0){
                                        greenIn=3;
                                    }else if(i==1){
                                        greenIn=5;
                                    }else{//i==2 + else
                                        greenIn=1;
                                    }
                                }
                            }
                            carouselIndex = Math.abs((greenIn + (greenPos*2) + 1) % CAROUSEL_POSITIONS.length);
                            timeout=runtime.milliseconds()+2000;
                            shootingState++;
                        }
                        else if(shootingState==1){
                            carouselIndex = Math.abs((carouselIndex-2) % CAROUSEL_POSITIONS.length);
                            timeout=runtime.milliseconds()+2000;
                            shootingState++;
                        }
                        else if(shootingState==2){
                            carouselIndex = Math.abs((carouselIndex-2) % CAROUSEL_POSITIONS.length);
                            timeout=runtime.milliseconds()+2000;
                            shootingState++;
                        }
                        else if(shootingState==3){
                            carouselIndex = Math.abs((carouselIndex-1) % CAROUSEL_POSITIONS.length);
                            shootingState++;
                            pathState++;
                            timeout=runtime.milliseconds()+2000;
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
            if(pathState==5||pathState==9||pathState==13){
                if(getDetectedColor()!='n'&&savedBalls[carouselIndex/2]=='n'){//detect one ball intake
                    savedBalls[carouselIndex/2]=getDetectedColor();
                    carouselIndex = (carouselIndex-2) % CAROUSEL_POSITIONS.length;
                }
                //if spindexer is full
                boolean full = true;
                for(int i=0;i<3;i++){
                    if(savedBalls[i]=='n'){
                        full=false;
                        break;
                    }
                }
                if(full){
                    follower.breakFollowing();
                    pathState++;
                    intake.setPower(0);
                }
            }
            //endregion

            //region CAROUSEL
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0/50.0; // fallback
            pidLastTimeMs = nowMs;
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
                    follower.breakFollowing();
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
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ToggleServo;

@Autonomous(name="Far Blue 1 (67)", group="Robot")
public class FarBlueAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //region PEDRO VARS
    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;
    private Pose startPose, pickup1, pickup2, pickup3, shoot1, shoot0;
    private PathChain pickupPath1, pickupPath2, pickupPath3, scorePath0, scorePath1, scorePath2, scorePath3;
    //endregion

    //region HARDWARE DECLARATIONS
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private Servo led = null;
    private Servo hood = null;
    private Servo trans = null;
    private ToggleServo hoodt = null;
    //endregion

    public void createPoses(){
        startPose = new Pose(55.5,7.5,Math.toRadians(90));
        pickup1 = new Pose(54,43,Math.toRadians(180));
        pickup2 = new Pose(52,43,Math.toRadians(180));
        pickup3 = new Pose(46,43,Math.toRadians(180));
        shoot1 = new Pose(60,20,2.02);
        shoot0 = new Pose(60,15,2.02);
    }

    public void createPaths(){
        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot0,pickup1))
                .setLinearHeadingInterpolation(shoot0.getHeading(),pickup1.getHeading())
                .setTimeoutConstraint(500)
                .build();
        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup2))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup2.getHeading())
                .setTimeoutConstraint(500)
                .build();
        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,pickup3))
                .setLinearHeadingInterpolation(shoot1.getHeading(),pickup3.getHeading())
                .setTimeoutConstraint(500)
                .build();
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shoot0))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,10))
                .build();
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1,shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,10))
                .build();
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2,shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,15))
                .build();
        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3,shoot1))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(144,17))
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //region MAIN VARS
        double transTime = 0;
        int pathState = 0;
        boolean flyOn=true;
        int flySpeed = 1600;
        //endregion

        //region HARDWARE INFO
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");

        //SERVOS
        led = hardwareMap.get(Servo.class,"led");
        hood = hardwareMap.get(Servo.class,"hood");
        trans =  hardwareMap.get(Servo.class,"t1");

        //TOGGLESERVO
        hoodt = new ToggleServo(hood,  new int[]{240, 255, 270, 285, 300}, Servo.Direction.FORWARD, 270);

        //MODES
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //DIRECTIONS
        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        trans.setDirection(Servo.Direction.REVERSE);
        //endregion

        //region INITIALIZE STUFF
        createPoses();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        createPaths();
        //endregion
        hoodt.setIndex(4);

        //WAIT
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            follower.update();

            double timeChange = runtime.milliseconds() - transTime;
            if(timeChange >= 250) {
                trans.setPosition(0);
            }

            if(flyOn){
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);
            }else{
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            //region PATH STUFF
            if(!follower.isBusy()&&timeChange >= 500){
                switch(pathState){
                    //CYCLE ZERO
                    case 0:
                        follower.followPath(scorePath0,true);
                        pathState++;
                        sleep(1500);
                        break;
                    //CYCLE ONE
                    case 3:
                        flySpeed = 1450;
                        intake.setPower(1);
                        follower.followPath(pickupPath1,true);
                        pathState++;
                        break;
                    case 4:
                        follower.followPath(scorePath1,true);
                        pathState++;
                        break;
                    //CYCLE TWO
                    case 7:
                        flySpeed = 1500;
                        intake.setPower(1);
                        follower.followPath(pickupPath2,true);
                        pathState++;
                        break;
                    case 8:
                        follower.followPath(scorePath2,true);
                        pathState++;
                        break;
                    //CYCLE THREE
                    case 11:
                        intake.setPower(1);
                        follower.followPath(pickupPath3,true);
                        pathState++;
                        break;
                    case 12:
                        follower.followPath(scorePath3,true);
                        pathState++;
                        break;
                    //OTHER STUFF IG
                    case 1:
                    case 2:
                    case 5:
                    case 6:
                    case 9:
                    case 10:
                    case 13:
                    case 14:
                        intake.setPower(0);
                        trans.setPosition(1);
                        transTime = runtime.milliseconds();
                        pathState++;
                        break;
                    default:
                        flyOn=false;
                        telemetry.addLine("Done!");
                        break;
                }


            }
            //endregion

            //region TELEMETRY
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("fly on",flyOn);
            telemetry.addData("actual fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.update();
            //endregion
        }
    }
}

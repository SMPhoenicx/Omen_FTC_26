package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Get Real Bot Pose for Auto", group="Robot")
public class TestGetPose extends LinearOpMode {
    private Pose startPose = new Pose(19.9,123.5,Math.toRadians(54));

    //region IDK
    private ElapsedTime runtime = new ElapsedTime();
    Follower follower;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake = null;
    //endregion

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        boolean intakeOn = false;

        //region MOTORS
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "in");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        //endregion

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        waitForStart();

        while(opModeIsActive()){
            follower.update();

            if(gamepad1.rightBumperWasPressed()){
                intakeOn = !intakeOn;
            }

            if(intakeOn){
                intake.setPower(1);
            }
            else if(!intakeOn){
                intake.setPower(0);
            }

            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x;

            moveRobot(drive,strafe,turn);

            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

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
}
package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="NeuralDetector", group="Conecpt")
public class NeuralDetector extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //region HARDWARE DECLARATIONS
    // Drive Motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    // Vision
    private Limelight3A limelight;
    //endregion

    //region VISION SYSTEM
    // AprilTag Configuration
    private LLResultTypes.FiducialResult desiredTag;
    private static final int DESIRED_TAG_ID = 20; //blue id is 20, red is 24

    // Tracking State
    private boolean facingGoal = false; //used for tracking
    private double lastKnownBearing = 0; //used for smoothing
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 400;
    private double txOffset = 0;

    // Heading PID
    private double lastHeadingError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN = 0.015;
    final double MAX_AUTO_TURN = 0.3;
    private double distCamOffset = 0;
    //endregion

    //region FLYWHEEL SYSTEM
    // Flywheel PID Constants
    double flyKp = 9.0;
    double flyKi = 0.945;
    double flyKd = 3.2;
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
    private double hoodAngle = 0;
    private double hoodOffset = 0;
    //endregion

    //region TELE EXTRA VARS
    private static final double[] RANGE_SAMPLES = {31, 36, 41, 46, 51, 71.5, 78, 85, 90, 93};
    private static final double[] FLY_SPEEDS = {1220, 1255, 1300, 1340, 1390, 1560, 1590, 1630, 1670, 1700};
    private static final double[] HOOD_ANGLES = {-6, -33, -40, -50, -80, -140, -180, -200, -187, -177};

    private double smoothedRange = 0;
    private double smoothedTx = 0;
    private boolean isInitialized = false;

    private static final double ALPHA = 0.9;
    //endregion

    @Override
    public void runOpMode() throws InterruptedException {
        //region OPERATIONAL VARIABLES
        // Camera State
        boolean targetFound = false;

        // Flywheel Control
        double flySpeed = 1160;
        double flyOffset = 0;

        // Localization
        boolean localizeApril = true;

        //Ball tracking
        double ballTx=0;
        double lastBallTracking=0;
        boolean trackingBall=false;

        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        //endregion

        //region HARDWARE INITIALIZATION
        // Initialize Drive Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        // Configure Motor Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        //endregion

        //region SUBSYSTEM INITIALIZATION
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);
        //endregion

        telemetry.update();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            //region VISION PROCESSING
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                List <LLResultTypes.DetectorResult> detector = result.getDetectorResults();
                double maxArea = 0;
                int maxAreaIndex=0;

                for (int i=0;i<detector.size();i++) { //checks all results of detector
                    LLResultTypes.DetectorResult detected = detector.get(i);

                    if(detected.getTargetArea()>maxArea) {
                        maxArea=detected.getTargetArea();
                        maxAreaIndex=i;
                    }
                }
                LLResultTypes.DetectorResult maxDetected = detector.get(maxAreaIndex);
                ballTx = maxDetected.getTargetXDegrees();
                double ty = maxDetected.getTargetYDegrees();
                double ta = maxDetected.getTargetArea();
                lastBallTracking=runtime.milliseconds();

                telemetry.addData("# of balls detected",detector.size());
                telemetry.addData("Closest ball ID",maxDetected.getClassId());
                telemetry.addData("Closest ball name",maxDetected.getClassName());
                telemetry.addData("Ball offset","X: %.3f Y: %.3f",ballTx,ty);
            } else {
                telemetry.addData("# of balls detected",0);
                if(lastBallTracking+1000<runtime.milliseconds()){
                    ballTx=0;
                }
            }
            //endregion

            //region BALL TRACKING
            if(gamepad1.squareWasPressed()){
                trackingBall=!trackingBall;
            }

            if(trackingBall) {
                turn   = Range.clip(-ballTx * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                telemetry.addLine("Tracking Ball!");
            } else{
                turn = -gamepad1.right_stick_x;
            }
            //endregion

            //region DRIVE
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;

            moveRobot(drive, strafe, turn);
            //endregion

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
    //endregion
}

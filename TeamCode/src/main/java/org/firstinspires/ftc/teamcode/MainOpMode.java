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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MainOpMode", group = "Concept")
public class MainOpMode extends LinearOpMode
{
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_TURN  = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel
    private DcMotor fly1 = null;
    private DcMotor fly2 = null;
    private DcMotor intake = null;
    private CRServo rspin = null;
    private CRServo lspin = null;
    private Servo feeder = null;
   private static final int DESIRED_TAG_ID = 20;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag;
    @Override public void runOpMode()
    {
        boolean targetFound     = false;
        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;
        double flySpeed = 0;
        boolean flyOn = false;
        double lastTime = 0;
        boolean feederUp = false;

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
        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotor.class, "fly1");
        fly2 = hardwareMap.get(DcMotor.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        rspin = hardwareMap.get(CRServo.class, "rspin");
        lspin = hardwareMap.get(CRServo.class, "lspin");
        feeder = hardwareMap.get(Servo.class, "gate");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rspin.setDirection(CRServo.Direction.FORWARD);
        lspin.setDirection(CRServo.Direction.FORWARD);
        feeder.setDirection(Servo.Direction.FORWARD);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        feeder.setPosition(1);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            //flywheel
            if(gamepad1.a && !a1Pressed)  {
                flyOn = !flyOn;
                flySpeed = 0.5;
            }
            if(flyOn) {
                fly1.setPower(flySpeed);
                fly2.setPower(flySpeed);
            }
            else {
                fly2.setPower(0);
                fly2.setPower(0);
            }
            if(gamepad1.right_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                flySpeed += (flySpeed < 1)? 0.05:0;
                lastTime = runtime.milliseconds();
            }
            if(gamepad1.left_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                flySpeed -= (flySpeed > 0)? 0.05:0;
                lastTime = runtime.milliseconds();
            }

            //carousel
            if(gamepad1.dpad_right) {
                lspin.setPower(1); rspin.setPower(1);
            }
            else if(gamepad1.dpad_left) {
                lspin.setPower(-1); rspin.setPower(-1);
            }
            else {
                lspin.setPower(0); rspin.setPower(0);
            }

            //gate
            if(gamepad1.dpad_up && !up1Pressed){
                feederUp = !feederUp;
                if(feederUp){
                    feeder.setPosition(0);
                }
                else{
                    feeder.setPosition(1);
//                    System.out.println("67");
                }
            }

            // Intake

            if(gamepad1.right_bumper && !rb1Pressed) {
                if(intake.getPower() <= 0) intake.setPower(1);
                else intake.setPower(0);
            }

            //Outtake
            if(gamepad1.left_bumper && !lb1Pressed) {
                intake.setPower(-0.6);
            }

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                        targetFound = true;
                        break;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Turn to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Turn to face target Automatically.
            if (gamepad1.dpad_down && targetFound) {

                // Heading error: angle left/right of camera center
                double headingError = desiredTag.ftcPose.bearing;

                // Apply proportional gain to generate turn power
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                // Zero out drive/strafe so it only rotates
                drive = 0;
                strafe = 0;

                telemetry.addData("Auto", "Turn %5.2f (headingErr %3.0f°)", turn, headingError);

            } else {
                // Manual control
                drive  = -gamepad1.left_stick_y;
                strafe = -gamepad1.left_stick_x;
                turn   = -gamepad1.right_stick_x;
            }

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);

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
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Fly state", flyOn);
            telemetry.addData("Fly power", flySpeed);
            telemetry.addData("Feeder Up",feederUp);
            telemetry.update();

            sleep(10);
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

    /**
     * Initialize the AprilTag processor.
     */
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
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
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
}

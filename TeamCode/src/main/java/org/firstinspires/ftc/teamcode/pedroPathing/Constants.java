package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.52544)
            .forwardZeroPowerAcceleration(-49.3516883945663)
            .lateralZeroPowerAcceleration(-71.04874194079048)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.05,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.00002,0.6,0.04))
            .centripetalScaling(0.01);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.89680284339842)
            .yVelocity(55.07917092588737);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .leftPodY(5.375)
            .rightPodY(-5.375)
            .strafePodX(-5.5)
            .leftEncoder_HardwareMapName("fl")
            .rightEncoder_HardwareMapName("fr")
            .strafeEncoder_HardwareMapName("bl")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.0029854818259260328)
            .strafeTicksToInches(-0.0029295215647804343)
            .turnTicksToInches(0.002824110133050991);
}

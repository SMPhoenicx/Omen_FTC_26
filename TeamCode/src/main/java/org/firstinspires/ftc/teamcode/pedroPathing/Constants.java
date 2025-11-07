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
            .mass(10.433)
            .forwardZeroPowerAcceleration(-36.360079640738)
            .lateralZeroPowerAcceleration(-75.304815650991)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.004, 0.04))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0.01,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.08,0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0,0.1,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0,0.00003,0.6,0.05))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.007,0,0.000001,0.6,0.1))
            .centripetalScaling(0.0001)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);

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
            .xVelocity(76.644973761374)
            .yVelocity(62.931638202634);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .leftPodY(6.75)
            .rightPodY(-6.75)
            .strafePodX(-6.75)
            .leftEncoder_HardwareMapName("fl")
            .rightEncoder_HardwareMapName("br")
            .strafeEncoder_HardwareMapName("fr")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.0029709051374068)
            .strafeTicksToInches(0.003018510707204)
            .turnTicksToInches(0.0029761537370001);
}

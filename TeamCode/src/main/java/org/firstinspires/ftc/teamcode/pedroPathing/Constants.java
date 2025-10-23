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
            .forwardZeroPowerAcceleration(-35.61107470369831)
            .lateralZeroPowerAcceleration(-81.26478251846179)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.005, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.08,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.00005,0.6,0.04))
            .centripetalScaling(0.0001);

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
            .xVelocity(83.213618641897)
            .yVelocity(61.276508546598414);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .leftPodY(5.125)
            .rightPodY(-5.125)
            .strafePodX(-7)
            .leftEncoder_HardwareMapName("skib")
            .rightEncoder_HardwareMapName("fl")
            .strafeEncoder_HardwareMapName("br")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardTicksToInches(0.0029906905942694607)
            .strafeTicksToInches(-0.0029906173594936272)
            .turnTicksToInches(0.0028762739224952347);
}

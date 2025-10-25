package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.62)
            .yVelocity(46.19)
            .useBrakeModeInTeleOp(true);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4)
            .strafePodX(-6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.677564)
            .forwardZeroPowerAcceleration(-46.29)
            .lateralZeroPowerAcceleration(-64.99)
            .centripetalScaling(5)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.02, 0))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.001, 0))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.02, 0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00001, 0.6, 0.01))
            .useSecondaryDrivePIDF(true)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.08, 0, 0.000005, 0.6, 0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

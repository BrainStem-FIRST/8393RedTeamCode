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
            .xVelocity(72.06)
            .yVelocity(31.37)
            .useBrakeModeInTeleOp(true);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.875) // 3.875
            .strafePodX(-6.25) // 6.25
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.58)
            .forwardZeroPowerAcceleration(-61.89)
            .lateralZeroPowerAcceleration(-71.66)
            .centripetalScaling(0.001)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0.01))
            .useSecondaryTranslationalPIDF(false)

            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0, 0.02))
            .useSecondaryHeadingPIDF(false)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0016, 0, 0, 0.6, 0.02))
            .useSecondaryDrivePIDF(false);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 6, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        pathConstraints.setHeadingConstraint(5 * Math.PI / 180);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

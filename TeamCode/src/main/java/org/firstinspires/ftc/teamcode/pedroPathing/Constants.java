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

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.001, 0.03))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.005, 0.03))

            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.001, 0.02))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.001, 0.02))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0011, 0, 0.00004, 0.6, 0.05))
            .useSecondaryDrivePIDF(false)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0013, 0, 0.00005, 0.6, 0.05));
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 10000, 8, 1.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        pathConstraints.setHeadingConstraint(3 * Math.PI / 180);
        followerConstants.setHeadingPIDFSwitch(0.1396);
        pathConstraints.setTranslationalConstraint(3);
        followerConstants.setTranslationalPIDFSwitch(5);
        followerConstants.setDrivePIDFSwitch(5);
//        followerConstants.setHeadingPIDFSwitch(10 * Math.PI / 180);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

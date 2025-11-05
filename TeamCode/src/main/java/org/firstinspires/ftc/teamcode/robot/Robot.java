package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.NullGamepadTracker;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.function.Supplier;

@Config
public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final GamepadTracker g1, g2;
    public final Follower follower;
    public final Intake intake;
    public final Indexer indexer;
    public final Transfer transfer;
    public final Shooter shooter;
    public final Parker parker;
    public final BinaryLight binaryLight;
    public final RGBLight rgbLight;
    private Supplier<PathChain> farPathChain, nearPathChain;

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;
    public static class Params {
        public double goalX = 3, goalY = 138;
        public double farX = 86.3, farY = 16, farHeading = -65;
        public double nearX = 52.7, nearY = 89.6, nearHeading = -45;
        public double turnCorrection = 0.14, turnAmpNormal = 0.8, turnAmpSlow = 0.2;
        public int greenPos = -1;
        public double width = 16.5, wheelToWheelL = 10.5;
        public double intakeToFrontWheelL = 4.5, backToBackWheelL = 1.625;
        public double intakeToWheelCenter = wheelToWheelL / 2 + intakeToFrontWheelL;
        public double backToWheelCenter = wheelToWheelL / 2 + backToBackWheelL;
        public double tickW = 0.8;
        public double kPFar = 0.5, kDFar = 0.0001, kF = 0;
        public double kPClose = 0.6, kDClose = 0;
        public double pidSwitch = 5;
        public double minShooterTurnFar = 0.12, minShooterTurnClose = 0.1, headingShootErrorNear = 3, headingShootErrorFar = 1.5;
        public boolean autoDone = false;
    }
    private PIDFController autoTurnPidFar, autoTurnPidClose;
    private boolean slowTurn;
    public static Params params = new Params();
    // used for auto
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose startPose) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        g1 = new NullGamepadTracker();
        g2 = new NullGamepadTracker();

        intake = new Intake(this);
        indexer = new Indexer(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        parker = new Parker(this);
        follower = Constants.createFollower(hardwareMap);
        binaryLight = new BinaryLight(this);
        rgbLight = new RGBLight(this);

        //Focals (pixels) - Fx: 628.438 Fy: 628.438
        //Optical center - Cx: 986.138 Cy: 739.836
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(628.438, 628.438, 986.138, 739.836)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(1920, 1200))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        follower.setStartingPose(startPose);
    }

    // used for tele
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadTracker g1, GamepadTracker g2, Pose startPose) {
        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;
        this.g1 = g1;
        this.g2 = g2;

        intake = new Intake(this);
        indexer = new Indexer(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        parker = new Parker(this);
        binaryLight = new BinaryLight(this);
        rgbLight = new RGBLight(this);
        follower = Constants.createFollower(hardwareMap);

        //Focals (pixels) - Fx: 628.438 Fy: 628.438
        //Optical center - Cx: 986.138 Cy: 739.836
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(628.438, 628.438, 986.138, 739.836)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(1920, 1200))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


        farPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(params.farX, params.farY))))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(params.farHeading))
                .build();
        nearPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(params.nearX, params.nearY))))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(params.nearHeading))
                .build();

        autoTurnPidFar = new PIDFController(new PIDFCoefficients(params.kPFar, 0, params.kDFar, params.kF));
        autoTurnPidClose = new PIDFController(new PIDFCoefficients(params.kPClose, 0, params.kDClose, params.kF));
        follower.setStartingPose(startPose);
    }
    public void updateSubsystems() {
        indexer.resetCaches();
        shooter.resetCaches();
        intake.update();
        indexer.update();
        transfer.update();
        shooter.update();
//        binaryLight.update();
        rgbLight.update();
    }
    public void updateTele() {
        updatePedroTele();
        updateSubsystems();
        updateAprilTag();
    }
    public void updateAprilTag() {
        telemetry.addData("camera state", visionPortal.getCameraState());
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if (!detections.isEmpty()) {
            for(int i = 0; i < detections.size(); i++) {
                AprilTagDetection tag = tagProcessor.getDetections().get(i);

                // calculating pattern
                if(params.greenPos == -1)
                    switch(tag.id) {
                        case 21: params.greenPos = 0; break;
                        case 22: params.greenPos = 1; break;
                        case 23: params.greenPos = 2; break;
                    }
            }
        }
        // 21: gpp, 22: pgp, 23: ppg
    }
    public void initPedroTele() {
        follower.startTeleopDrive();
        follower.update();
    }
    public void updatePedroTele() {
        double goalHeading = Math.atan2(follower.getPose().getY() - params.goalY, follower.getPose().getX() - params.goalX);
        if(g1.isFirstLeftBumper())
            slowTurn = !slowTurn;

        double turnPower = Range.clip(-g1.rightStickX() * params.turnAmpNormal + g1.leftStickX() * params.turnCorrection, -1, 1);
        if(slowTurn) {
            double error = goalHeading - follower.getHeading();
            if(Math.abs(error) < (shooter.getZone() == 0 ? params.headingShootErrorNear : params.headingShootErrorFar) * Math.PI / 180) {
                slowTurn = false;
            }
            else {
                if(Math.abs(error) > params.pidSwitch) {
                    telemetry.addLine("pid FAR");
                    autoTurnPidFar.updateError(error);
                    turnPower = autoTurnPidFar.run();
                    turnPower = Math.max(params.minShooterTurnFar, Math.abs(turnPower)) * Math.signum(turnPower);
                }
                else {
                    telemetry.addLine("pid CLOSE");
                    autoTurnPidClose.updateError(error);
                    turnPower = autoTurnPidClose.run();
                    turnPower = Math.max(params.minShooterTurnClose, Math.abs(turnPower)) * Math.signum(turnPower);
                }
            }
        }
        telemetry.addData("dx", params.goalX - follower.getPose().getX());
        telemetry.addData("dy", params.goalY - follower.getPose().getY());
        telemetry.addData("goalHeading", Math.floor(goalHeading * 180 / Math.PI));
        telemetry.addData("robot heading", Math.floor(follower.getHeading() * 180 / Math.PI));
        telemetry.addData("turn power", turnPower);

        follower.setTeleOpDrive(-g1.leftStickY(), -g1.leftStickX(), turnPower, true);
        follower.update();
    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public boolean isSlowTurn() {
        return slowTurn;
    }
}

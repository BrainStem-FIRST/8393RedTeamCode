package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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

    private boolean automatedDrive;
    private Supplier<PathChain> farPathChain, nearPathChain;

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;
    public static class Params {
        public double goalX = -72, goalY = -68;
        public double farX = 51, farY = -8, farHeading = -0.53;
        public double turnCorrection = 0.14;
        public int greenPos = -1;
    }
    public static Params params = new Params();


    // used for auto
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
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
    }

    // used for tele
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadTracker g1, GamepadTracker g2) {
        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;
        this.g1 = g1;
        this.g2 = g2;

        intake = new Intake(this);
        indexer = new Indexer(this);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        parker = new Parker(this);
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
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, params.farHeading, 0.8))
                .build();
        nearPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    public void updateTele() {
        updatePedroTele();
        updateAprilTag();
    }
    public void updateAuto() {
        updateAprilTag();
    }
    private void updateAprilTag() {
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
        else
            telemetry.addData("no tags found", "");
    }
    private void findPattern(ArrayList<AprilTagDetection> detections) {
    }
    public void initPedroTele() {
        follower.setStartingPose(new Pose(65.025, -9, Math.PI));
        follower.startTeleopDrive();
        follower.update();
    }
    public void updatePedroTele() {
        if(!automatedDrive) {
            if(g1.isFirstLeftBumper()) {
                automatedDrive = true;
                follower.followPath(farPathChain.get());
            }
            else
                follower.setTeleOpDrive(-g1.leftStickY(), -g1.leftStickX(), -g1.rightStickX() + g1.leftStickX() * params.turnCorrection, true);
        }
        else if(!follower.isBusy()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }
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
    public boolean automatedDrive() {
        return automatedDrive;
    }
}

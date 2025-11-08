package org.firstinspires.ftc.teamcode.robot;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;
    public static class Params {
        public boolean red = false;
        public double goalXNearBlue = 4, goalYNearBlue = 139, goalXFarBlue = 6, goalYFarBlue = 139;
        public double goalXNearRed = 140, goalYNearRed = 137, goalXFarRed = 138, goalYFarRed = 139;
        public double turnCorrection = 0.14, turnAmpNormal = 0.8, turnAmpSlow = 0.4;
        public int greenPos = -1;
        public double width = 16.75, wheelToWheelL = 10.75;
        public double intakeToFrontWheelL = 4.5, backToBackWheelL = 1.625;
        public double intakeToWheelCenter = 9.675;
        public double backToWheelCenter = wheelToWheelL / 2 + backToBackWheelL;
        public double tickW = 0.8;
        public double kPBig = 0.5, kDBig = 0, kF = 0;
        public double kPSmall = 1.1, kDSmall = 0;
        public double pidSwitch = 0.1345;
        public double minShooterTurn = 0.07, maxShooterTurn = 0.3, shootD2MoveAmp = 0.6;
        public double headingShootError = 0.03490658;
        public boolean autoDone = false;
        public double goalInc = 0.01;
    }
    private double goalX, goalY;
    private double headingLockOffset;
    private PIDFController autoTurnPidBig, autoTurnPidSmall;
    private double x, y, heading, goalHeading;
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

        autoTurnPidBig = new PIDFController(new PIDFCoefficients(params.kPBig, 0, params.kDBig, params.kF));
        autoTurnPidSmall = new PIDFController(new PIDFCoefficients(params.kPSmall, 0, params.kDSmall, params.kF));
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

        // updating pose
        goalHeading = Math.atan2(follower.getPose().getY() - goalY, follower.getPose().getX() - goalX);
        heading = follower.getHeading();
        x = follower.getPose().getX();
        y = follower.getPose().getY();
    }
    public void setGoalPos() {
        if(params.red) {
            if(shooter.getZone() == 0) {
                goalX = params.goalXNearRed;
                goalY = params.goalYNearRed;
            }
            else {
                goalX = params.goalXFarRed;
                goalY = params.goalYFarRed;
            }
        }
        else {
            if(shooter.getZone() == 0) {
                goalX = params.goalXNearBlue;
                goalY = params.goalYNearBlue;
            }
            else {
                goalX = params.goalXFarBlue;
                goalY = params.goalYFarBlue;
            }
        }
    }
    public void updateTele() {
        updatePedroTele();
        updateSubsystems();
        updateAprilTag();
        setGoalPos();
    }
    public void updateAprilTag() {
//        telemetry.addData("camera state", visionPortal.getCameraState());
        StringBuilder tags = new StringBuilder();
        try {
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                for (int i = 0; i < detections.size(); i++) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(i);
                    tags.append(tag.id).append(", ");

                    // calculating pattern
                    if (params.greenPos == -1)
                        switch (tag.id) {
                            case 21:
                                params.greenPos = 0;
                                break;
                            case 22:
                                params.greenPos = 1;
                                break;
                            case 23:
                                params.greenPos = 2;
                                break;
                        }
                }
            }
        } catch(IndexOutOfBoundsException ignored) {}
        // 21: gpp, 22: pgp, 23: ppg
//        telemetry.addData("tags", tags);
    }
    public void initPedroTele() {
        follower.startTeleopDrive();
        follower.update();
    }
    public void updatePedroTele() {
        double disp = goalHeading - heading + headingLockOffset;
        telemetry.addData("disp", disp);
        double turnPower = Range.clip(-g1.rightStickX() * params.turnAmpNormal + g1.leftStickX() * params.turnCorrection, -1, 1);
        slowTurn = g1.leftBumper();
        if(slowTurn) {
                if(Math.abs(disp) > params.pidSwitch) {
                    telemetry.addLine("pid BIG");
                    autoTurnPidBig.updateError(disp);
                    turnPower = autoTurnPidBig.run();
                }
                else {
                    telemetry.addLine("pid SMALL");
                    autoTurnPidSmall.updateError(disp);
                    turnPower = autoTurnPidSmall.run();
                }
                turnPower = Math.max(Math.min(Math.abs(turnPower), params.maxShooterTurn), params.minShooterTurn) * Math.signum(turnPower) - params.turnAmpSlow * g1.rightStickX();
        }
//        telemetry.addData("dx", params.goalX - follower.getPose().getX());
//        telemetry.addData("dy", params.goalY - follower.getPose().getY());
//        telemetry.addData("goalHeading", Math.floor(goalHeading * 180 / Math.PI * 100)/100);
//        telemetry.addData("robot heading", Math.floor(heading * 180 / Math.PI*100)/100);
//        telemetry.addData("turn power", turnPower);
//        telemetry.addData("slow turn", slowTurn);

        double axialPower = -g1.leftStickY();
        double lateralPower = -g1.leftStickX();
        if(Math.abs(g2.leftStickY()) > 0.03 || Math.abs(g2.leftStickX()) > 0.03 || Math.abs(g2.rightStickX()) > 0.03) {
            axialPower = -Math.sqrt(Math.abs(g2.leftStickY())) * Math.signum(g2.leftStickY()) * params.shootD2MoveAmp;
            lateralPower = -Math.sqrt(Math.abs(g2.leftStickX())) * Math.signum(g2.leftStickX()) * params.shootD2MoveAmp;
            turnPower = Range.clip(-g2.rightStickX() * params.turnAmpSlow + g2.leftStickX() * params.turnCorrection, -1, 1);
        }
        follower.setTeleOpDrive(axialPower, lateralPower, turnPower, true);
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
    public double getHeading() {
        return heading;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getGoalX() {
        return goalX;
    }
    public double getGoalY() {
        return goalY;
    }
    public double getGoalHeading() {
        return goalHeading;
    }
}

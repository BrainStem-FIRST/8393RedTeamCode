package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

import java.util.function.Supplier;

public class Robot {
    public double goalX = -72, goalY = -68;
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final GamepadTracker g1, g2;
    public final Follower follower;
    public final Intake intake;
    public final Indexer indexer;
    public final Shooter shooter;
    public final Parker parker;

    private boolean automatedDrive;
    private Supplier<PathChain> farPathChain, nearPathChain;
    public static class Params {

        public double farX = 51, farY = -8, farHeading = -0.53;
    }
    public static Params params = new Params();

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadTracker g1, GamepadTracker g2) {
        this.hardwareMap = hardwareMap;

        this.telemetry = telemetry;
        this.g1 = g1;
        this.g2 = g2;

        indexer = new Indexer(this);
        intake = new Intake(this);

        shooter = new Shooter(this);
        parker = new Parker(this);

        follower = Constants.createFollower(hardwareMap);

        farPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(params.farX, params.farY))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, params.farHeading, 0.8))
                .build();
        nearPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    public void update() {
        if(!automatedDrive) {
            updatePedroTele();
            if(g1.isFirstLeftBumper()) {
                automatedDrive = true;
                follower.followPath(farPathChain.get());
            }
        }
        else if(!follower.isBusy()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }
        follower.update();
    }
    public void initPedroTele() {
        follower.setStartingPose(new Pose(65.025, -9, 0));
        follower.startTeleopDrive();
        follower.update();
    }
    public void updatePedroTele() {
        follower.setTeleOpDrive(-g1.gamepad.left_stick_y, -g1.gamepad.left_stick_x, -g1.gamepad.right_stick_x, true);
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

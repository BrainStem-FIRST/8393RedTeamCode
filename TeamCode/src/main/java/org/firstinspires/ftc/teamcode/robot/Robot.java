package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

public class Robot {
    public double goalX, goalY;
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final GamepadTracker g1, g2;
    public final Follower follower;
    public final Intake intake;
    public final Indexer indexer;
    public final Shooter shooter;
    public final Parker parker;

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
    }
    public void initPedroTele() {
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        follower.update();
    }
    public void updatePedroTele() {
        follower.setTeleOpDrive(-g1.gamepad.left_stick_y, -g1.gamepad.left_stick_x, -g1.gamepad.right_stick_x, true);
        follower.update();
    }
}

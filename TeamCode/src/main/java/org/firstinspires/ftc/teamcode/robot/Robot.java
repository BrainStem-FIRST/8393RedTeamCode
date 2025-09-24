package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final GamepadTracker g1, g2;
    public final Follower follower;
    public final IntakeIndexer intakeIndexer;

    public final Shooter shooter;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadTracker g1, GamepadTracker g2, boolean teleop) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.g1 = g1;
        this.g2 = g2;

        intakeIndexer = new IntakeIndexer(this);
        shooter = new Shooter(this);

        follower = Constants.createFollower(hardwareMap);
        if(teleop)
            follower.startTeleopDrive();
    }
    public void update() {
        intakeIndexer.update();
        follower.update();
    }
}

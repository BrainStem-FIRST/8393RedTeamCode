package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Limelight {
    private final Limelight3A limelight;
    public final Robot robot;
    private Pose3D botPose;
    private final ElapsedTime lastUpdateTimer;
    public Limelight(Robot robot) {
        this.robot = robot;
        limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        lastUpdateTimer = new ElapsedTime();
    }
    public void update() {
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            robot.telemetry.addData("isValid", result.isValid());
            Pose3D botPose = result.getBotpose();
            if(botPose.getPosition().x != 0 && botPose.getPosition().y != 0 && botPose.getPosition().z != 0) {
                this.botPose = botPose;
                lastUpdateTimer.reset();
            }
        }
        else
            robot.telemetry.addLine("botpose null");
    }
    public Pose3D getBotPose() {
        return botPose;
    }
    public Position getBotPos() {
        return botPose.getPosition().toUnit(DistanceUnit.INCH);
    }
    public double getBotHeading() {
        return botPose.getOrientation().getYaw(AngleUnit.RADIANS);
    }
    public double timeSinceLastUpdate() {
        return lastUpdateTimer.seconds();
    }
}

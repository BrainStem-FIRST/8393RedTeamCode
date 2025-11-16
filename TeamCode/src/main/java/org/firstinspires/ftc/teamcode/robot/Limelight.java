package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Limelight {
    public static double offset = 0;
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
            robot.telemetry.addData("botPos", botPose.getPosition());
            robot.telemetry.addData("heading", botPose.getOrientation().getYaw(AngleUnit.DEGREES));
            if(botPose.getPosition().x != 0 && botPose.getPosition().y != 0) {
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
        double heading = botPose.getOrientation().getYaw(AngleUnit.RADIANS) + offset;
        if(Math.abs(heading) > Math.PI)
            return Math.PI * 2 - heading;
        return heading;
    }
    public double metersToInches(double m) {
        return m * 37.3701;
    }
    public double timeSinceLastUpdate() {
        return lastUpdateTimer.seconds();
    }
}

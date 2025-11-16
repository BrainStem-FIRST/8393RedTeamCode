package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.utils.math.Vector2;

public class Limelight {
    // i should tune the camera so that it gives me the turret center position
    private final Limelight3A limelight;
    public final Robot robot;
    private final Vector2 turretPos;
    private double turretHeading;
    private final Vector2 robotPos;
    private double robotHeading;
    private final Vector2 robotTurretVec;
    public Limelight(Robot robot) {
        this.robot = robot;
        limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        turretPos = new Vector2(0, 0);
        robotPos = new Vector2(0, 0);
        robotTurretVec = new Vector2(0, 0);
    }
    public void update(double turretAngle, double robotTurretOffset) {
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            robot.telemetry.addData("isValid", result.isValid());
            Pose3D turretPose = result.getBotpose();
            robot.telemetry.addData("camera pos", turretPose.getPosition());
            robot.telemetry.addData("heading", turretPose.getOrientation().getYaw(AngleUnit.DEGREES));

            if(turretPose.getPosition().x != 0 && turretPose.getPosition().y != 0) {
                Position temp = turretPose.getPosition().toUnit(DistanceUnit.INCH);
                this.turretPos.set(temp.x, temp.y);
                turretHeading = turretPose.getOrientation().getYaw(AngleUnit.RADIANS);

                robotHeading = (turretHeading - turretAngle + Math.PI * 2) % (Math.PI  * 2);
                if(robotHeading > Math.PI)
                    robotHeading -= Math.PI * 2;
                robotTurretVec.set(robotTurretOffset * Math.cos(robotHeading), robotTurretOffset * Math.sin(robotHeading));
                robotPos.set(turretPos.add(robotTurretVec));
            }
        }
        else
            robot.telemetry.addLine("camera pos is null");
    }

    // robotHeading should be in radians
    // robot turret offset is distance from center of turret to center of robot
    public Vector2 getRobotPos() {
        return robotPos;
    }
    public double getRobotHeading() {
        return robotHeading;
    }
    public Vector2 getTurretPos() {
        return turretPos;
    }
    public double getTurretHeading() {
        return turretHeading;
    }
}

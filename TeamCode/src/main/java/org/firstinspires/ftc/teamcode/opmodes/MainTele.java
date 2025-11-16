package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name="TELE")
public class MainTele extends LinearOpMode {
    public static double x = 0, y = 0, a = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2, new Pose(x, y, Math.toRadians(a)));
        Robot.params.autoDone = false;
        robot.initPedroTele();
        while(opModeInInit()) {
            g1.update();
            g2.update();
            robot.rgbLight.updateInit();
            if(g1.x())
                Robot.params.red = false;
            if(g1.b())
                Robot.params.red = true;
            if(g2.leftBumper())
                robot.indexer.resetEncoder();
            robot.setGoalPos();
        }
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        AprilTagDetection tag = null;
        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.updateTele();
            telemetry.addLine("SHOOTER----");
            telemetry.addData("vel", robot.shooter.getShooterVelocity());
            telemetry.addData("power", robot.shooter.getShooterPower());
            telemetry.addData("hood", robot.shooter.getHoodPos());
            telemetry.addData("zone", robot.shooter.getZone());
            telemetry.addLine("INDEXER----");
            telemetry.addData("  state", robot.indexer.getIndexerState());
//            telemetry.addData("  oscillate target", robot.indexer.getOscillateTargetEncoder());
//            telemetry.addData("  should oscillate", robot.indexer.shouldOscillate());
            telemetry.addData("  encoder", robot.indexer.getIndexerEncoder());
            telemetry.addData("  target", robot.indexer.getTargetEncoder());
//            telemetry.addData("  error", robot.indexer.getIndexerError());
            telemetry.addData("  power", robot.indexer.getIndexerPower());
//            telemetry.addData("  pid selected", robot.indexer.getPidSelected());
//            telemetry.addLine();
            telemetry.addData("  static shoot", robot.indexer.prettyMuchStaticShoot());
            telemetry.addData("  static cs", robot.indexer.prettyMuchStaticCS());
            telemetry.addData("  vel", robot.indexer.getIndexerVel());
//            telemetry.addData("  num balls", robot.indexer.getNumBalls());
//            telemetry.addData("  ballList", robot.indexer.getLabeledBalls());
//            telemetry.addData("  align offset", robot.indexer.getAlignShootOffset());
//            telemetry.addData("  intakeI, li, ri, si", robot.indexer.getIntakeI() + ", " + robot.indexer.getLeftIntakeI() + ", " + robot.indexer.getRightIntakeI() + ", " + robot.indexer.getShooterI());
//            telemetry.addData("  green pos, cur pattern i", Robot.params.greenPos + ", " + robot.indexer.getCurPatternI());
//            telemetry.addLine();
            telemetry.addData("  auto rotate cued", robot.indexer.isAutoRotateCued());
            telemetry.addData("  should auto rotate", robot.indexer.shouldAutoIndex());
            telemetry.addData("  sc left, mid, right", robot.indexer.shouldCheckLeftCS() + ", " + robot.indexer.shouldCheckMidCS() + ", " + robot.indexer.shouldCheckRightCS());
            telemetry.addData("  left, mid, right CS ball color", robot.indexer.getLeftCSColor() + ", " + robot.indexer.getMidCSColor() + ", " + robot.indexer.getRightCSColor());
//
            telemetry.addLine("");
            telemetry.addLine("TRANSFER SPECIFIC-----");
            telemetry.addData("  state", robot.transfer.getTransferState());
//
//
            telemetry.addLine();
            telemetry.addLine("INTAKE-----");
            telemetry.addData("  state", robot.intake.getIntakeState());
            telemetry.addData("  power", robot.intake.getIntakePower());

            telemetry.addLine("PARK----");
            telemetry.addData("  state", robot.parker.getParkState());
            telemetry.addData("  encoder", robot.parker.getMotorPos());
            telemetry.addLine();
//
            telemetry.addLine("LIGHT----");
            telemetry.addData("rgb value", robot.rgbLight.getLightValue());
//
            telemetry.addLine("PEDRO/LIMELIGHT----");
            telemetry.addData("  turret pos", robot.limelight.getTurretPos());
            telemetry.addData("  turret heading", robot.limelight.getTurretHeading());
            telemetry.addData("  robot pos", robot.limelight.getRobotPos());
            telemetry.addData("  robot heading", robot.limelight.getRobotHeading());
            telemetry.addData("  x", robot.getX());
            telemetry.addData("  y", robot.getY());
            telemetry.addData("  heading (deg)", robot.getHeading()*180/Math.PI);
            telemetry.addData("goalX", robot.getGoalX());
            telemetry.addData("goalY", robot.getGoalY());
            telemetry.addData("goal heading", Math.floor(robot.getGoalHeading() * 180 / Math.PI*100)/100);
            telemetry.update();
        }
    }
}

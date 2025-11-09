package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

import java.util.Arrays;

@Config
@TeleOp(name="TELE")
public class MainTele extends LinearOpMode {
    public static double x = 24, y = 72, a = 180;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2, new Pose(x, y, Math.toRadians(a)));
        Robot.params.autoDone = false;
        robot.initPedroTele();
        robot.indexer.setAutoBallList(1, ColorSensorBall.BallColor.N, ColorSensorBall.BallColor.N, ColorSensorBall.BallColor.N);
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
            telemetry.addData("x, b", g1.x() + ", " + g1.b());
            telemetry.addLine("x to set blue team");
            telemetry.addLine("b to set red team");
            telemetry.addData("cur team", Robot.params.red ? "RED" : "BLUE");
            telemetry.addData("startPose", robot.follower.getPose().getX() + ", " + robot.follower.getPose().getY() + ", " + Math.floor(robot.follower.getPose().getHeading() * 180/Math.PI));
            telemetry.addData("goalPos", robot.getGoalX() + ", " + robot.getGoalY());
            telemetry.update();
        }
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double prevSec = 0;
        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.updateTele();
//            telemetry.addData("april tags", robot.aprilTagDetector.getTagData());
//            telemetry.addData("location data", robot.aprilTagDetector.getLocationData(20));
//            telemetry.addLine("SHOOTER----");
//            telemetry.addData("goal error", robot.getGoalHeading() - robot.getHeading());
//            telemetry.addData("goal pos", robot.getGoalX() + ", " + robot.getGoalY());
//            telemetry.addData("vels", Arrays.toString(robot.transfer.vels.toArray()));
//            telemetry.addData("hoods", Arrays.toString(robot.transfer.hoods.toArray()));
//            telemetry.addData("  target and current power", robot.shooter.getTargetMotorPower() + ", " + Math.floor(robot.shooter.getShooterPower()*100/100));
//            telemetry.addData("  target and current veloc", robot.shooter.getTargetMotorVel() + ", " + Math.floor(robot.shooter.getShooterVelocity() * 100)/100);
//            telemetry.addData("  min vel, max vel", robot.shooter.getMinShooterVel() + ", " + robot.shooter.getMaxShooterVel());
//            telemetry.addData("  hood position", robot.shooter.getHoodPos());
//            telemetry.addData("  zone", robot.shooter.getZone());
//            telemetry.addData("resting", robot.shooter.isResting());
//            telemetry.addData("hood locked", robot.shooter.isHoodLocked());
//            telemetry.addData("dist", robot.shooter.goalDist());
//            telemetry.addLine();
//            telemetry.addData("fps", 1/(timer.seconds()-prevSec));
//            telemetry.addData("voltage", robot.getBatteryVoltage());
//            telemetry.addData("slow turn", robot.isSlowTurn());
            telemetry.addData("green pos", Robot.params.greenPos);
            telemetry.addData("cur pattern i", robot.indexer.getCurPatternI());

//            telemetry.addLine("INDEXER----");
//            telemetry.addData("  state", robot.indexer.getIndexerState());
//            telemetry.addData("  oscillate target", robot.indexer.getOscillateTargetEncoder());
//            telemetry.addData("  should oscillate", robot.indexer.shouldOscillate());
//            telemetry.addData("  encoder", robot.indexer.getIndexerEncoder());
//            telemetry.addData("  target", robot.indexer.getTargetEncoder());
//            telemetry.addData("  error", robot.indexer.getIndexerError());
//            telemetry.addData("  power", robot.indexer.getIndexerPower());
//            telemetry.addData("  pid selected", robot.indexer.getPidSelected());
//            telemetry.addLine();
//            telemetry.addData("  static shoot", robot.indexer.prettyMuchStaticShoot());
//            telemetry.addData("  static cs", robot.indexer.prettyMuchStaticCS());
//            telemetry.addData("  vel", robot.indexer.getIndexerVel());
//            telemetry.addData("  num balls", robot.indexer.getNumBalls());
//            telemetry.addData("  ballList", robot.indexer.getLabeledBalls());
//            telemetry.addData("  align offset", robot.indexer.getAlignShootOffset());
//            telemetry.addData("  intakeI, li, ri, si", robot.indexer.getIntakeI() + ", " + robot.indexer.getLeftIntakeI() + ", " + robot.indexer.getRightIntakeI() + ", " + robot.indexer.getShooterI());
//            telemetry.addData("  green pos, cur pattern i", Robot.params.greenPos + ", " + robot.indexer.getCurPatternI());
//            telemetry.addLine();
//            telemetry.addData("  auto rotate cued", robot.indexer.isAutoRotateCued());
//            telemetry.addData("  should auto rotate", robot.indexer.shouldAutoIndex());
//            telemetry.addData("  sc left, mid, right", robot.indexer.shouldCheckLeftCS() + ", " + robot.indexer.shouldCheckMidCS() + ", " + robot.indexer.shouldCheckRightCS());
//            telemetry.addData("  left, mid, right CS ball color", robot.indexer.getLeftCSColor() + ", " + robot.indexer.getMidCSColor() + ", " + robot.indexer.getRightCSColor());
//
//            telemetry.addLine("");
//            telemetry.addLine("TRANSFER SPECIFIC-----");
//            telemetry.addData("  state", robot.transfer.getTransferState());
//
//
//            telemetry.addLine();
//            telemetry.addLine("INTAKE-----");
//            telemetry.addData("  state", robot.intake.getIntakeState());
//            telemetry.addData("  power", robot.intake.getIntakePower());

            telemetry.addLine("PARK----");
            telemetry.addData("  state", robot.parker.getParkState());
            telemetry.addData("  encoder", robot.parker.getMotorPos());
            telemetry.addLine();

            telemetry.addLine("LIGHT----");
            telemetry.addData("binary value", robot.binaryLight.getLightValue());
            telemetry.addData("rgb value", robot.rgbLight.getLightValue());

            telemetry.addLine("PEDRO----");
            telemetry.addData("  x", robot.getX());
            telemetry.addData("  y", robot.getY());
            telemetry.addData("  heading (deg)", robot.getHeading()*180/Math.PI);
            telemetry.update();
            prevSec = timer.seconds();
        }
    }
}

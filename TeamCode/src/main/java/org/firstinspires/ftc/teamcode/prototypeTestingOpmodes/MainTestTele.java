package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.AutoFar;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="main test test tele")
public class MainTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2, new Pose(AutoFar.params.startX, AutoFar.params.startY, Math.toRadians(AutoFar.params.startA)));
        Robot.params.autoDone = false;
        robot.initPedroTele();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double prevSec = 0;
        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.updateTele();

            telemetry.addData("fps", 1/(timer.seconds()-prevSec));
            telemetry.addData("voltage", robot.getBatteryVoltage());
            telemetry.addData("slow turn", robot.isSlowTurn());
            telemetry.addData("green pos", Robot.params.greenPos);

            telemetry.addLine("INDEXER----");
            telemetry.addData("  state", robot.indexer.getIndexerState());
            telemetry.addData("  oscillate target", robot.indexer.getOscillateTargetEncoder());
            telemetry.addData("  should oscillate", robot.indexer.shouldOscillate());
            telemetry.addData("  encoder", robot.indexer.getIndexerEncoder());
            telemetry.addData("  target", robot.indexer.getTargetEncoder());
            telemetry.addData("  error", robot.indexer.getIndexerError());
            telemetry.addData("  power", robot.indexer.getIndexerPower());
            telemetry.addData("  pid selected", robot.indexer.getPidSelected());
            telemetry.addLine();
            telemetry.addData("  static shoot", robot.indexer.prettyMuchStaticShoot());
            telemetry.addData("  static cs", robot.indexer.prettyMuchStaticCS());
            telemetry.addData("  vel", robot.indexer.getIndexerVel());
            telemetry.addData("  num balls", robot.indexer.getNumBalls());
            telemetry.addData("  ballList", robot.indexer.getLabeledBalls());
            telemetry.addData("  find purple", robot.indexer.findBallI(ColorSensorBall.BallColor.P));
            telemetry.addData("  find green", robot.indexer.findBallI(ColorSensorBall.BallColor.G));
            telemetry.addData("  align offset", robot.indexer.getAlignIndexerOffset());
            telemetry.addData("  intakeI, li, ri, si", robot.indexer.getIntakeI() + ", " + robot.indexer.getLeftIntakeI() + ", " + robot.indexer.getRightIntakeI() + ", " + robot.indexer.getShooterI());
            telemetry.addData("  green pos, cur pattern i", Robot.params.greenPos + ", " + robot.indexer.getCurPatternI());
            telemetry.addLine();
            telemetry.addData("  auto rotate cued", robot.indexer.isAutoRotateCued());
            telemetry.addData("  should auto rotate", robot.indexer.shouldAutoIndex());
            telemetry.addData("  sc left, mid, right", robot.indexer.shouldCheckLeftCS() + ", " + robot.indexer.shouldCheckMidCS() + ", " + robot.indexer.shouldCheckRightCS());
            telemetry.addData("  left, mid, right CS ball color", robot.indexer.getLeftCSColor() + ", " + robot.indexer.getMidCSColor() + ", " + robot.indexer.getRightCSColor());

            telemetry.addLine("");
            telemetry.addLine("TRANSFER SPECIFIC-----");
            telemetry.addData("  state", robot.transfer.getTransferState());

            telemetry.addLine();
            telemetry.addLine("SHOOTER----");
            telemetry.addData("  target and current power", robot.shooter.getTargetMotorPower() + ", " + robot.shooter.getShooterPower());
            telemetry.addData("  target and current veloc", robot.shooter.getTargetMotorVel() + ", " + robot.shooter.getShooterVelocity());
            telemetry.addData("  hood position", robot.shooter.getHoodPos());
            telemetry.addData("  robot dist", robot.shooter.distance());
            telemetry.addData("  zone", robot.shooter.getZone());

            telemetry.addLine();
            telemetry.addLine("INTAKE-----");
            telemetry.addData("  state", robot.intake.getIntakeState());
            telemetry.addData("  power", robot.intake.getIntakePower());

            telemetry.addLine("PARK----");
            telemetry.addData("  state", robot.parker.getParkState());
            telemetry.addData("  encoder", robot.parker.getMotorPos());
            telemetry.addLine();

            telemetry.addLine("LIGHT----");
            telemetry.addData("value", robot.light.getLightValue());

            telemetry.addLine("PEDRO----");
            telemetry.addData("  x", robot.follower.getPose().getX());
            telemetry.addData("  y", robot.follower.getPose().getY());
            telemetry.addData("  heading (deg)", robot.follower.getPose().getHeading()*180/Math.PI);
            telemetry.update();
            prevSec = timer.seconds();
        }
    }
}

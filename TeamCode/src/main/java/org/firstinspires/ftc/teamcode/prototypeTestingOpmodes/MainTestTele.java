package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="main test test tele")
public class MainTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2);
        robot.initPedroTele();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double prevSec = 0;
        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.update();
            robot.intake.update();
            robot.indexer.update();
            robot.shooter.update(timer.seconds());
            robot.parker.update();

            telemetry.addData("fps", 1/(timer.seconds()-prevSec));
            telemetry.addData("voltage", robot.getBatteryVoltage());
            telemetry.addData("automated drive", robot.automatedDrive());
            telemetry.addData("green pos", Robot.params.greenPos);

            telemetry.addLine("INDEXER----");
            telemetry.addData("  indexer encoder", robot.indexer.getIndexerEncoder());
            telemetry.addData("  indexer target", robot.indexer.getTargetIndexerEncoder());
            telemetry.addData("  indexer error", robot.indexer.getIndexerError());
            telemetry.addData("  indexer power", robot.indexer.getIndexerPower());
            telemetry.addData("  pid selected", robot.indexer.getPidSelected());
            telemetry.addLine();
            telemetry.addData("  pretty much static", robot.indexer.prettyMuchStatic());
            telemetry.addData("  indexer vel", robot.indexer.getIndexerVel());
            telemetry.addData("  num balls", robot.indexer.getNumBalls());
            telemetry.addData("  ballList", robot.indexer.getLabeledBalls());
            telemetry.addData("  intakeI, li, ri, si", robot.indexer.getIntakeI() + ", " + robot.indexer.getLeftIntakeI() + ", " + robot.indexer.getRightIntakeI() + ", " + robot.indexer.getShooterI());
            telemetry.addData("  cur pattern i", robot.indexer.getCurPatternI());
            telemetry.addLine();
            telemetry.addData("auto rotate cued", robot.indexer.isAutoRotateCued());
            telemetry.addData("  should auto rotate", robot.indexer.shouldAutoIndex());
            telemetry.addData("  sc left, mid, right", robot.indexer.shouldCheckLeftCS() + ", " + robot.indexer.shouldCheckMidCS() + ", " + robot.indexer.shouldCheckRightCS());
            telemetry.addData("  left, mid, right CS ball color", robot.indexer.getLeftCSColor() + ", " + robot.indexer.getMidCSColor() + ", " + robot.indexer.getRightCSColor());

            telemetry.addLine("TRANSFER SPECIFIC-----");
            telemetry.addData("  transfer state", robot.indexer.getTransferState());

            telemetry.addLine();
            telemetry.addLine("SHOOTER----");
            telemetry.addData("should shoot", robot.shooter.getShouldShoot());
            telemetry.addData("  shooter power", robot.shooter.getShooterPower());
            telemetry.addData("  shooter velocity", robot.shooter.getShooterVelocity());
            telemetry.addData("  motor vel offset", robot.shooter.getMotorVelOffset());
            telemetry.addData("  target vel", robot.shooter.getTargetMotorVel());
            telemetry.addData("  hood position", robot.shooter.getHoodPos());
            telemetry.addData("  hood offset", robot.shooter.getHoodOffset());
            telemetry.addData("  robot dist", robot.shooter.distance());
            telemetry.addData("  zone", robot.shooter.getZone());

            telemetry.addLine();
            telemetry.addLine("INTAKE-----");
            telemetry.addData("  intake state", robot.intake.getIntakeState());
            telemetry.addData("  power", robot.intake.getIntakePower());

            telemetry.addLine("PARK----");
            telemetry.addData("  park state", robot.parker.getParkState());
            telemetry.addData("  park encoder", robot.parker.getMotorPos());

            telemetry.addLine("PEDRO----");
            telemetry.addData("  x", robot.follower.getPose().getX());
            telemetry.addData("  y", robot.follower.getPose().getY());
            telemetry.addData("  heading (deg)", robot.follower.getPose().getHeading()*180/Math.PI);
            telemetry.update();
            prevSec = timer.seconds();
        }
    }
}

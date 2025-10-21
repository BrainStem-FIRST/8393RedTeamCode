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
        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.intake.update();
            robot.indexer.update(timer.seconds());
            robot.shooter.update(timer.seconds());
            robot.updatePedroTele();

            telemetry.addData("timer seconds", timer.seconds());
            telemetry.addLine("SHOOTER SPECIFIC----");
            telemetry.addData("  shooter power", robot.shooter.getShooterPower());
            telemetry.addData("  target power", robot.shooter.getTargetMotorPower());
            telemetry.addData("  shooter velocity", robot.shooter.getShooterVelocity());
            telemetry.addData("  target vel", robot.shooter.getTargetMotorVel());
            telemetry.addData("  hood position", robot.shooter.getHoodPos());
            telemetry.addData("  robot dist", Math.sqrt(Math.pow(robot.follower.getPose().getX() - robot.goalX, 2) + Math.pow(robot.follower.getPose().getY() - robot.goalY, 2)));

            telemetry.addLine("SUBSYSTEM STATES---------------");
            telemetry.addData("  intake state", robot.intake.getIntakeState());
            telemetry.addData("  parker state", robot.parker.getParkState());
            telemetry.addData("  transfer state", robot.indexer.getTransferState());
            telemetry.addLine();

            telemetry.addLine("INDEXER SPECIFIC----");
            telemetry.addData("  indexer encoder", robot.indexer.getIndexerEncoder());
            telemetry.addData("  indexer target", robot.indexer.getTargetIndexerEncoder());
            telemetry.addData("  indexer error", robot.indexer.getIndexerError());
            telemetry.addData("  indexer power", robot.indexer.getIndexerPower());
            telemetry.addData("  use normal max power", robot.indexer.useNormalMaxPower());
            telemetry.addData("  pretty much static", robot.indexer.prettyMuchStatic());
            telemetry.addData("  num balls", robot.indexer.getNumBalls());
            telemetry.addLine();
            telemetry.addData("  left CS ball color", robot.indexer.getLeftCSColor());
            telemetry.addData("  mid CS ball color", robot.indexer.getMidCSColor());
            telemetry.addData("  right CS ball color", robot.indexer.getRightCSColor());
            telemetry.addData("  sc leftCS", robot.indexer.shouldCheckLeftCS());
            telemetry.addData("  sc midCS", robot.indexer.shouldCheckMidCS());
            telemetry.addData("  sc rightCS", robot.indexer.shouldCheckRightCS());
            telemetry.addData("  ballList", robot.indexer.getBallAt(0) + ", " + robot.indexer.getBallAt(1)
                                                + ", " + robot.indexer.getBallAt(2) + ", " + robot.indexer.getBallAt(3)
                                                + ", " + robot.indexer.getBallAt(4) + ", " + robot.indexer.getBallAt(5));
            telemetry.addData("  intakeI, li, ri, si", robot.indexer.getIntakeI() + ", " + robot.indexer.getLeftIntakeI() + ", " + robot.indexer.getRightIntakeI() + ", " + robot.indexer.getShooterI());
//            telemetry.addData("  indexer align offset", robot.indexer.getAlignIndexerOffset());
            telemetry.addData("  cur patternI", robot.indexer.getCurPatternI());

            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("INTAKE SPECIFIC-----");
            telemetry.addData("  power", robot.intake.getIntakePower());

            telemetry.addLine("PARK SPECIFIC----");
            telemetry.addData("park encoder", robot.parker.getMotorPos());

            telemetry.addLine("PEDRO SPECIFIC----");
            telemetry.addData("  x", robot.follower.getPose().getX());
            telemetry.addData("  y", robot.follower.getPose().getY());
            telemetry.addData("  heading (deg)", robot.follower.getPose().getHeading()*180/Math.PI);
            telemetry.update();
        }
    }
}

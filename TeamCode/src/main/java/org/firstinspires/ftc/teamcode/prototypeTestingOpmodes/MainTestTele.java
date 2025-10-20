package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.intake.update();
            robot.indexer.update();
            robot.shooter.update();
            robot.updatePedroTele();

            telemetry.addLine("SUBSYSTEM STATES---------------");
            telemetry.addData("  intake state", robot.intake.getIntakeState());
            telemetry.addData("  parker state", robot.parker.getParkState());
            telemetry.addData("  transfer state", robot.indexer.getTransferState());

            telemetry.addLine("INDEXER SPECIFIC----");
            telemetry.addData("  indexer encoder", robot.indexer.getIndexerEncoder());
            telemetry.addData("  target indexer encoder", robot.indexer.getTargetIndexerEncoder());
            telemetry.addData("  indexer power", robot.indexer.getIndexerPower());
            telemetry.addData("  num balls", robot.indexer.getNumBalls());
            telemetry.addData(  "current indexer max power", robot.indexer.getCurrentIndexerMaxPower());
            telemetry.addData("  left ball color", robot.indexer.getLeftCSColor());
            telemetry.addData("  right ball color", robot.indexer.getRightCSColor());

            telemetry.addLine("SHOOTER SPECIFIC----");
            telemetry.addData("  shooter power", robot.shooter.getShooterPower());
            telemetry.addData("  hood position", robot.shooter.getHoodPos());

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

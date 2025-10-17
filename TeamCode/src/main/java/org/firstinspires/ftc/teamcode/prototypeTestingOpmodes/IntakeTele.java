package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="intake test tele")
public class IntakeTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2);
        robot.initPedroTele();
        waitForStart();

        while (opModeIsActive()) {
            g1.update();
            g2.update();
            robot.intake.update();
            robot.updatePedroTele();

            telemetry.addData("intake state", robot.intake.getState());
            telemetry.addData("x", robot.follower.getPose().getX());
            telemetry.addData("y", robot.follower.getPose().getY());
            telemetry.addData("heading (deg)", robot.follower.getPose().getHeading()*180/Math.PI);
            telemetry.update();
        }
    }
}

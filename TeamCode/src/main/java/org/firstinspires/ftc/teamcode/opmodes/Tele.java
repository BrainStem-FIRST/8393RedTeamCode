package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="tele")
public class Tele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);

        Robot robot = new Robot(hardwareMap, telemetry, g1, g2);
        robot.initPedroTele();

        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            g2.update();
            robot.updatePedroTele();
            robot.intake.update();
            robot.parker.update();

            telemetry.update();
        }
    }
}

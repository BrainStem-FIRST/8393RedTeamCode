package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="tele")
public class Tele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, new GamepadTracker(gamepad1), new GamepadTracker(gamepad2), true);

        waitForStart();
        while(opModeIsActive()) {
            robot.update();
            telemetry.update();
        }
    }
}

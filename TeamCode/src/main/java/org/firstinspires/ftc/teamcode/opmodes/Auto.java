package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    private Robot robot;
    private int pathNum;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        robot = new Robot(hardwareMap, telemetry, new GamepadTracker(gamepad1), new GamepadTracker(gamepad2));
        pathNum = 0;

        waitForStart();

        while(opModeIsActive()) {
            updateAuto();
            telemetry.update();
        }
    }

    private void updateAuto() {
        switch(pathNum) {
            case 0:
                break;
            case 1:
                break;
        }
    }

}

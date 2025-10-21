package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

import java.util.Arrays;

@TeleOp(name="color sensor test")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        boolean mode = true;

        GamepadTracker g1 = new GamepadTracker(gamepad1);
        Robot r = new Robot(hardwareMap, telemetry, g1, new GamepadTracker(gamepad2));
        ColorSensorBall leftCS = new ColorSensorBall(r, "leftCS");
        ColorSensorBall rightCS = new ColorSensorBall(r, "rightCS");
        ColorSensorBall midCS = new ColorSensorBall(r, "midCS");

        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            double t1 = System.currentTimeMillis();
            if(g1.isFirstX())
                mode = !mode;
            telemetry.addData("mode", mode);

            leftCS.update();
            rightCS.update();
            midCS.update();
            if(mode) {
                telemetry.addData("leftCS color", leftCS.getBallColor());
                telemetry.addData("left CS rgb", leftCS.getRGBS());
                telemetry.addLine();
                telemetry.addData("rightCS color", rightCS.getBallColor());
                telemetry.addData("right CS rgb", rightCS.getRGBS());
                telemetry.addLine();
                telemetry.addData("midCS color", midCS.getBallColor());
                telemetry.addData("mid CS rgb", midCS.getRGBS());
                telemetry.addLine();
            }

            double t2 = System.currentTimeMillis();
            double dt = (t2 - t1) / 1000.0;

            telemetry.addData("main fps", 1/dt);
            telemetry.update();
        }
    }
}

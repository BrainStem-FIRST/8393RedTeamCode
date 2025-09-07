package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Configurable
@TeleOp(name="multithreading")
public class MultiThreadingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);
        boolean mode = true;

        ColorSensor c1 = hardwareMap.get(ColorSensor.class, "c1");
        ColorSensor c2 = hardwareMap.get(ColorSensor.class, "c2");
        ColorSensor c3 = hardwareMap.get(ColorSensor.class, "c3");



        waitForStart();
        while(opModeIsActive()) {
            double t1 = System.currentTimeMillis();
            mode = gamepad1.x;
            telemetry.addData("mode", mode);
            if(mode) {
                if (gamepad1.a) {
                    telemetry.addData("c1", c1.red() + ", " + c1.green() + ", " + c1.blue());
                }
                if (gamepad1.b) {
                    telemetry.addData("c2", c2.red() + ", " + c2.green() + ", " + c2.blue());
                }
                if (gamepad1.y) {
                    telemetry.addData("c3", c3.red() + ", " + c3.green() + ", " + c3.blue());
                }
            }
            else {
                if (gamepad1.a) {
                    int color = c1.argb();
                    telemetry.addData("c1", Color.red(color) + ", " + Color.green(color) + ", " + Color.blue(color));
                }
                if (gamepad1.b) {
                    int color = c2.argb();
                    telemetry.addData("c2", Color.red(color) + ", " + Color.green(color) + ", " + Color.blue(color));
                }
                if (gamepad1.y) {
                    int color = c3.argb();
                    telemetry.addData("c3", Color.red(color) + ", " + Color.green(color) + ", " + Color.blue(color));
                }
            }

            double t2 = System.currentTimeMillis();
            double dt = (t2 - t1) / 1000.0;

            telemetry.addData("main fps", 1/dt);
            telemetry.update();
        }
    }
}

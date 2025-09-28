package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="Motor Test Tele")
public class MotorTest extends LinearOpMode {
    public static String motorName = "motor", servoName = "servo";
    private final DcMotorEx[] motors = new DcMotorEx[4];
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        GamepadTracker g1 = new GamepadTracker(gamepad1);
        for(int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, "motor" + i);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double motorPower = 0.3;
        boolean motorOn = false;

        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            if(g1.isFirstDpadUp())
                motorPower += 0.05;
            else if(g1.isFirstDpadDown())
                motorPower -= 0.05;

            else if(g1.isFirstA())
                motorOn = !motorOn;



            if(motorOn)
                setMotorPowers(motorPower);
            else
                setMotorPowers(0);

            telemetry.addData("motor on", motorOn);
            telemetry.addData("motor powers", motorPower);
            telemetry.addData("motor port 0 current", motors[0].getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine("press a to toggle motors");
            telemetry.addLine("dpad up and down to increment motor powers");
            telemetry.update();
        }
    }

    private void setMotorPowers(double power) {
        for(int i = 0; i < 4; i++)
            motors[i].setPower(power);
    }
}

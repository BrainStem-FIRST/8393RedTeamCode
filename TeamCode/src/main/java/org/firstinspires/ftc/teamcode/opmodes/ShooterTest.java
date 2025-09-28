package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadTracker g1 = new GamepadTracker(gamepad1);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx pivot = hardwareMap.get(ServoImplEx.class, "pivot");
        pivot.setPwmRange(new PwmControl.PwmRange(500, 2000));

        double power = 0.99;
        boolean on = false;


        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            if(g1.isFirstA())
                on = !on;

            if(on)
                motor.setPower(power);
            else
                motor.setPower(0);

            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}

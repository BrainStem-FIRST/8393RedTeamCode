package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Motor power test tele")
public class MotorPowerTestTele extends LinearOpMode {
    public static String motorName = "BR";
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            motor.setPower(-gamepad1.right_stick_y);

            telemetry.addData("gamepad1 right y", gamepad1.right_stick_y);
            telemetry.addData("power", motor.getPower());
            telemetry.update();
        }
    }
}

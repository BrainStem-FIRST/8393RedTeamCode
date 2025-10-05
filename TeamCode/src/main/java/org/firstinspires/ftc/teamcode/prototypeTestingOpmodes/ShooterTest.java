package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="shooter test tele")
public class ShooterTest extends LinearOpMode {
    private DcMotorEx shooter1, shooter2;

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadTracker g1 = new GamepadTracker(gamepad1);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx pivot = hardwareMap.get(ServoImplEx.class, "pivot");
        pivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pivot.setPosition(0.5);

        double shooterPower = 0;

        waitForStart();
        while(opModeIsActive()) {
            g1.update();

            // pivot angle gamepad input
            if(g1.isFirstDpadLeft())
                pivot.setPosition(pivot.getPosition() + 0.02);
            else if(g1.isFirstDpadRight())
                pivot.setPosition(pivot.getPosition() - 0.02);
            else if(g1.gamepad.dpad_up)
                pivot.setPosition(pivot.getPosition() + 0.01);
            else if(g1.gamepad.dpad_down)
                pivot.setPosition((pivot.getPosition() - 0.01));
            // shooter power gamepad input
            if(g1.isFirstY())
                shooterPower += 0.025;
            else if(g1.isFirstA())
                shooterPower -= 0.025;
            if(g1.isFirstX())
                shooterPower = 0;
            setShooterPower(shooterPower);

            telemetry.addData("shooter power", shooterPower);
            telemetry.addData("pivot pos", pivot.getPosition());
            telemetry.update();
        }
    }
    private void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }
}

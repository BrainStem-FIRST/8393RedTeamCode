package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="shooter test tele")
public class ShooterTest extends LinearOpMode {
    private DcMotorEx shooter1, shooter2;
    private CRServo pivot1, pivot2;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadTracker g1 = new GamepadTracker(gamepad1);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivot1 = hardwareMap.get(CRServo.class, "pivot1");
        pivot2 = hardwareMap.get(CRServo.class, "pivot2");

        double shooterPower = 0;
        double pivotPower = 0.5;
        double currentPivotPower;

        waitForStart();
        while(opModeIsActive()) {
            g1.update();

            // pivot angle gamepad input
            if(g1.isFirstDpadLeft())
                pivotPower += 0.05;
            else if(g1.isFirstDpadRight())
                pivotPower -= 0.05;

            if(g1.isFirstDpadUp())
                currentPivotPower = pivotPower;
            else if(g1.isFirstDpadDown())
                currentPivotPower = -pivotPower;
            else
                currentPivotPower = 0;

            setPivotPower(currentPivotPower);

            // shooter power gamepad input
            if(g1.isFirstY())
                shooterPower += 0.05;
            else if(g1.isFirstA())
                shooterPower -= 0.05;
            if(g1.isFirstX())
                shooterPower = 0;
            setShooterPower(shooterPower);

            telemetry.addData("shooter power", shooterPower);
            double x = (shooter1.getVelocity(AngleUnit.DEGREES) + shooter2.getVelocity(AngleUnit.DEGREES))*0.5;
            telemetry.addData("shooter avg vel (deg)", Math.floor(x*100)/100);
            telemetry.addData("pivot power", currentPivotPower);
            telemetry.update();
        }
    }
    private void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }
    private void setPivotPower(double power) {
        pivot1.setPower(-power);
        pivot2.setPower(power);
    }
}

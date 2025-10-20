package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="shooter test tele")
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2);
        robot.initPedroTele();
        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            robot.updatePedroTele();
            robot.intake.update();
            robot.indexer.update();

            robot.shooter.update();
            telemetry.addData("shooter state", robot.shooter.getState());

            telemetry.addData("shooter power", robot.shooter.getShooterPower());
            telemetry.update();
        }
    }
}

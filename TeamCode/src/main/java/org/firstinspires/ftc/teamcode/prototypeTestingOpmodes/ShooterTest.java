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
        boolean testing = true;
        double inc = 0.02;
        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            robot.updatePedroTele();

            if(testing) {
                // moving shooter
                if(g1.isFirstY())
                    robot.shooter._setShooterPower(robot.shooter.getShooterPower()+inc);
                else if(g1.isFirstA())
                    robot.shooter._setShooterPower(robot.shooter.getShooterPower()-inc);
                else if(g1.isFirstX())
                    robot.shooter._setShooterPower(0);

                // moving pivot
                if(gamepad1.dpad_up)
                    robot.shooter._setHoodPower(0.1);
                else if(gamepad1.dpad_down)
                    robot.shooter._setHoodPower(-0.1);
                else if(gamepad1.dpad_left)
                    robot.shooter._setHoodPower(0.5);
                else if(gamepad1.dpad_right)
                    robot.shooter._setHoodPower(-0.5);
                else
                    robot.shooter._setHoodPower(0);

                // moving transfer
                if(gamepad1.right_bumper)
                    robot.indexer._setTransferPower(0.99);
                else if(gamepad1.left_bumper)
                    robot.indexer._setTransferPower(-0.99);
                else if(gamepad1.right_trigger > 0.1)
                    robot.indexer._setTransferPower(0);
            }
            else {
                robot.shooter.update();
                telemetry.addData("shooter state", robot.shooter.getState());
            }
            telemetry.addData("shooter power", robot.shooter.getShooterPower());
            telemetry.addData("hood power", robot.shooter._getHoodPower());
            telemetry.addData("transfer power", robot.indexer.getTransferPower());
            telemetry.update();
        }
    }
}

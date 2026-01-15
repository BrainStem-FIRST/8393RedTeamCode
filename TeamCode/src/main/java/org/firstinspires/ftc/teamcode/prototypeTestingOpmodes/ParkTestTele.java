package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="park test tele")
public class ParkTestTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        GamepadTracker g2 = new GamepadTracker(gamepad2);
        Robot robot = new Robot(hardwareMap, telemetry, g1, g2);
        robot.initPedroTele();
        boolean testing = true;
        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            g2.update();

            if(g1.isFirstA())
                testing = !testing;
            if(g1.isFirstB())
                robot.parker._resetEncoder();

            if(testing)
                robot.parker._setMotorPower(-gamepad1.right_stick_y);
            else
                robot.parker.update();
            telemetry.addData("testing", testing);
            telemetry.addData("park encoder", robot.parker.getMotorPos());
            telemetry.addData("park power", robot.parker.getMotorPower());
            telemetry.addData("park state", robot.parker.getParkState());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="drivetrain test tele")
public class drivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        follower.update();

        waitForStart();
        while(opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, true);
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading (deg)", follower.getPose().getHeading() * 180 / Math.PI);
            telemetry.update();
        }
    }
}

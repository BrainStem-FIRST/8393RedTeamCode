package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@TeleOp(name="drivetrain test tele")
public class drivetrainTest extends LinearOpMode {
    private DcMotorEx fl, fr, bl, br;
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
        follower.update();

        boolean pedro = false;
        GamepadTracker g1 = new GamepadTracker(gamepad1);
        waitForStart();
        while(opModeIsActive()) {
            g1.update();
            if(g1.isFirstA())
                pedro = !pedro;

            if(pedro) {
                follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
                follower.update();
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading (deg)", follower.getPose().getHeading() * 180 / Math.PI);
            }
            else{
                setDrivePowers(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
            telemetry.addData("pedro", pedro);
            telemetry.update();
        }
    }
    public void setDrivePowers(double lateralPower, double axialPower, double headingPower) {
        lateralPower *= -1; // because we define left as positive lateral (it is how it is defined in the field coordinate space)
        double addValue = Math.round((100 * (axialPower * Math.abs(axialPower) + lateralPower * Math.abs(lateralPower)))) / 100.;
        double subtractValue = Math.round((100 * (axialPower * Math.abs(axialPower) - lateralPower * Math.abs(lateralPower)))) / 100.;
        //double addValue = Math.round((100 * (axialPower + lateralPower))) / 100.;
        //double subtractValue = Math.round((100 * (axialPower - lateralPower))) / 100.;

        setMotorPowers((addValue - headingPower), (subtractValue + headingPower), (subtractValue - headingPower), (addValue + headingPower));
    }
    private void setMotorPowers(double FL, double FR, double BL, double BR) {
        fl.setPower(FL);
        fr.setPower(FR);
        bl.setPower(BL);
        br.setPower(BR);
    }
}

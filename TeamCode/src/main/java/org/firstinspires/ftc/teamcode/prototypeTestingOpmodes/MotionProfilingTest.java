package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Indexer;
import org.firstinspires.ftc.teamcode.utils.math.Cos;

@Config
@TeleOp(name="test motion profiling")
public class MotionProfilingTest extends LinearOpMode {
    public static boolean rotateCued;
    public static double target = 0, time = 1;
    public static double kP = 0.01, kI = 0, kD = 0, minPower = 0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        CRServo indexer = hardwareMap.get(CRServo.class, "indexer");
        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "FL");
        PIDFController pid = new PIDFController(new PIDFCoefficients(kP, kI, kD, 0));
        Cos v = new Cos(0, 0, 0);
        double a = 0, b = 0, c = 0;
        double prevPos = 0, prevTime = 0;
        boolean rotating = false;
        double indexerV = 0, targetV = 0, indexerVError = 0;

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(rotateCued) {
                rotateCued = false;
                a = -(target - encoder.getCurrentPosition()) / time;
                b = 2 * Math.PI / time;
                c = -a;
                v = new Cos(a, b, c);
                timer.reset();
                prevPos = encoder.getCurrentPosition();
                prevTime = 0;
                rotating = true;
            }
            else if(timer.seconds() < time) {
                if(rotating) {
                    indexerV = (encoder.getCurrentPosition() - prevPos) / (timer.seconds() - prevTime);
                    targetV = v.eval(timer.seconds());
                    indexerVError = targetV - indexerV;
                    pid.updateError(indexerVError);
                    double power = pid.run();
                    indexer.setPower((Math.abs(power) + minPower) * Math.signum(power));
                    prevTime = timer.seconds();
                }
            }
            else {
                rotating = false;
                indexer.setPower(0);
            }

            telemetry.addData("rotating", rotating);
            telemetry.addData("rotateCued", rotateCued);
            telemetry.addData("encoder", encoder.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("time", timer.seconds());
            telemetry.addData("v target", v.eval(timer.seconds()));
            telemetry.addData("v indexer", indexerV);
            telemetry.addData("power", indexer.getPower());
            telemetry.addData("a" ,a);
            telemetry.addData("b", b);
            telemetry.addData("c", c);
            telemetry.update();
        }
    }
}

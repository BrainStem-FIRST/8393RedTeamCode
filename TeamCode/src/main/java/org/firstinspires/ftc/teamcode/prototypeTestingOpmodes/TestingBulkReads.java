package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

import java.util.List;

@TeleOp(name="testing bulk reads")
@Config
public class TestingBulkReads extends LinearOpMode {
    public static boolean bulkReadOn = false;
    public static boolean readSensors = false;
    public static int frameTrackLength = 10;
    public static int repeatChecks = 1;
    private double shooterE = 0, indexE = 0, parkE = 0, hoodPos = 0;
    DcMotorEx shooter1, shooter2, park;
    DcMotorEx indexerEncoder;
    ServoImplEx hood, transfer, transferStopper;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        GamepadTracker g1 = new GamepadTracker(gamepad1);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        if(bulkReadOn)
            for (LynxModule hub : allHubs)
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        else
            for (LynxModule hub : allHubs)
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);


        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        park = hardwareMap.get(DcMotorEx.class, "park");
        indexerEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        hood = hardwareMap.get(ServoImplEx.class, "hoodServo");
        transfer = hardwareMap.get(ServoImplEx.class, "transfer");
        transferStopper = hardwareMap.get(ServoImplEx.class, "transferStopper");

        int i = 0;
        double sum = 0;

        waitForStart();
        double prev = System.currentTimeMillis();
        while (opModeIsActive()) {
            g1.update();

            if(bulkReadOn)
                for (LynxModule hub : allHubs)
                    hub.clearBulkCache();

            if(readSensors)
                for(int j = 0; j < repeatChecks; j++)
                    checkEncoders();
            double cur = System.currentTimeMillis();
            double dt = cur - prev;
            if(i == 0)
                sum = 0;
            else
                sum += dt;
            i = (i + 1) % frameTrackLength;
            telemetry.addData("dt (ms)", dt);
            telemetry.addData("avg dt (ms)", sum / i);
            telemetry.addData("bulk read on", bulkReadOn);
            telemetry.addData("read sensors", readSensors);
            telemetry.addData("shooter", shooterE);
            telemetry.addData("indexE", indexE);
            telemetry.addData("parkE", parkE);
            telemetry.addData("hoodE", hoodPos);
            prev = cur;
            telemetry.update();
        }
    }
    private void checkEncoders() {
        shooterE = shooter1.getCurrentPosition();
        indexE = indexerEncoder.getCurrentPosition();
        parkE = park.getCurrentPosition();
        hoodPos = hood.getPosition();
        transferStopper.setPosition(0.5);
        transfer.setPosition(0.5);

    }
}

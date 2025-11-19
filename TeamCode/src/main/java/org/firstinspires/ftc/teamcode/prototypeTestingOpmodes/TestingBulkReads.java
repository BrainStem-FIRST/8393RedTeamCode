package org.firstinspires.ftc.teamcode.prototypeTestingOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

import java.util.List;

@TeleOp(name="testing bulk reads")
public class TestingBulkReads extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        GamepadTracker g1 = new GamepadTracker(gamepad1);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        DcMotorEx park = hardwareMap.get(DcMotorEx.class, "park");
        DcMotorEx indexerEncoder = hardwareMap.get(DcMotorEx.class, "FL");

        boolean bulkReadOn = true;
        double prev = System.currentTimeMillis();
        while (opModeIsActive()) {
            g1.update();

            if(g1.isFirstA()) {
                bulkReadOn = !bulkReadOn;
                if(!bulkReadOn)
                    for(LynxModule hub : allHubs)
                        hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                else
                    for(LynxModule hub : allHubs)
                        hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            if(bulkReadOn)
                for (LynxModule hub : allHubs)
                    hub.clearBulkCache();

            if(g1.b()) {
                telemetry.addData("s1", shooter1.getCurrentPosition());
                telemetry.addData("s2", shooter2.getCurrentPosition());
                park.getCurrentPosition();
                indexerEncoder.getCurrentPosition();
            }
            double cur = System.currentTimeMillis();
            telemetry.addData("dt (ms)", cur - prev);
            telemetry.addData("bulk read on", bulkReadOn);
            prev = cur;
        }
    }
}

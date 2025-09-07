package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;

public class MultiThreadingTest2 extends OpMode {
    private volatile boolean running;
    private volatile String threadOutput;
    private ColorSensor c1, c2, c3;
    private void initThread() {
//        Thread thread = new Thread() {
//            @Override
//            public void run() {
//                while(running) {
//                    double prevTime = System.currentTimeMillis();
//                    c2.
//                    threadOutput = "c1" + ", " + c1.red() + ", " + c1.green() + ", " + c1.blue();
//
//                    double dt = System.currentTimeMillis() - prevTime;
//                    telemetry.addData("thread fps", 1000./dt);
//                }
//            }
//        };
//        thread.start();
    }
    @Override
    public void init() {
        initThread();
    }

    @Override
    public void loop() {

    }
}

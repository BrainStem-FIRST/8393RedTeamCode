package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final GamepadTracker g1, g2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadTracker g1, GamepadTracker g2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.g1 = g1;
        this.g2 = g2;
    }
}

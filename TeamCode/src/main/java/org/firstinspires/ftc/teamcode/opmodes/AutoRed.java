package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class AutoRed extends AutoNear {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.params.red = true;
        super.runOpMode();
    }
}

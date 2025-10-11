package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Parker {
    public static final int upEncoder = 100;
    public static final double minUpPower = -0.3;
    private final Robot robot;
    private final DcMotorEx parkMotor;
    private final PIDController pid;
    private double motorPower;

    private enum ParkState {
        DOWN, UP
    }
    private ParkState parkState;
    public Parker(Robot robot) {
        this.robot = robot;
        parkMotor = robot.hardwareMap.get(DcMotorEx.class, "parker");
        parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parkMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pid = new PIDController(0, 0, 0);
        pid.setTarget(upEncoder);
        motorPower = 0;

        parkState = ParkState.DOWN;
    }

    public void update() {
        switch(parkState) {
            case DOWN:
                if(robot.g1.isFirstX()) {
                    motorPower = getUpPower();
                    parkState = ParkState.UP;
                }
                break;
            case UP:
                if(robot.g1.isFirstX()) {
                    motorPower = 0;
                    parkState = ParkState.DOWN;
                }
                else
                    motorPower = getUpPower();
                break;
        }
        parkMotor.setPower(motorPower);
    }
    private int getMotorPos() {
        return parkMotor.getCurrentPosition();
    }
    private double getUpPower() {
        return Math.min(minUpPower, pid.update(getMotorPos()));
    }
}

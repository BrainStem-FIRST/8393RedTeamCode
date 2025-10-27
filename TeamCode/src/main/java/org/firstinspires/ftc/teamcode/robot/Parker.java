package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Parker {
    public static final int upEncoder = 6800;
    public static final double minUpPower = 0.15;
    private final Robot robot;
    private final DcMotorEx parkMotor;
    private final PIDController pid;
    private double motorPower, prevMotorPower;

    public enum ParkState {
        DOWN, UP
    }
    private ParkState parkState;
    public Parker(Robot robot) {
        this.robot = robot;
        parkMotor = robot.hardwareMap.get(DcMotorEx.class, "park");
        parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parkMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(0.01, 0, 0);
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
        if(motorPower != prevMotorPower)
            parkMotor.setPower(motorPower);
        prevMotorPower = motorPower;
    }
    public int getMotorPos() {
        return parkMotor.getCurrentPosition();
    }
    public double getMotorPower() {
        return parkMotor.getPower();
    }
    public ParkState getParkState() {
        return parkState;
    }
    private double getUpPower() {
        double pidPower = pid.update(getMotorPos());
        return Math.max(minUpPower, pidPower);
    }
    public void _setMotorPower(double power) {
        parkMotor.setPower(power);
    }
    public void _resetEncoder() {
        parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parkMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

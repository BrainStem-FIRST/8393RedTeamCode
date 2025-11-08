package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Parker {
    public static final int bridgeEncoder = 7650, kickStandEncoder = 1300;
    public static final double minUpPower = 0.2;
    private final Robot robot;
    private final DcMotorEx parkMotor;
    private final PIDController pid;
    private double motorPower, prevMotorPower;
    private int motorPos;


    public enum ParkState {
        DOWN, KICKSTAND, BRIDGE
    }
    private ParkState parkState;
    public Parker(Robot robot) {
        this.robot = robot;
        parkMotor = robot.hardwareMap.get(DcMotorEx.class, "park");
        parkMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parkMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(0.01, 0, 0);
        motorPower = 0;
        parkState = ParkState.DOWN;
    }

    public void update() {
        motorPos = parkMotor.getCurrentPosition();
        switch(parkState) {
            case DOWN:
                if(robot.g1.isFirstX()) {
                    pid.setTarget(kickStandEncoder);
                    motorPower = getUpPower();
                    parkState = ParkState.KICKSTAND;
                }
                break;
            case KICKSTAND:
                if(robot.g1.isFirstX()) {
                    pid.setTarget(bridgeEncoder);
                    parkState = ParkState.BRIDGE;
                }
                motorPower = getUpPower();
                break;
            case BRIDGE:
                if(robot.g1.isFirstX()) {
                    parkState = ParkState.DOWN;
                    motorPower = 0;
                }
        }
        if(Math.abs(robot.g2.rightStickY()) > 0.1)
            motorPower = -robot.g2.rightStickY();
        if(motorPower != prevMotorPower)
            parkMotor.setPower(motorPower);
        prevMotorPower = motorPower;
    }
    public int getMotorPos() {
        return motorPos;
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

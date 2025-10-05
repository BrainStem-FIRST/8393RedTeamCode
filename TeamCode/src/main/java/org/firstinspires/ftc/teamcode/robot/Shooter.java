package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {

    public static final double triggerPressThreshold = 0.1;
    public static final double shootPower = 0.99; //TODO: what power works here?
    private final Robot robot;

    // OFF: motor is unmoving
    // SHOOTING: motor is running to shoot the balls from the indexer
    public enum State {
        OFF, SHOOTING
    }

    private State state;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private double motorPower;

    public Shooter(Robot robot) {
        this.robot = robot;
        state = Shooter.State.OFF;

        motor1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorPower = 0;
    }

    public void update() {
        switch(state){
            case OFF:
                if (!shootTriggerPressed()) {
                    setStateOff();
                    setShooterPower(motorPower);
                }
                break;
            case SHOOTING:
                if (shootTriggerPressed()) {
                    setStateShooting();
                    setShooterPower(motorPower);

                }
                break;
        }
    }

    private void setStateOff(){
        state = State.OFF;
        motorPower = 0;
    }

    private void setStateShooting(){
        state = State.SHOOTING;
        motorPower = shootPower;
    }
    private boolean shootTriggerPressed() {
        // checking in-taking
        return robot.g1.gamepad.right_trigger > triggerPressThreshold;
    }

    private void setShooterPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }
    private int getMotor1Pos() {
        return motor1.getCurrentPosition();
    }
    private int getMotor2Pos() {
        return motor2.getCurrentPosition();
    }

}
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    public static double postCollectTime = 0.8;
    public static double collectPower = 0.99;
    private final Robot robot;

    // state descriptions:
    // OFF: motor not moving
    // COLLECTING: motor running to intake
    public enum IntakeState {
        OFF, COLLECTING, POST_COLLECTING
    }

    private IntakeState intakeState;
    private final DcMotorEx intake;
    private double motorPower, prevMotorPower;
    private final ElapsedTime postCollectTimer;
    private boolean shouldIntake, intakeSafely;
    public Intake(Robot robot) {
        this.robot = robot;
        intakeState = IntakeState.OFF;
        intake = robot.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = 0;
        postCollectTimer = new ElapsedTime();
    }
    public void update() {
        switch(intakeState) {
            case POST_COLLECTING:
                if(postCollectTimer.seconds() > postCollectTime)
                    setStateOff();
            case OFF:
                if(intakeSafely && !robot.indexer.prettyMuchStatic())
                    return;

                if(listenCollectInput() || shouldIntake)
                    setStateCollecting(collectPower);
                else if(listenExtakeInput())
                    setStateCollecting(-collectPower);
                break;
            case COLLECTING:
                if(intakeSafely && !robot.indexer.prettyMuchStatic()) // auto checking for indexer movement
                    setStateOff();
                else if(!listenCollectInput() && !listenExtakeInput() && !shouldIntake)
                    setStatePostCollecting();
                break;
        }
        if(motorPower != prevMotorPower)
            intake.setPower(motorPower);
        prevMotorPower = motorPower;
    }
    private void setStatePostCollecting() {
        postCollectTimer.reset();
        motorPower = 0;
        intakeState = IntakeState.POST_COLLECTING;
    }
    private void setStateOff() {
        intakeState = IntakeState.OFF;
    }
    private void setStateCollecting(double power) {
        intakeState = IntakeState.COLLECTING;
        motorPower = power;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public double getIntakePower() {
        return intake.getPower();
    }
    private boolean listenCollectInput() {
        return robot.g1.rightTrigger() > 0.1;
    }
    private boolean listenExtakeInput() { return robot.g1.leftTrigger() > 0.1; }
    public void setIntake(boolean shouldIntake) {
        this.shouldIntake = shouldIntake;
    }
    public void setIntakeSafely(boolean intakeSafely) {
        this.intakeSafely = intakeSafely;
    }
}

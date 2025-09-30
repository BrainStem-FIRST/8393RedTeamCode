package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    public static final double triggerPressThreshold = 0.1;
    public static final double collectPower = 0.99;
    private final Robot robot;

    // state descriptions:
    // OFF: motor not moving
    // COLLECTING: motor running to intake
    public enum State {
        OFF, COLLECTING
    }

    private State state;
    private final DcMotorEx intake;
    private double motorPower;
    public Intake(Robot robot) {
        this.robot = robot;
        state = State.OFF;
        intake = robot.hardwareMap.get(DcMotorEx.class, "intakeIndexer");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = 0;
    }

    public void update() {
        switch(state) {
            case OFF:
                if(listenCollectToggleInput())
                    setStateCollecting(collectPower);
                break;
            case COLLECTING:
                if(listenCollectToggleInput() || robot.indexer.getNumBalls() == 3)
                    setStateOff();
                else if(robot.indexer.getState() == Indexer.State.INDEXING)
                    motorPower = 0;
                else
                    motorPower = collectPower;
                break;
        }
        intake.setPower(motorPower);
    }

    public void setStateOff() {
        state = State.OFF;
        motorPower = 0;
    }
    public void setStateCollecting(double power) {
        state = State.COLLECTING;
        motorPower = power;
    }

    private boolean listenCollectToggleInput() {
        return robot.g1.isFirstLeftBumper();
    }
}

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    public static double collectPower = 0.70, indexIntakePower = 0.1;
    private final Robot robot;

    // state descriptions:
    // OFF: motor not moving
    // COLLECTING: motor running to intake
    public enum IntakeState {
        OFF, COLLECTING
    }

    private IntakeState intakeState;
    private final DcMotorEx intake;
    private double motorPower;
    public Intake(Robot robot) {
        this.robot = robot;
        intakeState = IntakeState.OFF;
        intake = robot.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = 0;
    }
    public IntakeState getState() {
        return intakeState;
    }
    public void update() {
        switch(intakeState) {
            case OFF:
                if(listenCollectToggleInput())
                    setStateCollecting(collectPower);
                break;
            case COLLECTING:
                if(listenCollectToggleInput() || robot.indexer.getNumBalls() == 3)
                    setStateOff();
                else if(robot.indexer.getIndexerState() == Indexer.IndexerState.INDEXING)
                    motorPower = indexIntakePower;
                else
                    motorPower = collectPower;
                break;
        }
        intake.setPower(motorPower);
    }

    public void setStateOff() {
        intakeState = IntakeState.OFF;
        motorPower = 0;
    }
    public void setStateCollecting(double power) {
        intakeState = IntakeState.COLLECTING;
        motorPower = power;
    }

    private boolean listenCollectToggleInput() {
        return robot.g1.isFirstLeftBumper();
    }
}

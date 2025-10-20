package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    public static double collectPower = 0.99, indexIntakePower = 0.1;
    private final Robot robot;

    // state descriptions:
    // OFF: motor not moving
    // COLLECTING: motor running to intake
    public enum IntakeState {
        OFF, COLLECTING
    }

    private IntakeState intakeState;
    private final DcMotorEx intake;
    private double desiredMotorPower;
    public Intake(Robot robot) {
        this.robot = robot;
        intakeState = IntakeState.OFF;
        intake = robot.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        desiredMotorPower = 0;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public void update() {
        double motorPower = desiredMotorPower;
        switch(intakeState) {
            case OFF:
                if(robot.g1.gamepad.right_trigger > 0.1)
                    setStateCollecting(collectPower);
                else if(robot.g1.gamepad.left_trigger > 0.1)
                    setStateCollecting(-collectPower);
                break;
            case COLLECTING:
                if(robot.indexer.getNumBalls() == 3 || !listenCollectInput())
                    setStateOff();
                else if(Math.abs(robot.indexer.getIndexerError()) > Indexer.errorThreshold * 2)
                    motorPower = indexIntakePower;
                break;
        }
        intake.setPower(motorPower);
    }

    public void setStateOff() {
        intakeState = IntakeState.OFF;
        desiredMotorPower = 0;
    }
    public void setStateCollecting(double power) {
        intakeState = IntakeState.COLLECTING;
        desiredMotorPower = power;
    }

    private boolean listenCollectInput() {
        return robot.g1.gamepad.right_trigger > 0.1;
    }
}

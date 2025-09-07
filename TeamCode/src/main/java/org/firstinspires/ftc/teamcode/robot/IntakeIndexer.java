package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;

@Configurable
public class IntakeIndexer {
    public static final double triggerPressThreshold = 0.1;
    public static final double collectPower = 0.99;
    public static final int indexRotateAmount = 200; // encoders needed to rotate indexer 120 degrees
    public static final int indexerAlignedThreshold = 10;
    private final Robot robot;

    // state descriptions:
    // OFF: motor not moving, but INDEXER IS ALIGNED so all 3 color sensors can read
    // COLLECTING: motor running to intake
    // INDEXING: motor running to align indexer for ball to be shot
    public enum State {
        OFF, COLLECTING, INDEXING
    }

    private State state;
    private final DcMotorEx motor;
    private double motorPower;
    private final PIDController indexerPid;
    public IntakeIndexer(Robot robot) {
        this.robot = robot;
        state = State.OFF;
        motor = robot.hardwareMap.get(DcMotorEx.class, "intakeIndexer");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = 0;
        indexerPid = new PIDController(0.05, 0, 0);
    }

    public void update() {
        switch(state) {
            case OFF:
                // checking when driver collects/spits
                if(robot.g1.gamepad.left_trigger > triggerPressThreshold)
                    setStateCollecting(collectPower);
                else if(robot.g1.gamepad.left_bumper)
                    setStateCollecting(-collectPower);
                break;
            case COLLECTING:
                motor.setPower(collectPower);
                break;
            case INDEXING:
                if(indexerAligned())
                    setStateOff();
                else
                    motorPower = indexerPid.update(getIndexerEncoder());
                break;
        }
        motor.setPower(motorPower);
    }

    public int getIndexerEncoder() {
        return motor.getCurrentPosition();
    }
    public boolean indexerAligned() {
        return getIndexerEncoder() % indexRotateAmount < indexerAlignedThreshold;
    }
    public void alignIndexer() {
        // calculating closest "third" encoder to align current position with
        int dist = getIndexerEncoder() % indexRotateAmount;
        int target = getIndexerEncoder() / indexRotateAmount * indexRotateAmount;
        if(dist > indexRotateAmount/2)
            target += indexRotateAmount;
        setStateIndexing(target);
    }
    public void setStateOff() {
        state = State.OFF;
        motorPower = 0;
    }
    public void setStateCollecting(double power) {
        state = State.COLLECTING;
        motorPower = power;
    }
    public void setStateIndexing(int target) {
        state = State.INDEXING;
        indexerPid.setTarget(target);
    }
}

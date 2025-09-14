package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Configurable
public class IntakeIndexer {
    public static final double triggerPressThreshold = 0.1;
    public static final double collectPower = 0.99;
    public static final int indexRotateAmount = 200; // encoders needed to rotate indexer 120 degrees
    public static final int indexerAlignedThreshold = 10;
    private final Robot robot;

    // state descriptions:
    // OFF: motor not moving
    // COLLECTING: motor running to intake
    // INDEXING: motor running to align indexer for ball to be shot
    public enum State {
        OFF, COLLECTING, INDEXING
    }

    // both index types align so that a ball is ready to be shot
    // align_closest aligns the closest ball to be shot
    // align_pattern aligns the ball of the correct color to be shot
    public enum IndexType {
        ALIGN_CLOSEST, ALIGN_PATTERN
    }

    private State state;
    private final DcMotorEx motor;
    private double motorPower;
    private final PIDController indexerPid;
    private boolean indexerAligned, ballColorsUpdated;

    // these list indexes are defined in such order:
    // index 0: where ball is shot from
    // index 1: counter-clockwise of 0
    // index 2: counter-clockwise of 1
    private final ColorSensorBall[] colorSensors;
    private final BallColor[] ballColors;
    private BallColor patternColor; // color that is needed to fill the next spot in the pattern
    public IntakeIndexer(Robot robot) {
        this.robot = robot;
        state = State.OFF;
        motor = robot.hardwareMap.get(DcMotorEx.class, "intakeIndexer");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = 0;
        indexerPid = new PIDController(0.05, 0, 0);
        indexerAligned = true;
        ballColorsUpdated = false;

        colorSensors = new ColorSensorBall[] {
            new ColorSensorBall(robot, "c1"),
            new ColorSensorBall(robot, "c2"),
            new ColorSensorBall(robot, "c3")
        };
        ballColors = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};
    }

    public void update() {
        updateColorSensors();
        switch(state) {
            case OFF:
                if(listenCollectInput())
                    break;
                // checking for when driver wants to read ball colors
                else if(robot.g1.isFirstLeftBumper())
                    setStateIndexing(IndexType.ALIGN_CLOSEST);
                else if(indexerAligned && !ballColorsUpdated)
                    updateBallColors();
                break;
            case COLLECTING:
                if(listenCollectInput())
                    break;
                else
                    setStateOff();
                break;
            case INDEXING:
                if(indexerAligned()) {
                    setStateOff();
                    indexerAligned = true;
                }
                else
                    motorPower = indexerPid.update(getIndexerEncoder());
                break;
        }
        motor.setPower(motorPower);
    }

    private void setStateOff() {
        state = State.OFF;
        motorPower = 0;
    }
    private void setStateCollecting(double power) {
        state = State.COLLECTING;
        motorPower = power;
        indexerAligned = false;
        ballColorsUpdated = false;
    }
    private void setStateIndexing(IndexType indexType) {
        state = State.INDEXING;

        if(indexType == IndexType.ALIGN_CLOSEST)
            indexerPid.setTarget(getClosestAlignEncoder());
        else if(indexType == IndexType.ALIGN_PATTERN) {
            int desiredIndex = findIndex(patternColor);
            indexerPid.setTarget(getAlignEncoder(desiredIndex));
        }
    }

    private boolean listenCollectInput() {
        // checking in-taking
        if(robot.g1.gamepad.left_trigger > triggerPressThreshold) {
            setStateCollecting(collectPower);
            return true;
        }
        // checking ex-taking
        else if(robot.g1.gamepad.left_bumper) {
            setStateCollecting(-collectPower);
            return true;
        }
        return false;
    }

    // return indexer encoder so that the closest index is aligned to be shot
    private int getClosestAlignEncoder() {
        // calculating closest "third" encoder to align current position with
        int dist = getIndexerEncoder() % indexRotateAmount;
        int target = getIndexerEncoder() / indexRotateAmount * indexRotateAmount;
        if(dist > indexRotateAmount/2)
            target += indexRotateAmount;
        return target;
    }
    // return indexer encoder so that the ball at the specified index is ready to be shot
    private int getAlignEncoder(int index) {
        return 0;
    }
    private int findIndex(BallColor desiredColor) {
        for(int i = 0; i < ballColors.length; i++)
            if(ballColors[i] == desiredColor)
                return i;
        return -1;
    }

    // color sensor helper functions
    private void updateColorSensors() {
        for(ColorSensorBall colorSensor : colorSensors)
            colorSensor.update();
    }
    private void updateBallColors() {
        for(int i = 0; i < colorSensors.length; i++)
            ballColors[i] = colorSensors[i].getBallColor();
        ballColorsUpdated = true;
    }

    // public getters
    public int getIndexerEncoder() {
        return motor.getCurrentPosition();
    }
    public boolean indexerAligned() {
        return getIndexerEncoder() % indexRotateAmount < indexerAlignedThreshold;
    }

}

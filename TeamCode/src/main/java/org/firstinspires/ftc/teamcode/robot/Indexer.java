package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Indexer {
    private final int thirdRotateAmount = 100; // encoders needed to rotate 120 degrees
    private final int errorThreshold = 10;
    private final Robot robot;
    private CRServo indexer;
    private PIDController indexerPid;
    private double indexPower;

    public enum State {
        OFF, INDEXING
    }
    private State state;

    // these list indexes are defined in such order:
    // index 0: where ball is shot from
    // index 1: counter-clockwise of 0
    // index 2: counter-clockwise of 1
    private BallColor[] ballColors;
    private int numBalls; // index within ball color list in which the balls enter indexer from
    private ColorSensorBall colorSensor;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");
        indexPower = 0;

        state = State.OFF;

        ballColors = new BallColor[3];
        numBalls = 0;

        colorSensor = new ColorSensorBall(robot, "color sensor");
    }

    public void update() {
        colorSensor.update();
        switch(state) {
            case OFF:
                BallColor ballColor = colorSensor.getBallColor();
                // rotating 120 degrees to next open spot
                if(ballColor != BallColor.NONE && numBalls < 3) {
                    ballColors[numBalls] = ballColor;
                    numBalls ++;
                    if(numBalls != 3)
                        setStateIndexing(thirdRotateAmount);
                }

                break;
            case INDEXING:
                if(Math.abs(getIndexerEncoder() - indexerPid.getTarget()) < errorThreshold)
                    setStateOff();
                else
                    indexPower = indexerPid.update(getIndexerEncoder());
                break;
        }
        indexer.setPower(indexPower);
    }

    public void setStateIndexing(int offsetGoalEncoder) {
        indexerPid.setTarget(getIndexerEncoder() + offsetGoalEncoder);
        indexPower = indexerPid.update(getIndexerEncoder());
        state = State.INDEXING;
    }

    public void setStateOff() {
        indexPower = 0;
        state = State.OFF;
    }

    public int getIndexerEncoder() {
        return 0;
    }

    public State getState() {
        return state;
    }

    public int getNumBalls() {
        return numBalls;
    }

}

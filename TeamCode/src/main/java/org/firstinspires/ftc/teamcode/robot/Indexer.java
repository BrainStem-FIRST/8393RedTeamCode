package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Indexer {
    public static int thirdRotateAmount = 100; // encoders needed to rotate 120 degrees
    public static int errorThreshold = 10;
    public static double transferServoPower = 0.99;
    public static double minTransferTime = 0.5;
    private final Robot robot;
    private final CRServo indexer;
    private final PIDController indexerPid;
    private double indexPower;

    private final CRServo transfer;
    private double transferPower;
    private final ElapsedTime transferTimer; // tracks time spent transferring


    // in off collect, indexer is aligned so there is opening in intake
    // in off shoot, indexer is aligned so there is opening for a ball to be shot
    public enum IndexerState {
        OFF, INDEXING
    }
    private IndexerState indexerState;
    public enum AlignMode {
        INTAKE, SHOOT
    }

    private AlignMode alignMode;

    public enum TransferState {
        OFF, TRANSFERRING
    }
    private TransferState transferState;

    private final BallColor[] ballColors;
    private int intakeBallI; // 0-5: represents 6 possible places where ball could be in the indexer
    private int numBalls;
    private final ColorSensorBall colorSensor;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");
        indexerPid = new PIDController(0, 0, 0);
        indexPower = 0;

        transfer = robot.hardwareMap.get(CRServo.class, "transfer");
        transferPower = 0;
        transferTimer = new ElapsedTime();

        indexerState = IndexerState.OFF;
        alignMode = AlignMode.INTAKE;
        transferState = TransferState.OFF;


        ballColors = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};
        intakeBallI = 0;
        numBalls = 0;

        colorSensor = new ColorSensorBall(robot, "color sensor");
    }

    public void update() {
        updateIndexer();
        updateTransfer();
    }
    private void updateIndexer() {
        colorSensor.update();
        switch(indexerState) {
            case OFF:
                BallColor ballColor = colorSensor.getBallColor();
                // listening for indexer rotation input
                // dpad left/right is 120 degree rotations
                // dpad up/down is 60 degree rotations
                boolean autoDetectBall = numBalls < 3 && ballColor != BallColor.NONE && intakeBallI % 2 == 0;
                if(robot.g1.isFirstDpadLeft()
                || autoDetectBall) {
                    if(autoDetectBall) {
                        ballColors[intakeBallI/2] = ballColor;
                        numBalls++;
                    }
                    // don't rotate indexer if automatically detected and if indexer is filled
                    if(!autoDetectBall || numBalls < 3) {
                        setIndexerIndexing(thirdRotateAmount);
                        updateIntakeBallI(2);
                    }
                }
                else if(robot.g1.isFirstDpadRight()) {
                    setIndexerIndexing(-thirdRotateAmount);
                    updateIntakeBallI(-2);
                }
                else if(robot.g1.isFirstDpadUp() || (numBalls == 3 && alignMode == AlignMode.INTAKE)) {
                    updateIntakeBallI(1);
                    toggleAlignMode(thirdRotateAmount / 2);
                }
                else if(robot.g1.isFirstDpadDown()) {
                    updateIntakeBallI(-1);
                    toggleAlignMode(-thirdRotateAmount / 2);
                }
                break;
            case INDEXING:
                if(Math.abs(getIndexerEncoder() - indexerPid.getTarget()) < errorThreshold)
                    setIndexerOff();
                else
                    indexPower = indexerPid.update(getIndexerEncoder());
                break;
        }
        indexer.setPower(indexPower);
    }
    private void updateTransfer() {
        switch(transferState) {
            case OFF:
                // listening for input to transfer ball to shooter
                if(robot.g1.isFirstA() && alignMode == AlignMode.SHOOT)
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() >= minTransferTime) {
                    setTransferOff();
                    // automatically rotate indexer if still more balls left
                    if(numBalls > 0) {
                        setIndexerIndexing(thirdRotateAmount);
                        updateIntakeBallI(2);
                    }
                }
                break;
        }
        transfer.setPower(transferPower);
    }
    private void toggleAlignMode(int rotateAmount) {
        setIndexerIndexing(rotateAmount);
        alignMode = alignMode == AlignMode.INTAKE ? AlignMode.SHOOT : AlignMode.INTAKE;
    }
    private void setIndexerIndexing(int offsetGoalEncoder) {
        indexerPid.setTarget(getIndexerEncoder() + offsetGoalEncoder);
        indexPower = indexerPid.update(getIndexerEncoder());
        indexerState = IndexerState.INDEXING;
    }
    private void setIndexerOff() {
        indexPower = 0;
        indexerState = IndexerState.OFF;
    }
    private void setTransferTransferring() {
        transferPower = transferServoPower;
        transferState = TransferState.TRANSFERRING;
        transferTimer.reset();
        if(ballColors[getShooterBallI()] != BallColor.NONE) {
            numBalls--;
            ballColors[getShooterBallI()] = BallColor.NONE;
        }
    }
    private void setTransferOff() {
        transferPower = 0;
        transferState = TransferState.OFF;
    }
    private void updateIntakeBallI(int n) {
        intakeBallI = (intakeBallI + n) % 6;
    }
    private int getShooterBallI() {
        return (intakeBallI + 3) % 6; // returns position of ball that is currently aligned to shoot
    }
    public int getIndexerEncoder() {
        return 0;
    }
    public IndexerState getIndexerState() {
        return indexerState;
    }
    public int getNumBalls() {
        return numBalls;
    }
}

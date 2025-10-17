package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Indexer {
    public static int sixthRotateAmount = 100; // encoders needed to rotate 120 degrees
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

    private DcMotorEx indexerEncoder;

    private final BallColor[] ballColors;
    private int intakeBallI; // 0-5: represents 6 possible places where ball could be in the indexer
    private int numBalls;
    private final ColorSensorBall colorSensor1, colorSensor2;

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

        indexerEncoder = robot.hardwareMap.get(DcMotorEx.class, "FL");

        ballColors = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};
        intakeBallI = 0;
        numBalls = 0;

        colorSensor1 = new ColorSensorBall(robot, "colorSensor1");
        colorSensor2 = new ColorSensorBall(robot, "colorSensor2");
    }

    public void update() {
        updateIndexer();
        updateTransfer();
        robot.telemetry.addData("indexer encoder", getIndexerEncoder());
    }
    private void updateIndexer() {
        colorSensor1.update();
        switch(indexerState) {
            case OFF:
                break;
            case INDEXING:
                break;
        }
        indexer.setPower(indexPower);
    }
    private void updateTransfer() {
        switch(transferState) {
            case OFF:
                break;
            case TRANSFERRING:
                break;
        }
        transfer.setPower(transferPower);
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
    public int getIndexerEncoder() {
        return indexerEncoder.getCurrentPosition();
    }
    public IndexerState getIndexerState() {
        return indexerState;
    }
    public int getNumBalls() {
        return numBalls;
    }


    private int listenIndexerDpadInput() {
        if (robot.g1.isFirstDpadRight()) {
            return 1;
        } else if (robot.g1.isFirstDpadLeft()) {
            return -1;
        }
        return 0;
    }

    public void _setTransferPower(double power) {
        transfer.setPower(power);
    }
    public double getTransferPower() {
        return transferPower;
    }

}

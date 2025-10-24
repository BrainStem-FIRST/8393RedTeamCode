package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
public class IndexerNew {
    public static class Params {
        public double indexerVelocityStaticThreshold = 1, prettyMuchStatic = 200;
        public double minPowerNorm = 0.09, maxPower = 0.5;
        public double negIndexerErrorAmp = 1.5, negErrorAmpActivateRange = 600;
        public double kPNorm = 0.00015;
        public double kI = 0.000001;
        public double kD = 0;
        public double revolutionAmount = 8200, sixthRotateAmount = 1366.66667; // encoders needed to rotate 120 degrees
        public double intakeWaitingPower = 0.3;
        public int errorThreshold = 75;
        public double transferMoveTime = 0.2, transferStopperMoveTime = 0.05;
    }
    public static Params params = new Params();

    private final Robot robot;
    private final CRServo indexer;
    public enum RotateType {
        PID, CONSTANT_POWER
    }
    private RotateType rotateType;
    private int rotateDir;
    private final PIDController indexerPid;
    private double indexPower;

    private final ServoImplEx transfer;
    public static int transferInPwm = 620, transferShootPwm = 850;
    private final ServoImplEx transferStopper;
    public static int transferStopperDownPwm = 2310, transferStopperUpPwm = 2450;
    private final ElapsedTime transferTimer; // tracks time spent transferring

    public enum TransferState {
        OFF, RESETTING, TRANSFERRING
    }
    private TransferState transferState;

    private final DcMotorEx indexerEncoder;
    private double targetIndexerEncoder;
    private double intakePos;
    private final BallColor[] ballColors;
    private int numBalls;
    private final ColorSensorBall leftCS, rightCS, midCS; // color sensors
    private boolean shouldCheckLeftCS, shouldCheckRightCS, shouldCheckMidCS;
    private int curPatternI; // current pattern color selected
    private boolean shouldAutoIndex;
    private double time; // in seconds
    private boolean justTransferred;

    public IndexerNew(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        rotateType = RotateType.CONSTANT_POWER;
        indexPower = 0;
        indexerEncoder = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetIndexerEncoder = 0;
        intakePos = 0;
        indexerPid = new PIDController(params.kPNorm, params.kI, params.kD);

        transfer = robot.hardwareMap.get(ServoImplEx.class, "transfer");
        transfer.setPwmRange(new PwmControl.PwmRange(transferInPwm, transferShootPwm));
        transfer.setPosition(0);

        transferStopper = robot.hardwareMap.get(ServoImplEx.class, "transferStopper");
        transferStopper.setPwmRange(new PwmControl.PwmRange(transferStopperDownPwm, transferStopperUpPwm));
        transferStopper.setPosition(0);

        transferTimer = new ElapsedTime();
        transferState = TransferState.OFF;

        ballColors = new BallColor[] {BallColor.N, BallColor.N, BallColor.N, BallColor.N, BallColor.N, BallColor.N};
        numBalls = 0;

        leftCS = new ColorSensorBall(robot, "leftCS");
        rightCS = new ColorSensorBall(robot, "rightCS");
        midCS = new ColorSensorBall(robot, "midCS");

        shouldCheckLeftCS = true;
        shouldCheckRightCS = true;
        shouldCheckMidCS = true;

        curPatternI = 0;
        time = 0;
        shouldAutoIndex = true;
        rotateIndexerPower(1);
    }

    public void update(double totalTime) {
        updateColorSensors();
        updateIndexer();
        updateTransfer();
        time = totalTime;
    }
    private void updateColorSensors() {
        leftCS.update();
        rightCS.update();
        midCS.update();
    }
    private void updateIndexer() {
        intakePos = (getIndexerEncoder() + params.revolutionAmount) % params.sixthRotateAmount;
        intakePos /= params.sixthRotateAmount;

        // listening for gamepad input to index
        if (transferState == TransferState.OFF) {
            if (robot.g1.isFirstB())
                rotateIndexerPid(1);
            else if (robot.g1.isFirstA())
                rotateIndexerPid(-1);

            // listening for color sensor input
            if(shouldAutoIndex && numBalls != 3) {
                    int leftI = leftCS.getBallColor() != BallColor.N ? getLeftIntakeI() : -10;
                    int midI = midCS.getBallColor() != BallColor.N ? getIntakeI() : -10;
                    int rightI = rightCS.getBallColor() != BallColor.N ? getRightIntakeI() : -10;

                    boolean ballDetected = false;
                    if (leftI != -10) {
                        ballColors[getLeftIntakeI()] = leftCS.getBallColor();
                        ballDetected = true;
                    }
                    else if (midI != -10) {
                        ballColors[getIntakeI()] = midCS.getBallColor();
                        ballDetected = true;
                    }
                    else if(rightI != -10) {
                        ballColors[getRightIntakeI()] = rightCS.getBallColor();
                        ballDetected = true;
                    }

                    if (ballDetected) {
                        if(numBalls == 0)
                            rotateIndexerPower(leftI != -10 ? -3 : rightI != -10 ? 3 : 4);
                        else if(numBalls == 1)
                            rotateIndexerPid(leftI != -10 ? -1 : rightI != -10 ? 1 : !emptyAt(getOffsetI(getShooterI(), 1)) ? -2 : 2);
                        else
                            rotateIndexerPid(getAlignIndexerOffset());
                        numBalls++;
                    }
            }
        }

    /// calculating indexer power
        // precondition: rotateType should only equal constant power when collecting (aka when numBalls < 2)
        if(rotateType == RotateType.CONSTANT_POWER) {
            // oscillating indexer
            if(Math.abs(getIndexerError()) < params.errorThreshold || Math.signum(getIndexerError()) != rotateDir) {
                if (numBalls == 0)
                    rotateIndexerPower(2 * -rotateDir);
                else
                    rotateIndexerPower(-rotateDir);
                indexPower = params.intakeWaitingPower * rotateDir;
            }
        }
        else {
            if (Math.abs(getIndexerError()) < params.errorThreshold)
                indexPower = 0;
            else {
                indexPower = indexerPid.updateWithError(getIndexerError() < 0 && Math.abs(getIndexerError()) < params.negErrorAmpActivateRange ? getIndexerError() * params.negIndexerErrorAmp : getIndexerError());
                indexPower = Range.clip(Math.abs(indexPower), params.minPowerNorm, params.maxPower) * Math.signum(indexPower);
            }
        }
        indexer.setPower(indexPower);

        //misc
        if(robot.g1.isFirstStart())
            shouldAutoIndex = !shouldAutoIndex;
    }
    private void updateTransfer() {
        justTransferred = false;
        switch(transferState) {
            case OFF:
                if(robot.g1.gamepad.right_bumper)
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > params.transferStopperMoveTime) {
                    numBalls = Math.max(0, numBalls - 1);
                    ballColors[getShooterI()] = BallColor.N;
                    curPatternI = (curPatternI + 1) % 3; // assuming you score every time you shoot
                    transfer.setPosition(0.99);
                    justTransferred = true;
                }
                if(transferTimer.seconds() > params.transferStopperMoveTime + params.transferMoveTime)
                    setTransferResetting();
                break;
            case RESETTING:
                if(transferTimer.seconds() > Math.max(params.transferStopperMoveTime, params.transferMoveTime)) {
                    setTransferOff();
                    rotateIndexerPid(2);
                }
        }
    }
    private void setTransferTransferring() {
        transferState = TransferState.TRANSFERRING;
        transferTimer.reset();
        transferStopper.setPosition(0.99);
    }
    private void setTransferResetting() {
        transferState = TransferState.RESETTING;
        transferTimer.reset();
        transfer.setPosition(0);
        transferStopper.setPosition(0);
    }
    private void setTransferOff() {
        transferState = TransferState.OFF;
    }
    private void resetIndexerEncoder() {
        indexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // returns offset to rotate indexer to align it to next pattern color (pos=clockwise, neg=ccw, -10=no valid pattern color)
    public int getAlignIndexerOffset() {
        int desiredI = findBallI(BallColor.G);
        if(numBalls == 0) // means don't need to rotate
            return 0;
        if(desiredI == -1) // finding index of other color ball if there is no balls of correct color
            desiredI = findBallI(BallColor.P);
        return getShooterI() - desiredI;
    }
    private void updateIndexerTarget(int sixth) {
        if(Math.abs(sixth) >= 6)
            throw new IllegalArgumentException("cannot rotate >=360 deg in one rotation");
        targetIndexerEncoder += sixth * params.sixthRotateAmount;
    }
    private void rotateIndexerPid(int sixth) {
        rotateType = RotateType.PID;
        indexerPid.setTarget(targetIndexerEncoder);
        updateIndexerTarget(sixth);
        shouldCheckMidCS = emptyAt(getIntakeI());
        shouldCheckLeftCS = emptyAt(getLeftIntakeI());
        shouldCheckRightCS = emptyAt(getRightIntakeI());
    }
    private void rotateIndexerPower(int sixth) {
        rotateType = RotateType.CONSTANT_POWER;
        updateIndexerTarget(sixth);
        rotateDir = (int)Math.signum(sixth);
    }
    public int getIndexerEncoder() {
        return indexerEncoder.getCurrentPosition();
    }
    public double getIndexerPower() {
        return indexer.getPower();
    }
    public int getNumBalls() {
        return numBalls;
    }
    public void incNumBalls() {
        numBalls++;
    }
    public void decNumBalls() {
        numBalls--;
    }
    public double getTargetIndexerEncoder() {
        return targetIndexerEncoder;
    }
    public double getIndexerError() {
        return targetIndexerEncoder - getIndexerEncoder();
    }
    public BallColor getLeftCSColor() {
        return leftCS.getBallColor();
    }
    public BallColor getRightCSColor() {
        return rightCS.getBallColor();
    }
    public BallColor getMidCSColor() {
        return midCS.getBallColor();
    }
    public TransferState getTransferState() {
        return transferState;
    }
    public boolean prettyMuchStatic() {
        return indexerEncoder.getVelocity() < params.indexerVelocityStaticThreshold && Math.abs(getIndexerError()) < params.prettyMuchStatic;
    }
    public double getIndexerVel() {
        return indexerEncoder.getVelocity();
    }
    public boolean emptyAt(int i) {
        return ballColors[i] == BallColor.N;
    }

    // returns a continuous number to represent current position of indexer
    public int getIntakeI() {
        return (int) intakePos;
    }
    public double getIntakePos() {
        return intakePos;
    }
    public int getLeftIntakeI() {
        return getOffsetI(getIntakeI(), -1);
    }
    public int getRightIntakeI() {
        return getOffsetI(getIntakeI(), 1);
    }
    public int getOffsetI(int i, int offset) {
        return (i + offset + 6) % 6;
    }
    public int getShooterI() {
        return (getIntakeI() + 3) % 6;
    }
    public int findBallI(BallColor ballColor) {
        for(int i = 0; i < 4; i++) {
            int index = (getShooterI() + i) % 6;
            if(getBallAt(index) == ballColor)
                return index;
            index = (getShooterI() - i + 6) % 6;
            if(getBallAt(index) == ballColor)
                return index;
        }
        return -1;
    }
    public BallColor getBallAt(int i) {
        return ballColors[i];
    }
    public boolean shouldCheckLeftCS() {
        return shouldCheckLeftCS;
    }
    public boolean shouldCheckRightCS() {
        return shouldCheckRightCS;
    }
    public boolean shouldCheckMidCS() {
        return shouldCheckMidCS;
    }
    public String getLabeledBalls() {
        StringBuilder label = new StringBuilder();
        for(int i = 0; i < ballColors.length; i++) {
            if(i == getIntakeI())
                label.append("{").append(ballColors[i]).append("}, ");
            else if(i == getShooterI())
                label.append("[").append(ballColors[i]).append("], ");
            else
                label.append(ballColors[i]).append(", ");
        }
        return label.toString();
    }
    public boolean shouldAutoIndex() {
        return shouldAutoIndex;
    }
    public boolean justTransferred() {
        return justTransferred;
    }
}

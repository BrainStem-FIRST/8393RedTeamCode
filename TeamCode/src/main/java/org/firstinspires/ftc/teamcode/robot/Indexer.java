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
public class Indexer {
    public static double minPower0Balls = 0.09, minPower3Balls = 0.1;
    public static double normalMaxIndexerPower = 0.5, shootMaxIndexerPower = 0.4;
    public static double smallPIDCoefAmp = 3.1, smallPIDActivationRange = 200;
    public static double negIndexerErrorAmp = 1.4;
    public static double kP0Balls = 0.00015, kP3Balls = 0.0002;
    public static double kI0Balls = 0, kI3Balls = 0.0001;
    public static double kD0Balls = 0, kD3Balls = 0;

    public static double thirdRotateAmount = 2733.333333; // encoders needed to rotate 120 degrees
    public static int errorThreshold = 50;
    public static double transferMoveTime = 0.2, transferStopperMoveTime = 0.05;

    private final Robot robot;
    private final CRServo indexer;
    private double currentMaxIndexerPower;
    private final PIDController indexerPid;
    private double indexPower;

    private final ServoImplEx transfer;
    public static int transferInPwm = 620, transferShootPwm = 850;
    private final ServoImplEx transferStopper;
    public static int
            transferStopperDownPwm = 2300, transferStopperUpPwm = 2450;
    private final ElapsedTime transferTimer; // tracks time spent transferring

    public enum TransferState {
        OFF, RESETTING, TRANSFERRING
    }
    private TransferState transferState;

    private final DcMotorEx indexerEncoder;
    private double targetIndexerEncoder;

    private final BallColor[] ballColors;
    private int intakeI; // 0-5: represents 6 possible places where ball could be in the indexer; index 0 is index of central intake spot, indexes increase in clockwise order
    private int numBalls;
    private final ColorSensorBall leftCS, rightCS, midCS; // color sensors
    private boolean shouldCheckLeftCS, shouldCheckRightCS, shouldCheckMidCS;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        indexPower = 0;
        indexerEncoder = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetIndexerEncoder = 0;
        indexerPid = new PIDController(kP0Balls, kI0Balls, kD0Balls);
        currentMaxIndexerPower = normalMaxIndexerPower;

        transfer = robot.hardwareMap.get(ServoImplEx.class, "transfer");
        transfer.setPwmRange(new PwmControl.PwmRange(transferInPwm, transferShootPwm));

        transferStopper = robot.hardwareMap.get(ServoImplEx.class, "transferStopper");
        transferStopper.setPwmRange(new PwmControl.PwmRange(transferStopperDownPwm, transferStopperUpPwm));

        transferTimer = new ElapsedTime();
        transferState = TransferState.OFF;


        ballColors = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};
        intakeI = 0;
        numBalls = 0;

        leftCS = new ColorSensorBall(robot, "leftCS");
        rightCS = new ColorSensorBall(robot, "rightCS");
        midCS = new ColorSensorBall(robot, "midCS");

        shouldCheckLeftCS = true;
        shouldCheckRightCS = true;
        shouldCheckMidCS = true;
    }

    public void update() {
        updateColorSensors();
        updateIndexer();
        updateTransfer();
    }
    private void updateColorSensors() {
        leftCS.update();
        rightCS.update();
        midCS.update();
    }
    private void updateIndexer() {
        // listening for gamepad input to index
        if(robot.g1.isFirstB()) {
            rotateIndexerNormal(-1);
        }
        else if(robot.g1.isFirstA()) {
            rotateIndexerNormal(1);
        }
        // checking for automatic indexing if indexer is pretty much not moving
        else if(Math.abs(getIndexerError()) < errorThreshold * 2) {
            // potentially check left and right sensors if indexer is at correct offset
            if(intakeI % 2 == 1) {
                boolean ballAtLeft = shouldCheckLeftCS && emptyAt(getLeftIntakeI()) && leftCS.getBallColor() != BallColor.NONE;
                if (ballAtLeft) {
                    ballColors[getLeftIntakeI()] = leftCS.getBallColor();
                    numBalls++;
                }
                boolean ballAtRight = shouldCheckRightCS && emptyAt(getRightIntakeI()) && rightCS.getBallColor() != BallColor.NONE;
                if (ballAtRight) {
                    ballColors[getRightIntakeI()] = rightCS.getBallColor();
                    numBalls++;
                }

                if(ballAtLeft && ballAtRight)
                    //rotating 180 degrees clockwise
                    rotateIndexerNormal(3);
                else if(ballAtLeft)
                    // rotating 120 degrees clockwise
                    rotateIndexerNormal(2);
                else if(ballAtRight)
                    // rotating 120 degrees counter clockwise
                    rotateIndexerNormal(-2);
            }
            // potentially check middle sensor if indexer at correct offset
            else {
                if(shouldCheckMidCS && emptyAt(intakeI) && midCS.getBallColor() != BallColor.NONE) {
                    ballColors[intakeI] = midCS.getBallColor();
                    numBalls++;
                    // rotating 180 degrees clockwise
                    rotateIndexerNormal(3);
                }
            }
        }

        // calculating indexer power
        if(Math.abs(getIndexerEncoder() - targetIndexerEncoder) < errorThreshold)
            indexPower = 0;
        else {
            double t = numBalls/3.;
            indexerPid.setKP(lerp(kP0Balls, kP3Balls, t));
            indexerPid.setKI(lerp(kI0Balls, kI3Balls, t));
            indexerPid.setKD(lerp(kD0Balls, kD3Balls, t));
            if(Math.abs(getIndexerEncoder() - targetIndexerEncoder) < smallPIDActivationRange)
                indexerPid.setKP(indexerPid.getKP() * smallPIDCoefAmp);
            // TODO: maybe also scale KI

            double error = getIndexerError();
            indexPower = Range.clip(indexerPid.updateWithError(error < 0 ? error * negIndexerErrorAmp : error), -currentMaxIndexerPower, currentMaxIndexerPower);

            double minPower = lerp(minPower0Balls, minPower3Balls, t);
            indexPower = Math.signum(indexPower) * Math.max(minPower, Math.abs(indexPower));
        }
        indexer.setPower(indexPower);

        //misc
        if(robot.g1.isFirstX())
            resetIndexerEncoder();

        if(robot.g1.isFirstStart())
            robot.indexer.incNumBalls();
        else if(robot.g1.isFirstBack())
            robot.indexer.decNumBalls();
    }
    private void updateTransfer() {
        switch(transferState) {
            case OFF:
                if(robot.g1.gamepad.right_bumper)
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > transferStopperMoveTime)
                    transfer.setPosition(0.99);
                if(transferTimer.seconds() > transferStopperMoveTime + transferMoveTime)
                    setTransferResetting();
                break;
            case RESETTING:
                if(transferTimer.seconds() > Math.max(transferStopperMoveTime, transferMoveTime)) {
                    setTransferOff();
                    rotateIndexerShoot();
                }
        }
    }
    private void setTransferTransferring() {
        transferState = TransferState.TRANSFERRING;
        transferTimer.reset();
        transferStopper.setPosition(0.99);
        numBalls = Math.max(numBalls-1, 0);
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
    // positive value = clockwise rotation, vice versa
    private void rotateIndexerNormal(int sixth) {
        intakeI = (intakeI - sixth) % 6;

        shouldCheckMidCS = emptyAt(intakeI);
        shouldCheckLeftCS = emptyAt(getLeftIntakeI());
        shouldCheckRightCS = emptyAt(getRightIntakeI());

        targetIndexerEncoder -= sixth * thirdRotateAmount/2;
        currentMaxIndexerPower = normalMaxIndexerPower;
    }
    private void rotateIndexerShoot() {
        intakeI = (intakeI - 2) % 6;
        targetIndexerEncoder -= 2 * thirdRotateAmount/2;
        currentMaxIndexerPower = shootMaxIndexerPower;
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
    public double getCurrentIndexerMaxPower() {
        return currentMaxIndexerPower;
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

    public TransferState getTransferState() {
        return transferState;
    }
    public boolean emptyAt(int i) {
        return ballColors[i] == BallColor.NONE;
    }
    public int getLeftIntakeI() {
        return (intakeI + 1) % 6;
    }
    public int getRightIntakeI() {
        return (intakeI - 1) % 6;
    }
    private double lerp(double a, double b, double t) {
        return a + (b-a) * t;
    }
}

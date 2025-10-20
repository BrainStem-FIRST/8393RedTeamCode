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
    public static double normalMaxIndexerPower = 0.5, shootIndexerMaxPower = 0.4;
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

    public enum AlignMode {
        INTAKE, SHOOT
    }

    private AlignMode alignMode;

    public enum TransferState {
        OFF, RESETTING, TRANSFERRING
    }
    private TransferState transferState;

    private final DcMotorEx indexerEncoder;
    private double targetIndexerEncoder;

    private final BallColor[] ballColors;
    private int intakeBallI; // 0-5: represents 6 possible places where ball could be in the indexer
    private int numBalls;
    private final ColorSensorBall leftCS, rightCS, midCS;

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

        alignMode = AlignMode.INTAKE;
        transferState = TransferState.OFF;


        ballColors = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};
        intakeBallI = 0;
        numBalls = 0;

        leftCS = new ColorSensorBall(robot, "leftCS");
        rightCS = new ColorSensorBall(robot, "rightCS");
        midCS = new ColorSensorBall(robot, "midCS");
    }

    public void update() {
        leftCS.update();
        rightCS.update();
        if(numBalls < 3) {
            leftCS.getBallColor();
            rightCS.getBallColor();
        }
        updateIndexer();
        updateTransfer();
    }
    public BallColor getLeftCSColor() {
        return leftCS.getBallColor();
    }
    public BallColor getRightCSColor() {
        return rightCS.getBallColor();
    }
    private double lerp(double a, double b, double t) {
        return a + (b-a) * t;
    }
    private void updateIndexer() {
        // listening for gamepad input
        if(robot.g1.isFirstDpadLeft()) {
            targetIndexerEncoder += thirdRotateAmount/2;
            currentMaxIndexerPower = normalMaxIndexerPower;
        }
        else if(robot.g1.isFirstDpadRight()) {
            targetIndexerEncoder -= thirdRotateAmount/2;
            currentMaxIndexerPower = normalMaxIndexerPower;
        }
        if(robot.g1.isFirstX())
            resetIndexerEncoder();

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

            double error = targetIndexerEncoder - getIndexerEncoder();
            indexPower = Range.clip(indexerPid.updateWithError(error < 0 ? error * negIndexerErrorAmp : error), -currentMaxIndexerPower, currentMaxIndexerPower);

            double minPower = lerp(minPower0Balls, minPower3Balls, t);
            indexPower = Math.signum(indexPower) * Math.max(minPower, Math.abs(indexPower));
        }
        indexer.setPower(indexPower);

        //misc
        if(robot.g1.isFirstStart())
            robot.indexer.incNumBalls();
        else if(robot.g1.isFirstBack())
            robot.indexer.decNumBalls();
    }
    private void updateTransfer() {
        switch(transferState) {
            case OFF:
                if(robot.g1.isFirstB())
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
                    targetIndexerEncoder -= thirdRotateAmount;
                    currentMaxIndexerPower = shootIndexerMaxPower;
                }
        }
    }
    private void resetIndexerEncoder() {
        indexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public TransferState getTransferState() {
        return transferState;
    }
    public double getCurrentIndexerMaxPower() {
        return currentMaxIndexerPower;
    }

}

package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
public class Indexer {
    public static double minPower0Balls = 0.08, minPower3Balls = 0.1;
    public static double kP0Balls = 0.0002, kP3Balls = 0.00036;
    public static double kI0Balls = 0, kI3Balls = 0;
    public static double kD0Balls = 0, kD3Balls = 0;

    public static double thirdRotateAmount = 2733.333333; // encoders needed to rotate 120 degrees
    public static int errorThreshold = 50;
    public static double transferServoPower = 0.99;
    public static double minTransferTime = 0.5;
    private final Robot robot;
    private final CRServo indexer;
    private final PIDController indexerPid;
    private double indexPower;

    private final CRServo transfer;
    private double transferPower;
    private final ElapsedTime transferTimer; // tracks time spent transferring

    public enum AlignMode {
        INTAKE, SHOOT
    }

    private AlignMode alignMode;

    public enum TransferState {
        OFF, TRANSFERRING
    }
    private TransferState transferState;

    private final DcMotorEx indexerEncoder;
    private double targetIndexerEncoder;

    private final BallColor[] ballColors;
    private int intakeBallI; // 0-5: represents 6 possible places where ball could be in the indexer
    private int numBalls;
    private final ColorSensorBall colorSensor1, colorSensor2;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        indexPower = 0;
        indexerEncoder = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetIndexerEncoder = 0;
        indexerPid = new PIDController(kP0Balls, kI0Balls, kD0Balls);

        transfer = robot.hardwareMap.get(CRServo.class, "transfer");
        transferPower = 0;
        transferTimer = new ElapsedTime();

        alignMode = AlignMode.INTAKE;
        transferState = TransferState.OFF;


        ballColors = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};
        intakeBallI = 0;
        numBalls = 0;

        colorSensor1 = new ColorSensorBall(robot, "colorSensor1");
        colorSensor2 = new ColorSensorBall(robot, "colorSensor2");
    }

    public void update() {
        updateIndexer();
        updateTransfer();
    }
    private double lerp(double a, double b, double t) {
        return a + (b-a) * t;
    }
    private void updateIndexer() {
        colorSensor1.update();
        colorSensor2.update();

        // listening for gamepad input
        if(robot.g1.isFirstDpadLeft()) {
            targetIndexerEncoder += thirdRotateAmount;
        }
        else if(robot.g1.isFirstDpadRight()) {
            targetIndexerEncoder -= thirdRotateAmount;
        }
        if(robot.g1.isFirstX())
            resetIndexerEncoder();

        // updating pid's
        indexerPid.setTarget(targetIndexerEncoder);

        // calculating indexer power
        if(Math.abs(getIndexerEncoder() - targetIndexerEncoder) < errorThreshold)
            indexPower = 0;
        else {
            double t = numBalls/3.;
            indexerPid.setKP(lerp(kP0Balls, kP3Balls, t));
            indexerPid.setKI(lerp(kI0Balls, kI3Balls, t));
            indexerPid.setKD(lerp(kD0Balls, kD3Balls, t));
            indexPower = indexerPid.update(getIndexerEncoder());
            double minPower = lerp(minPower0Balls, minPower3Balls, t);
            indexPower = Math.signum(indexPower) * Math.max(minPower, Math.abs(indexPower));
            robot.telemetry.addData("min power", minPower);
        }
        indexer.setPower(indexPower);

        //misc
        if(robot.g1.isFirstStart())
            robot.indexer.incNumBalls();
        else if(robot.g1.isFirstBack())
            robot.indexer.decNumBalls();

        robot.telemetry.addData("target indexer encoder", targetIndexerEncoder);
        robot.telemetry.addData("indexer power", indexer.getPower());
        robot.telemetry.addData("kP", indexerPid.getKP());
    }
    private void updateTransfer() {
        switch(transferState) {
            case OFF:
                if(robot.g1.isFirstY())
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > minTransferTime)
                    setTransferOff();
                break;
        }
        transfer.setPower(transferPower);
    }
    private void resetIndexerEncoder() {
        indexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void setTransferTransferring() {
        transferTimer.reset();
        transferState = TransferState.TRANSFERRING;
        transferPower = transferServoPower;
    }
    private void setTransferOff() {
        transferState = TransferState.OFF;
        transferPower = 0;
    }
    public int getIndexerEncoder() {
        return indexerEncoder.getCurrentPosition();
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

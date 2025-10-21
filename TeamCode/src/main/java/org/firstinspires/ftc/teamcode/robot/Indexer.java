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
    public static class Params {
        public int greenPos = 0; // ranges 0-2 (0=green should be shot first, 2=green should be shot last)

        public double minPower0Balls = 0.1, minPower3Balls = 0.11;
        public double normalMaxIndexerPower0 = 0.5, shootMaxIndexerPower0 = 0.4;
        public double normalMaxIndexerPower3 = 0.6, shootMaxIndexerPower3 = 0.5;
        public double smallPIDCoefAmp = 1, smallPIDActivationRange = 300;
        public double negIndexerErrorAmp = 1.1;
        public double kP0Balls = 0.00015, kP3Balls = 0.00015;
        public double kI0Balls = 0, kI3Balls = 0;
        public double kD0Balls = 0, kD3Balls = 0;
        public double thirdRotateAmount = 2733.333333; // encoders needed to rotate 120 degrees
        public int errorThreshold = 50;
        public double transferMoveTime = 0.2, transferStopperMoveTime = 0.05;
    }
    public static Params params = new Params();

    private final Robot robot;
    private final CRServo indexer;
    private boolean useNormalMaxPower;
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

    private final BallColor[] ballColors;
    private int intakeI; // 0-5: represents 6 possible places where ball could be in the indexer; index 0 is index of central intake spot, indexes increase in clockwise order
    private int numBalls;
    private final ColorSensorBall leftCS, rightCS, midCS; // color sensors
    private boolean shouldCheckLeftCS, shouldCheckRightCS, shouldCheckMidCS;
    private int curPatternI; // current pattern color selected
    private boolean autoIndexToPattern;
    private double time; // in seconds

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        indexPower = 0;
        indexerEncoder = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetIndexerEncoder = 0;
        indexerPid = new PIDController(params.kP0Balls, params.kI0Balls, params.kD0Balls);
        useNormalMaxPower = true;

        transfer = robot.hardwareMap.get(ServoImplEx.class, "transfer");
        transfer.setPwmRange(new PwmControl.PwmRange(transferInPwm, transferShootPwm));

        transferStopper = robot.hardwareMap.get(ServoImplEx.class, "transferStopper");
        transferStopper.setPwmRange(new PwmControl.PwmRange(transferStopperDownPwm, transferStopperUpPwm));

        transferTimer = new ElapsedTime();
        transferState = TransferState.OFF;

        ballColors = new BallColor[] {BallColor.N, BallColor.N, BallColor.N, BallColor.N, BallColor.N, BallColor.N};
        intakeI = 0;
        numBalls = 0;

        leftCS = new ColorSensorBall(robot, "leftCS");
        rightCS = new ColorSensorBall(robot, "rightCS");
        midCS = new ColorSensorBall(robot, "midCS");

        shouldCheckLeftCS = true;
        shouldCheckRightCS = true;
        shouldCheckMidCS = true;

        curPatternI = 0;
        time = 0;
        autoIndexToPattern = true;
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
        // listening for gamepad input to index
        if(transferState == TransferState.OFF) {
            if (robot.g1.isFirstB()) {
                rotateIndexerNormal(-1);
            } else if (robot.g1.isFirstA()) {
                rotateIndexerNormal(1);
            }
            // checking for automatic color sensor indexing if indexer is pretty much not moving and not shooting
            else if (prettyMuchStatic()) {
                // potentially check left and right sensors if indexer is at correct offset
                if (intakeI % 2 == 1) {
                    boolean ballAtLeft = shouldCheckLeftCS && emptyAt(getLeftIntakeI()) && leftCS.getBallColor() != BallColor.N;
                    if (ballAtLeft) {
                        ballColors[getLeftIntakeI()] = leftCS.getBallColor();
                        numBalls++;
                    }
                    boolean ballAtRight = shouldCheckRightCS && emptyAt(getRightIntakeI()) && rightCS.getBallColor() != BallColor.N;
                    if (ballAtRight) {
                        ballColors[getRightIntakeI()] = rightCS.getBallColor();
                        numBalls++;
                    }

                    if (numBalls == 3)
                        rotateIndexerNormal(getAlignIndexerOffset());
                    else if (ballAtLeft && ballAtRight)
                        //rotating 180 degrees clockwise
                        rotateIndexerNormal(3);
                    else if (ballAtLeft)
                        if (numBalls == 1)
                            // rotating 120 degrees clockwise
                            rotateIndexerNormal(2);
                        else
                            rotateIndexerNormal(1);
                    else if (ballAtRight)
                        if (numBalls == 1)
                            // rotating 120 degrees counter clockwise
                            rotateIndexerNormal(-2);
                        else
                            rotateIndexerNormal(-1);
                }
                // potentially check middle sensor if indexer at correct offset
                else if (shouldCheckMidCS && emptyAt(intakeI) && midCS.getBallColor() != BallColor.N) {
                    ballColors[intakeI] = midCS.getBallColor();
                    numBalls++;
                    if (numBalls == 3)
                        rotateIndexerNormal(getAlignIndexerOffset());
                    else
                        // rotating 180 degrees clockwise
                        rotateIndexerNormal(3);
                }
            }
        }

        // calculating indexer power
        if(Math.abs(getIndexerEncoder() - targetIndexerEncoder) < params.errorThreshold)
            indexPower = 0;
        else {
            double t = numBalls/3.;
            indexerPid.setKP(lerp(params.kP0Balls, params.kP3Balls, t));
            indexerPid.setKI(lerp(params.kI0Balls, params.kI3Balls, t));
            indexerPid.setKD(lerp(params.kD0Balls, params.kD3Balls, t));

            double error = getIndexerError();
            if(Math.abs(error) < params.smallPIDActivationRange)
                indexerPid.setKP(indexerPid.getKP() * params.smallPIDCoefAmp);
            double max = useNormalMaxPower ? lerp(params.normalMaxIndexerPower0, params.normalMaxIndexerPower3, t) : lerp(params.shootMaxIndexerPower0, params.shootMaxIndexerPower3, t);
            indexPower = Range.clip(indexerPid.updateWithError(error < 0 ? error * params.negIndexerErrorAmp : error), -max, max);

            double minPower = lerp(params.minPower0Balls, params.minPower3Balls, t);
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
                if(robot.g1.gamepad.right_bumper)
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > params.transferStopperMoveTime)
                    transfer.setPosition(0.99);
                if(transferTimer.seconds() > params.transferStopperMoveTime + params.transferMoveTime)
                    setTransferResetting();
                break;
            case RESETTING:
                if(transferTimer.seconds() > Math.max(params.transferStopperMoveTime, params.transferMoveTime)) {
                    setTransferOff();
                    rotateIndexerShoot();
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
        int desiredI = findBallI(curPatternI == params.greenPos ? BallColor.G : BallColor.P);
        if(numBalls == 0) // means don't need to rotate
            return 0;
        if(desiredI == -1) // finding index of other color ball if there is no balls of correct color
            desiredI = findBallI(curPatternI == params.greenPos ? BallColor.P : BallColor.G);
        return getShooterI() - desiredI;
    }
    // positive value = clockwise rotation, vice versa
    private void rotateIndexer(int sixth) {
        intakeI = (intakeI - sixth + 6) % 6;
        targetIndexerEncoder -= sixth * params.thirdRotateAmount/2;

        shouldCheckMidCS = emptyAt(intakeI);
        shouldCheckLeftCS = emptyAt(getLeftIntakeI());
        shouldCheckRightCS = emptyAt(getRightIntakeI());
    }
    private void rotateIndexerNormal(int sixth) {
        rotateIndexer(sixth);
        useNormalMaxPower = true;
    }

    private void rotateIndexerShoot() {
        curPatternI = (curPatternI + 1) % 3; // assuming you score every time you shoot
        if(!emptyAt(getShooterI())) {
            numBalls = Math.max(numBalls-1, 0);
            ballColors[getShooterI()] = BallColor.N;
            rotateIndexer(getAlignIndexerOffset());
            useNormalMaxPower = false;
        }
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
    public boolean useNormalMaxPower() {
        return useNormalMaxPower;
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
        return Math.abs(getIndexerError()) < params.errorThreshold * 2;
    }
    public boolean emptyAt(int i) {
        return ballColors[i] == BallColor.N;
    }
    public int getIntakeI() {
        return intakeI;
    }
    public int getLeftIntakeI() {
        return (intakeI + 1) % 6;
    }
    public int getRightIntakeI() {
        return (intakeI + 5) % 6;
    }
    public int getShooterI() {
        return (intakeI + 3) % 6;
    }
    public int findBallI(BallColor ballColor) {
        for(int i = 0; i < 4; i++) {
            int index = (getShooterI() - i + 6) % 6;
            if(getBallAt(index) == ballColor)
                return index;
            index = (getShooterI() + i) % 6;
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
    public int getCurPatternI() {
        return curPatternI;
    }
    private double lerp(double a, double b, double t) {
        return a + (b-a) * t;
    }
}

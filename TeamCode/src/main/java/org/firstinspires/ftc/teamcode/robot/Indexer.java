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
        public double indexerVelocityStaticThreshold = 1, prettyMuchStatic = 400;
        public double manualIndexPowerAmp = 0.2;
        public int greenPos = 0; // ranges 0-2 (0=green should be shot first, 2=green should be shot last)
        public double csResponseDelay = 0;
        public double minPowerWeak = 0.08, minPowerNorm = 0.09, minPowerStrong = 0.11, offsetMaxPower = 0.4, maxPower = 0.5, strongMaxPower = 0.6;
        public double negIndexerErrorAmp = 1.5, negErrorAmpActivateRange = 600;
        public double pidPowerOffset = 0.05;
        public double kPNorm = 0.00015;
        public double kI = 0.000001;
        public double kD = 0;
        public double kPWeak = 0.0001;
        public double kPWeakNorm = 0.00013;
        public double kPStrong = 0.0002;
        public double thirdRotateAmount = 2733.333333; // encoders needed to rotate 120 degrees
        public int errorThreshold = 75;
        public double transferMoveTime = 0.2, transferStopperMoveTime = 0.05;
    }
    public static Params params = new Params();

    private final Robot robot;
    private final CRServo indexer;
    private final PIDController indexerPid;
    private final PIDController pidWeakPosOffset, pidWeakNegOffset, pidNormalPosOffset, pidNormalNegOffset, pidStrongPosOffset, pidStrongNegOffset, pidNormalWeakPosOffset;
    private final PIDController pidWeak, pidWeakNorm, pidNormal, pidStrong;
    private double indexPower;
    private final ElapsedTime indexerAutoRotateTimer;
    private boolean autoRotateCued;

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
    private boolean ballAtLeft, ballAtRight, ballAtMid;
    private int curPatternI; // current pattern color selected
    private String pidSelected;
    private boolean shouldAutoRotate;
    private double time; // in seconds
    private boolean justTransferred;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        indexPower = 0;
        indexerEncoder = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetIndexerEncoder = 0;
        indexerPid = new PIDController(params.kPNorm, params.kI, params.kD);

        // pos or neg offset means direction (& sign of power) offset
        pidWeakPosOffset = new PIDController(params.kPWeak, params.kI, params.kD);
        pidWeakPosOffset.setPowerOffset(params.pidPowerOffset);
        pidWeakNegOffset = new PIDController(params.kPWeak, params.kI, params.kD);
        pidWeakNegOffset.setPowerOffset(-params.pidPowerOffset);
        pidNormalPosOffset = new PIDController(params.kPNorm, params.kI, params.kD);
        pidNormalPosOffset.setPowerOffset(params.pidPowerOffset);
        pidNormalNegOffset = new PIDController(params.kPNorm, params.kI, params.kD);
        pidNormalNegOffset.setPowerOffset(-params.pidPowerOffset);
        pidStrongPosOffset = new PIDController(params.kPStrong, params.kI, params.kD);
        pidStrongPosOffset.setPowerOffset(params.pidPowerOffset);
        pidStrongNegOffset = new PIDController(params.kPStrong, params.kI, params.kD);
        pidStrongNegOffset.setPowerOffset(-params.pidPowerOffset);
        pidNormalWeakPosOffset = new PIDController(params.kPWeakNorm, params.kI, params.kD);
        pidNormalWeakPosOffset.setPowerOffset(params.pidPowerOffset);

        pidWeak = new PIDController(params.kPWeak, params.kI, params.kD);
        pidWeakNorm = new PIDController(params.kPWeakNorm, params.kI, params.kD);
        pidNormal = new PIDController(params.kPNorm, params.kI, params.kD);
        pidStrong = new PIDController(params.kPStrong, params.kI, params.kD);

        autoRotateCued = false;
        indexerAutoRotateTimer = new ElapsedTime();

        transfer = robot.hardwareMap.get(ServoImplEx.class, "transfer");
        transfer.setPwmRange(new PwmControl.PwmRange(transferInPwm, transferShootPwm));
        transfer.setPosition(0);

        transferStopper = robot.hardwareMap.get(ServoImplEx.class, "transferStopper");
        transferStopper.setPwmRange(new PwmControl.PwmRange(transferStopperDownPwm, transferStopperUpPwm));
        transferStopper.setPosition(0);

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
        shouldAutoRotate = false;
        pidSelected = "normal";
    }

    public void update(double totalTime) {
        updateColorSensors();
        updateIndexer();
        updateIndexerAutoRotate();
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
        if (transferState == TransferState.OFF) {
            if (robot.g1.isFirstB())
                rotateIndexer(1);
            else if (robot.g1.isFirstA())
                rotateIndexer(-1);

            // checking for automatic color sensor indexing if indexer is pretty much not moving and not shooting
            else if (prettyMuchStatic()) {
                // potentially check left and right sensors if indexer is at correct offset
                if (intakeI % 2 == 1) {
                    boolean ballAtLeft = shouldCheckLeftCS && emptyAt(getLeftIntakeI()) && leftCS.getBallColor() != BallColor.N;
                    if (ballAtLeft) {
                        ballColors[getLeftIntakeI()] = leftCS.getBallColor();
                        numBalls++;
                        this.ballAtLeft = true;
                    }
                    boolean ballAtRight = shouldCheckRightCS && emptyAt(getRightIntakeI()) && rightCS.getBallColor() != BallColor.N;
                    if (ballAtRight) {
                        ballColors[getRightIntakeI()] = rightCS.getBallColor();
                        numBalls++;
                        this.ballAtRight = true;
                    }
                    if (ballAtLeft || ballAtRight) {
                        indexerAutoRotateTimer.reset();
                        autoRotateCued = true;
                    }
                    ballAtMid = false;
                }
                // potentially check middle sensor if indexer at correct offset
                else if (shouldCheckMidCS && emptyAt(intakeI) && midCS.getBallColor() != BallColor.N) {
                    ballAtMid = true;
                    ballAtLeft = false;
                    ballAtRight = false;
                    ballColors[intakeI] = midCS.getBallColor();
                    numBalls++;
                    indexerAutoRotateTimer.reset();
                    autoRotateCued = true;
                }
            }
        }

        // calculating indexer power
        if(Math.abs(getIndexerError()) < params.errorThreshold)
            indexPower = 0;
        else {
            if(robot.g2.gamepad.right_trigger > 0.05)
                indexPower = Math.signum(getIndexerError()) * robot.g2.gamepad.right_trigger * params.manualIndexPowerAmp;
            else {
                double error = targetIndexerEncoder - getIndexerEncoder();
                indexPower = indexerPid.updateWithError(error);
                indexPower = Range.clip(Math.abs(indexPower), pidSelected.contains("weak") ? params.minPowerWeak : pidSelected.contains("strong") ? params.minPowerStrong :params.minPowerNorm, pidSelected.contains("strong") ? params.strongMaxPower : pidSelected.contains("offset") ? params.offsetMaxPower : params.maxPower) * Math.signum(indexPower);
            }
        }
        indexer.setPower(indexPower);

        //misc
        if(robot.g1.isFirstStart())
            shouldAutoRotate = !shouldAutoRotate;
    }
    private void updateIndexerAutoRotate() {
        if(!shouldAutoRotate || !autoRotateCued || indexerAutoRotateTimer.seconds() < params.csResponseDelay)
            return;
        if (numBalls == 3)
            rotateIndexer(-1);
        else if (ballAtLeft && ballAtRight)
            //rotating 180 degrees clockwise
            rotateIndexer(3);
        else if (ballAtLeft)
            if (numBalls == 1)
                // rotating 120 degrees clockwise
                rotateIndexer(-2);
            else if(!emptyAt(getShooterI()))
                // rotating 60 degrees clockwise
                rotateIndexer(-1);
            else
                // rotating 180 degrees clockwise
                rotateIndexer(3);
        else if(ballAtRight) {
            if (numBalls == 1)
                // rotating 120 degrees counter clockwise
                rotateIndexer(2);
            else if(!emptyAt(getShooterI()))
                // rotating 60 degrees counter clockwise
                rotateIndexer(1);
            else
                // rotating 180 degrees clockwise
                rotateIndexer(3);
        }

        else if(ballAtMid) {
            if(!emptyAt(getOffsetI(getShooterI(), 1)))
                // rotating 120 degrees clockwise
                rotateIndexer(2);
            else
                rotateIndexer(-2);
        }
        autoRotateCued = false;
        ballAtLeft = false;
        ballAtMid = false;
        ballAtRight = false;
    }
    private void updateTransfer() {
        justTransferred = false;
        switch(transferState) {
            case OFF:
                if(robot.g1.gamepad.right_bumper)
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > params.transferStopperMoveTime && transfer.getPosition() < 0.5) {
                    transfer.setPosition(0.99);
                    numBalls = Math.max(numBalls-1, 0);
                    ballColors[getShooterI()] = BallColor.N;
                    justTransferred = true;
                }
                if(transferTimer.seconds() > params.transferStopperMoveTime + params.transferMoveTime)
                    setTransferResetting();
                break;
            case RESETTING:
                if(transferTimer.seconds() > Math.max(params.transferStopperMoveTime, params.transferMoveTime)) {
                    setTransferOff();
                    rotateIndexer(-2);
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
        if(desiredI == -1) // finding index of other color ball if there is no balls of correct color
            desiredI = findBallI(BallColor.P);
        return getShooterI() - desiredI;
    }
    private PIDController calcRotationPid(int sixth) {
        if(!shouldAutoRotate) {
            pidSelected = "strong";
            return pidStrong;
        }
        switch(numBalls) {
            case 1:
                if(!emptyAt(getShooterI())) {
                    if(sixth == 3) {
                        pidSelected = "weak";
                        return pidWeak;
                    }
                    pidSelected = "normWeak";
                    return pidWeakNorm;
                }
                if(!emptyAt(intakeI)) {
                    pidSelected = "normWeak";
                    return pidWeakNorm;
                }

                // if ball is on left side of indexer
                if(!emptyAt(getOffsetI(intakeI, 1)) || !emptyAt(getOffsetI(intakeI, 2)))
                    if(sixth > 0) {
                        pidSelected = "normal";
                        return pidNormal;
                    }
                    else {
                        pidSelected = "weak neg offset";
                        return pidWeakNegOffset;
                    }

                // if ball is on right side of indexer
                if(sixth > 0) {
                    pidSelected = "weakNorm pos offset";
                    return pidNormalWeakPosOffset;
                }
                pidSelected = "normal";
                return pidNormal;

            case 2:
                if(!emptyAt(getOffsetI(getShooterI(), 1)) && !emptyAt(getOffsetI(getShooterI(), -1))) {
                    pidSelected = "normal";
                    return pidNormal;
                }
                if(!emptyAt(getLeftIntakeI()) && !emptyAt(getRightIntakeI())) {
                    pidSelected = "normWeak";
                    return pidWeakNorm;
                }
                // if one ball is on left side (the other must be on top or bottom)
                if(!emptyAt(getOffsetI(intakeI, 1)) || !emptyAt(getOffsetI(intakeI, 2))) {
                    if(sixth > 0) {
                        pidSelected = "Strong";
                        return pidStrong;
                    }
                    pidSelected = "Strong negOffset";
                    return pidStrongNegOffset;
                }
                // if one ball is on right side (the other must be on top or bottom)
                if(sixth > 0) {
                    pidSelected = "Strong";
                    return pidStrong;
                }
                pidSelected = "normal";
                return pidNormal;
            case 3:
                pidSelected = "strong";
                return pidStrong;
        }
        pidSelected = "normal";
        return pidNormal; // return if have 0 or 3 balls
    }
    // positive value = counter clockwise rotation
    private void rotateIndexer(int sixth) {
        intakeI = (intakeI + sixth + 6) % 6;
        targetIndexerEncoder += sixth * params.thirdRotateAmount/2;
        indexerPid.set(calcRotationPid(sixth));
        indexerPid.setTarget(targetIndexerEncoder);

        shouldCheckMidCS = emptyAt(intakeI);
        shouldCheckLeftCS = emptyAt(getLeftIntakeI());
        shouldCheckRightCS = emptyAt(getRightIntakeI());
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
    public int getIntakeI() {
        return intakeI;
    }
    public int getLeftIntakeI() {
        return getOffsetI(intakeI, -1);
    }
    public int getRightIntakeI() {
        return getOffsetI(intakeI, 1);
    }
    public int getOffsetI(int i, int offset) {
        return (i + offset + 6) % 6;
    }
    public int getShooterI() {
        return (intakeI + 3) % 6;
    }
    public int findBallI(BallColor ballColor) {
        for(int i = 0; i < 4; i++) {
            int index = (getShooterI() + i) % 6;
            if(getBallAt(index) == ballColor)
                return index;
            index = (getShooterI() - i+6) % 6;
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
    public String getLabeledBalls() {
        StringBuilder label = new StringBuilder();
        for(int i = 0; i < ballColors.length; i++) {
            if(i == intakeI)
                label.append("{").append(ballColors[i]).append("}, ");
            else if(i == getShooterI())
                label.append("[").append(ballColors[i]).append("], ");
            else
                label.append(ballColors[i]).append(", ");
        }
        return label.toString();
    }
    public String getPidSelected() {
        return pidSelected;
    }
    public boolean isBallAtLeft() {
        return ballAtLeft;
    }
    public boolean isBallAtMid() {
        return ballAtMid;
    }
    public boolean isBallAtRight() {
        return ballAtRight;
    }
    public boolean isAutoRotateCued() {
        return autoRotateCued;
    }
    public double autoIndexCueTime() {
        return indexerAutoRotateTimer.seconds();
    }
    public boolean shouldAutoIndex() {
        return shouldAutoRotate;
    }
    public boolean justTransferred() {
        return justTransferred;
    }
}

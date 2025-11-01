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
    public static class Params {
        public double indexerVelocityStaticThreshold = 1, prettyMuchStatic = 400;
        public double manualIndexPowerAmp = 0.4;
        public int greenPos = 0; // ranges 0-2 (0=green should be shot first, 2=green should be shot last)
        public double csResponseDelay = 0;
        public double kPClockwise60 = 0.000242, kPCounter60 = 0.00032, kPClockwise120 = 0.000185, kPCounter120 = 0.000241;
        public double kI = 0.000001;
        public double kD = 0;
        public double thirdRotateAmount = 2733.333333; // encoders needed to rotate 120 degrees
        public int errorThreshold = 75;
    }
    public static Params params = new Params();

    private final Robot robot;
    // indexer stuff
    private final CRServo indexer;
    private final DcMotorEx indexerEncoderTracker;
    private double targetIndexerEncoder;
    private final PIDController cwPid60, ccwPid60, cwPid120, ccwPid120;
    private final PIDController indexerPid;
    private double indexPower, prevIndexPower;


    // logic stuff
    private final BallColor[] ballColors;
    private int intakeI; // 0-5: represents 6 possible places where ball could be in the indexer; index 0 is index of central intake spot, indexes increase in clockwise order
    private int numBalls;
    private final ColorSensorBall leftCS, rightCS, midCS; // color sensors
    private boolean shouldCheckLeftCS, shouldCheckRightCS, shouldCheckMidCS;
    private boolean ballAtLeft, ballAtRight, ballAtMid;
    private int curPatternI; // current pattern color selected
    private String pidSelected;
    private boolean shouldAutoRotate;
    private boolean autoRotateCued;
    private final ElapsedTime indexerAutoRotateTimer;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        indexPower = 0;
        indexerEncoderTracker = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetIndexerEncoder = 0;
        cwPid60 = new PIDController(params.kPClockwise60, params.kI, params.kD);
        ccwPid60 = new PIDController(params.kPCounter60, params.kI, params.kD);
        cwPid120 = new PIDController(params.kPClockwise120, params.kI, params.kD);
        ccwPid120 = new PIDController(params.kPCounter120, params.kI, params.kD);
        indexerPid = new PIDController(params.kPClockwise60, params.kI, params.kD);

        autoRotateCued = false;
        indexerAutoRotateTimer = new ElapsedTime();

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
        shouldAutoRotate = false;
        pidSelected = "normal";
    }

    public void update() {
        updateColorSensors();
        updateIndexer();
        updateIndexerAutoRotate();
    }
    private void updateColorSensors() {
        leftCS.update();
        rightCS.update();
        midCS.update();
    }
    private void updateIndexer() {
        // listening for gamepad input to index
        if (robot.transfer.getTransferState() == Transfer.TransferState.OFF) {
            if (robot.g1.isFirstB())
                rotate(1);
            else if (robot.g1.isFirstA())
                rotate(-1);

            // updating color sensor values
            else if (shouldAutoRotate && prettyMuchStatic() && robot.intake.getIntakeState() != Intake.IntakeState.OFF) {
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
            if(robot.g2.rightTrigger() > 0.05)
                indexPower = Math.signum(getIndexerError()) * robot.g2.rightTrigger() * params.manualIndexPowerAmp;
            else {
                double error = targetIndexerEncoder - getIndexerEncoderTracker();
                indexPower = indexerPid.updateWithError(error);
                indexPower *= -1;
            }
        }
        if(indexPower != prevIndexPower)
            indexer.setPower(indexPower);
        prevIndexPower = indexPower;

        //misc
        if(robot.g1.isFirstStart())
            shouldAutoRotate = !shouldAutoRotate;
    }
    private void updateIndexerAutoRotate() {
        if(!shouldAutoRotate || !autoRotateCued || indexerAutoRotateTimer.seconds() < params.csResponseDelay)
            return;
        if (numBalls == 3)
            rotate(getAlignIndexerOffset());
        else if (ballAtLeft && ballAtRight)
            //rotating 180 degrees clockwise
            rotate(3);
        else if (ballAtLeft)
            if (numBalls == 1)
                // rotating 120 degrees clockwise
                rotate(-2);
            else if(!emptyAt(getShooterI()))
                // rotating 60 degrees clockwise
                rotate(-1);
            else
                // rotating 180 degrees clockwise
                rotate(3);
        else if(ballAtRight) {
            if (numBalls == 1)
                // rotating 120 degrees counter clockwise
                rotate(2);
            else if(!emptyAt(getShooterI()))
                // rotating 60 degrees counter clockwise
                rotate(1);
            else
                // rotating 180 degrees clockwise
                rotate(3);
        }

        else if(ballAtMid) {
            if(!emptyAt(getOffsetI(getShooterI(), 1)))
                // rotating 120 degrees clockwise
                rotate(-2);
            else
                rotate(2);
        }
        autoRotateCued = false;
        ballAtLeft = false;
        ballAtMid = false;
        ballAtRight = false;
    }
    private void resetIndexerEncoder() {
        indexerEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerEncoderTracker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // returns offset to rotate indexer to align it to next pattern color (pos=clockwise, neg=ccw, -10=no valid pattern color)
    public int getAlignIndexerOffset() {
        BallColor desiredColor = curPatternI == Robot.params.greenPos ? BallColor.G : BallColor.P;
        int desiredI = findBallI(desiredColor);
        if(desiredI == -1) // finding index of other color ball if there is no balls of correct color
            desiredI = findBallI(desiredColor == BallColor.G ? BallColor.P : BallColor.G);
        return getShooterI() - desiredI;
    }
    private PIDController calcRotationPid(int sixth) {
        if(sixth == 1) {
            pidSelected = "counter clockwise 60";
            return ccwPid60;
        }
        else if(sixth == -1) {
            pidSelected = "clockwise 60";
            return cwPid60;
        }
        else if(sixth == 2) {
            pidSelected = "counter clockwise 120";
            return ccwPid120;
        }
        pidSelected = "clockwise 120";
        return cwPid120;
//        if(!shouldAutoRotate) {
//            pidSelected = "strong";
//            return pidStrong;
//        }
//        switch(numBalls) {
//            case 1:
//                if(!emptyAt(getShooterI())) {
//                    if(sixth == 3) {
//                        pidSelected = "weak";
//                        return pidWeak;
//                    }
//                    pidSelected = "normWeak";
//                    return pidWeakNorm;
//                }
//                if(!emptyAt(intakeI)) {
//                    pidSelected = "normWeak";
//                    return pidWeakNorm;
//                }
//
//                // if ball is on left side of indexer
//                if(!emptyAt(getOffsetI(intakeI, 1)) || !emptyAt(getOffsetI(intakeI, 2)))
//                    if(sixth > 0) {
//                        pidSelected = "normal";
//                        return pidNormal;
//                    }
//                    else {
//                        pidSelected = "weak neg offset";
//                        return pidWeakNegOffset;
//                    }
//
//                // if ball is on right side of indexer
//                if(sixth > 0) {
//                    pidSelected = "weakNorm pos offset";
//                    return pidNormalWeakPosOffset;
//                }
//                pidSelected = "normal";
//                return pidNormal;
//
//            case 2:
//                if(!emptyAt(getOffsetI(getShooterI(), 1)) && !emptyAt(getOffsetI(getShooterI(), -1))) {
//                    pidSelected = "normal";
//                    return pidNormal;
//                }
//                if(!emptyAt(getLeftIntakeI()) && !emptyAt(getRightIntakeI())) {
//                    pidSelected = "normWeak";
//                    return pidWeakNorm;
//                }
//                // if one ball is on left side (the other must be on top or bottom)
//                if(!emptyAt(getOffsetI(intakeI, 1)) || !emptyAt(getOffsetI(intakeI, 2))) {
//                    if(sixth > 0) {
//                        pidSelected = "Strong";
//                        return pidStrong;
//                    }
//                    pidSelected = "Strong negOffset";
//                    return pidStrongNegOffset;
//                }
//                // if one ball is on right side (the other must be on top or bottom)
//                if(sixth > 0) {
//                    pidSelected = "Strong";
//                    return pidStrong;
//                }
//                pidSelected = "normal";
//                return pidNormal;
//            case 3:
//                pidSelected = "strong";
//                return pidStrong;
//        }
//        pidSelected = "normal";
//        return pidNormal; // return if have 0 or 3 balls
    }
    // positive value = counter clockwise rotation
    public void rotate(int sixth) {
        intakeI = (intakeI - sixth + 6) % 6;
        targetIndexerEncoder += sixth * params.thirdRotateAmount/2;
        indexerPid.set(calcRotationPid(sixth));
        indexerPid.setTarget(targetIndexerEncoder);

        shouldCheckMidCS = emptyAt(intakeI);
        shouldCheckLeftCS = emptyAt(getLeftIntakeI());
        shouldCheckRightCS = emptyAt(getRightIntakeI());
    }
    public void simulateShot() {
        numBalls = Math.max(0, numBalls-1);
        ballColors[getShooterI()] = ColorSensorBall.BallColor.N;
        curPatternI = (curPatternI + 1) % 3;
    }
    public int getIndexerEncoderTracker() {
        return indexerEncoderTracker.getCurrentPosition();
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
        return targetIndexerEncoder - getIndexerEncoderTracker();
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
    public boolean prettyMuchStatic() {
        return indexerEncoderTracker.getVelocity() < params.indexerVelocityStaticThreshold && Math.abs(getIndexerError()) < params.prettyMuchStatic;
    }
    public double getIndexerVel() {
        return indexerEncoderTracker.getVelocity();
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
            int index = getOffsetI(getShooterI(), i);
            if(getBallAt(index) == ballColor)
                return index;
            index = getOffsetI(getShooterI(), -i);
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
    public String getLabeledBalls() {
        StringBuilder label = new StringBuilder();
        for(int i = 0; i < ballColors.length; i++) {
            if(i == intakeI)
                label.append("i{").append(ballColors[i]).append("}, ");
            else if(i == getShooterI())
                label.append("s[").append(ballColors[i]).append("], ");
            else
                label.append(" ").append(ballColors[i]).append(", ");
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
    public boolean shouldAutoIndex() {
        return shouldAutoRotate;
    }
    public void setAutoRotate(boolean shouldAutoRotate) {
        this.shouldAutoRotate = shouldAutoRotate;
    }
}

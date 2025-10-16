package org.firstinspires.ftc.teamcode.utils;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;
public class PIDController {
    protected double target;
    protected double kP, kI, kD;
    protected double proportional, integral, derivative;
    protected boolean shouldReset;

    protected double previousTime, previousError;

    protected double lowerInputBound = Double.NEGATIVE_INFINITY, higherInputBound = Double.POSITIVE_INFINITY;
    protected double lowerOutputBound = -1, higherOutputBound = 1;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        shouldReset = true;
    }

    public void setInputBounds(double lowerInputBound, double higherInputBound) {
        this.lowerInputBound = lowerInputBound;
        this.higherInputBound = higherInputBound;
    }

    public void setOutputBounds(double lowerOutputBound, double higherOutputBound) {
        this.lowerOutputBound = lowerOutputBound;
        this.higherOutputBound = higherOutputBound;
    }


    // returns motor power given current position of motor
    public double update(double value) {
        value = Range.clip(value, lowerInputBound, higherInputBound);

        // BEFORE: double error = value - target;
        double error = target - value;

        return Range.clip(updateWithError(error), lowerOutputBound, higherOutputBound);
    }

    private double updateWithError(double error) {
        if (Double.isNaN(error) || Double.isInfinite(error))
            return 0;

        proportional = kP * error;

        double currentTime = System.currentTimeMillis() / 1000.0;

        if (shouldReset) {
            shouldReset = false;
            integral = 0;
            derivative = 0;
        }
        else {
            double dT = currentTime - previousTime;

            integral += kI * error * dT;

            derivative = kD * (error - previousError) / dT;
        }

        previousTime = currentTime;
        previousError = error;

        double correction = proportional + integral + derivative;

        return Math.signum(correction) * Range.clip(Math.abs(correction),
                lowerOutputBound, higherOutputBound);
    }


    // getters/setters
    public double getKP() {
        return kP;
    }
    public double getKI() {
        return kI;
    }
    public double getKD() {
        return kD;
    }
    // the target position of motor
    public double getTarget() {
        return target;
    }
    public void setTarget(double target) {
        shouldReset = true;
        this.target = target;
    }
    @NonNull
    public String toString() {
        return "kP:" + kP + " | kI: " + kI + " | kD: " + kD;
    }
}
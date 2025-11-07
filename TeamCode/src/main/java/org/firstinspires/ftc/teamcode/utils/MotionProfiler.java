package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.math.Func;

public class MotionProfiler {
    public Func f; // outputs velocity as function of time
    public final ElapsedTime timer;
    public final PIDFController pid;
    public double curVel;
    private final double predictDt;
    private double prevTime, t;
    public MotionProfiler(PIDFController pid, double predictDt) {
        timer = new ElapsedTime();
        this.pid = pid;
        this.predictDt = predictDt;
    }
    public void start(Func f) {
        this.f = f;
        timer.reset();
        prevTime = 0;
        curVel = 0;
    }
    public double update(double dy) {
        t = timer.seconds();
        curVel = dy / (t - prevTime);
        pid.updateError(f.eval(t+predictDt) - curVel);
//        pid.updateFeedForwardInput(1);
        prevTime = timer.seconds();
        return pid.run();
    }
}

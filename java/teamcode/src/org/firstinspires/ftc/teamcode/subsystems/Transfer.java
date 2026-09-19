package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Feeds one ball at a time and owns the cycle time. This class, not the world, is the
 * authority on how fast the robot may shoot -- so the number the team tunes here is the
 * number that ships.
 */
public class Transfer {
    private DcMotorEx motor;
    private Servo gate;
    private RobotConfig cfg;

    private final ElapsedTime sinceFeed = new ElapsedTime();
    private final ElapsedTime pulse = new ElapsedTime();
    private boolean pulsing = false;
    private boolean gateOpen = false;
    private boolean beltOn = false;

    public void init(HardwareMap hw, RobotConfig config) {
        cfg = config;
        motor = hw.get(DcMotorEx.class, config.TRANSFER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gate = config.gateEnabled ? hw.tryGet(Servo.class, config.GATE) : null;
        sinceFeed.reset();
    }

    public boolean canFeed() { return sinceFeed.seconds() >= cfg.cycleTimeS && !pulsing; }
    public double secondsSinceFeed() { return sinceFeed.seconds(); }

    /** Ask for one ball. Returns true if the request was actually taken. */
    public boolean feedOne() {
        if (!canFeed()) return false;
        pulsing = true;
        pulse.reset();
        sinceFeed.reset();
        return true;
    }

    public void hold() { pulsing = false; }

    /**
     * THE GATE STAYS OPEN ONLY WHILE THE SHOT IS STILL GOOD.
     *
     * feedOne() commits about four tenths of a second before the ball actually leaves -- the
     * feed pulse plus the climb up the tube -- and that used to be final. Every axis keeps
     * tracking in the meantime so the AIM at release is current; what went stale was the
     * PERMISSION, and it showed up as a small tail of badly wrong shots rather than as a loss
     * of precision. Call this every loop with whether the shot would still be allowed now.
     */
    public void stillGood(boolean ok) { if (pulsing && !ok) pulsing = false; }

    public void setGate(boolean open) { gateOpen = open; }

    /**
     * THE BELT RUNS WHENEVER THE SHOOTER IS ARMED, and the gate meters. This ran the belt only
     * for the 0.25 s feed pulse, so a ball had a quarter of a second to climb from the bin to
     * the wheel -- about 0.7 s of travel -- and the tube emptied back into the bin between
     * shots. That is the README's "the Java decides to fire and the ball never leaves": the
     * simulator's mirror (BuiltinTeleOp) has run the belt continuously while armed since the
     * gate became a real plate, and the deliverable had not caught up. Robot.update() sets
     * this from the flywheel's target.
     */
    public void setBeltOn(boolean on) { beltOn = on; }

    public void update() {
        if (pulsing && pulse.seconds() >= cfg.feedPulseS) pulsing = false;
        motor.setPower(beltOn || pulsing ? 1.0 : 0.0);
        if (gate != null) gate.setPosition(gateOpen || pulsing ? cfg.gateOpen : cfg.gateClosed);
    }

    public void telemetry(Telemetry t) {
        t.addData("since feed", "%.2f / %.2f s", sinceFeed.seconds(), cfg.cycleTimeS);
        t.addData("feeding", pulsing);
    }
}

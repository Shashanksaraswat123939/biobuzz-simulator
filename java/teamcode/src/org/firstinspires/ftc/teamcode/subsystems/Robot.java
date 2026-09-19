package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.control.Localizer;
import org.firstinspires.ftc.teamcode.control.ShotTable;
import org.firstinspires.ftc.teamcode.control.FusedLocalizer;
import org.firstinspires.ftc.teamcode.control.TagCamera;
import org.firstinspires.ftc.teamcode.control.TagTargetProvider;
import org.firstinspires.ftc.teamcode.control.TargetProvider;

import java.util.List;

/**
 * Everything the robot is, assembled once. OpModes compose this and call update().
 * No threads, no static mutable state: the SDK does not forgive either.
 */
public class Robot {
    public final RobotConfig cfg = new RobotConfig();
    public final Drivetrain drive = new Drivetrain();
    public final IntakeLine intake = new IntakeLine();
    public final Transfer transfer = new Transfer();
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Flywheel flywheel = new Flywheel();
    public final Hopper hopper = new Hopper();
    public final ShotTable shots = cfg.shotTable();

    private TargetProvider target;
    /** Non-null when the target is coming off a camera, which is the only real case. */
    private TagTargetProvider tagTarget;
    /** Odometry with tag corrections folded in. Null with no camera or no raw localizer. */
    private FusedLocalizer fused;
    private final ElapsedTime loop = new ElapsedTime();
    private double dt = 0.02;

    public void init(HardwareMap hw) {
        // Bulk caching AUTO: one I/O round trip per loop instead of one per read. On the
        // hub this is the difference between a 10 ms loop and a 40 ms one.
        List<LynxModule> hubs = hw.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hw, cfg);
        intake.init(hw, cfg);
        transfer.init(hw, cfg);
        turret.init(hw, cfg);
        hood.init(hw, cfg);
        flywheel.init(hw, cfg);
        hopper.init(hw, cfg);

        // WHERE THE GOAL IS, from a camera. On the hub `tagcam` is an AprilTagProcessor
        // wrapper; here it is the world's pipeline, with its lens, its frame rate and its
        // latency. The provider is TeamCode's own, so the fusion that turns late detections
        // into an aim ships to the robot rather than living in the simulator.
        //
        // There is NO FALLBACK any more, and that is deliberate. The oracle that used to sit
        // behind `target` -- SimTargetProvider, reading the world's exact bearing and range --
        // is deleted, not disabled, so it cannot quietly come back and flatter a measurement.
        // No camera means no target, and AimController holds fire, which is what a robot with
        // no vision actually does.
        TagCamera cam = hw.tryGet(TagCamera.class, cfg.TAGCAM);
        // TAG FIXES CORRECT THE POSE. Odometry drifts -- heading error rotates every inch
        // driven after it, so position error grows with DISTANCE and never comes back. The
        // fuser wraps the raw localizer and IS one, so everything downstream gets the
        // corrected pose without knowing it exists.
        if (cam != null && drive.getLocalizer() != null) {
            fused = new FusedLocalizer(drive.getLocalizer(), cam,
                    RobotConstants.FUSE_GAIN, RobotConstants.FUSE_HEADING_GAIN,
                    RobotConstants.FUSE_REJECT_OVER_IN);
        }
        if (cam != null) {
            tagTarget = new TagTargetProvider(cam, localizer(), cfg.tagHoldS, cfg.tagMaxFireAgeS,
                    cfg.tagScanRateDps, cfg.turretMinDeg, cfg.turretMaxDeg,
                    cfg.tagMouthDxA, cfg.tagMouthDxB, cfg.mouthFacingXA, cfg.mouthFacingXB,
                    cfg.anchorPriorX, cfg.anchorPriorY,
                    cfg.mouthFromAnchorA, cfg.mouthFromAnchorB, cfg.anchorAlpha,
                    cfg.fireOnOdometry, cfg.startCellId);
            target = tagTarget;
        } else {
            target = hw.tryGet(TargetProvider.class, "target");
        }
        loop.reset();
    }

    public TargetProvider target() { return target; }
    /** The camera-backed provider, or null on a world with no `tag` block. Telemetry only. */
    public TagTargetProvider tagTarget() { return tagTarget; }
    /** The best pose available: corrected by tag fixes when there is a camera to do it. */
    public Localizer localizer() { return fused != null ? fused : drive.getLocalizer(); }

    /** The fuser itself, for telemetry. Null when there is nothing to fuse. */
    public FusedLocalizer fused() { return fused; }
    public double dt() { return dt; }

    public void update() {
        dt = Math.max(1e-3, Math.min(0.2, loop.seconds()));
        loop.reset();
        drive.update();
        // THE POSE, for OpModes that never call AimController -- the tuning ones. When the aim
        // did run this loop it stepped the same estimator already, and update() is idempotent
        // on an unchanged raw reading, so calling it twice is free rather than wrong.
        if (fused != null) fused.update();
        else if (drive.getLocalizer() != null) drive.getLocalizer().update();
        intake.update();
        // The magazine stays loaded against the gate while the wheel is spinning, so the
        // next shot is a gate pulse and not a climb from the bin.
        transfer.setBeltOn(flywheel.gate().getTargetRpm() > 0);
        transfer.update();
        turret.update();
        hood.update();
        flywheel.update();
    }

    public void telemetry(Telemetry t) {
        drive.telemetry(t);
        intake.telemetry(t);
        transfer.telemetry(t);
        turret.telemetry(t);
        hood.telemetry(t);
        flywheel.telemetry(t);
        hopper.telemetry(t);
    }

    public void stop() {
        drive.stop();
        intake.stop();
        transfer.hold();
        flywheel.stop();
        update();
    }
}

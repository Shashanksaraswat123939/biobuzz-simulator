package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * The first thing that must work: drive off the wall (LEAVE, 3 pts) and come back to the
 * LOADING ZONE (PARK, 5 pts). Together with an alliance partner that is the 16 points the
 * SWARM ranking point needs.
 */
@Autonomous(name = "Auto Leave + Park", group = "match", preselectTeleOp = "TeleOp Main")
public class AutoLeavePark extends LinearOpMode {

    private static final double LEAVE_IN = 30.0;
    private static final double DRIVE_POWER = 0.45;
    private static final double HEADING_KP = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);

        telemetry.addLine("AutoLeavePark ready");
        telemetry.update();
        waitForStart();
        robot.intake.collect();   // holds the preloads in under braking; see AutoOneTip
        if (!opModeIsActive()) return;

        ElapsedTime timer = new ElapsedTime();
        driveForward(robot, LEAVE_IN, DRIVE_POWER, 6.0, "leaving");
        sleep(250);
        driveForward(robot, -LEAVE_IN, DRIVE_POWER, 6.0, "parking");

        robot.stop();
        telemetry.addData("auto done", "%.1f s", timer.seconds());
        telemetry.update();
    }

    /**
     * Straight-line move on the drive encoders, holding heading on the IMU, with a timeout
     * so it can never hang. The heading term is not optional: open loop, a couple of degrees
     * of yaw early turns into a foot of lateral error over a 30 inch leg.
     */
    private void driveForward(Robot robot, double inches, double power, double timeoutS, String label)
            throws InterruptedException {
        resetAndSettle(robot);
        double holdHeading = robot.drive.getHeadingDeg();
        ElapsedTime t = new ElapsedTime();
        double sign = inches >= 0 ? 1 : -1;

        while (opModeIsActive() && t.seconds() < timeoutS) {
            double done = robot.drive.getForwardIn();
            double remaining = Math.abs(inches) - Math.abs(done);
            if (remaining <= 1.0) break;
            double p = power * Math.min(1.0, Math.max(0.25, remaining / 12.0));
            double err = Units.wrapDeg(holdHeading - robot.drive.getHeadingDeg());
            double turn = Units.clamp(HEADING_KP * err, -0.35, 0.35);
            robot.drive.driveRobotCentric(sign * p, 0, turn);
            robot.update();
            telemetry.addData(label, "%.1f / %.1f in", Math.abs(done), Math.abs(inches));
            telemetry.addData("heading err", "%.1f deg", err);
            telemetry.update();
            idle();
        }
        robot.drive.stop();
        robot.update();
    }

    /**
     * Zero the encoders and wait until they actually read zero.
     *
     * STOP_AND_RESET_ENCODER does not take effect until the next loop -- the hub has to make
     * the round trip, and so does the bridge. Reading the count in the same pass returns the
     * value from before the reset, which on the second leg of a there-and-back looks like
     * "already arrived", and the move is skipped entirely.
     */
    private void resetAndSettle(Robot robot) throws InterruptedException {
        robot.drive.resetEncoders();
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < 0.5 && Math.abs(robot.drive.getForwardIn()) > 1.0) {
            robot.drive.stop();
            robot.update();
            idle();
        }
    }
}

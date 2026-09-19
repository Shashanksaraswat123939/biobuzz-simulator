package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.control.AimController;
import org.firstinspires.ftc.teamcode.control.DriveToRange;
import org.firstinspires.ftc.teamcode.control.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * Leave, get to a range the shot table can actually solve, empty the preload into the up
 * CELL, park. One tip is 20 points, worth more than everything else autonomous can reach.
 *
 * The "get to a range" step is not decoration. Leaving the wall drives the robot TOWARDS the
 * HIVE, and 26 inches of LEAVE puts it about 22 inches from the CELL -- inside the table's
 * 30 inch floor, where no hood angle and no RPM will drop a ball in. The robot has to back
 * off before it can shoot.
 */
@Autonomous(name = "Auto One Tip", group = "match", preselectTeleOp = "TeleOp Main")
public class AutoOneTip extends LinearOpMode {

    private static final double HEADING_KP = 0.03;
    private static final double LEAVE_IN = 26.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);
        AimController aim = new AimController(robot);
        DriveToRange toRange = new DriveToRange(robot.shots, 6.0, 0.02);
        robot.hopper.setCount(robot.cfg.hopperCapacity);

        telemetry.addLine("AutoOneTip ready");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        ElapsedTime match = new ElapsedTime();
        // THE INTAKE RUNS THE WHOLE TIME, as the simulator's own brain has it ("always
        // running, because a real one is"). The roller pressing on the preloads is what holds
        // them in the bin under braking: with it off, the stop at the end of LEAVE threw all
        // four out through the mouth and the dead-reckoned hopper count went on saying 4.
        // That is the README's "counts its hopper down from 6 to 2 and the world records
        // shots 0" -- measured with tools/headless.ts --dumpact.
        robot.intake.collect();
        resetAndSettle(robot);
        double hold = robot.drive.getHeadingDeg();

        // 1. leave the wall, so the alliance banks LEAVE whatever else happens
        while (opModeIsActive() && robot.drive.getForwardIn() < LEAVE_IN && match.seconds() < 5) {
            robot.drive.driveRobotCentric(0.45, 0, headingHold(robot, hold));
            robot.update();
            telemetry.addData("leaving", "%.1f / %.1f in", robot.drive.getForwardIn(), LEAVE_IN);
            telemetry.update();
            idle();
        }
        robot.drive.stop();

        // 2. back off until the shot table has an answer for this range
        while (opModeIsActive() && match.seconds() < 12) {
            aim.update(false);
            double range = aim.rangeIn();
            if (robot.shots.usable(range) && robot.shots.marginFor(range) >= 0.03) break;
            double forward = toRange.update(range, robot.target() == null ? 0 : robot.target().getAzimuthDeg());
            robot.drive.driveRobotCentric(forward, 0, headingHold(robot, hold));
            robot.update();
            telemetry.addData("ranging", "%.1f in, margin %.1f%%", range, robot.shots.marginFor(range) * 100);
            Localizer lz = robot.localizer();
            if (lz != null) telemetry.addData("pose", "%.1f, %.1f @ %.0f", lz.getX(), lz.getY(), lz.getHeadingDeg());
            telemetry.update();
            idle();
        }
        robot.drive.stop();

        // 3. spin up and empty the hopper into the CELL
        while (opModeIsActive() && match.seconds() < 25 && !robot.hopper.isEmpty()) {
            if (aim.update(true)) aim.fireIfReady();
            robot.drive.driveRobotCentric(0, 0, headingHold(robot, hold));
            robot.update();
            telemetry.addData("status", aim.status());
            telemetry.addData("range", "%.1f in", aim.rangeIn());
            telemetry.addData("left", robot.hopper.getCount());
            robot.telemetry(telemetry);
            telemetry.update();
            idle();
        }

        // 4. back to the LOADING ZONE for PARK
        robot.flywheel.stop();
        resetAndSettle(robot);
        while (opModeIsActive() && robot.drive.getForwardIn() > -40 && match.seconds() < 29.5) {
            robot.drive.driveRobotCentric(-0.6, 0, headingHold(robot, hold));
            robot.update();
            telemetry.addData("parking", "%.1f in", robot.drive.getForwardIn());
            telemetry.update();
            idle();
        }
        robot.stop();
    }

    private static double headingHold(Robot robot, double target) {
        return Units.clamp(HEADING_KP * Units.wrapDeg(target - robot.drive.getHeadingDeg()), -0.35, 0.35);
    }

    /** See AutoLeavePark: STOP_AND_RESET_ENCODER only lands on the next loop. */
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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.AimController;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.control.DriveToRange;
import org.firstinspires.ftc.teamcode.control.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/** Driver practice: robot-centric drive, intake always on (left trigger reverses), auto-aim on X. */
@TeleOp(name = "TeleOp Main", group = "match")
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);
        AimController aim = new AimController(robot);
        DriveToRange toRange = new DriveToRange(robot.shots, 6.0, 0.02);

        boolean autoAim = true;
        boolean lastX = false;
        boolean spinUp = false;
        boolean lastA = false;

        telemetry.addLine("TeleOpMain ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && !lastX) autoAim = !autoAim;
            lastX = gamepad1.x;
            if (gamepad1.a && !lastA) spinUp = !spinUp;
            lastA = gamepad1.a;

            double slow = gamepad1.left_bumper ? 0.35 : 1.0;
            double forward = -gamepad1.left_stick_y * slow;
            double left = -gamepad1.left_stick_x * slow;
            double turn = -gamepad1.right_stick_x * slow;

            // THE SPEED CAP, m/s. Mirrors BuiltinTeleOp: a feed-forward fraction so the robot
            // never gets up to an illegal speed, plus a feedback trim because power is not
            // speed and the open-loop fraction is only close. Translation only -- rotation is
            // not what outruns the ball. 0 means no cap, which is the shipping default.
            if (RobotConstants.MAX_SPEED_MPS > 0) {
                double cap = RobotConstants.MAX_SPEED_MPS;
                double k = RobotConstants.FREE_SPEED_MPS > 0
                        ? Math.min(1.0, cap / RobotConstants.FREE_SPEED_MPS) : 1.0;
                Localizer loc = robot.localizer();
                double vx = loc == null ? 0 : loc.getVx() * 0.0254;
                double vy = loc == null ? 0 : loc.getVy() * 0.0254;
                double now = Math.hypot(vx, vy);
                if (now > cap) k *= cap / now;
                forward *= k;
                left *= k;
            }

            boolean canShoot = aim.update(spinUp);

            if (autoAim && spinUp && Math.abs(forward) < 0.15 && Math.abs(left) < 0.15) {
                // Hands off the sticks: close the range to the table's best band.
                forward = toRange.update(aim.rangeIn(), robot.target() == null ? 0 : robot.target().getAzimuthDeg());
            }
            // ROBOT-CENTRIC: forward is whichever way the INTAKE points, always.
            //
            // This was field-centric, rotating the stick into the field frame so up meant
            // "away from the driver station" whichever way the robot faced. The argument was
            // that a turret lets the chassis point anywhere so the driver should not have to
            // track its nose -- which is right about the SHOT and wrong about the driver. You
            // do not drive a robot at the field, you drive it at a BALL, and the intake is the
            // only end that can pick one up. Hold Y for the old behaviour.
            if (gamepad1.y) {
                robot.drive.driveFieldCentric(forward, left, turn);
            } else {
                robot.drive.driveRobotCentric(forward, left, turn);
            }

            // ALWAYS RUNNING, reversed on the left trigger -- the same as BuiltinTeleOp. A
            // roller that only turns while a trigger is held lets the bin empty itself out of
            // the mouth on every hard stop.
            if (gamepad1.left_trigger > 0.1) {
                robot.intake.eject();
            } else {
                robot.intake.collect();
                robot.hopper.setCount(robot.hopper.getCount()); // real counter goes here
            }

            if ((gamepad1.right_bumper || gamepad1.b) && canShoot) aim.fireIfReady();
            robot.transfer.setGate(gamepad1.y);

            robot.update();

            telemetry.addData("status", aim.status());
            telemetry.addData("auto-aim", autoAim);
            telemetry.addData("range", "%.1f in", aim.rangeIn());
            robot.telemetry(telemetry);
            telemetry.update();
        }
        robot.stop();
    }
}

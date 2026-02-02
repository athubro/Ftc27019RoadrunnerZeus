package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ZeusTeleOPBlue", group = "TeleOp")
public class ZeusTeleOPBlue extends LinearOpMode {

    private Turret turret;
    private Intake intake;
    private Pose2d initialPose = new Pose2d(0, 0, 0);

    private double speedRatio = 0.75;

    private boolean usingOdomTracking = false;

    @Override
    public void runOpMode() {

        turret = new Turret(hardwareMap, telemetry, initialPose);
        intake = new Intake(hardwareMap, telemetry);

        turret.PARAMS.TARGET_TAG_ID = 20;
        turret.setAutoAngleEnabled(false);
        turret.setAutoRPMEnabled(false);
        turret.setTrackingMode(false);
        turret.setContinuousTracking(true);
        turret.setUseOdometryTracking(false);

        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("Turret + Drive + Intake Initialized");
        telemetry.addLine("Target Tag ID: 20");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1: Drive + Intake");
        telemetry.addLine("Gamepad 2: Turret Controls");
        telemetry.addLine();
        telemetry.addLine("GP2 B: Toggle FULL AUTO (Vision/RPM/Angle)");
        telemetry.addLine("GP2 A: Toggle Vision vs Odom Tracking");
        telemetry.addLine("GP2 Right Stick X: MANUAL TURRET YAW (when tracking OFF)");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // GAMEPAD 1: DRIVE CONTROLS
            // =========================

            double forward = -speedRatio * gamepad1.left_stick_y;
            double strafe = -speedRatio * gamepad1.left_stick_x;
            double rotation = -speedRatio * gamepad1.right_stick_x;

            if (gamepad1.right_bumper)      speedRatio = 1.0;
            else if (gamepad1.left_bumper)  speedRatio = 0.3;
            else                            speedRatio = 0.75;

            // =========================
            // GAMEPAD 2: TURRET CONTROLS
            // =========================

            // Toggle FULL AUTO MODE (Vision + RPM + Angle) - using B button
            if (gamepad2.b) {
                boolean newState = !turret.trackingMode;
                turret.setTrackingMode(newState);
                turret.setAutoAngleEnabled(newState);
                turret.setAutoRPMEnabled(newState);
                sleep(200); // simple debounce
            }

            // Toggle Vision vs Odom tracking - using A button
            if (gamepad2.a) {
                usingOdomTracking = !usingOdomTracking;
                turret.setUseOdometryTracking(usingOdomTracking);
                sleep(200); // debounce
            }

            // Shooting
            turret.setShootingEnabled(gamepad2.right_trigger > 0.2);

            // Manual RPM (D-pad up/down)
            if (gamepad2.dpad_up) {
                turret.setTargetRPM(turret.getTargetRPM() + 50.0);
            }
            if (gamepad2.dpad_down) {
                turret.setTargetRPM(Math.max(0, turret.getTargetRPM() - 50.0));
            }
            if (gamepad1.dpadUpWasPressed()) {
                turret.PARAMS.turretKF += 1;
            }
            if (gamepad1.dpadDownWasPressed()) {
                turret.PARAMS.turretKF -= 1;
            }
            if (gamepad1.dpadRightWasPressed()) {
                turret.PARAMS.turretKP += 1;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                turret.PARAMS.turretKP -= 1;
            }

            // Manual turret angle (up/down) - D-pad left/right
            if (gamepad2.dpad_right) {
                turret.setTurretAngleCommand(1);
            } else if (gamepad2.dpad_left) {
                turret.setTurretAngleCommand(-1);
            } else {
                turret.setTurretAngleCommand(0);
            }

            // =========================
            // GAMEPAD 2: INTAKE CONTROLS
            // =========================

            if (gamepad2.x) {
                intake.openGate();
                intake.setIntakePower(1.0);
            } else if (gamepad2.y) {
                intake.closeGate();
                intake.setIntakePower(1.0);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setIntakePower(gamepad2.left_trigger);
            } else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                intake.setIntakePower(gamepad2.left_stick_y);
            } else {
                intake.setIntakePower(0);
            }

            if (gamepad2.right_bumper) intake.openGate();
            if (gamepad2.left_bumper)  intake.closeGate();
            if (gamepad2.b)            intake.toggleGate();  // now active

            // =========================
            // UPDATE ALL SYSTEMS
            // =========================

            turret.update(forward, strafe, rotation);

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Speed Mode", speedRatio == 1.0 ? "FAST" : (speedRatio == 0.3 ? "SLOW" : "NORMAL"));
            telemetry.addData("Position", "X: %.1f  Y: %.1f  H: %.1f°",
                    turret.getPose().position.x,
                    turret.getPose().position.y,
                    Math.toDegrees(turret.getPose().heading.toDouble()));

            telemetry.addLine();
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.trackingMode ? "AUTO" : "MANUAL");
            telemetry.addData("Tracking Source", usingOdomTracking ? "ODOMETRY (-60, -60)" : "LIMELIGHT");
            telemetry.addData("Aligned", turret.isAligned() ? "YES" : "NO");
            telemetry.addData("Auto RPM", turret.autoRPMEnabled ? "ON" : "OFF");
            telemetry.addData("Auto Angle", turret.autoAngleEnabled ? "ON" : "OFF");
            telemetry.addData("Tag Found", turret.isTagFound() ? "YES" : "NO");
            telemetry.addData("Distance", "%.1f in", turret.getDistanceToTarget());
            telemetry.addData("Tracking Error", "%.1f°", turret.getTrackingError());
            telemetry.addData("Turret Yaw Power", "%.3f", turret.turretMotor.getPower());
            telemetry.addData("Turret Yaw Deg", "%.1f°", turret.turretMotor.getCurrentPosition() / turret.PARAMS.TICKS_PER_BIG_GEAR_DEGREE);

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Shooting", turret.shootingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Left RPM", "%.0f", turret.getCurrentRPMLeft());
            telemetry.addData("Right RPM", "%.0f", turret.getCurrentRPMRight());
            telemetry.addData("Target RPM", "%.0f", turret.getTargetRPM());
            telemetry.addData("Up to Speed", turret.isUpToSpeed() ? "READY" : "SPINNING UP");

            telemetry.addLine();
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Power", "%.2f", intake.getIntakePower());
            telemetry.addData("Gate", intake.isGateOpen() ? "OPEN" : "CLOSED");
            telemetry.addData("Turret KP", turret.PARAMS.turretKP);
            telemetry.addData("Turret KF", turret.PARAMS.turretKF);
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("GP1: Drive (L-stick), Intake (A/B/X/LT)");
            telemetry.addLine("GP1: RB=Fast, LB=Slow");
            telemetry.addLine("GP2-B: Toggle FULL AUTO (Vision + RPM + Angle)");
            telemetry.addLine("GP2-A: Toggle Vision vs Odom Tracking");
            telemetry.addLine("GP2 Right Stick X: MANUAL TURRET YAW (when tracking OFF)");
            telemetry.addLine("GP2-RT: Shooting");
            telemetry.addLine("GP2-Dpad: RPM (up/down) / Angle (left/right)");
            telemetry.addLine("GP2-B: Toggle Gate");

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ZeusTeleOPBlue", group = "TeleOp")
public class ZeusTeleOPBlue extends LinearOpMode {

    private Turret turret;
    private Intake intake; //
    private Pose2d initialPose = new Pose2d(0, 0, 0);

    private double speedRatio = 0.75;

    // NEW: Track which aiming mode is active
    private boolean usingOdomTracking = false;

    @Override
    public void runOpMode() {

        // Initialize all systems
        turret = new Turret(hardwareMap, telemetry, initialPose);
        intake = new Intake(hardwareMap, telemetry);

        // Configure turret
        turret.PARAMS.TARGET_TAG_ID = 20;
        turret.setAutoAngleEnabled(false);  // Start with manual angle control
        turret.setAutoRPMEnabled(false);    // Start with manual RPM control
        turret.setTrackingMode(false);      // Start with manual tracking
        turret.setContinuousTracking(true); // Enable smooth continuous tracking
        turret.setUseOdometryTracking(false); // Start with vision (Limelight)

        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("Turret + Drive + Intake Initialized");
        telemetry.addLine("Target Tag ID: 20");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1: Drive Controls");
        telemetry.addLine("Gamepad 2: Turret & Intake Controls");
        telemetry.addLine();
        telemetry.addLine("New: GP2 Right Stick Button → Toggle Vision vs Odom Tracking");
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

            // Speed control
            if (gamepad1.right_bumper) {
                speedRatio = 1.0;  // Full speed
            } else if (gamepad1.left_bumper) {
                speedRatio = 0.3;  // Slow speed
            } else {
                speedRatio = 0.75; // Normal speed
            }

            // =========================
            // GAMEPAD 2: TURRET CONTROLS
            // =========================

            // Toggle ALL tracking features with A button
            if (gamepad2.a) {
                boolean newState = !turret.trackingMode;
                turret.setTrackingMode(newState);
                turret.setAutoAngleEnabled(newState);
                turret.setAutoRPMEnabled(newState);
            }

            // NEW: Toggle between Limelight vision and Odometry tracking
            if (gamepad2.right_stick_button) {
                usingOdomTracking = !usingOdomTracking;
                turret.setUseOdometryTracking(usingOdomTracking);
                // Small delay to prevent rapid toggling (debounce)
                sleep(200);
            }

            // Enable/disable shooting with right trigger
            if (gamepad2.right_trigger > 0.2) {
                turret.setShootingEnabled(true);
            } else {
                turret.setShootingEnabled(false);
            }

            // Manual RPM adjustment (D-pad up/down)
            if (gamepad2.dpad_up) {
                turret.setTargetRPM(turret.getTargetRPM() + 50.0);
            }
            if (gamepad2.dpad_down) {
                turret.setTargetRPM(Math.max(0, turret.getTargetRPM() - 50.0));
            }

            // Manual turret angle control (D-pad left/right)
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

            // Gate control (independent)
            if (gamepad2.right_bumper) {
                intake.openGate();
            }
            if (gamepad2.left_bumper) {
                intake.closeGate();
            }

            // Toggle gate (B button)
            if (gamepad2.b) {
                intake.toggleGate();
            }

            // =========================
            // UPDATE ALL SYSTEMS
            // =========================

            turret.update(forward, strafe, rotation);

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Speed Mode", speedRatio == 1.0 ? "FAST" : (speedRatio == 0.3 ? "SLOW" : "NORMAL"));
            telemetry.addData("Position", "X: %.1f, Y: %.1f, H: %.1f°",
                    turret.getPose().position.x,
                    turret.getPose().position.y,
                    Math.toDegrees(turret.getPose().heading.toDouble()));

            telemetry.addLine();
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Auto Mode", turret.trackingMode ? "FULL AUTO" : "MANUAL");
            telemetry.addData("Tracking", turret.trackingMode ? "ON" : "OFF");
            telemetry.addData("Tracking Source", usingOdomTracking ? "ODOMETRY (-60,-60)" : "LIMELIGHT VISION");
            telemetry.addData("Aligned", turret.isAligned() ? "YES" : "NO");
            telemetry.addData("Auto RPM", turret.autoRPMEnabled ? "ON" : "OFF");
            telemetry.addData("Auto Angle", turret.autoAngleEnabled ? "ON" : "OFF");
            telemetry.addData("Tag Found", turret.isTagFound() ? "YES" : "NO");
            telemetry.addData("Distance", "%.1f in", turret.getDistanceToTarget());
            telemetry.addData("Tracking Error", "%.1f°", turret.getTrackingError());

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Shooting", turret.shootingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Left RPM", "%.0f", turret.getCurrentRPMLeft());
            telemetry.addData("Right RPM", "%.0f", turret.getCurrentRPMRight());
            telemetry.addData("Target RPM", "%.0f", turret.getTargetRPM());
            telemetry.addData("Up to Speed", turret.isUpToSpeed() ? "READY" : "SPINNING UP");
            telemetry.addData("Turret Angle", "%.2f", turret.getTurretAnglePosition());

            telemetry.addLine();
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Intake Power", "%.2f", intake.getIntakePower());
            telemetry.addData("Gate", intake.isGateOpen() ? "OPEN" : "CLOSED");

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("GP1: Drive (L-stick move, R-stick rotate)");
            telemetry.addLine("GP1: RB=Fast, LB=Slow");
            telemetry.addLine("GP2-A: Toggle FULL AUTO MODE (Vision + RPM + Angle)");
            telemetry.addLine("GP2-Right Stick Btn: Toggle Vision vs Odom Tracking");
            telemetry.addLine("GP2-X: Intake + Gate OPEN");
            telemetry.addLine("GP2-Y: Intake + Gate CLOSED");
            telemetry.addLine("GP2-RT: Enable Shooting");
            telemetry.addLine("GP2-LT: Manual Intake Power");
            telemetry.addLine("GP2-RB: Open Gate, LB: Close Gate");
            telemetry.addLine("GP2-B: Toggle Gate");
            telemetry.addLine("GP2-Dpad: Manual RPM (up/down) / Angle (left/right)");

            telemetry.update();
        }
    }
}
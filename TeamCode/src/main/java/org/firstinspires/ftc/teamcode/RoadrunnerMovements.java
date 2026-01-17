package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RoadRunner Movement Tester", group = "Testing")
public class RoadrunnerMovements extends LinearOpMode {

    // Movement type enum
    private enum MovementType {
        SPLINE_TO_CONSTANT_HEADING("SplineToConstantHeading"),
        SPLINE_TO_LINEAR_HEADING("SplineToLinearHeading"),
        SPLINE_TO_SPLINE_HEADING("SplineToSplineHeading"),
        LINE_TO("LineTo"),
        LINE_TO_X("LineToX"),
        LINE_TO_Y("LineToY"),
        STRAFE_TO("StrafeTo"),
        TURN("Turn");

        private final String displayName;

        MovementType(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    private MecanumDrive drive;
    private MovementType currentMovementType = MovementType.SPLINE_TO_CONSTANT_HEADING;

    // Target position and heading
    private double targetX = 24.0;
    private double targetY = 24.0;
    private double targetHeading = Math.toRadians(90); // 90 degrees
    private double tangent = 0.0; // Tangent for spline movements

    // Step sizes
    private final double POSITION_STEP = 6.0; // inches
    private final double HEADING_STEP = Math.toRadians(15); // 15 degrees
    private final double TANGENT_STEP = Math.toRadians(30); // 30 degrees

    // Button state tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("RoadRunner Movement Tester");
        telemetry.addLine("---------------------------");
        telemetry.addLine("Controls:");
        telemetry.addLine("D-Pad Up/Down: Adjust Y position");
        telemetry.addLine("D-Pad Left/Right: Adjust X position");
        telemetry.addLine("Left/Right Trigger: Adjust heading");
        telemetry.addLine("Left/Right Bumper: Adjust tangent");
        telemetry.addLine("B: Cycle movement type");
        telemetry.addLine("X: Reset target to 24, 24, 90°");
        telemetry.addLine("Y: Reset robot pose to 0, 0, 0°");
        telemetry.addLine("A: Execute movement");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update pose estimate
            drive.updatePoseEstimate();

            // Get current pose
            Pose2d currentPose = drive.localizer.getPose();

            // Handle button presses
            handleInput();

            // Display telemetry
            displayTelemetry(currentPose);
        }
    }

    private void handleInput() {
        // D-Pad controls for X and Y position
        if (gamepad1.dpad_up && !lastDpadUp) {
            targetY += POSITION_STEP;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            targetY -= POSITION_STEP;
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            targetX += POSITION_STEP;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            targetX -= POSITION_STEP;
        }

        // Trigger controls for heading (analog)
        if (gamepad1.left_trigger > 0.3) {
            targetHeading -= HEADING_STEP * gamepad1.left_trigger * 0.05;
        }
        if (gamepad1.right_trigger > 0.3) {
            targetHeading += HEADING_STEP * gamepad1.right_trigger * 0.05;
        }

        // Bumper controls for tangent
        if (gamepad1.left_bumper && !lastLeftBumper) {
            tangent -= TANGENT_STEP;
        }
        if (gamepad1.right_bumper && !lastRightBumper) {
            tangent += TANGENT_STEP;
        }

        // Normalize heading to -PI to PI range
        while (targetHeading > Math.PI) targetHeading -= 2 * Math.PI;
        while (targetHeading < -Math.PI) targetHeading += 2 * Math.PI;

        // Normalize tangent to -PI to PI range
        while (tangent > Math.PI) tangent -= 2 * Math.PI;
        while (tangent < -Math.PI) tangent += 2 * Math.PI;

        // B button - cycle through movement types
        if (gamepad1.b && !lastB) {
            int nextIndex = (currentMovementType.ordinal() + 1) % MovementType.values().length;
            currentMovementType = MovementType.values()[nextIndex];
        }

        // X button - reset target position
        if (gamepad1.x && !lastX) {
            targetX = 24.0;
            targetY = 24.0;
            targetHeading = Math.toRadians(90);
            tangent = 0.0;
        }

        // Y button - reset robot pose
        if (gamepad1.y && !lastY) {
            drive.localizer.setPose(new Pose2d(0, 0, 0));
        }

        // A button - execute movement
        if (gamepad1.a && !lastA) {
            executeMovement();
        }

        // Update button states
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }

    private void executeMovement() {
        Action action = null;
        Pose2d currentPose = drive.localizer.getPose();

        telemetry.addLine("=== EXECUTING MOVEMENT ===");
        telemetry.addData("Type", currentMovementType.getDisplayName());
        telemetry.update();

        switch (currentMovementType) {
            case SPLINE_TO_CONSTANT_HEADING:
                action = drive.actionBuilder(currentPose)
                        .splineToConstantHeading(new Vector2d(targetX, targetY), tangent)
                        .build();
                break;

            case SPLINE_TO_LINEAR_HEADING:
                action = drive.actionBuilder(currentPose)
                        .splineToLinearHeading(new Pose2d(targetX, targetY, targetHeading), tangent)
                        .build();
                break;

            case SPLINE_TO_SPLINE_HEADING:
                action = drive.actionBuilder(currentPose)
                        .splineToSplineHeading(new Pose2d(targetX, targetY, targetHeading), tangent)
                        .build();
                break;

            case LINE_TO:
                action = drive.actionBuilder(currentPose)
                        .lineToX(targetX)
                        .lineToY(targetY)
                        .build();
                break;

            case LINE_TO_X:
                action = drive.actionBuilder(currentPose)
                        .lineToX(targetX)
                        .build();
                break;

            case LINE_TO_Y:
                action = drive.actionBuilder(currentPose)
                        .lineToY(targetY)
                        .build();
                break;

            case STRAFE_TO:
                action = drive.actionBuilder(currentPose)
                        .strafeTo(new Vector2d(targetX, targetY))
                        .build();
                break;

            case TURN:
                action = drive.actionBuilder(currentPose)
                        .turn(targetHeading - currentPose.heading.toDouble())
                        .build();
                break;
        }

        // Run the action
        if (action != null) {
            com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();

            while (opModeIsActive() && !isStopRequested()) {
                packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
                boolean running = action.run(packet);
                drive.updatePoseEstimate();

                telemetry.addLine("=== MOVEMENT IN PROGRESS ===");
                telemetry.addData("Status", running ? "Running..." : "Complete");
                displayTelemetry(drive.localizer.getPose());

                if (!running) break;
            }

            telemetry.addLine("=== MOVEMENT COMPLETE ===");
            telemetry.update();
            sleep(500);
        }
    }

    private void displayTelemetry(Pose2d currentPose) {
        telemetry.addLine("=== CURRENT STATUS ===");
        telemetry.addData("Movement Type", currentMovementType.getDisplayName());
        telemetry.addLine();

        telemetry.addLine("=== CURRENT POSE ===");
        telemetry.addData("X", "%.2f in", currentPose.position.x);
        telemetry.addData("Y", "%.2f in", currentPose.position.y);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.addLine();

        telemetry.addLine("=== TARGET ===");
        telemetry.addData("Target X", "%.2f in", targetX);
        telemetry.addData("Target Y", "%.2f in", targetY);
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.addData("Tangent", "%.1f°", Math.toDegrees(tangent));
        telemetry.addLine();

        telemetry.addLine("=== DELTA ===");
        telemetry.addData("ΔX", "%.2f in", targetX - currentPose.position.x);
        telemetry.addData("ΔY", "%.2f in", targetY - currentPose.position.y);
        telemetry.addData("ΔHeading", "%.1f°",
                Math.toDegrees(targetHeading - currentPose.heading.toDouble()));
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("D-Pad: Adjust X/Y (±" + POSITION_STEP + " in)");
        telemetry.addLine("Triggers: Adjust Heading");
        telemetry.addLine("Bumpers: Adjust Tangent (±" + Math.toDegrees(TANGENT_STEP) + "°)");
        telemetry.addLine("B: Change Type | X: Reset Target");
        telemetry.addLine("Y: Reset Pose | A: Execute");

        telemetry.update();
    }
}
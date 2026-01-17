package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Apollo 1 - Optimized", group = "Autonomous")
public class AutonomousSample extends LinearOpMode {

    private String[] pattern = {"P", "G", "P"}; // Default pattern
    private ElapsedTime autoTimer = new ElapsedTime();
    private volatile boolean timeExpired = false;
    private static final double TIME_LIMIT = 28.0; // seconds

    @Override
    public void runOpMode() {
        // =======================================
        // INITIALIZATION
        // =======================================
        Pose2d startPose = new Pose2d(-44.97, -50.58, Math.toRadians(-38.7));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        TurretCopy turret = new TurretCopy(hardwareMap, telemetry);
        StorageWLoaderCopy storage = new StorageWLoaderCopy(hardwareMap, turret);

        // Initialize the reusable actions class
        SSMyRobotCopy actions = new SSMyRobotCopy(drive, turret, storage, this);
        turret.PARAMS.TARGET_TAG_ID = 20;
        // Setup limelight
        turret.limelight.pipelineSwitch(0);
        turret.limelight.setPollRateHz(100);
        turret.limelight.start();

        turret.setShootingEnabled(true);
        turret.setTrackingMode(false);

        telemetry.addLine("AUTO READY - OPTIMIZED");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Start the timer
        autoTimer.reset();

        // Start a thread to monitor the timer
        Thread timerThread = new Thread(() -> {
            while (!timeExpired && !isStopRequested()) {
                if (autoTimer.seconds() >= TIME_LIMIT) {
                    timeExpired = true;
                    break;
                }
                try {
                    Thread.sleep(50); // Check every 50ms
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        timerThread.start();

        // =======================================
        // AUTONOMOUS SEQUENCE - OPTIMIZED
        // =======================================
        try {
            Actions.runBlocking(
                    new SequentialAction(
                            // Move to motif detection position
                            drive.actionBuilder(startPose)
                                    .splineToLinearHeading(
                                            new Pose2d(-24.45, -30, Math.toRadians(-43.4)),
                                            Math.toRadians(30)
                                    )
                                    .build(),

                            // Detect which pattern to shoot based on AprilTags
                            actions.detectMotif(pattern),

                            // Turn to shooting heading (23.4°)
                            drive.actionBuilder(drive.localizer.getPose())
                                    .turnTo(Math.toRadians(23.4))
                                    .build(),

                            // Enable tracking and shoot the pattern
                            actions.turnOnTracking(0.0, 2.0),
                            actions.shootPattern(pattern, 5.0),
                            actions.turnOffTracking(),

                            // OPTIMIZED: Single spline to intake positions with intake running
                            actions.startIntake(1.0),

                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(
                                            new Pose2d(-10.3, -33.3, Math.toRadians(-63.4)),
                                            Math.toRadians(-90)
                                    )
                                    .splineToLinearHeading(
                                            new Pose2d(-4.95, -57.2, Math.toRadians(-91.2)),
                                            Math.toRadians(-90)
                                    )
                                    .build(),

                            // Stop intake after collecting
                            actions.stopIntake(),

                            // Check timer before continuing
                            (telemetryPacket) -> {
                                if (timeExpired) throw new RuntimeException("Timer expired");
                                return false;
                            },

                            // OPTIMIZED: Direct spline back to shooting position
                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(
                                            new Pose2d(-24.45, -30, Math.toRadians(23.4)),
                                            Math.toRadians(23.4)
                                    )
                                    .build(),

                            // Shoot second set
                            actions.turnOnTracking(0.0, 2.0),
                            actions.shootPattern(pattern, 5.0),
                            actions.turnOffTracking(),

                            // OPTIMIZED: Continuous spline through second intake path
                            actions.startIntake(1.0),

                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(-60))
                                    .splineToLinearHeading(
                                            new Pose2d(8.4, -35.4, Math.toRadians(-60.13)),
                                            Math.toRadians(-60.13)
                                    )
                                    .splineToLinearHeading(
                                            new Pose2d(18, -58.216, Math.toRadians(-72.37)),
                                            Math.toRadians(-72.37)
                                    )
                                    .build(),

                            // Stop intake
                            actions.stopIntake(),

                            // OPTIMIZED: Direct return to final shooting position
                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(
                                            new Pose2d(-24.45, -30, Math.toRadians(23.4)),
                                            Math.toRadians(23.4)
                                    )
                                    .build(),

                            // Final shooting sequence
                            actions.turnOnTracking(0.0, 2.0),
                            actions.shootPattern(pattern, 5.0),
                            actions.turnOffTracking()
                    )
            );
        } catch (Exception e) {
            // Timer expired or other exception - proceed to parking position
            telemetry.addLine("Sequence interrupted - moving to park position");
            telemetry.update();
        }

        // If time expired or sequence finished, go to parking position
        if (timeExpired || autoTimer.seconds() >= TIME_LIMIT) {
            telemetry.addData("Timer", "%.1f seconds - Going to park", autoTimer.seconds());
            telemetry.update();

            // Go to parking position
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(
                                    new Pose2d(-8.397934080108882, -51.16613673412894, Math.toRadians(-138.78761344396153)),
                                    Math.toRadians(-138.78761344396153)
                            )
                            .build()
            );
        }

        turret.setShootingEnabled(false);
        telemetry.addLine("AUTO FINISHED");
        telemetry.addData("Total Time", "%.1f seconds", autoTimer.seconds());
        telemetry.update();
    }
}
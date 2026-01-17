package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Apollo 2.0 - Red Near Zone", group = "Autonomous")
public class ApolloNearZoneRed extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private StorageWLoader kickers;
    private Pose2d startPose = new Pose2d(-44.97, 50.58, Math.toRadians(38.7));
    private Pose2d shotingPos = new Pose2d(-24.45, 30, Math.toRadians(-23.4));
    private Vector2d shotingVector = new Vector2d(-24.45,30);
    private Pose2d detect = new Pose2d (-24.45, 30, Math.toRadians(43.4));
    private Pose2d firstStartIntake = new Pose2d(-8.1687, 29.5788, Math.toRadians(65.05));
    private Pose2d firstFinishIntake = new Pose2d(-8.95, 55.2,Math.toRadians(91.2));
    private SSMyRobot myRobot;

    @Override
    public void runOpMode() {
        Action motiffSequence;
        Action motiffSequence2;
        Action motiffSequence3;

        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, startPose);
        kickers = new StorageWLoader(hardwareMap, turretSystem);
        myRobot = new SSMyRobot(hardwareMap, drive, kickers, turretSystem, startPose);


        waitForStart();

        kickers.generalTimerReset();
        // =========================
        // Initial Setup
        // =========================
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 24;
        kickers.update();
        kickers.loadingUpdate();

        // =========================
        // PHASE 1: DRIVE TO DETECTION POSITION
        // =========================
        //Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .splineToLinearHeading(
                                detect,
                                Math.toRadians(30)
                        )
                        .build()
                // myRobot.turnOffUpdate()

        );

        drive.updatePoseEstimate();

        // Now detect motif at the detection position
        Actions.runBlocking(myRobot.motiffUpdate());

        telemetry.addData("Motiff 0", turretSystem.motiff[0]);
        telemetry.addData("Motiff 1", turretSystem.motiff[1]);
        telemetry.addData("Motiff 2", turretSystem.motiff[2]);
        telemetry.update();

        // Build shooting sequences based on detected motif
        if (turretSystem.motiff[0].equals("G")) { // GPP
            motiffSequence = new SequentialAction(
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
            motiffSequence2 = new SequentialAction(
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
            motiffSequence3 = new SequentialAction(
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
        } else if (turretSystem.motiff[1].equals("G")) { // PGP
            motiffSequence = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
            motiffSequence2 = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
            motiffSequence3 = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
        } else { // PPG
            motiffSequence = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.lastAfterLoad()
            );
            motiffSequence2 = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.lastAfterLoad()
            );
            motiffSequence3 = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.lastAfterLoad()
            );
        }

        // =========================
        // PHASE 2: Turn to shooting position and shoot preloads
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.reverseTransfer(),
                new SequentialAction(
                        // Turn to shooting heading
                        drive.actionBuilder(drive.localizer.getPose())
                                .splineToLinearHeading(shotingPos,Math.toRadians(0))
                                .build(),
                        myRobot.turnOnTracking(),
                        myRobot.reverseTransferStop(),
                        myRobot.turnOffUpdate()
                )
        ));

        // Shoot preloads
        Actions.runBlocking(motiffSequence);

        drive.updatePoseEstimate();

        // =========================
        // PHASE 3: First intake cycle
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.turnOffTracking(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .splineToLinearHeading(firstStartIntake, Math.toRadians(35.05))
                                .build(),
                        myRobot.intake(1.0),
                        myRobot.turnOffUpdate()

                )
        ));

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .splineToLinearHeading(firstFinishIntake, Math.toRadians(0), new TranslationalVelConstraint(8))

                                .build(),

                        myRobot.turnOffUpdate()
                )
        ));

        drive.updatePoseEstimate();

        // =========================
        // PHASE 4: Return to shooting position and shoot first cycle
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.reverseTransfer(),

                new SequentialAction(
                        myRobot.intake(0),
                        drive.actionBuilder(drive.localizer.getPose())

                                .setReversed(true).splineToLinearHeading(shotingPos, Math.toRadians(0))
                                .build(),

                        myRobot.reverseTransferStop(),
                        myRobot.turnOffUpdate()

                )
        ));

        drive.updatePoseEstimate();

        Actions.runBlocking(new SequentialAction(
                myRobot.turnOnTracking(),
                myRobot.shooterSpinUp(),
                motiffSequence2
        ));

        // =========================
        // PHASE 5: Second intake cycle
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.turnOffTracking(),
                        drive.actionBuilder(drive.localizer.getPose())

                                .splineToLinearHeading(new Pose2d(8.4, 31.4,Math.toRadians(60.13)), Math.toRadians(-60.13))
                                .build(),
                        myRobot.intake(1.0),
                        myRobot.turnOffUpdate()
                )
        ));

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .splineTo(new Vector2d(18, 55.216), Math.toRadians(72.37), new TranslationalVelConstraint(8))
                                .build(),
                        myRobot.intake(0),
                        myRobot.turnOffUpdate()
                )
        ));

        drive.updatePoseEstimate();

        // =========================
        // PHASE 6: Final shooting position
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.reverseTransfer(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())

                                .strafeToLinearHeading(shotingVector, Math.toRadians(-23.4))
                                .build(),
                        myRobot.reverseTransferStop(),
                        myRobot.turnOffUpdate()
                )
        ));

        drive.updatePoseEstimate();

        Actions.runBlocking(new SequentialAction(
                myRobot.turnOnTracking(),
                myRobot.shooterSpinUp(),
                motiffSequence3,
                myRobot.turnOffTracking()
        ));

        //=========================
        // Park
        //=========================

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.reverseTransfer(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())

                                .strafeToLinearHeading(new Vector2d(4,35.4), Math.toRadians(-23.4))
                                .build(),
                        myRobot.reverseTransferStop(),
                        myRobot.turnOffUpdate()
                )
        ));

        // =========================
        // Telemetry loop for debugging
        // =========================
        while (opModeIsActive()) {
            telemetry.addData("timer", kickers.generalTimer);
            telemetry.addData("change flag trigger value", kickers.changeFlagTrigger);
            telemetry.addData("flag", kickers.flag);
            telemetry.addData("front dis", kickers.dis);
            telemetry.addData("front pos", kickers.frontPos);
            telemetry.addData("middle pos", kickers.middlePos);
            telemetry.addData("back pos", kickers.backPos);
            telemetry.addData("Kick Target", kickers.kickTarget);
            telemetry.addData("Ball Count", kickers.count);
            telemetry.addData("Front Slot", kickers.ballArray[0]);
            telemetry.addData("Middle Slot", kickers.ballArray[1]);
            telemetry.addData("Back Slot", kickers.ballArray[2]);
            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            telemetry.addData("Target RPM", "%.1f", turretSystem.targetRPM);
            telemetry.addData("shotdetected", turretSystem.shotDetected);
            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", turretSystem.trackingMode ? "ON" : "OFF");
            telemetry.addData("Tag Visible", turretSystem.telemetryData.tagFound);
            telemetry.addData("Error Angle (deg)", "%.2f", turretSystem.telemetryData.errorAngleDeg);
            telemetry.addData("Turret Power", "%.3f", turretSystem.telemetryData.turretPower);
            telemetry.addData("calculated distance in inches ", turretSystem.disToAprilTag);
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ZeusAutoNearZone", group = "Blue Near")
public class ZeusAutoNearZone extends LinearOpMode {

   // private NewRobot robot;

    // ─── POSES (same as your old auto) ──────────────────────────────────────
    private final Pose2d startPose      = new Pose2d(-46.74, -51.76, Math.toRadians(-37.9));
    private final Vector2d detectVec    = new Vector2d(-25.5, -21.7);
    private final double   detectHead   = Math.toRadians(-38.1);

    private final Vector2d shoot1Vec    = new Vector2d(-34.66, -26.08);
    private final double   shoot1Head   = Math.toRadians(45);

    private final Pose2d   intake1Start = new Pose2d(-21.75, -32.5, Math.toRadians(-45.2));
    private final Pose2d   intake1End   = new Pose2d(-8.6,   -51.0, Math.toRadians(-55.4));

    private final Pose2d   intake2Start = new Pose2d(  2.25, -32.5, Math.toRadians(-45.2));
    private final Pose2d   intake2End   = new Pose2d( 15.4,  -51,   Math.toRadians(-55.4));

    private final Vector2d shoot2Vec    = new Vector2d(-35.4, -19);
    private final double   shoot2Head   = Math.toRadians(50);

    @Override
    public void runOpMode() {
/*
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);  // your drive class
        robot = new NewRobot(hardwareMap, drive, startPose);

        // Wait for start
        waitForStart();
        if (isStopRequested()) return;

        robot.turret.PARAMS.TARGET_TAG_ID = 20;

        // ─── PHASE 1: Detect motif ──────────────────────────────────────────
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(detectVec, detectHead)
                        .build()
        );

        // Update motiff
        Actions.runBlocking(robot.motiffUpdate());
        sleep(400); // extra settle time for vision

        String[] shootOrder = robot.getShootOrder();

        telemetry.addData("Motiff", String.join(",", robot.turret.motiff));
        telemetry.addData("Shoot Order", String.join(" → ", shootOrder));
        telemetry.update();

        // ─── PHASE 2: First shooting position + preloads ────────────────────
        Actions.runBlocking(
                new ParallelAction(
                        robot.subsystemUpdate(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(shoot1Vec, shoot1Head)
                                .build()
                )
        );

        Actions.runBlocking(robot.spinUpFlywheel());
        sleep(600); // wait for spin-up

        Actions.runBlocking(robot.trackingOn());

        // Store balls in order and shoot 3 preloads
        Actions.runBlocking(robot.storeForOrder(shootOrder[0], shootOrder[1], shootOrder[2]));
        for (int i = 0; i < 3; i++) {
            Actions.runBlocking(robot.waitForNextShotReady(4.0));
            sleep(400); // shot settle time
        }

        Actions.runBlocking(robot.trackingOff());
        Actions.runBlocking(robot.stopFlywheel());

        // ─── PHASE 3: First intake cycle ────────────────────────────────────
        Actions.runBlocking(
                new ParallelAction(
                        robot.subsystemUpdate(),
                        new SequentialAction(
                                robot.intakeOn(0.95),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .setReversed(true)
                                        .splineToLinearHeading(intake1Start, Math.toRadians(0))
                                        .splineToLinearHeading(intake1End, Math.toRadians(0))
                                        .build(),
                                robot.intakeOff()
                        )
                )
        );

        // ─── PHASE 4: Return + shoot cycle 1 ───────────────────────────────
        Actions.runBlocking(
                new ParallelAction(
                        robot.subsystemUpdate(),
                        new SequentialAction(
                                robot.spinUpFlywheel(),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .strafeToLinearHeading(shoot1Vec, shoot1Head)
                                        .build(),
                                robot.trackingOn()
                        )
                )
        );

        // Store and shoot 3 more
        Actions.runBlocking(robot.storeForOrder(shootOrder[0], shootOrder[1], shootOrder[2]));
        for (int i = 0; i < 3; i++) {
            Actions.runBlocking(robot.waitForNextShotReady(4.0));
            sleep(400);
        }

        Actions.runBlocking(robot.trackingOff());
        Actions.runBlocking(robot.stopFlywheel());

        // ─── PHASE 5: Second intake cycle ───────────────────────────────────
        Actions.runBlocking(
                new ParallelAction(
                        robot.subsystemUpdate(),
                        new SequentialAction(
                                robot.intakeOn(0.95),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .setReversed(true)
                                        .splineToLinearHeading(intake2Start, Math.toRadians(0))
                                        .splineToLinearHeading(intake2End, Math.toRadians(0))
                                        .build(),
                                robot.intakeOff()
                        )
                )
        );

        // ─── PHASE 6: Final shooting position + shoot ───────────────────────
        Actions.runBlocking(
                new ParallelAction(
                        robot.subsystemUpdate(),
                        new SequentialAction(
                                robot.spinUpFlywheel(),
                                drive.actionBuilder(drive.localizer.getPose())
                                        .strafeToLinearHeading(shoot2Vec, shoot2Head)
                                        .build(),
                                robot.trackingOn()
                        )
                )
        );

        // Store and shoot remaining
        Actions.runBlocking(robot.storeForOrder(shootOrder[0], shootOrder[1], shootOrder[2]));
        for (int i = 0; i < 3; i++) {
            Actions.runBlocking(robot.waitForNextShotReady(4.0));
            sleep(400);
        }

        Actions.runBlocking(robot.trackingOff());
        Actions.runBlocking(robot.stopFlywheel());

        // Optional park
        // Actions.runBlocking(drive.actionBuilder(...).strafeToLinearHeading(...).build());

        // End
        while (opModeIsActive() && !isStopRequested()) {
            robot.turret.update();
            robot.intake.storageUpdate();
            telemetry.addData("Balls", robot.intake.ballCount);
            telemetry.addData("Storage", String.join(" ", robot.intake.storage));
            telemetry.update();
        }

        */
    }
}
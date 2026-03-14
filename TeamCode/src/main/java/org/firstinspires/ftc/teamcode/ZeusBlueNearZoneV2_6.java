package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ZeusBlueNearZoneV2.6", group = "Autonomous")
public class ZeusBlueNearZoneV2_6 extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Intake intake;
    private RobotInfoStorage info;
    private Pose2d startPose = new Pose2d(-45.2766, -61.5312, Math.toRadians(-127.6875));
    private Pose2d firstShootingPos = new Pose2d(-15, -30, Math.toRadians(-115.56));
    private Pose2d shotingPos = new Pose2d(-9.923, -25.695, Math.toRadians(-115.56));
    private Pose2d firstSpikeStart = new Pose2d(-9.746, -34.06, Math.toRadians(-84.316));
    private Pose2d firstSpikeEnd = new Pose2d(-8.5665, -54.037, Math.toRadians(-91.45));
    private Pose2d firstSpikeFurther = new Pose2d(-8.8385, -60.4869, Math.toRadians(-91.697));
    private Pose2d secondSpikeStart = new Pose2d(14.44, -35.727, Math.toRadians(-81.94));
    private Pose2d secondSpikeEnd = new Pose2d(16.352, -59.976, Math.toRadians(-84.186));
    private Pose2d secondSpikeFurther = new Pose2d(17.835, -68.849, Math.toRadians(-91.67));
    private Pose2d gatePrepare = new Pose2d(19, -61.996, Math.toRadians(-113.167));
    private Pose2d gateOpen = new Pose2d(15.7, -70.0368, Math.toRadians(-121.8748));
    private Pose2d thirdSpikeStart = new Pose2d(37.5592, -34.6989, Math.toRadians(-79.336));
    private Pose2d thirdSpikeEnd = new Pose2d(39.354, -58.91, Math.toRadians(-90.622));
    private Pose2d thirdSpikeFurther = new Pose2d(39.148, -66.8586, Math.toRadians(-91.583));
    private Pose2d park = new Pose2d(0.366, -48.9, Math.toRadians(-0.305));
    private Pose2d finalShootingPos = new Pose2d(-35.4, -22, Math.toRadians(-91));

    private SSMyRobot myRobot;

    @Override
    public void runOpMode() {

        // Initialize all systems
        drive = new MecanumDrive(hardwareMap, startPose);
        info = new RobotInfoStorage();
        turretSystem = new Turret(hardwareMap, drive, telemetry, startPose);
        turretSystem.resetTurretEncoder();
        intake = new Intake(hardwareMap, telemetry);
        myRobot = new SSMyRobot(hardwareMap, drive, intake, turretSystem, startPose);

        Actions.runBlocking(myRobot.setTurretAnlge(-12));
        turretSystem.targetRPM = 2500;

        waitForStart();

        intake.generalTimerReset();
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 20;
        intake.storageUpdate();
        RobotInfoStorage.autoEndPose = startPose;

        // =========================
        // PHASE 1: Drive to first shooting position and shoot
        // shooterSpinUp() is called before the drive so the shooter
        // is at speed by the time we arrive
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.contCalcRPMAndAngle(),
                new SequentialAction(
                        myRobot.turnOnTracking(),
                        myRobot.shooterSpinUp(),  // spin up before driving
                        drive.actionBuilder(startPose)
                                .strafeToLinearHeading(firstShootingPos.position, firstShootingPos.heading,
                                        new TranslationalVelConstraint(25))
                                .build(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 2: Drive to second spike to intake
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(secondSpikeStart.position, secondSpikeStart.heading)
                                .strafeToLinearHeading(secondSpikeEnd.position, secondSpikeEnd.heading,
                                        new TranslationalVelConstraint(70))
                                .build(),
                        myRobot.intakePower(0.5),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 3: Shoot — spin up before driving back to shooting pos
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),  // spin up before driving
                        drive.actionBuilder(drive.localizer.getPose())
                                .setReversed(true)
                                .splineToLinearHeading(shotingPos, Math.toRadians(170))
                                .build(),
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 4: Drive to gate to intake
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(secondSpikeStart.position, secondSpikeStart.heading)
                                .strafeToLinearHeading(gatePrepare.position, gatePrepare.heading)
                                .strafeToLinearHeading(gateOpen.position, gateOpen.heading,
                                        new TranslationalVelConstraint(45))
                                .build(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitFullStorage(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 5: Shoot second load — spin up before driving
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),  // spin up before driving
                        drive.actionBuilder(drive.localizer.getPose())
                                .setReversed(true)
                                .splineToLinearHeading(shotingPos, Math.toRadians(170))
                                .build(),
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 6: Drive to gate again to intake
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(secondSpikeStart.position, secondSpikeStart.heading)
                                .strafeToLinearHeading(gatePrepare.position, gatePrepare.heading)
                                .strafeToLinearHeading(gateOpen.position, gateOpen.heading,
                                        new TranslationalVelConstraint(45))
                                .build(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitFullStorage(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 7: Shoot third load — spin up before driving
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),  // spin up before driving
                        drive.actionBuilder(drive.localizer.getPose())
                                .setReversed(true)
                                .splineToLinearHeading(shotingPos, Math.toRadians(170))
                                .build(),
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),
                        myRobot.turnOffTracking(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 8: Drive to first spike to intake
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(firstSpikeStart.position, firstSpikeStart.heading)
                                .strafeToLinearHeading(firstSpikeEnd.position, firstSpikeEnd.heading,
                                        new TranslationalVelConstraint(70))
                                .build(),
                        myRobot.intakePower(0.5),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 9: Final shoot — spin up before driving to finalShootingPos
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),  // spin up before driving
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(finalShootingPos.position, finalShootingPos.heading)
                                .build(),
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // PHASE 10: Park
        // =========================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(park.position, park.heading)
                                .build(),
                        myRobot.turnOffUpdate())
        ));

        // =========================
        // Telemetry loop
        // =========================
        while (opModeIsActive()) {
            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            telemetry.addData("Target RPM", "%.1f", turretSystem.targetRPM);
            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", turretSystem.trackingMode ? "ON" : "OFF");
            telemetry.addData("Calculated distance (in)", turretSystem.disToAprilTag);
            telemetry.update();
        }
    }
}
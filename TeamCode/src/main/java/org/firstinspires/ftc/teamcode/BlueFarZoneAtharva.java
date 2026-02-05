package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ZeusBlueFarZoneAtharva - Gate Eating", group = "Autonomous")
public class BlueFarZoneAtharva extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Intake intake;
    private Pose2d startPose = new Pose2d(61.743, -30.1168, Math.toRadians(-179.8));
    private Pose2d intakeStart = new Pose2d(67.257, -60.174, Math.toRadians(-84.43));
    private Pose2d intakeEnd = new Pose2d(67.544, -67.495, Math.toRadians(-94.685));
    private Pose2d extraIntakeStart = new Pose2d(62.371, -62.0515, Math.toRadians(-91.017));
    private Pose2d extraIntakeEnd = new Pose2d(60.216, -67.157, Math.toRadians(-89.701));
    private Pose2d shootingPos = new Pose2d(59.1073, -22.21, Math.toRadians(-107.479));
    private SSMyRobot myRobot;

    @Override
    public void runOpMode() {
        // Initialize all systems
        drive = new MecanumDrive(hardwareMap, startPose);
        turretSystem = new Turret(hardwareMap, drive, telemetry, startPose);
        intake = new Intake(hardwareMap, telemetry);
        myRobot = new SSMyRobot(hardwareMap, drive, intake, turretSystem, startPose);

        Actions.runBlocking(myRobot.setTurretAnlge(-12));
        turretSystem.targetRPM = 2500;

        waitForStart();

        intake.generalTimerReset();

        // =========================
        // Initial Setup
        // =========================
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 20;
        intake.storageUpdate();

        // =========================
        // PHASE 1: INTAKE AND SHOOT (first cycle)
        // =========================
        drive.updatePoseEstimate();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.contCalcRPMAndAngle(),
                myRobot.shooterSpinUp(),
                new SequentialAction(
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate()
                ),
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(intakeStart.position, intakeStart.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(intakeEnd.position, intakeEnd.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(shootingPos.position, shootingPos.heading, new TranslationalVelConstraint(25))
                        .build()
        ));

        // =========================
        // PHASE 2: INTAKE AND SHOOT (second cycle with extra intake)
        // =========================
        drive.updatePoseEstimate();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.contCalcRPMAndAngle(),
                myRobot.shooterSpinUp(),
                new SequentialAction(
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate()
                ),
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(intakeStart.position, intakeStart.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(intakeEnd.position, intakeEnd.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(extraIntakeStart.position, extraIntakeStart.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(extraIntakeEnd.position, extraIntakeEnd.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(shootingPos.position, shootingPos.heading, new TranslationalVelConstraint(25))
                        .build()
        ));

        // =========================
        // PHASE 3: INTAKE AND SHOOT (third cycle with extra intake)
        // =========================
        drive.updatePoseEstimate();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.contCalcRPMAndAngle(),
                myRobot.shooterSpinUp(),
                new SequentialAction(
                        myRobot.turnOnTracking(),
                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate()
                ),
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(intakeStart.position, intakeStart.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(intakeEnd.position, intakeEnd.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(extraIntakeStart.position, extraIntakeStart.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(extraIntakeEnd.position, extraIntakeEnd.heading, new TranslationalVelConstraint(25))
                        .strafeToLinearHeading(shootingPos.position, shootingPos.heading, new TranslationalVelConstraint(25))
                        .build()
        ));

        // =========================
        // Telemetry loop for debugging
        // =========================
        while (opModeIsActive()) {
            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            telemetry.addData("Target RPM", "%.1f", turretSystem.targetRPM);
            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", turretSystem.trackingMode ? "ON" : "OFF");
            telemetry.addData("calculated distance in inches ", turretSystem.disToAprilTag);
            telemetry.update();
        }
    }
}
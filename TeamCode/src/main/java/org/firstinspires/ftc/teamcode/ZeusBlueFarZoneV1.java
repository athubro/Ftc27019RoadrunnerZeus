package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ZeusBlueFarZoneV1", group = "Autonomous")
public class ZeusBlueFarZoneV1 extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Intake intake;
    private Pose2d startPose = new Pose2d(61.743, -30.1168, Math.toRadians(-179.8));
    private Pose2d lastSpikeStart = new Pose2d(41.3775, -37.8307, Math.toRadians(-99.955));//(-32.66, -24.08, Math.toRadians(45));
    private Pose2d lastSpikeEnd = new Pose2d(40.012, -56.67, Math.toRadians(-90.128));
    private Pose2d lastSpikeFurther = new Pose2d(39.766, -66.571, Math.toRadians(-90.5));

    private Pose2d shotingPos = new Pose2d(59.1073, -22.21, Math.toRadians(-107.479));
    private Pose2d cornerStart = new Pose2d(67.257, -60.174, Math.toRadians(-84.43));
    private Pose2d cornerEnd = new Pose2d(67.544, -67.495, Math.toRadians(-94.685));
    private Pose2d cornerSlideBack = new Pose2d(62.371, -62.0515, Math.toRadians(-91.017));
    private Pose2d cornerSlideFront = new Pose2d(60.216, -67.157, Math.toRadians(-89.701));
    //with intake
    private Pose2d scoopPrepare = new Pose2d(64.697, -45.2048, Math.toRadians(-82.46));

    private Pose2d scoopFront = new Pose2d(65.916, -65.511, Math.toRadians(-91.316));

    private Pose2d sideScoopPrepare = new Pose2d(52.8477, -47.0567, Math.toRadians(-95.688));

    private Pose2d sideScoopFront = new Pose2d(47.781, -65.438, Math.toRadians(-102.629));
    private Pose2d park = new Pose2d(53.697, -33.378, Math.toRadians(-106.992));




    private SSMyRobot myRobot;

    @Override
    public void runOpMode() {
        Action motiffSequence;
        Action motiffSequence2;
        Action motiffSequence3;

        Action alternateMotifSeq;
        Action alternateMotifSeq2;
        Action alternateMotifSeq3;

        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry, startPose);
        drive = new MecanumDrive(hardwareMap, startPose);
        intake = new Intake(hardwareMap, telemetry);
        myRobot = new SSMyRobot(hardwareMap, drive, intake, turretSystem, startPose);

        Actions.runBlocking (myRobot.setTurretAnlge(5));
        turretSystem.targetRPM=2400;



        waitForStart();

        intake.generalTimerReset();
        // =========================
        // Initial Setup
        // =========================
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 20;
        intake.storageUpdate();

        // =========================
        // PHASE 1: DRIVE TO DETECTION POSITION
        // =========================

        //drive.updatePoseEstimate();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.setShooterAngle(0.5),
                        myRobot.setTargetRPM(2600),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose()).
                        strafeToLinearHeading(lastSpikeStart.position,lastSpikeStart.heading).strafeToLinearHeading(lastSpikeEnd.position,lastSpikeEnd.heading,new TranslationalVelConstraint(13)).build(),
                        myRobot.intakePower(0.1),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));


        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading)
                        .build(),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(cornerStart.position,cornerStart.heading)
                        .strafeToLinearHeading(cornerEnd.position,cornerEnd.heading, new TranslationalVelConstraint(12))
                        .strafeToLinearHeading(cornerSlideBack.position,cornerSlideBack.heading)
                        .strafeToLinearHeading(cornerSlideFront.position,cornerSlideFront.heading, new TranslationalVelConstraint(12)).build(),
                        //myRobot.resetIntakeTimer(),
                        //myRobot.waitFullStorage(),

                        myRobot.turnOffUpdate())));

        //========================================================================

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading)

                        .build(),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(scoopPrepare.position,scoopPrepare.heading)
                        .strafeToLinearHeading(scoopFront.position,scoopFront.heading, new TranslationalVelConstraint(12))
                        .strafeToLinearHeading(sideScoopPrepare.position,sideScoopPrepare.heading)
                        .strafeToLinearHeading(sideScoopFront.position,sideScoopFront.heading, new TranslationalVelConstraint(12)).build(),
                       // myRobot.resetIntakeTimer(),
                       // myRobot.waitFullStorage(),

                        myRobot.turnOffUpdate())));
        //========================================================================

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())

                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading)
                        .build(),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())));
/*
/*

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .splineToLinearHeading(secondFinishIntake,0,new TranslationalVelConstraint(13)).build(), //, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();

        // =========================
        // PHASE 6: Final shooting position
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());

        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.intake(0),
                        drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(secondShootingVector,secondShootingHeading).build(),
                        myRobot.reverseTransferStop(),
                        myRobot.turnOnTracking(),
                        myRobot.shooterSpinUp(),
                        myRobot.turnOffUpdate() )));


        frontColor=kickers.ballArray[0];
        if (frontColor.equals("G")||frontColor.equals("U")){
            Actions.runBlocking(alternateMotifSeq3);
        } else{
            Actions.runBlocking(motiffSequence3);
        }
*/
        drive.updatePoseEstimate();

        //=========================
        // Park
        //=========================
        /* no final park, shooting inside the zone.
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(parkVector,parkheading).build(),
                        myRobot.turnOffUpdate() )));
        */
        // =========================
        // Telemetry loop for debugging
        // =========================
        while (opModeIsActive()) {
            /*
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

             */
            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            telemetry.addData("Target RPM", "%.1f", turretSystem.targetRPM);
            //telemetry.addData("shotdetected", turretSystem.shotDetected);
            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", turretSystem.trackingMode ? "ON" : "OFF");
            //telemetry.addData("Tag Visible", turretSystem.telemetryData.tagFound);
           // telemetry.addData("Error Angle (deg)", "%.2f", turretSystem.telemetryData.errorAngleDeg);
           // telemetry.addData("Turret Power", "%.3f", turretSystem.telemetryData.turretPower);
            telemetry.addData("calculated distance in inches ", turretSystem.disToAprilTag);
            telemetry.update();
        }
    }
}
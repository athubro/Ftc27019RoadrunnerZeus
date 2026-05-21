package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ZeusRedNearZoneV1", group = "Autonomous")
public class ZeusRedNearZoneV1 extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Intake intake;
    private RobotInfoStorage info;
    private Pose2d startPose = new Pose2d(-54.811, 40, Math.toRadians(129.052));
    private Pose2d firstShootingPos = new Pose2d(-22.8958, 8.795, Math.toRadians(120.458));
    private Pose2d shotingPos = new Pose2d(-22.8958, 8.795, Math.toRadians(120.458));//(-32.66, -24.08, Math.toRadians(45)); //==========???????????????
    private Pose2d firstSpikeStart = new Pose2d(-18.6545, 20.9864, Math.toRadians(83.1));
    private Pose2d firstSpikeEnd = new Pose2d(-16.574, 41.1757, Math.toRadians(90.924));

    private Pose2d firstSpikeFurther = new Pose2d(-16.5344, 46.313, Math.toRadians(90.07));
    private Pose2d secondSpikeStart = new Pose2d(3.915, 21.7272, Math.toRadians(80.22));
    private Pose2d secondSpikeEnd = new Pose2d(6.746, 47.138, Math.toRadians(89.12));
    private Pose2d secondSpikeFurther = new Pose2d(6.746, 51.7, Math.toRadians(90.4));
    private Pose2d gatePrepare = new Pose2d(7.5839, 45.11, Math.toRadians(123.18));
    //with intake
    private Pose2d gateOpen = new Pose2d(7.64, 50, Math.toRadians(120.61));
    private Pose2d gateRetrive = new Pose2d(10.24, 48.58, Math.toRadians(119.61));

    private Pose2d thirdSpikeStart = new Pose2d(26.58, 22.07, Math.toRadians(77.5));

    private Pose2d thirdSpikeEnd = new Pose2d(29.7, 41.75, Math.toRadians(92.6));

    private Pose2d thirdSpikeFurther = new Pose2d(29.8,50 , Math.toRadians(94.3));
    private Pose2d park = new Pose2d(-0.3, 40.8, Math.toRadians(16));
    private Pose2d finalShootingPos = new Pose2d(-40.8958, 8.795, Math.toRadians(110.458));
    //private Vector2d fina = new Vector2d(-35.4,-19);
    //private double secondShootingHeading = Math.toRadians(50);



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
        drive = new MecanumDrive(hardwareMap, startPose);
        info = new RobotInfoStorage();

        turretSystem = new Turret(hardwareMap, drive, telemetry, startPose);
        turretSystem.resetTurretEncoder();
        //turretSystem.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = new Intake(hardwareMap, telemetry);
        myRobot = new SSMyRobot(hardwareMap, drive, intake, turretSystem, startPose);
        turretSystem.LLFarZoneOffset = -2;
        turretSystem.targetPos = new Vector2d(-53, 60);
        Actions.runBlocking (myRobot.setTurretAnlge(12));
        turretSystem.targetRPM=2500;



        waitForStart();

        intake.generalTimerReset();
        // =========================
        // Initial Setup
        // =========================
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 24;
        intake.storageUpdate();
        RobotInfoStorage.autoEndPose = startPose;

        // =========================
        // PHASE 1: DRIVE TO DETECTION POSITION
        // =========================

        //drive.updatePoseEstimate();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
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
                                myRobot.turnOffUpdate()),
                        drive.actionBuilder(startPose)
                                .strafeToLinearHeading(firstShootingPos.position,firstShootingPos.heading,new TranslationalVelConstraint(25)).build()
                        // myRobot.setShooterAngle(0.5),
                        // myRobot.setTargetRPM(2600),
                )
        );




        //===========================================================================================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose()).
                        strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading).strafeToLinearHeading(secondSpikeEnd.position,secondSpikeEnd.heading,new TranslationalVelConstraint(70)).build(),
                        myRobot.intakePower(0.5),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));


        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),
                        drive.actionBuilder(drive.localizer.getPose()).setReversed(true).splineToLinearHeading(shotingPos,Math.toRadians(-170))
                                .build(),
                        myRobot.turnOnTracking(),

                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),

                        myRobot.shooterStop(),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
                        .strafeToLinearHeading(gatePrepare.position,gatePrepare.heading)
                        .strafeToLinearHeading(gateOpen.position,gateOpen.heading, new TranslationalVelConstraint(45)).waitSeconds(0.1)
                        .strafeToLinearHeading(gateRetrive.position,gateRetrive.heading).build(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitFullStorage(),

                        myRobot.turnOffUpdate())));

        //========================================================================

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .setReversed(true).splineToLinearHeading(shotingPos,Math.toRadians(-170))
                                .build(),
                        myRobot.turnOnTracking(),

                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),

                        myRobot.shooterStop(),
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
                        .strafeToLinearHeading(gateOpen.position,gateOpen.heading, new TranslationalVelConstraint(45)).waitSeconds(0.1)
                        .strafeToLinearHeading(gateRetrive.position,gateRetrive.heading).build(), // speed was 40 to gate open
                        myRobot.resetIntakeTimer(),
                        myRobot.waitFullStorage(),

                        myRobot.turnOffUpdate())));
        //========================================================================

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction(

                        myRobot.shooterSpinUp(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .setReversed(true).splineToLinearHeading(shotingPos,Math.toRadians(-170))
                                .build(),
                        myRobot.turnOnTracking(),

                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),

                        myRobot.shooterStop(),
                        //myRobot.intakePower(0),
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose()).
                        strafeToLinearHeading(firstSpikeStart.position,firstSpikeStart.heading).strafeToLinearHeading(firstSpikeEnd.position,firstSpikeEnd.heading,new TranslationalVelConstraint(70)).build(),
                        myRobot.intakePower(0.5),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));


        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction(

                        myRobot.shooterSpinUp(),
                        drive.actionBuilder(drive.localizer.getPose())

                                .strafeToLinearHeading(finalShootingPos.position,finalShootingPos.heading)
                                .build(),
                        myRobot.turnOnTracking(),

                        myRobot.fireBalls(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),
                        myRobot.turnOffTracking(),
                        //myRobot.intakePower(0),
                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(park.position,park.heading)
                        .build(),


                        myRobot.turnOffUpdate())));





/*
        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose()).
                        strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading).strafeToLinearHeading(secondSpikeEnd.position,secondSpikeEnd.heading,new TranslationalVelConstraint(13)).build(),
                        myRobot.intakePower(0.1),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));


        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading).build(),
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
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
                        .strafeToLinearHeading(gatePrepare.position,gatePrepare.heading)
                        .strafeToLinearHeading(gateOpen.position,gateOpen.heading, new TranslationalVelConstraint(12)).build(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitFullStorage(),

                        myRobot.turnOffUpdate())));

        //========================================================================

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
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
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
                        .strafeToLinearHeading(gatePrepare.position,gatePrepare.heading)
                        .strafeToLinearHeading(gateOpen.position,gateOpen.heading, new TranslationalVelConstraint(12)).build(),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitFullStorage(),

                        myRobot.turnOffUpdate())));
        //========================================================================

        drive.updatePoseEstimate();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(secondSpikeStart.position,secondSpikeStart.heading)
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

        */
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
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

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
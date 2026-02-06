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

@Autonomous(name = "ZeusRedFarZoneV1", group = "Autonomous")
public class ZeusRedFarZoneV1 extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private RobotInfoStorage info;
    private Intake intake;
    private Pose2d startPose = new Pose2d(61.51, -0.357, Math.toRadians(-179.8));
    private Pose2d lastSpikeStart = new Pose2d(33.69, 17.87, Math.toRadians(110.47));//(-32.66, -24.08, Math.toRadians(45));
    private Pose2d lastSpikeEnd = new Pose2d(30.34, 38.62, Math.toRadians(90.9));
    private Pose2d lastSpikeFurther = new Pose2d(30.7, 49.57, Math.toRadians(93.26));

    private Pose2d shotingPos = new Pose2d(48.66, 3.731, Math.toRadians(107.5));
    private Pose2d cornerStart = new Pose2d(54.3645, 46.97, Math.toRadians(87.5));
    private Pose2d cornerEnd = new Pose2d(54.09, 52.846, Math.toRadians(80.5));
    private Pose2d cornerSlideBack = new Pose2d(46.1,42.41 , Math.toRadians(98.6));
    private Pose2d cornerSlideFront = new Pose2d(42.6, 51.7, Math.toRadians(90.3));
    //with intake
    private Pose2d scoopPrepare = new Pose2d(50.06, 32.3, Math.toRadians(82.9));

    private Pose2d scoopFront = new Pose2d(51.3, 51.3, Math.toRadians(89.5));

    private Pose2d sideScoopPrepare = new Pose2d(39.76, 34.5, Math.toRadians(99.5));

    private Pose2d sideScoopFront = new Pose2d(35.5, 52.4, Math.toRadians(88.5));
    private Pose2d park = new Pose2d(38.35, 17.8, Math.toRadians(125));




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
     //   turretSystem.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSystem.LLFarZoneOffset = -2;
        turretSystem.targetPos = new Vector2d(-53, 60);
        intake = new Intake(hardwareMap, telemetry);
        myRobot = new SSMyRobot(hardwareMap, drive, intake, turretSystem, startPose);
        Actions.runBlocking (myRobot.setTurretAnlge(-20));
        turretSystem.targetRPM=2400;



        waitForStart();

        intake.generalTimerReset();
        // =========================
        // Initial Setup
        // =========================
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 24;
        intake.storageUpdate();

        // =========================
        // PHASE 1: DRIVE TO DETECTION POSITION
        // =========================
        RobotInfoStorage.autoEndPose = startPose;
        //drive.updatePoseEstimate();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction(
                        /*

                        myRobot.calcRPMAndAngle(),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffUpdate())));
                         */
                        myRobot.turnOnTracking(),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffTracking(),
                        myRobot.turnOffUpdate())));



        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();


        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose()).
                        strafeToLinearHeading(lastSpikeStart.position,lastSpikeStart.heading).strafeToLinearHeading(lastSpikeEnd.position,lastSpikeEnd.heading,new TranslationalVelConstraint(30)).build(),
                        myRobot.intakePower(0.1),//, new TranslationalVelConstraint(10)
                        myRobot.setTurretAnlge(40),
                        myRobot.turnOffUpdate())));


        drive.updatePoseEstimate();

        RobotInfoStorage.autoEndPose=  drive.localizer.getPose();
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading)
                        .build(),
                        myRobot.turnOnTracking(),

                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffTracking(),

                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

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
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading)

                        .build(),
                        myRobot.turnOnTracking(),


                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),
                        myRobot.turnOffTracking(),

                        myRobot.turnOffUpdate())));

        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

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
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),
                new SequentialAction( drive.actionBuilder(drive.localizer.getPose())

                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading)
                        .build(),
                        myRobot.turnOnTracking(),

                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.resetIntakeTimer(),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),
                        myRobot.turnOffTracking(),

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
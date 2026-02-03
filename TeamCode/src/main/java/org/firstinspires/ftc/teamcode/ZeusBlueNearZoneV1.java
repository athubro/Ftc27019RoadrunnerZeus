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

@Autonomous(name = "ZeusBlueNearZoneV1", group = "Autonomous")
public class ZeusBlueNearZoneV1 extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Intake intake;
    private Pose2d startPose = new Pose2d(-45.2766, -61.5312, Math.toRadians(-127.6875));
    private Pose2d shotingPos = new Pose2d(-8.923, -24.695, Math.toRadians(-115.56));//(-32.66, -24.08, Math.toRadians(45));
    private Pose2d firstSpikeStart = new Pose2d(-9.746, -34.06, Math.toRadians(-84.316));
    private Pose2d firstSpikeEnd = new Pose2d(-8.5665, -54.037, Math.toRadians(-91.45));

    private Pose2d firstSpikeFurther = new Pose2d(-8.8385, -60.4869, Math.toRadians(-91.697));
    private Pose2d secondSpikeStart = new Pose2d(14.44, -35.727, Math.toRadians(-81.94));
    private Pose2d secondSpikeEnd = new Pose2d(16.352, -59.976, Math.toRadians(-84.186));
    private Pose2d secondSpikeFurther = new Pose2d(17.835, -68.849, Math.toRadians(-91.67));
    private Pose2d gatePrepare = new Pose2d(20.0447, -61.996, Math.toRadians(-113.167));
    //with intake
    private Pose2d gateOpen = new Pose2d(16.7857, -70.0368, Math.toRadians(-121.8748));

    private Pose2d thirdSpikeStart = new Pose2d(37.5592, -34.6989, Math.toRadians(-79.336));

    private Pose2d thirdSpikeEnd = new Pose2d(39.354, -58.91, Math.toRadians(-90.622));

    private Pose2d thirdSpikeFurther = new Pose2d(39.148, -66.8586, Math.toRadians(-91.583));
    private Pose2d park = new Pose2d(0.366, -48.9, Math.toRadians(-0.305));




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

        Actions.runBlocking (myRobot.setTurretAnlge(-23));
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
                new SequentialAction( drive.actionBuilder(startPose)
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading).build(),
                        myRobot.setShooterAngle(0.5),
                        myRobot.setTargetRPM(2600),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.turnOffUpdate())));

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
                        .strafeToLinearHeading(shotingPos.position,shotingPos.heading).build(),
                        myRobot.shooterSpinUp(),
                        myRobot.waitSpinUp(),
                        myRobot.openGate(),
                        myRobot.intakePower(1),
                        myRobot.waitEmptyStorage(),
                        myRobot.closeGate(),
                        myRobot.shooterStop(),//, new TranslationalVelConstraint(10)
                        myRobot.turnOffUpdate())));
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
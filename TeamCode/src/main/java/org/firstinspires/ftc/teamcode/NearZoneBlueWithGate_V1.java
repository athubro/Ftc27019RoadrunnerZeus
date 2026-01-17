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

@Autonomous(name = "V1 - Blue Near Zone With Gate", group = "Autonomous")
public class NearZoneBlueWithGate_V1 extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private StorageWLoader kickers;
    private Pose2d startPose = new Pose2d(-46.74, -51.76, Math.toRadians(-37.9));
    private Pose2d shotingPos = new Pose2d(-34.66, -26.08, Math.toRadians(45));//(-32.66, -24.08, Math.toRadians(45));
    private Vector2d shotingVector = new Vector2d(-34.66,-26.06);
    private double shotingHeading = Math.toRadians(45);
    private Pose2d detect = new Pose2d (-25.5, -21.7, Math.toRadians(-38.1));
    private Vector2d detectVector = new Vector2d(-25.5, -21.7);
    private double detectorHeading = Math.toRadians(-38.1);
    private Pose2d firstStartIntake = new Pose2d(-14.9, -32.5, Math.toRadians(-60.2)); //(-7.66, -32.66, Math.toRadians(-86.44));
    private Pose2d firstFinishIntake = new Pose2d(-6, -52,Math.toRadians(-90.40)); //(-6.92, -57.7,Math.toRadians(-90.48));
    private Vector2d gatePath =  new Vector2d(6, -55);
    private double gatePathHeading = Math.toRadians(-90);
    private Pose2d gatePos = new Pose2d(6, -63, Math.toRadians(-95));
    private Pose2d secondStartIntake = new Pose2d(2.25, -32.5, Math.toRadians(-45.2)); //(11.60, -32.0, Math.toRadians(-75.73));
    private Pose2d secondFinishIntake = new Pose2d(17.4, -53,Math.toRadians(-57.4)); //(19.83, -56.8,Math.toRadians(-75))
    private Pose2d secondShootingPos = new Pose2d(-35.4,-19,Math.toRadians(50)); //(-9.9,-17.50,Math.toRadians(36.33));
    private Vector2d secondShootingVector = new Vector2d(-35.4,-19);
    private double secondShootingHeading = Math.toRadians(50);
    private Pose2d finalParkPos = new Pose2d(-2.41,-29.1,Math.toRadians(54.1));
    private Vector2d parkVector = new Vector2d(-2.41,-29.1);
    private double parkheading = Math.toRadians(54.1);

    private Pose2d gateReady = new Pose2d(-7.49, -46.7, Math.toRadians(1));
    private Pose2d openGate = new Pose2d(-5.87, -53.515, Math.toRadians(-2));
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
        turretSystem = new Turret(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, startPose);
        kickers = new StorageWLoader(hardwareMap, turretSystem);
        myRobot = new SSMyRobot(hardwareMap, drive, kickers, turretSystem, startPose);

        Actions.runBlocking (myRobot.lookMotifBlue());
        turretSystem.targetRPM=2400;
        kickers.rpmFieldOffset=0.99;



        waitForStart();

        kickers.generalTimerReset();
        // =========================
        // Initial Setup
        // =========================
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 20;
        kickers.update();
        kickers.loadingUpdate();

        // =========================
        // PHASE 1: DRIVE TO DETECTION POSITION
        // =========================

        Actions.runBlocking (myRobot.motiffCheck());
        //Actions.runBlocking(myRobot.turnOnUpdate());

        if (turretSystem.motiff[0].equals("N")){
            Actions.runBlocking(myRobot.aimInit());
            Actions.runBlocking(drive.actionBuilder(startPose).strafeToLinearHeading(detectVector,detectorHeading).build());

            Actions.runBlocking (myRobot.reverseTransfer());

            Actions.runBlocking (myRobot.motiffUpdate());
        } else{
            Actions.runBlocking(myRobot.aimInit());
        }



        // Now detect motif at the detection position


        telemetry.addData("Motiff 0", turretSystem.motiff[0]);
        telemetry.addData("Motiff 1", turretSystem.motiff[1]);
        telemetry.addData("Motiff 2", turretSystem.motiff[2]);
        String frontColor = kickers.ballArray[0];
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
                    myRobot.loadGreenAction(), myRobot.afterLoadNTer(),
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer(),
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer()
            );
            alternateMotifSeq = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
            alternateMotifSeq2 = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoad(),
                    myRobot.loadGreenAction(), myRobot.afterLoad(),
                    myRobot.loadPurpleAction(), myRobot.lastAfterLoad()
            );
            alternateMotifSeq3 = new SequentialAction(
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer(),
                    myRobot.loadGreenAction(), myRobot.afterLoadNTer(),
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer()
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
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer(),
                    myRobot.loadGreenAction(), myRobot.afterLoadNTer(),
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer()
            );
            alternateMotifSeq=motiffSequence;
            alternateMotifSeq2=motiffSequence2;
            alternateMotifSeq3=motiffSequence3;
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
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer(),
                    myRobot.loadPurpleAction(), myRobot.afterLoadNTer(),
                    myRobot.loadGreenAction(), myRobot.afterLoadNTer()
            );

            alternateMotifSeq=motiffSequence;
            alternateMotifSeq2=motiffSequence2;
            alternateMotifSeq3=motiffSequence3;
        }

        // =========================
        // PHASE 2: Turn to shooting position and shoot preloads
        // =========================
        drive.updatePoseEstimate();


        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(myRobot.setColor());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(myRobot.shooterSpinUp(),
                        drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(shotingVector,shotingHeading).build(),
                        myRobot.turnOffUpdate())));
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(), myRobot.reverseTransfer(),new SequentialAction(myRobot.turnOnTrackingOnly(),   myRobot.reverseTransferStop(),myRobot.turnOffUpdate())));


        // Shoot preloads
        frontColor=kickers.ballArray[0];
        if (frontColor.equals("G")||frontColor.equals("U")){
            Actions.runBlocking(alternateMotifSeq);
        } else{
            Actions.runBlocking(motiffSequence);
        }


        drive.updatePoseEstimate();

        // =========================
        // PHASE 3: First intake cycle & open gate
        // =========================
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.turnOffTracking(),
                        myRobot.intake(1.0),
                        drive.actionBuilder(drive.localizer.getPose())
                                .splineToLinearHeading(firstStartIntake, 0).splineToLinearHeading(firstFinishIntake,0, new TranslationalVelConstraint(12)).strafeToLinearHeading(gatePath,gatePathHeading).splineToLinearHeading(gatePos,0,new TranslationalVelConstraint(50))
                                .build(),
                        myRobot.intake(0),

                        myRobot.turnOffUpdate()

                )
        ));

        drive.updatePoseEstimate();
        //open gate --------------------

 //       Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
   //             .strafeToLinearHeading(gateReady.position, gateReady.heading, new TranslationalVelConstraint(15)).strafeToLinearHeading(openGate.position,openGate.heading, new TranslationalVelConstraint(15))
     //           .build());
       // drive.updatePoseEstimate();
        // =========================
        // PHASE 4: Return to shooting position and shoot first cycle
        // =========================




        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                myRobot.reverseTransfer(),
                new SequentialAction(
                        myRobot.shooterSpinUp(),

                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(shotingVector, shotingHeading).build(),
                        myRobot.turnOnTracking(),

                        myRobot.reverseTransferStop(),

                        myRobot.turnOffUpdate()
                )
        ));

        drive.updatePoseEstimate();

        frontColor=kickers.ballArray[0];
        if (frontColor.equals("G")||frontColor.equals("U")){
            Actions.runBlocking(alternateMotifSeq2);
        } else{
            Actions.runBlocking(motiffSequence2);
        }



        // =========================
        // PHASE 5: Second intake cycle
        // =========================

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        myRobot.turnOffTracking(),
                        drive.actionBuilder(drive.localizer.getPose())
                                .splineToLinearHeading(secondStartIntake,0).build(),
                        myRobot.intake(1),
                        myRobot.turnOffUpdate())));



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
                        drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(secondShootingVector,secondShootingHeading).waitSeconds(0.3).build(),
                        myRobot.reverseTransferStop(),
                        myRobot.shooterSpinUp(),
                        myRobot.turnOnTracking(),

                        myRobot.turnOffUpdate() )));


        frontColor=kickers.ballArray[0];
        if (frontColor.equals("G")||frontColor.equals("U")){
            Actions.runBlocking(alternateMotifSeq3);
        } else{
            Actions.runBlocking(motiffSequence3);
        }

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
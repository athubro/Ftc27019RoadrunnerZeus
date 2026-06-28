package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ZeusSimpleIntakeZone", group = "Autonomous")
public class ZeusSimpleIntakeZone extends LinearOpMode {


    private MecanumDrive drive;
    private Intake intake;
    private RobotInfoStorage info;
    private SSMyRobot myRobot;

    // Positioning coordinates mapped from your original setup
    private Pose2d startPose = new Pose2d(-45.2766, -61.5312, Math.toRadians(-127.6875));
    private Pose2d nearSpikeStart = new Pose2d(14.44, -35.727, Math.toRadians(-81.94));
    private Pose2d nearSpikeEnd = new Pose2d(16.352, -59.976, Math.toRadians(-84.186));

    @Override
    public void runOpMode() {
        // Initialize all subsystems
        drive = new MecanumDrive(hardwareMap, startPose);
        info = new RobotInfoStorage();

        Turret turretSystem = new Turret(hardwareMap, drive, telemetry, startPose);
        turretSystem.resetTurretEncoder();

        intake = new Intake(hardwareMap, telemetry);
        myRobot = new SSMyRobot(hardwareMap, drive, intake, turretSystem, startPose);

        // Pre-match setups
        Actions.runBlocking(myRobot.setTurretAnlge(-12));

        waitForStart();

        // Reset variables at match start
        intake.generalTimerReset();
        turretSystem.update();
        intake.storageUpdate();
        RobotInfoStorage.autoEndPose = startPose;

        // ===================================================================================
        // MOVEMENT SECTION: Drive towards the near cluster of balls while spinning the intake
        // ===================================================================================
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(
                myRobot.updateRobot(),
                new SequentialAction(
                        drive.actionBuilder(drive.localizer.getPose())
                                .strafeToLinearHeading(nearSpikeStart.position, nearSpikeStart.heading)
                                .strafeToLinearHeading(nearSpikeEnd.position, nearSpikeEnd.heading, new TranslationalVelConstraint(70))
                                .build(),
                        // Activate intake at 50% power once coordinates are reached
                        myRobot.intakePower(0.5),
                        myRobot.turnOffUpdate()
                )
        ));

        // Update tracking variables to finish out the cycle cleanly
        drive.updatePoseEstimate();
        RobotInfoStorage.autoEndPose = drive.localizer.getPose();

        // ==========================================
        // Debugging Telemetry Keep-Alive Loop
        // ==========================================
        while (opModeIsActive()) {
            telemetry.addLine("=== SYSTEM STATUS ===");
            telemetry.addData("Robot Pose X", "%.2f", drive.localizer.getPose().position.x);
            telemetry.addData("Robot Pose Y", "%.2f", drive.localizer.getPose().position.y);
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            telemetry.update();
        }
    }
}
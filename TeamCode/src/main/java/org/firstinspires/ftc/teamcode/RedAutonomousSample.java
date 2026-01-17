package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Apollo 1 - Red", group = "Autonomous")
public class RedAutonomousSample extends LinearOpMode {

    private String[] pattern = {"P", "G", "P"}; // Default pattern

    @Override
    public void runOpMode() {
        // =======================================
        // INITIALIZATION
        // =======================================
        Pose2d startPose = new Pose2d(-44.97, 50.58, Math.toRadians(38.7));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        TurretCopy turret = new TurretCopy(hardwareMap, telemetry);
        StorageWLoaderCopy storage = new StorageWLoaderCopy(hardwareMap, turret);

        // Initialize the reusable actions class
        SSMyRobotCopy actions = new SSMyRobotCopy(drive, turret, storage, this);
        turret.PARAMS.TARGET_TAG_ID = 24;
        // Setup limelight
        turret.limelight.pipelineSwitch(0);
        turret.limelight.setPollRateHz(100);
        turret.limelight.start();

        turret.setShootingEnabled(true);
        turret.setTrackingMode(false);

        telemetry.addLine("AUTO READY");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // =======================================
        // AUTONOMOUS SEQUENCE
        // =======================================
        Actions.runBlocking(
                new SequentialAction(
                        // Move to motif detection position
                        drive.actionBuilder(startPose)
                                .splineToLinearHeading(
                                        new Pose2d(-24.45, 30, Math.toRadians(43.4)),
                                        Math.toRadians(30)
                                )
                                .build(),

                        // Detect which pattern to shoot based on AprilTags
                        actions.detectMotif(pattern),

                        // Turn to shooting heading (23.4°)
                        actions.moveToWithHeading(
                                new Vector2d(-24.45, 30),
                                Math.toRadians(-23.4),
                                Math.toRadians(-23.4)
                        ),

                        // Enable tracking and shoot the pattern
                        actions.turnOnTracking(0.0, 2.0),
                        actions.shootPattern(pattern, 5.0),
                        actions.turnOffTracking(),

                        // Move to first intake position
                        actions.moveToWithHeading(
                                new Vector2d(-10.3, 33.3),
                                Math.toRadians(63.4),
                                Math.toRadians(90)
                        ),

                        // Start intake
                        actions.startIntake(1.0),

                        // Move to second intake position
                        actions.moveToWithHeading(
                                new Vector2d(-4.95, 57.2),
                                Math.toRadians(91.2),
                                Math.toRadians(90)
                        ),

                        // Stop intake
                        actions.stopIntake(),



                        // shooting
                        actions.moveToWithHeading(
                                new Vector2d(-24.45, 30),
                                Math.toRadians(-23.4),
                                Math.toRadians(-23.4)
                        ),
                        actions.turnOnTracking(0.0, 2.0),
                        actions.shootPattern(pattern, 5.0),
                        actions.turnOffTracking(),
                        //First Second Intake
                        actions.moveToWithHeading(
                                new Vector2d(8.4, 35.4),
                                Math.toRadians(60.13),
                                Math.toRadians(60.13)
                        ),

                        // Start intake
                        actions.startIntake(1.0),

                        //Finish Second Intake


                        actions.moveToWithHeading(
                                new Vector2d(18, 58.216),
                                Math.toRadians(72.37),
                                Math.toRadians(72.37)
                        ),

                        // Stop intake
                        actions.stopIntake(),

                        // Park at starting position
                        actions.moveToWithHeading(
                                new Vector2d(-24.45, 30),
                                Math.toRadians(-23.4),
                                Math.toRadians(-23.4)
                        ),
                        actions.turnOnTracking(0.0, 2.0),
                        actions.shootPattern(pattern, 5.0),
                        actions.turnOffTracking()



                )
        );

        turret.setShootingEnabled(false);
        telemetry.addLine("AUTO FINISHED");
        telemetry.update();
    }
}
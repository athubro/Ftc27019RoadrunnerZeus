package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AutoToTeleopInfoTest", group = "TeleOp")
public class AutoToTeleopInfoTest extends LinearOpMode {

    public RobotInfoStorage info;
    public  MecanumDrive myDrive;
    private Turret turret;
    private Intake intake;
    private Pose2d initialPose = new Pose2d(0, 0, 0);
    private boolean usingOdomTracking = false;
    private boolean ableResetTime = true;
    private ElapsedTime sortTimer = new ElapsedTime();
    private Pose2d targetPose = new Pose2d(0, 0, 0);

    private Servo rgbIndicator;

    public String[] motiff = {"P", "P", "G"};


    private double manualTurretDegrees = 0;

    private double speedRatio = 0.75;

    @Override
    public void runOpMode() {

        // Initialize all systems
        info = new RobotInfoStorage();
        initialPose = info.autoEndPose;
        myDrive= new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap, myDrive ,telemetry, initialPose);
        intake = new Intake(hardwareMap, telemetry);

        rgbIndicator = hardwareMap.get(Servo.class, "rgbLight");

        // Configure turret
        turret.PARAMS.TARGET_TAG_ID = 20;
        turret.setAutoAngleEnabled(false);  // Start with manual angle control
        turret.setAutoRPMEnabled(false);    // Start with manual RPM control
        turret.setTrackingMode(false);      // Start with manual heading control

        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("Turret + Drive + Intake Initialized");
        telemetry.addLine("Target Tag ID: 20");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1: Drive Controls");
        telemetry.addLine("Gamepad 2: Turret & Intake Controls");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();
        boolean autoDrive = false;
        waitForStart();

        // Button state trackers for toggles
        boolean trackingToggleLast = false;
        boolean autoRPMToggleLast = false;
        boolean autoAngleToggleLast = false;
        boolean gateToggleLast = false;


        while (opModeIsActive()) {
            intake.storageUpdate();
            // =========================
            // GAMEPAD 1: DRIVE CONTROLS
            // =========================

            //double forward = -speedRatio * gamepad1.left_stick_y;
           /// double strafe = -speedRatio * gamepad1.left_stick_x;
           // double rotation = -speedRatio * gamepad1.right_stick_x;



            // =========================
            // UPDATE ALL SYSTEMS
            // =========================
                    //==========================================================================================
            if ((!gamepad1.aWasPressed()) ) {
                if (!autoDrive) {
                    Vector2d translation = new Vector2d((speedRatio * (-gamepad1.left_stick_y)), (speedRatio * (-gamepad1.left_stick_x)));
                    double rotation = -speedRatio * gamepad1.right_stick_x;
                    myDrive.setDrivePowers(new PoseVelocity2d(translation, rotation));
                } else {
                    if (Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01) {
                        autoDrive =false;
                    }
                }
            } else {
                //hmmmmmm
                //Pose2d curPose2D = drive.localizer.getPose();
                autoDrive = true;

                //   Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                //           .strafeToLinearHeading(new Vector2d(38, -33), 0).build());//38,-33
                // } else if (gamepad2.dpadLeftWasPressed()) {
                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose())
                        .strafeToLinearHeading(targetPose.position, targetPose.heading).build());
                // }
            }
            if (gamepad1.xWasPressed())  {
                autoDrive = false;
            }
            myDrive.updatePoseEstimate();
            if (gamepad1.startWasPressed()) {
                targetPose = myDrive.localizer.getPose();
            }
            if (gamepad1.bWasPressed()) {
                targetPose = new Pose2d(40, 32, 0 );
            }

            //==========================================================================================

            // Update turret with drive controls
            turret.update(0, 0, 0);

            /*
            turret.update(forward, strafe, rotation);
            if (gamepad1.dpadLeftWasPressed()) {
                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose())
                        .strafeToLinearHeading(targetPose.position, targetPose.heading).build());
            }
            drive.updatePoseEstimate();
            if (gamepad2.startWasPressed()) {
                closeShotPose = drive.localizer.getPose();
            }
             */

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("motiff 0 ",motiff[0]);
            telemetry.addData("motiff 1 ",motiff[1]);
            telemetry.addData("motiff 2 ",motiff[2]);
            telemetry.addData("slot 0 ",intake.storage[0]);
            telemetry.addData("slot 1 ",intake.storage[1]);
            telemetry.addData("slot 2 ",intake.storage[2]);
            telemetry.addData("x" ,myDrive.localizer.getPose().position.x);
            telemetry.addData("y ",myDrive.localizer.getPose().position.y);
            telemetry.addData("slot 2 ",myDrive.localizer.getPose().heading);



            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue TeleOp 2.0", group = "TeleOp")
public class SolarStorm27019TeleOPBlueAutoDrive extends LinearOpMode {
    boolean autoDrive = false;
    private Turret turretSystem;
    private MecanumDrive drive;
    //private Storage kickers;  // intake-free storage class
    private StorageWLoader kickers;
    private Pose2d pose = new Pose2d(0, 0, 0);
    private Pose2d closeShotPose = new Pose2d(0, 0, 0);
    private boolean waitingForConfig = true;
    private double speedRatio = 0.5;
    private double speedRatioTurning = 0.4;


// auto drive

    //Pose2d targetPose = new Pose2d(,,);
    boolean autoDriving = false;

    // Tune these!
    double kP_xy = 0.8;     // translational gain
    double kP_heading = 3.0;
    @Override
    public void runOpMode() {

        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry);
        turretSystem.autoSpinUp=true;

        //drive = new MecanumDrive(hardwareMap, pose);
        //drive.PARAMS.maxWheelVel=30;

        //kickers = new Storage(hardwareMap, turretSystem);  // intake-free kicker class
        kickers = new StorageWLoader(hardwareMap, turretSystem);
        turretSystem.PARAMS.TARGET_TAG_ID = 20;
       /* while (waitingForConfig){speedRatio
            telemetry.addLine("DO NOT hit START for NOW!!!");
            telemetry.addLine("Turret + Drive + Kickers Ready");
            telemetry.addLine("Press pad-1 X to increase the speed ratio by 0.05");
            telemetry.addLine("Press pad-1 Y to decrease the speed ratio by 0.05");
            telemetry.addLine("Press pad-1 A when finish!");
            telemetry.addData("Speed Ratio", speedRatio);
            telemetry.update();
            if (gamepad1.xWasReleased()) {+=0.05;}
            if (gamepad1.yWasReleased()) {speedRatio-=0.05;}
            if (gamepad1.aWasReleased()) {waitingForConfig=false;}
        }
        telemetry.addLine("Now you can start!");
        telemetry.update();

        */
        boolean flag = true;
        boolean poseSelected = false;
        turretSystem.turretManualPos(turretSystem.turretPos);

        telemetry.clear();
        telemetry.update();
        telemetry.addLine("=== STARTING POSE SELECTION \uD83E\uDEF5\uD83E\uDD23 ===");
        telemetry.addLine();
        telemetry.addLine("Select your starting position (default: 0, 0 ,0):");
        telemetry.addLine("• Press B for FarZone Auto V3 (9, -13, 0°)"); //(9,-13,0)
        telemetry.addLine("• Press A for NearZone Auto V2 (-35.4, -19, 50°)"); //(-35.4,-19,Math.toRadians(50))
        telemetry.addLine("• Press Y for Pos of FarZone V2 (40, -19, 0°)");
        telemetry.addLine("• Press D-Pad Right for Atharva Auto Pose");
        telemetry.addLine("  (4, -35.4, 23.4°)");
        telemetry.addLine();
        telemetry.addLine("After selecting pose, press X to confirm 😊");
        telemetry.addLine("and initialize drive system.");
        telemetry.update();
        while (flag) {
            // Clear telemetry each loop
            //telemetry.clear();

            // Show instructions only if pose hasn't been selected yet

            // Handle pose selection
            if (gamepad1.bWasPressed()) {
                telemetry.clear();
                telemetry.update();
                pose = new Pose2d(9,-13,0);
                poseSelected = true;
                //telemetry.clear();
                telemetry.addLine("Selected Pose FarZone Auto V3");
                telemetry.addLine("Press X to confirm");
                telemetry.update();
            } else if (gamepad1.aWasPressed()) {
                telemetry.clear();
                telemetry.update();
                pose = new Pose2d(-35.4,-19,Math.toRadians(50));
                poseSelected = true;
                //telemetry.clear();
                telemetry.addLine("Selected Pose NearZone Auto V2");
                telemetry.addLine("Press X to confirm");
                telemetry.update();
            } else if (gamepad1.yWasPressed()) {
                telemetry.clear();
                telemetry.update();
                pose = new Pose2d(40,-19,0);
                poseSelected = true;
                //telemetry.clear();
                telemetry.addLine("Selected Pose FarZone V2 (40, -19, 0°)");
                telemetry.addLine("Press X to confirm");
                telemetry.update();
            } else if (gamepad1.dpadRightWasPressed()) {
                telemetry.clear();
                telemetry.update();
                pose = new Pose2d(4,-35.4, Math.toRadians(23.4));
                poseSelected = true;
                //telemetry.clear();
                telemetry.addLine("Selected Pose Atharva Auto Pose (4, -35.4, 23.4°) 😊");
                telemetry.addLine("Press X to confirm");
                telemetry.update();
            }

            telemetry.update();

            if (gamepad1.x) {
                flag = false;
            }
        }

        drive = new MecanumDrive(hardwareMap, pose);

        telemetry.clear();
        telemetry.addLine("Now you can start!");
        telemetry.update();
        waitForStart();

        // Button state trackers for toggles
        boolean aPressedLast = false;
        boolean dpadUpLast = false;
        boolean dpadDownLast = false;

        while (opModeIsActive()) {
            //turretSystem.trackingMode = true;
            // =========================
            // Turret & Shooter Controls (Gamepad1)
            // =========================
            // turretSystem.setShootingEnabled(gamepad1.right_trigger > 0.1);

            if (gamepad1.dpad_up && !dpadUpLast)
                turretSystem.setTargetRPM(turretSystem.getTargetRPM() + 50.0);
            if (gamepad1.dpad_down && !dpadDownLast)
                turretSystem.setTargetRPM(Math.max(0, turretSystem.getTargetRPM() - 50.0));

            dpadUpLast = gamepad1.dpad_up;
            dpadDownLast = gamepad1.dpad_down;

            if (gamepad1.aWasPressed())
                turretSystem.setTrackingMode(true);
            if (gamepad1.bWasPressed())
                turretSystem.setTrackingMode(false);
            //aPressedLast = gamepad1.a;
            if (!turretSystem.trackingMode){
                turretSystem.turretPos-=gamepad1.right_stick_x*0.05;
                //turretSystem.turretManualPos(turretSystem.turretPos);
            }
            //turretSystem.setManualTurretPower(gamepad1.right_stick_x);

            if (gamepad1.dpad_right)
                turretSystem.setTurretAngleCommand(1);
            else if (gamepad1.dpad_left)
                turretSystem.setTurretAngleCommand(-1);
            else
                turretSystem.setTurretAngleCommand(0);



            // =========================
            // Mecanum Drive Controls (Gamepad2)
            // =========================
            if ((!gamepad2.dpadLeftWasPressed()) ) {
                if (!autoDrive) {
                    Vector2d translation = new Vector2d((speedRatio * (-gamepad2.left_stick_y)), (speedRatio * (-gamepad2.left_stick_x)));
                    double rotation = -speedRatioTurning * gamepad2.right_stick_x;
                    drive.setDrivePowers(new PoseVelocity2d(translation, rotation));
                } else {
                    if (Math.abs(gamepad2.left_stick_y) > 0.01 || Math.abs(gamepad2.left_stick_x) > 0.01 || Math.abs(gamepad2.right_stick_x) > 0.01) {
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
                Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(closeShotPose.position, closeShotPose.heading).build());
                // }
            }
            if (gamepad2.xWasPressed())  {
                autoDrive = false;
            }
            drive.updatePoseEstimate();
            if (gamepad2.startWasPressed()) {
                closeShotPose = drive.localizer.getPose();
            }
            if (gamepad2.backWasPressed()) {
                closeShotPose = new Pose2d(40, 32, 0 );
            }

            turretSystem.update();
            // =========================
            // Kicker / Storage Updates
            // =========================
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            //
            //
            //
            shotDetectReset();
            if(gamepad2.right_trigger > 0.1) kickers.setIntakePower(gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.1)kickers.setIntakePower(-gamepad2.left_trigger);
            else kickers.setIntakePower(0);
            if ((!(gamepad2.right_trigger >0.1)) && (!(gamepad2.left_trigger >0.1))) {
                kickers.transferPower(gamepad1.left_stick_y);
            }

            if (gamepad2.right_bumper) kickers.openGate();
            if (gamepad2.left_bumper) kickers.closeGate();
            // Automatic loading into nearest empty slot
            if (gamepad1.x) kickers.loadGreen();
            if (gamepad1.y) kickers.loadPurple();
            if (gamepad2.b) kickers.resetKick();
            //if (gamepad1.bWasPressed()) turretSystem.aimingMode = true;
            if (gamepad1.left_bumper) kickers.loadOne();

            if (gamepad1.right_trigger>0.2) turretSystem.setShootingEnabled(true);
            else turretSystem.setShootingEnabled(false);




            // Manual kicking using D-pad
            if (gamepad2.dpadUpWasPressed()) kickers.setBacUp();
            if (gamepad2.dpadRightWasPressed()) kickers.setMiddleUp();
            if (gamepad2.dpadDownWasPressed()) kickers.setFrontUp();

            // Optional telemetry for debugging
            telemetry.addData("timer", kickers.generalTimer);
            telemetry.addData("change flag trigger value", kickers.changeFlagTrigger);
            telemetry.addData("flag", kickers.flag);
            //telemetry.addData("RPM", turretSystem.getTargetRPM());


            //telemetry.addData("front dis", kickers.dis);
            //telemetry.addData("front pos", kickers.frontPos);
            //telemetry.addData("middle pos", kickers.middlePos);
            //telemetry.addData("back pos", kickers.backPos);
            telemetry.addData("Kick Target", kickers.kickTarget);

            telemetry.addData("Ball Count", kickers.count);
            telemetry.addData("Front Slot", kickers.ballArray[0]);
            //telemetry.addData("Front reading Red", kickers.redReading1);
            //telemetry.addData("Front reading Green", kickers.greenReading1);
            //telemetry.addData("Front reading blue", kickers.blueReading1);
            telemetry.addData("Middle Slot", kickers.ballArray[1]);
            //telemetry.addData("Middle reading Red", kickers.redReading2);
            //telemetry.addData("Middle reading Green", kickers.greenReading2);
            //telemetry.addData("Middle reading blue", kickers.blueReading2);
            telemetry.addData("Back Slot", kickers.ballArray[2]);
            //telemetry.addData("Back reading Red", kickers.redReading3);
            //telemetry.addData("Back reading Green", kickers.greenReading3);
            //telemetry.addData("Back reading blue", kickers.blueReading3);

            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            //telemetry.addData("Left Motor power", "%.2f", turretSystem.leftMotor.getPower());
            //telemetry.addData("Right Motor power", "%.2f", turretSystem.rightMotor.getPower());
            //telemetry.addData("Left ticks", turretSystem.leftMotor.getCurrentPosition());
            //telemetry.addData("Right ticks", turretSystem.rightMotor.getCurrentPosition());
            telemetry.addData("Target RPM", "%.1f", turretSystem.targetRPM);
            // telemetry.addData("Left RPM Derivative",  "%.1f", turretSystem.leftDerivative);
            //telemetry.addData("Right RPM Derivative",  "%.1f", turretSystem.rightDerivative);
            telemetry.addData("shotdetected", turretSystem.shotDetected);
            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", turretSystem.trackingMode ? "ON" : "OFF");
            telemetry.addData("Tag Visible", turretSystem.telemetryData.tagFound);
            telemetry.addData("Error Angle (deg)", "%.2f", turretSystem.telemetryData.errorAngleDeg);
            //telemetry.addData("Turret Power", "%.3f", turretSystem.telemetryData.turretPower);
            telemetry.addData("calculated distance in inches ", turretSystem.disToAprilTag);
            telemetry.addData("measured angle of april tag ", turretSystem.ATAngle);
            telemetry.addLine("=== TURRET ANGLE ===");
            telemetry.addData("Turret Angle Position", "%.3f", turretSystem.telemetryData.turretAnglePower);
            telemetry.addData("Turret aiming position",turretSystem.turretPos);

            telemetry.update();
        }
    }

    public void shotDetectReset() {
        if (kickers.kickUp && turretSystem.shotDetected) {
            kickers.resetKick();
            turretSystem.shotDetected = false;
        }
    }
    /*
    public void rapidFire(){
        int count = kickers.ballCount();
        kickers.update();
        for (int i = 0; i < 3; i++) {
            if (kickers.ballArray[i] == "P") {
                kickers.loadPurple();
                while (!kickers.flag && turretSystem.shotDetected) {
                    kickers.update();
                    turretSystem.update();

                }
                shotDetectReset();
                sleep(400);

            } else if (kickers.ballArray[i] == "G") {
                kickers.loadGreen();
                kickers.loadPurple();
                while (!kickers.flag && turretSystem.shotDetected) {
                    kickers.update();
                    turretSystem.update();

                }
                shotDetectReset();
                sleep(400);

            } else {

            }
        }
    }
    */


}
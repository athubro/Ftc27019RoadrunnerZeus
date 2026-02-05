package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ZeusTeleOPBlue v1", group = "TeleOp")
public class ZeusTeleOPBlue extends LinearOpMode {
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

            // Speed control
            if (gamepad1.right_bumper) {
                speedRatio = 1.0;  // Full speed
            } else if (gamepad1.left_bumper) {
                speedRatio = 0.3;  // Slow speed
            } else {
                speedRatio = 0.75; // Normal speed
            }

            // =========================
            // GAMEPAD 2: TURRET CONTROLS
            // =========================

            // Toggle tracking mode (A button)
             /*
            if (gamepad2.aWasPressed()) {
                turret.setTrackingMode(!turret.trackingMode);
                turret.setAutoAngleEnabled(!turret.autoAngleEnabled);
                turret.setAutoRPMEnabled(!turret.autoRPMEnabled);

            }
*/


            // Toggle FULL AUTO MODE (Vision + RPM + Angle)
            if (gamepad2.bWasPressed()) {
                boolean newState = !turret.trackingMode;
                turret.setTrackingMode(newState);
                turret.setAutoAngleEnabled(newState);
                turret.setAutoRPMEnabled(newState);
            }

            // Toggle Vision vs Odom tracking
            if (gamepad2.aWasPressed()) {
                usingOdomTracking = !usingOdomTracking;
                turret.setUseOdometryTracking(usingOdomTracking);
            }



            if (turret.flywheelUpToSpeed) {
                rgbIndicator.setPosition(0.62);
            } else {
                rgbIndicator.setPosition(0.333);
            }
            // Storage/Sorting controls - D-pad
            if (gamepad1.dpad_up) {
                motiff[0] = "G";
                motiff[1] = "P";
                motiff[2] = "P";
                rgbIndicator.setPosition(0.5);
                intake.storeBalls(motiff);
            }
            if (gamepad1.dpad_down) {
                motiff[0] = "P";
                motiff[1] = "P";
                motiff[2] = "G";
                rgbIndicator.setPosition(0.722);
                intake.storeBalls(motiff);
            }
            if (gamepad1.dpad_left) {

                motiff[0] = "P";
                motiff[1] = "G";
                motiff[2] = "P";
                rgbIndicator.setPosition(0.722);
                intake.storeBalls(motiff);
            }
            if ((!intake.firstStep.equals("N")) || (!intake.secondStep.equals("N"))) {
                // Reset all compartments to pass-through
                if (intake.ballCount == 0 && gamepad2.left_trigger > 0.1 && sortTimer.seconds() > 1) {
                    intake.executeNextStep();
                }
            }


            // Enable/disable shooting with triggers
            if (gamepad2.right_trigger > 0.2) {
                turret.setShootingEnabled(true);
            } else {
                turret.setShootingEnabled(false);
            }

            // Manual RPM adjustment (D-pad up/down)
            if (gamepad2.dpadUpWasPressed()) {
                turret.setTargetRPM(turret.getTargetRPM() + 50.0);
            }
            if (gamepad2.dpadDownWasPressed()) {
                turret.setTargetRPM(Math.max(0, turret.getTargetRPM() - 50.0));
            }


            // Manual turret angle control (D-pad left/right)
            if (gamepad2.dpadRightWasPressed()) {
                turret.setTurretAngleCommand(1);
                turret.updateTurretAngle();
            } else if (gamepad2.dpadLeftWasPressed()) {
                turret.setTurretAngleCommand(-1);
                turret.updateTurretAngle();
            } else {
                turret.setTurretAngleCommand(0);
                turret.updateTurretAngle();
            }


            if (!turret.trackingMode) {
                manualTurretDegrees+= -gamepad2.right_stick_x*2;
                manualTurretDegrees = turret.clamper(manualTurretDegrees, turret.PARAMS.TURRET_MIN_DEG, turret.PARAMS.TURRET_MAX_DEG);
                turret.manualTurretAngle(manualTurretDegrees);
            } else {
                if (turret.tagFound&&!usingOdomTracking) {
                    turret.setShootingEnabled(true); //= true;
                }
            }

            // =========================
            // GAMEPAD 2: INTAKE CONTROLS
            // =========================

            // Intake motor control with left trigger (intake) and left stick Y (outtake)
            if (gamepad1.right_trigger > 0.1) {
                intake.setIntakePower(gamepad1.right_trigger);  // Intake
                intake.closeGate();
            } else if (Math.abs(gamepad1.left_trigger) > 0.1) {
                intake.setIntakePower(-gamepad1.left_trigger);  // Manual control
            } else {
                intake.setIntakePower(0);  // Stop
            }
            if (gamepad2.left_trigger > 0.1) {
                intake.setIntakePower(gamepad2.left_trigger);  // Intake
                if (ableResetTime) {
                    sortTimer.reset();
                    ableResetTime = false;
                }
                intake.openGate();
            } else {
                ableResetTime = true;
            }

            // Gate control
            if (gamepad2.rightBumperWasPressed()) {
                intake.openGate();
            }
            if (gamepad2.leftBumperWasPressed()) {
                intake.closeGate();
            }

            // Toggle gate (B button)
            if (gamepad2.bWasPressed()) {
                intake.toggleGate();
            }


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

            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Speed Mode", speedRatio == 1.0 ? "FAST" : (speedRatio == 0.3 ? "SLOW" : "NORMAL"));
            telemetry.addData("Position", "X: %.1f  Y: %.1f  H: %.1f°",
                    turret.getPose().position.x,
                    turret.getPose().position.y,
                    Math.toDegrees(turret.getPose().heading.toDouble()));

            telemetry.addLine();
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.trackingMode ? "AUTO" : "MANUAL");
            telemetry.addData("Tracking Source", usingOdomTracking ? "ODOMETRY (-60, -60)" : "LIMELIGHT");
            telemetry.addData("ShooterAngle", turret.turretAngle.getPosition());
            telemetry.addData("Aligned", turret.isAligned() ? "YES" : "NO");
            telemetry.addData("Auto RPM", turret.autoRPMEnabled ? "ON" : "OFF");
            telemetry.addData("Auto Angle", turret.autoAngleEnabled ? "ON" : "OFF");
            telemetry.addData("Tag Found", turret.isTagFound() ? "YES" : "NO");
            telemetry.addData("Distance", "%.1f in", turret.getDistanceToTarget());
            telemetry.addData("Tracking Error", "%.1f°", turret.getTrackingError());
            telemetry.addData("Turret Yaw Power", "%.3f", turret.turretMotor.getPower());
            telemetry.addData("Turret Yaw Deg", "%.1f°", turret.turretMotor.getCurrentPosition() / turret.PARAMS.TICKS_PER_BIG_GEAR_DEGREE);

            telemetry.addLine();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Shooting", turret.shootingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("limelight TY", turret.ATAngle);
            telemetry.addData("Left RPM", "%.0f", turret.getCurrentRPMLeft());
            telemetry.addData("Right RPM", "%.0f", turret.getCurrentRPMRight());
            telemetry.addData("Target RPM", "%.0f", turret.getTargetRPM());
            telemetry.addData("shooterAngle", "%.0f", turret.turretAnglePos);

            telemetry.addData("Up to Speed", turret.isUpToSpeed() ? "READY" : "SPINNING UP");

            telemetry.addLine();
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Power", "%.2f", intake.getIntakePower());
            telemetry.addData("Gate", intake.isGateOpen() ? "OPEN" : "CLOSED");
            telemetry.addData("Turret KD", turret.PARAMS.turretKD);
            telemetry.addData("Turret KF", turret.PARAMS.turretKF);
            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("GP1: Drive (L-stick), Intake (A/B/X/LT)");
            telemetry.addLine("GP1: RB=Fast, LB=Slow");
            telemetry.addLine("GP2-B: Toggle FULL AUTO (Vision + RPM + Angle)");
            telemetry.addLine("GP2-A: Toggle Vision vs Odom Tracking");
            telemetry.addLine("GP2 Right Stick X: MANUAL TURRET YAW (when tracking OFF)");
            telemetry.addLine("GP2-RT: Shooting");
            telemetry.addLine("GP2-Dpad: RPM (up/down) / Angle (left/right)");
            telemetry.addLine("GP2-B: Toggle Gate");

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ZeusTeleOPRed v2", group = "TeleOp")
public class ZeusTeleOPRedV2 extends LinearOpMode {

    public RobotInfoStorage info;
    public  MecanumDrive myDrive;
    private Turret turret;
    private Intake intake;
    private Pose2d initialPose = new Pose2d(0, 0, 0);
    private boolean usingOdomTracking = false;
    private boolean ableResetTime = true;
    private ElapsedTime sortTimer = new ElapsedTime();
    private ElapsedTime autoShootingTimer = new ElapsedTime();
    private Pose2d targetPose = new Pose2d(0, 0, 0);

    private Pose2d gatePos = new Pose2d(7, 41, Math.toRadians(115)); //Pose2d(9.6, 43.5, Math.toRadians(115))
    private Pose2d gateOpenPos = new Pose2d(4.5, 47, Math.toRadians(112.7));//Pose2d(14.0, -67.1, Math.toRadians(-107.3));
    private Pose2d colletingPos = new Pose2d(12, 47, Math.toRadians(132));
    private Pose2d shootingPos = new Pose2d(-14.9, 1, Math.toRadians(135)); // Pose2d(-23.4, 6.9, Math.toRadians(135))
    private Pose2d loadingZone = new Pose2d(39.5, -36.4, Math.toRadians(-1.5));
    private Pose2d gateResetPos= new Pose2d(-13.1,47.6, Math.toRadians(87.9));

    private Servo rgbIndicator;

    public String[] motiff = {"P", "P", "G"};

    private boolean keepIntake = false;
    private boolean keepFlyingWheelOn = false;

    private boolean autoShootFlag=false;
    private double manualTurretDegrees = 0;

    private double speedRatio = 0.75;

    private boolean holdingBall=false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all systems
        info = new RobotInfoStorage();
        // initialPose = info.autoEndPose;
        initialPose = RobotInfoStorage.autoEndPose;;
        myDrive= new MecanumDrive(hardwareMap, initialPose);
        turret = new Turret(hardwareMap, myDrive ,telemetry, initialPose);
        intake = new Intake(hardwareMap, telemetry);
        // turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        rgbIndicator = hardwareMap.get(Servo.class, "rgbLight");

        // Configure turret
        turret.PARAMS.TARGET_TAG_ID = 20;
        turret.setAutoAngleEnabled(false);  // Start with manual angle control
        turret.setAutoRPMEnabled(false);    // Start with manual RPM control
        turret.setTrackingMode(false);      // Start with manual heading control
        turret.actionFlagForTurning=false;
        //turret.fineAdjustmentFlag=false;
        turret.teleOpOnly=true;
        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("Turret + Drive + Intake Initialized");
        telemetry.addLine("Target Tag ID: 20");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1: Drive Controls");
        telemetry.addLine("Gamepad 2: Turret & Intake Controls");
        telemetry.addData("Turret Position", turret.turretMotor.getCurrentPosition());
        telemetry.addLine("Press START to begin");
        telemetry.update();
        boolean autoDrive = false;

        //sleep (5000);
        turret.resetTurretEncoder();
        telemetry.addData("Turret Position", turret.turretMotor.getCurrentPosition());
        // sleep(3000);
        turret.updateTurretPID();
        turret.updateTurretVelocity(400);
        turret.continuousTracking=true;

        turret.PARAMS.TARGET_TAG_ID = 24;
        turret.LLFarZoneOffset = -2;

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
            if (myDrive.localizer.getPose().position.x> 30){
                turret.updateTurretVelocity(700);
                turret.continuousTracking=false;
            } else {
                turret.updateTurretVelocity(400);
                turret.continuousTracking=true;
            }
            // Speed control
            if (gamepad1.right_bumper) {
                speedRatio = 1.0;  // Full speed
            } else if (gamepad1.left_bumper) {
                speedRatio = 0.3;  // Slow speed
            } else {
                speedRatio = 0.75; // Normal speed
            }
            if (gamepad1.backWasPressed()){
                myDrive.localizer.setPose(gateResetPos);
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
                boolean tempState = !turret.trackingMode;
                turret.setTrackingMode(tempState);
                turret.setAutoAngleEnabled(tempState);
                turret.setAutoRPMEnabled(tempState);
                usingOdomTracking = true;
                //usingOdomTracking = !usingOdomTracking;
                turret.setUseOdometryTracking(true);
                // if (turret.trackingMode) {
                //     turret.updateTurretAiming();
                //}
            }

            // Toggle Vision vs Odom tracking
            if (gamepad2.aWasPressed()) {
                boolean tempState ;
                if (usingOdomTracking) {
                    tempState= true;
                } else{
                    tempState = !turret.trackingMode;
                }

                turret.setTrackingMode(tempState);
                turret.setAutoAngleEnabled(tempState);
                turret.setAutoRPMEnabled(tempState);
                usingOdomTracking = false;
                //usingOdomTracking = !usingOdomTracking;
                turret.setUseOdometryTracking(false);
                // if (turret.trackingMode) {
                //     turret.updateTurretAiming();
                // }

                autoShootFlag=true;
                autoShootingTimer.reset();
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

            if (gamepad1.dpad_right) {

                motiff[0] = "P";
                motiff[1] = "G";
                motiff[2] = "P";
                //rgbIndicator.setPosition(0.722);
                intake.resetAll();
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
                keepFlyingWheelOn=false;
            } else {
                if (!keepFlyingWheelOn) {
                    turret.setShootingEnabled(false);
                }

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
                manualTurretDegrees+= -gamepad2.right_stick_x*5;
                manualTurretDegrees = turret.clamper(manualTurretDegrees, turret.PARAMS.TURRET_MIN_DEG, turret.PARAMS.TURRET_MAX_DEG);
                turret.manualTurretAngle(manualTurretDegrees);
            } else {
                manualTurretDegrees=turret.turretMotor.getCurrentPosition()*1.0/turret.PARAMS.TICKS_PER_BIG_GEAR_DEGREE;

                if (turret.tagFound&&!usingOdomTracking) {
                    //temporary turn off shootngenable at hotel!
                    turret.setShootingEnabled(true); //= true;
                }
            }

            // =========================
            // GAMEPAD 2: INTAKE CONTROLS
            // =========================

            // Intake motor control with left trigger (intake) and left stick Y (outtake)
            if (gamepad1.right_trigger > 0.1) {
                keepIntake=false;
                holdingBall=false;
                intake.setIntakePower(gamepad1.right_trigger);  // Intake
                intake.closeGate();
                if (intake.ballCount == 3) {
                    rgbIndicator.setPosition(0.47);
                } else {
                    rgbIndicator.setPosition(0.36);
                }
            } else if (Math.abs(gamepad1.left_trigger) > 0.1) {
                keepIntake=false;
                holdingBall=false;
                intake.setIntakePower(-gamepad1.left_trigger);  // Manual control
            } else {
                if (!keepIntake && !holdingBall){
                    intake.setIntakePower(0);  // Stop
                } else if (holdingBall){
                    intake.setIntakePower(0.3);
                }
            }
            if (gamepad2.left_trigger > 0.1) {
                keepIntake=false;
                intake.setIntakePower(gamepad2.left_trigger);  // Intake
                if (ableResetTime) {
                    sortTimer.reset();
                    ableResetTime = false;
                }
                intake.openGate();
            } else {
                ableResetTime = true;
            }
            if (autoShootFlag){
                if (turret.tagFound && isInShootingZone(myDrive.localizer.getPose()) && turret.flywheelUpToSpeed){
                    keepIntake=true;
                    holdingBall=false;
                    intake.setIntakePower(1);
                    intake.openGate();
                }

                if (autoShootingTimer.seconds()>4) {
                    if (isInShootingZone(myDrive.localizer.getPose())){
                        autoShootFlag=false;
                        holdingBall=false;
                        intake.setIntakePower(0);
                        intake.closeGate();
                        turret.setTrackingMode(false);
                    } else{
                        autoShootFlag=false;
                    }

                }
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
            if (gamepad1.rightBumperWasPressed()) {
                autoDrive = true;
                keepIntake=false;
                holdingBall=true;
                //turret.setShootingEnabled(true);

                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose())
                        .strafeToLinearHeading(shootingPos.position, shootingPos.heading).build());



                turret.setTrackingMode(true);
                turret.setAutoAngleEnabled(true);
                turret.setAutoRPMEnabled(true);
                usingOdomTracking = false;
                //usingOdomTracking = !usingOdomTracking;
                turret.setUseOdometryTracking(false);
                autoShootFlag=true;
                autoShootingTimer.reset();
                //usingOdomTracking = !usingOdomTracking;

            }else if (gamepad1.leftBumperWasPressed()) {

                autoDrive = true;

                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose())
                        .strafeToLinearHeading(gatePos.position, gatePos.heading).build()); //.strafeToLinearHeading(colletingPos.position, colletingPos.heading)
                intake.closeGate();
                intake.setIntakePower(1);

                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose()).strafeToLinearHeading(gateOpenPos.position, gateOpenPos.heading).strafeToLinearHeading(colletingPos.position, colletingPos.heading).build());
                keepIntake=true;
            } else if (gamepad1.yWasPressed()){
                autoDrive = true;

                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose())
                        .strafeToLinearHeading(loadingZone.position, loadingZone.heading).build());


            } else if (gamepad1.aWasPressed()){
                //hmmmmmm
                //Pose2d curPose2D = drive.localizer.getPose();
                autoDrive = true;

                //   Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                //           .strafeToLinearHeading(new Vector2d(38, -33), 0).build());//38,-33
                // } else if (gamepad2.dpadLeftWasPressed()) {
                Actions.runBlocking(myDrive.actionBuilder(myDrive.localizer.getPose())
                        .strafeToLinearHeading(targetPose.position, targetPose.heading).build());
                // }
            } else  {
                if (!autoDrive) {
                    Vector2d translation = new Vector2d((speedRatio * (-gamepad1.left_stick_y)), (speedRatio * (-gamepad1.left_stick_x)));
                    double rotation = -0.6 * gamepad1.right_stick_x;
                    myDrive.setDrivePowers(new PoseVelocity2d(translation, rotation));
                } else {
                    if (Math.abs(gamepad1.left_stick_y) > 0.01 || Math.abs(gamepad1.left_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_x) > 0.01) {
                        autoDrive = false;
                    }
                }
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
            turret.update();

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
            telemetry.addData("error Deg",turret.errorDeg);
            telemetry.addData("aligned? ",turret.hasAligned);
            telemetry.addData("desired Deg", turret.TurretDesiredDeg);
            telemetry.addData("previous desired deg", turret.previousDesiredDeg);
            telemetry.addData("Turret Yaw Power", "%.3f", turret.turretMotor.getPower());
            telemetry.addData("Turret Yaw Deg", "%.1f°", turret.turretMotor.getCurrentPosition() / turret.PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
            telemetry.addData("actionFlagForTurning", turret.actionFlagForTurning);
            telemetry.addData("FineAdjustmentFlag ", turret.fineAdjustmentFlag);
            telemetry.addData("teleOpOnly", turret.teleOpOnly);
            telemetry.addLine();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Shooting", turret.shootingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("limelight TY", turret.ATAngle);
            telemetry.addData("Left RPM", "%.0f", turret.getCurrentRPMLeft());
            telemetry.addData("Right RPM", "%.0f", turret.getCurrentRPMRight());
            telemetry.addData("Target RPM", "%.0f", turret.getTargetRPM());
            telemetry.addData("shooterAngle", "%.0f", turret.turretAnglePos);

            telemetry.addData("Up to Speed", turret.isUpToSpeed() ? "READY" : "SPINNING UP");
            //telemetry.addData("adjustmentFlag", turret.adjustmentFlag);

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

    public static boolean isInShootingZone (Pose2d position){
        double line1A=-1;
        double line1B=7;
        double line2A=1;
        double line2B=21;
        if ( (position.position.x<position.position.y*line1A+line1B) && (position.position.x<position.position.y*line2A+line2B)){
            return true;
        } else {
            return false;
        }
    }
}
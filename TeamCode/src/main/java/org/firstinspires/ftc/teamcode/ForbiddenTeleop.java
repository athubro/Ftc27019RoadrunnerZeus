package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Forbidden TeleOp 1.0", group = "TeleOp")
public class ForbiddenTeleop extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    //private Storage kickers;  // intake-free storage class
    private StorageWLoader kickers;
    private Pose2d pose = new Pose2d(0, 0, 0);
    private boolean waitingForConfig = true;
    private double speedRatio = 0.5;
    @Override
    public void runOpMode() {

        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry);
        turretSystem.autoSpinUp=true;

        drive = new MecanumDrive(hardwareMap, pose);
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
        if (gamepad1.bWasPressed()) {
            pose = new Pose2d(10, -13, 0);
        } else if (gamepad1.aWasPressed()) {
            pose = new Pose2d(0, 0, 0);
        }
        waitForStart();

        // Button state trackers for toggles
        boolean aPressedLast = false;
        boolean dpadUpLast = false;
        boolean dpadDownLast = false;

        while (opModeIsActive()) {

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

            if (gamepad1.a && !aPressedLast)
                turretSystem.setTrackingMode(!turretSystem.trackingMode);

            aPressedLast = gamepad1.a;

          //  turretSystem.setManualTurretPower(gamepad1.right_stick_x);

            if (gamepad1.dpad_right)
                turretSystem.setTurretAngleCommand(1);
            else if (gamepad1.dpad_left)
                turretSystem.setTurretAngleCommand(-1);
            else
                turretSystem.setTurretAngleCommand(0);



            // =========================
            // Mecanum Drive Controls (Gamepad2)
            // =========================
            Vector2d translation = new Vector2d(( speedRatio*(-gamepad1.left_stick_y)), ( speedRatio*(-gamepad1.left_stick_x)));
            double rotation = -speedRatio*gamepad1.right_stick_x;
            drive.setDrivePowers(new PoseVelocity2d(translation, rotation));
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
            if(gamepad1.right_trigger > 0.1) kickers.setIntakePower(gamepad1.right_trigger);
            else if (gamepad1.left_trigger > 0.1)kickers.setIntakePower(-gamepad1.left_trigger);
            else kickers.setIntakePower(0);
            if ((!(gamepad2.right_trigger >0.1)) && (!(gamepad2.left_trigger >0.1))) {
                kickers.transferPower(gamepad1.left_stick_y);
            }

            if (gamepad1.right_bumper) turretSystem.setShootingEnabled(true);
            if (gamepad1.left_bumper) turretSystem.setShootingEnabled(false);
            // Automatic loading into nearest empty slot
            if (gamepad1.x) kickers.loadGreen();
            if (gamepad1.y) kickers.loadPurple();
            if (gamepad1.dpad_left) kickers.resetKick();
            if (gamepad1.bWasPressed()) turretSystem.aimingMode = true; turretSystem.turretSetPos();
            if (gamepad1.left_bumper) kickers.loadAll();
//
          //  if (gamepad1.right_trigger>0.2) turretSystem.setShootingEnabled(true);
          //   else turretSystem.setShootingEnabled(false);


           

            // Manual kicking using D-pad
            if (gamepad2.dpad_up) kickers.kickBack();
            if (gamepad2.dpad_left) kickers.kickMiddle();
            if (gamepad2.dpad_down) kickers.kickFront();

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
            telemetry.addData("Turret Aiming Position", "%.3f", turretSystem.telemetryData.turretAimPos);
            telemetry.addData("calculated distance in inches ", turretSystem.disToAprilTag);
            telemetry.addData("measured angle of april tag ", turretSystem.ATAngle);
            telemetry.addLine("=== TURRET ANGLE ===");
            telemetry.addData("Turret Angle Position", "%.3f", turretSystem.telemetryData.turretAnglePower);

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

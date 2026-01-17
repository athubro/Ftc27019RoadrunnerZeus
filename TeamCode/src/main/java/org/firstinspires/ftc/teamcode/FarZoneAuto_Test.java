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

@Autonomous(name = "FarZoneAutoTest", group = "Autonomous")
public class FarZoneAuto_Test extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    //private Storage kickers;  // intake-free storage class
    private StorageWLoader kickers;
    private Pose2d startPose = new Pose2d(62.7, -18.7, 0); //36.84, -14.96 0; delta x should be 25.9
    private boolean waitingForConfig = true;
    private double speedRatio = 0.4;

    private SSMyRobot myRobot;
    @Override
    public void runOpMode() {
        Action motiffSequence;
        Action motiffSequence2;
        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, startPose);
        //drive.PARAMS.maxWheelVel=30;

        //kickers = new Storage(hardwareMap, turretSystem);  // intake-free kicker class
        kickers = new StorageWLoader(hardwareMap, turretSystem);

        myRobot= new SSMyRobot(hardwareMap,drive,kickers,turretSystem,startPose);
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
        waitForStart();

        // Button state trackers for toggles
        boolean aPressedLast = false;
        boolean dpadUpLast = false;
        boolean dpadDownLast = false;
        turretSystem.update();
        turretSystem.PARAMS.TARGET_TAG_ID = 20;

        // =========================
        // Kicker / Storage Updates
        // =========================
        kickers.update();  // senses colors and updates slot states
        kickers.loadingUpdate();
        Actions.runBlocking (myRobot.motiffUpdate());

        telemetry.addData("Motiff 0", turretSystem.motiff[0]);
        telemetry.addData("Motiff 1", turretSystem.motiff[1]);
        telemetry.addData("Motiff 2", turretSystem.motiff[2]);
        if (turretSystem.motiff[0].equals("G")) { //GPP
            motiffSequence = new SequentialAction(myRobot.loadGreenAction(),myRobot.afterLoad() , myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()); //

            motiffSequence2 = new SequentialAction(myRobot.loadGreenAction(),myRobot.afterLoad() , myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()); //
        } else if (turretSystem.motiff[1].equals("G")) { //PGP
            motiffSequence = new SequentialAction(myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadGreenAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()); //, myRobot.loadGreenAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()
            motiffSequence2 = new SequentialAction(myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadGreenAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()); //, myRobot.loadGreenAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()

        } else { //PPG
            motiffSequence = new SequentialAction(myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadGreenAction(),myRobot.afterLoad()); //
            motiffSequence2 = new SequentialAction(myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadGreenAction(),myRobot.afterLoad()); //

        }
        /*



        */
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(), myRobot.reverseTransfer(),new SequentialAction(myRobot.turnOnTrackingOnly(), myRobot.waitForTracking(),  myRobot.reverseTransferStop(),myRobot.turnOffUpdate())));
        Actions.runBlocking(motiffSequence );

        drive.updatePoseEstimate();

        // drive.actionBuilder(pose)
        //                .turn( Math.PI / 4).build(), myRobot.turnOnTracking(), myRobot.loadMotiff())
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(49.614, -38.1), Math.toRadians(-108.4)).build());
//Pose2d(28.9,-15.0375, Math.toRadians(10)
        while (opModeIsActive()) {

            // =========================
            // Turret & Shooter Controls (Gamepad1)
            // =========================
           // turretSystem.setShootingEnabled(gamepad1.right_trigger > 0.1);



            // Optional telemetry for debugging
            telemetry.addData("timer", kickers.generalTimer);
            telemetry.addData("change flag trigger value", kickers.changeFlagTrigger);
            telemetry.addData("flag", kickers.flag);
            //telemetry.addData("RPM", turretSystem.getTargetRPM());


            telemetry.addData("front dis", kickers.dis);
            telemetry.addData("front pos", kickers.frontPos);
            telemetry.addData("middle pos", kickers.middlePos);
            telemetry.addData("back pos", kickers.backPos);
            telemetry.addData("Kick Target", kickers.kickTarget);

            telemetry.addData("Ball Count", kickers.count);
            telemetry.addData("Front Slot", kickers.ballArray[0]);
            telemetry.addData("Front reading Red", kickers.redReading1);
            telemetry.addData("Front reading Green", kickers.greenReading1);
            telemetry.addData("Front reading blue", kickers.blueReading1);
            telemetry.addData("Middle Slot", kickers.ballArray[1]);
            telemetry.addData("Middle reading Red", kickers.redReading2);
            telemetry.addData("Middle reading Green", kickers.greenReading2);
            telemetry.addData("Middle reading blue", kickers.blueReading2);
            telemetry.addData("Back Slot", kickers.ballArray[2]);
            telemetry.addData("Back reading Red", kickers.redReading3);
            telemetry.addData("Back reading Green", kickers.greenReading3);
            telemetry.addData("Back reading blue", kickers.blueReading3);

            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", turretSystem.currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", turretSystem.currentRPMRight);
            //telemetry.addData("Left Motor power", "%.2f", turretSystem.leftMotor.getPower());
            //telemetry.addData("Right Motor power", "%.2f", turretSystem.rightMotor.getPower());
            //telemetry.addData("Left ticks", turretSystem.leftMotor.getCurrentPosition());
            //telemetry.addData("Right ticks", turretSystem.rightMotor.getCurrentPosition());
            telemetry.addData("Target RPM", "%.1f", turretSystem.targetRPM);
            telemetry.addData("Left RPM Derivative",  "%.1f", turretSystem.leftDerivative);
            telemetry.addData("Right RPM Derivative",  "%.1f", turretSystem.rightDerivative);
            telemetry.addData("shotdetected", turretSystem.shotDetected);
            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", turretSystem.trackingMode ? "ON" : "OFF");
            telemetry.addData("Tag Visible", turretSystem.telemetryData.tagFound);
            telemetry.addData("Error Angle (deg)", "%.2f", turretSystem.telemetryData.errorAngleDeg);
            telemetry.addData("Turret Power", "%.3f", turretSystem.telemetryData.turretPower);
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

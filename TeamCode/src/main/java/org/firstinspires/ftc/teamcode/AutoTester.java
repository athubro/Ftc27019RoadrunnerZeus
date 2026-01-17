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

@Autonomous(name = "AutoTest", group = "Autonomous")
public class AutoTester extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    //private Storage kickers;  // intake-free storage class
    private StorageWLoader kickers;
    private Pose2d startPose = new Pose2d(36.84, -14.96, 0);
    private boolean waitingForConfig = true;
    private double speedRatio = 0.4;

    private SSMyRobot myRobot;
    @Override
    public void runOpMode() {
        Action motiffSequence;
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
        } else if (turretSystem.motiff[1].equals("G")) { //PGP
            motiffSequence = new SequentialAction(myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadGreenAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()); //, myRobot.loadGreenAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad()

        } else { //PPG
            motiffSequence = new SequentialAction(myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadPurpleAction(),myRobot.afterLoad(), myRobot.loadGreenAction(),myRobot.afterLoad()); //

        }
        /*



        */
        Actions.runBlocking(new SequentialAction(myRobot.constantPRM(3100),myRobot.shooterSpinUp()));
        Actions.runBlocking(myRobot.turnOnUpdate());
        Actions.runBlocking(new ParallelAction(myRobot.updateRobot(),myRobot.reverseTransfer(),new SequentialAction( drive.actionBuilder(startPose).splineToLinearHeading(new Pose2d(0,0,0),0).build(), myRobot.turnOffUpdate())));
        Actions.runBlocking(new ParallelAction(myRobot.reverseTransfer(),drive.actionBuilder(startPose).turn(Math.toRadians(45)).build()));
        Actions.runBlocking(new SequentialAction(myRobot.turnOnUpdate(), new ParallelAction(myRobot.updateRobot(),new SequentialAction(myRobot.turnOnTracking(), drive.actionBuilder(drive.localizer.getPose()).turn(Math.toRadians(-10)).waitSeconds(3).build(),myRobot.turnOffTracking(),myRobot.turnOffUpdate()))));
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).turn(Math.toRadians(-80)).build());
        Actions.runBlocking(new SequentialAction(myRobot.turnOnUpdate(), new ParallelAction(myRobot.updateRobot(), myRobot.turnOnTracking(), new SequentialAction(drive.actionBuilder(drive.localizer.getPose()).turn(Math.toRadians(35)).waitSeconds(3).turn(Math.toRadians(45)).waitSeconds(5).build(),myRobot.turnOffUpdate()))));
       /* Actions.runBlocking(new SequentialAction( myRobot.reverseTransfer(), drive.actionBuilder(startPose)
                .lineToX(28.89).build(), myRobot.turnOnTracking(),
                motiffSequence

                ));

        drive.updatePoseEstimate();

        // drive.actionBuilder(pose)
        //                .turn( Math.PI / 4).build(), myRobot.turnOnTracking(), myRobot.loadMotiff())

        Actions.runBlocking(new SequentialAction( myRobot.turnOffTracking(),drive.actionBuilder(drive.localizer.getPose())
                .turn(Math.toRadians(-110.67)).splineTo(new Vector2d(24.714, -38.1), Math.toRadians(-108.4)).build(),myRobot.intake(1)));
        drive.updatePoseEstimate(); // ---------------------------------^^24.714    ^^-38.1                 ^-108.4

        Actions.runBlocking(new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(16.05, -59.746), Math.toRadians(-103.7), new TranslationalVelConstraint(10)).build(),
                myRobot.intake(0))); // ^^^.lineToY(-58.746, new TranslationalVelConstraint(7))
        drive.updatePoseEstimate();

        Actions.runBlocking(new SequentialAction( drive.actionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(28.9,-15.0375, Math.toRadians(10)), Math.toRadians(10)).build(), myRobot.reverseTransfer(), myRobot.turnOnTracking(), motiffSequence ));

*/
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

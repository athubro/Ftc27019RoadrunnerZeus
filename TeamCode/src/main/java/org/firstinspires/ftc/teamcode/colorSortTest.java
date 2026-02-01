package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ColorSortTest", group = "TeleOp")
public class colorSortTest extends LinearOpMode {

  //  private Turret turret;
    private Intake intake;
    private Pose2d initialPose = new Pose2d(0, 0, 0);

    private double speedRatio = 0.75;

    public String[] motiff = {"P", "P" ,"G"};

    @Override
    public void runOpMode() {

        // Initialize all systems
      //  turret = new Turret(hardwareMap, telemetry, initialPose);
        intake = new Intake(hardwareMap, telemetry);

        // Configure turret
  //      turret.PARAMS.TARGET_TAG_ID = 20;
    //    turret.setAutoAngleEnabled(false);  // Start with manual angle control
      //  turret.setAutoRPMEnabled(false);    // Start with manual RPM control
        //turret.setTrackingMode(false);      // Start with manual heading control

        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("Turret + Drive + Intake Initialized");
        telemetry.addLine("Target Tag ID: 20");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1: Drive Controls");
        telemetry.addLine("Gamepad 2: Turret & Intake Controls");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

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
            if (gamepad1.aWasPressed()) {
                intake.storeBalls(motiff);
            }
            if (gamepad1.bWasPressed()) {
                intake.executeNextStep();
            }
            if (gamepad1.xWasPressed()) {
                intake.storeMiddle();
            }
            if (gamepad1.yWasPressed()) {
                intake.storeTop();
            }
            if (gamepad1.rightBumperWasPressed()) {
                intake.resetAll();
            }
            if (gamepad1.dpadUpWasPressed()) {
                motiff[0] = "P";
                motiff[1] = "P";
                motiff[2] = "G";

            }
            if (gamepad1.dpadDownWasPressed()) {

                motiff[0] = "G";
                motiff[1] = "P";
                motiff[2] = "P";

            }if (gamepad1.dpadLeftWasPressed()) {

                motiff[0] = "P";
                motiff[1] = "G";
                motiff[2] = "P";



            }
            intake.setIntakePower(gamepad1.right_trigger);



            // =========================
            // UPDATE ALL SYSTEMS
            // =========================

            // Update turret with drive controls

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("back(top)", intake.storage[0]);
            telemetry.addData("middle ", intake.storage[1]);
            telemetry.addData("front (bottom)", intake.storage[2]);

            telemetry.addData("front dis", intake.frontDis);
            telemetry.addData("middle dis", intake.middleDis);
            telemetry.addData("back dis", intake.backDis);
            telemetry.addData("front hue", intake.frontHue);
            telemetry.addData("middle hue", intake.middleHue);
            telemetry.addData("back hue", intake.backHue);
            telemetry.addData("motiff0", motiff[0]);
            telemetry.addData("motiff1", motiff[1]);
            telemetry.addData("motiff2", motiff[2]);

            telemetry.addData("firststep", intake.firstStep);
            telemetry.addData("secondstep", intake.secondStep);
            telemetry.addData("ballCount", intake.ballCount);
            telemetry.addData("ball stored?", intake.ballsStored);





            telemetry.update();
        }
    }
}
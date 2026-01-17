package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@TeleOp(name = "ErrorTurretPID", group = "Autonomous")
public class ErrorTurretPID extends LinearOpMode {

    private CRServo turret;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    private static final int TARGET_TAG_ID = 20;
    private static final double TOLERANCE_DEG = 1.0;
    private static final double BASE_POWER = 0.15;  // max speed
    private static final double MIN_POWER = 0.07;   // minimum correction
    private static final double KP = 0.02;         // proportional gain (new)
    private static final double SEARCH_POWER = 0.08;
    private static final long TRACK_SLEEP_MS = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(CRServo.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight.setPollRateHz(100);
        limelight.start();

        boolean trackingMode = false;
        boolean aPressedLast = false;
        double lastDirection = 1.0; // 1 = right, -1 = left

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --- Toggle tracking mode ---
            if (gamepad1.a && !aPressedLast) {
                trackingMode = !trackingMode;
            }
            aPressedLast = gamepad1.a;

            double turretPower = 0.0;
            double errorAngleDeg = 0.0;
            boolean tagFound = false;

            // --- Get Limelight data ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        tagFound = true;
                        errorAngleDeg = fid.getTargetXDegrees();
                        break;
                    }
                }
            }

            if (trackingMode) {
                if (tagFound) {
                    // --- Proportional correction ---
                    if (Math.abs(errorAngleDeg) > TOLERANCE_DEG) {
                        double proportionalPower = KP * Math.abs(errorAngleDeg);

                        // Limit the power range
                        proportionalPower = Math.min(proportionalPower, BASE_POWER);
                        proportionalPower = Math.max(proportionalPower, MIN_POWER);

                        if (errorAngleDeg > 0) {
                            // Tag is to the right → turn turret right (corrected direction)
                            turretPower = -proportionalPower;
                            lastDirection = -1;
                        } else {
                            // Tag is to the left → turn turret left
                            turretPower = proportionalPower;
                            lastDirection = 1;
                        }
                    } else {
                        turretPower = 0.0;
                    }
                } else {
                    // --- Tag lost: scan opposite of last direction ---
                    turretPower = -lastDirection * SEARCH_POWER;
                }
            } else {
                // --- Manual mode ---
                turretPower = gamepad1.right_stick_x;
            }

            turret.setPower(turretPower);

            // --- Telemetry ---
            telemetry.addData("Tracking Mode", trackingMode ? "ON" : "OFF");
            telemetry.addData("Turret Power", "%.3f", turretPower);
            if (tagFound) {
                telemetry.addData("Tag ID", TARGET_TAG_ID);
                telemetry.addData("Error Angle (deg)", "%.2f", errorAngleDeg);
            } else {
                telemetry.addLine("Tag " + TARGET_TAG_ID + " not detected (searching...)");
            }
            telemetry.update();

            if (trackingMode) sleep(TRACK_SLEEP_MS);
        }

        limelight.close();
    }
}

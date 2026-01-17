package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "PIDtest", group = "Autonomous")
public class PIDtest extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ElapsedTime timer = new ElapsedTime();

    // PID constants
    double kP = 0.015;
    double kI = 0.000001;
    double kD = 0.000000;

    double targetRPM = 4500;
    double tolerance = 15; // ±15 RPM

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotor.class, "left motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right motor");

        // Reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse right motor direction
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("PID Speed Control Ready");
        telemetry.addLine("Motors will try to maintain 4000 RPM ±15");
        telemetry.update();

        waitForStart();

        // PID variables
        double integral = 0;
        double lastError = 0;
        int lastPosition = leftMotor.getCurrentPosition();
        double lastTime = timer.seconds();
        double output = 0; // last applied motor power

        int loopCount = 0;
        final int UPDATE_INTERVAL = 1000; // PID + telemetry every 1000 loops

        while (opModeIsActive()) {
            loopCount++;

            if (loopCount % UPDATE_INTERVAL == 0) {
                int currentPosition = leftMotor.getCurrentPosition();
                double currentTime = timer.seconds();

                int deltaTicks = currentPosition - lastPosition;
                double deltaTime = currentTime - lastTime;

                if (deltaTime <= 0) deltaTime = 0.001;

                double revsPerSec = (deltaTicks / 28.0) / deltaTime; // 28 ticks per rev
                double currentRPM = revsPerSec * 60.0;

                double error = targetRPM - currentRPM;

                if (Math.abs(error) > tolerance) {
                    integral += error * deltaTime;
                }

                double derivative = (error - lastError) / deltaTime;
                output = (kP * error) + (kI * integral) + (kD * derivative);

                output = Math.max(0, Math.min(1, output));

                // Apply same power to both motors
                leftMotor.setPower(output);
                rightMotor.setPower(output);

                // --- FTC Dashboard Telemetry ---
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Target RPM", targetRPM);
                packet.put("Current RPM", currentRPM);
                packet.put("Error", error);
                packet.put("Motor Power", output);
                dashboard.sendTelemetryPacket(packet);

                // --- Driver Station Telemetry ---
                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Current RPM", "%.2f", currentRPM);
                telemetry.addData("Error", "%.2f", error);
                telemetry.addData("Power", "%.3f", output);
                telemetry.update();

                lastError = error;
                lastPosition = currentPosition;
                lastTime = currentTime;
            }

            // Keep applying last power to maintain speed
            leftMotor.setPower(output);
            rightMotor.setPower(output);
        }
    }
}

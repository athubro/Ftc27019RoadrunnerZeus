package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "SpeedAntiPID", group = "Testing")
public class SpeedAntiPID extends LinearOpMode {

    private DcMotor leftMotor;
    private ElapsedTime timer = new ElapsedTime();

    // --- SETTINGS ---
    double testPower = 0.6;   // constant power to test
    double ticksPerRev = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotor.class, "left motor");

        // Reset encoder
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Constant Power Speed Test Ready");
        telemetry.addData("Power", testPower);
        telemetry.update();

        waitForStart();

        int lastPosition = leftMotor.getCurrentPosition();
        double lastTime = timer.seconds();

        // start motor at constant power
        leftMotor.setPower(testPower);

        int loopCount = 0;
        final int UPDATE_INTERVAL = 1000; // every 1000 loops, update telemetry

        while (opModeIsActive()) {
            loopCount++;

            if (loopCount % UPDATE_INTERVAL == 0) {
                int currentPosition = leftMotor.getCurrentPosition();
                double currentTime = timer.seconds();

                int deltaTicks = currentPosition - lastPosition;
                double deltaTime = currentTime - lastTime;

                if (deltaTime <= 0) deltaTime = 0.001;

                // Calculate current RPM
                double revsPerSec = (deltaTicks / ticksPerRev) / deltaTime;
                double currentRPM = revsPerSec * 60.0;

                // --- FTC Dashboard Telemetry ---
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Current RPM", currentRPM);
                packet.put("Motor Power", testPower);
                dashboard.sendTelemetryPacket(packet);

                // --- Driver Station Telemetry ---
                telemetry.addData("Current RPM", "%.2f", currentRPM);
                telemetry.addData("Power", "%.3f", testPower);
                telemetry.update();

                // Update previous values
                lastPosition = currentPosition;
                lastTime = currentTime;
            }
        }

        // stop motor when done
        leftMotor.setPower(0);
    }
}

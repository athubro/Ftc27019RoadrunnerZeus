package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PIDF Velocity Control", group ="TeleOP")
public class PIDSetup extends LinearOpMode {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx intake;

    public static double kP = 13.6;
    public static double kI = 0;
    public static double kD = 0.0;
    public static double kF = 30.5;

    public static double targetRPM = 3150;
    public static double TICKS_PER_REV = 28.0;

    // ---- tuning helpers ----
    private final ElapsedTime buttonTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime recoveryTimer = new ElapsedTime();

    private boolean isRecoveryTest = false;
    private double recoveryTime = 0;

    private static final double UPDATE_DELAY = 0.1;   // seconds
    private static final double BASE_STEP = 0.001;
    private static final double MAX_STEP = 0.05;
    private static final double GROWTH_RATE = 1.2;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        rightMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("PIDF Velocity Control Ready");
        telemetry.update();

        waitForStart();

        buttonTimer.reset();
        holdTimer.reset();

        while (opModeIsActive()) {

            leftMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            rightMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            double targetVelocity = (targetRPM / 60.0) * TICKS_PER_REV;

            // ---- HOLD-BASED TUNING ----
            if (buttonTimer.seconds() >= UPDATE_DELAY) {

                double holdTime = holdTimer.seconds();
                double step = BASE_STEP * Math.exp(holdTime * GROWTH_RATE);
                step = Math.min(step, MAX_STEP);
                step = Math.round(step * 100.0) / 100.0;

                // enforce minimum usable step
                if (step < 0.01) {
                    step = 0.01;
                }

                boolean anyHeld =
                        gamepad1.dpad_up || gamepad1.dpad_down ||
                                gamepad1.dpad_left || gamepad1.dpad_right;

                if (anyHeld) {

                    if (gamepad1.dpad_up) kP += step;
                    if (gamepad1.dpad_down) kP -= step;
                    if (gamepad1.dpad_left) kF += step;
                    if (gamepad1.dpad_right) kF -= step;

                    buttonTimer.reset();
                } else {
                    holdTimer.reset();
                }
            }

            if (gamepad1.yWasPressed()) targetRPM += 50;
            if (gamepad1.aWasPressed()) targetRPM -= 50;

            if (gamepad2.left_trigger > 0.1) {
                intake.setPower(gamepad2.left_trigger);
            } else {
                intake.setPower(0);
            }


                // ---- RECOVERY TEST ----
                if (gamepad1.xWasPressed() && !isRecoveryTest) {
                    isRecoveryTest = true;
                    recoveryTimer.reset();
                }

                double leftRPM = (leftMotor.getVelocity() / TICKS_PER_REV) * 60.0;
                double rightRPM = (rightMotor.getVelocity() / TICKS_PER_REV) * 60.0;

                if (isRecoveryTest) {
                    double elapsed = recoveryTimer.seconds();

                    if (elapsed < 5.0) {
                        // Set velocity to 0 for 2 seconds
                        leftMotor.setVelocity(0);
                        rightMotor.setVelocity(0);
                    } else {
                        // Spin back up
                        leftMotor.setVelocity(targetVelocity);
                        rightMotor.setVelocity(targetVelocity);

                        // Check if both motors have reached 95% of target
                        double avgRPM = (leftRPM + rightRPM) / 2.0;
                        if (avgRPM >= targetRPM) {
                            recoveryTime = elapsed - 5.0; // Subtract the stop time
                            isRecoveryTest = false;
                        }
                    }
                } else if (gamepad1.right_trigger > 0.1) {
                    leftMotor.setVelocity(targetVelocity);
                    rightMotor.setVelocity(targetVelocity);
                } else {
                    leftMotor.setVelocity(0);
                    rightMotor.setVelocity(0);
                }

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Target RPM", targetRPM);
                packet.put("Left RPM", leftRPM);
                packet.put("Right RPM", rightRPM);
                packet.put("kP", kP);
                packet.put("kF", kF);
                packet.put("Recovery Time (s)", recoveryTime);
                packet.put("Testing", isRecoveryTest);
                dashboard.sendTelemetryPacket(packet);

                telemetry.addData("Target RPM", targetRPM);
                telemetry.addData("Left RPM", "%.1f", leftRPM);
                telemetry.addData("Right RPM", "%.1f", rightRPM);
                telemetry.addData("kP", "%.5f", kP);
                telemetry.addData("kF", "%.5f", kF);
                telemetry.addData("Recovery Time", "%.2f s", recoveryTime);
                if (isRecoveryTest) {
                    telemetry.addLine(">> RECOVERY TEST RUNNING <<");
                }
                telemetry.update();
            }
        }
    }

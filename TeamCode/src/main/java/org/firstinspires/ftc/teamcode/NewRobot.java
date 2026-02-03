package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NewRobot {

    public final Turret turret;
    public final Intake intake;
    public final MecanumDrive drive;   // assuming you have this class

    private final ElapsedTime timer = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public NewRobot(HardwareMap hardwareMap, MecanumDrive drive, Pose2d initialPose) {
        this.drive = drive;
        this.turret = new Turret(hardwareMap, null, initialPose);  // telemetry null or pass if needed
        this.intake = new Intake(hardwareMap, null);               // telemetry null or pass
        turret.setAutoRPMEnabled(true);
        turret.setAutoAngleEnabled(true);
        turret.setUseOdometryTracking(true);
        turret.setContinuousTracking(true);
        timer.reset();
    }

    // ────────────────────────────────────────────────
    // Basic subsystem controls as Actions
    // ────────────────────────────────────────────────

    public Action intakeOn(double power) {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                intake.setIntakePower(power);
                intake.openGate();
                return false;
            }
        };
    }

    public Action intakeOff() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                intake.setIntakePower(0);
                intake.closeGate();
                return false;
            }
        };
    }

    public Action trackingOn() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                turret.setUseOdometryTracking(true);
                turret.setTrackingMode(true);
                return false;
            }
        };
    }

    public Action trackingOff() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                turret.setTrackingMode(false);
                return false;
            }
        };
    }

    public Action spinUpFlywheel() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                turret.setShootingEnabled(true);
                return false;
            }
        };
    }

    public Action stopFlywheel() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                turret.setShootingEnabled(false);
                return false;
            }
        };
    }

    // ────────────────────────────────────────────────
    // Motiff update (added)
    // ────────────────────────────────────────────────

    public Action motiffUpdate() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                turret.updateMotiff();
                packet.put("Motiff", String.join(",", turret.motiff));
                return false;
            }
        };
    }

    // ────────────────────────────────────────────────
    // Store balls in desired shooting order
    // ────────────────────────────────────────────────

    public Action storeForOrder(String first, String second, String third) {
        return new Action() {
            boolean done = false;
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                if (done) return false;

                String[] desired = {first, second, third};  // first = first to shoot
                intake.storageUpdate();

                if (intake.finishedStoring) {
                    if (!intake.ballsStored) {
                        intake.storeBalls(desired);
                    }
                    done = true;
                    return false;
                }
                return true;  // still working / waiting
            }
        };
    }

    // ────────────────────────────────────────────────
    // Wait until next ball is ready to shoot (after previous shot)
    // ────────────────────────────────────────────────

    public Action waitForNextShotReady(double timeoutSec) {
        return new Action() {
            double start = timer.seconds();
            boolean shotWasDetected = false;
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                if (timer.seconds() - start > timeoutSec) return false;

                turret.update();
                intake.storageUpdate();

                // Detect shot (simple RPM drop – tune threshold)
                boolean shotNow = (turret.currentRPMLeft < turret.targetRPM * 0.75 &&
                        turret.currentRPMRight < turret.targetRPM * 0.75);

                if (shotNow && !shotWasDetected) {
                    shotWasDetected = true;
                    intake.executeNextStep();   // release next ball
                }

                // Wait until compartments are back to reset position
                // and we have at least one ball ready
                boolean compartmentsReset = (intake.middleCompartment.getPosition() < 0.1 &&
                        intake.topCompartment.getPosition() < 0.1);

                boolean hasBallReady = intake.ballCount > 0;

                return !(shotWasDetected && compartmentsReset && hasBallReady);
            }
        };
    }

    // ────────────────────────────────────────────────
    // Continuous subsystem update (call in parallel)
    // ────────────────────────────────────────────────

    public Action subsystemUpdate() {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                turret.update();
                intake.storageUpdate();
                packet.put("RPM L/R", String.format("%.0f / %.0f", turret.currentRPMLeft, turret.currentRPMRight));
                packet.put("Dist", String.format("%.1f", turret.disToAprilTag));
                packet.put("Error deg", String.format("%.1f", turret.errorAngleDeg));
                packet.put("Balls", intake.ballCount);
                packet.put("Storage", String.join(",", intake.storage));
                return true;   // keep running
            }
        };
    }

    // ────────────────────────────────────────────────
    // Quick preset turret positions (if needed)
    // ────────────────────────────────────────────────

    public Action setTurretDegrees(double degrees) {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket packet) {
                double clamped = Math.max(-90, Math.min(90, degrees));
                int ticks = (int)(clamped * Turret.Params.TICKS_PER_BIG_GEAR_DEGREE);
                turret.turretMotor.setTargetPosition(ticks);
                return false;
            }
        };
    }

    // ────────────────────────────────────────────────
    // Get shoot order from motiff (added)
    // ────────────────────────────────────────────────

    public String[] getShootOrder() {
        String[] order = {"P", "P", "P"}; // default
        if (turret.motiff[0].equals("G")) { // GPP
            order = new String[]{"G", "P", "P"};
        } else if (turret.motiff[1].equals("G")) { // PGP
            order = new String[]{"P", "G", "P"};
        } else if (turret.motiff[2].equals("G")) { // PPG
            order = new String[]{"P", "P", "G"};
        }
        return order;
    }
}
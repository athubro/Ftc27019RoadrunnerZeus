package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret subsystem - with moveable turret base for aiming
 */
public final class Turret {

    // ==================== PARAMETERS ====================
    public class Params {
        public static final double PID_INTERVAL = 0.1;

        // PID Coefficients (flywheels)
        public double kP = 50.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 10.0;

        // Motor Parameters
        public static final double TICKS_PER_REV = 28.0;
        public double toleranceRPM = 70.0;

        // Vision Parameters
        public int TARGET_TAG_ID = 20;
        public static final double TOLERANCE_DEG = 1.0;

        // Turret motor control (velocity mode)
        public static final double KP_TURRET_VEL     = 0.18;   // ← tune this (start 0.10–0.30)
        public static final double MIN_TURRET_POWER  = 0.05;
        public static final double MAX_TURRET_POWER  = 0.65;
        public static final double AIM_TOLERANCE_DEG = 1.2;    // deadband / tolerance
        public static final double DEADZONE_DEG      = 0.9;

        // Turret Gear Calculations
        public static final double SMALL_GEAR_TEETH = 39.0;
        public static final double BIG_GEAR_TEETH   = 160.0;
        public static final double TICKS_PER_SMALL_REV = 285.0;
        public static final double GEAR_RATIO = BIG_GEAR_TEETH / SMALL_GEAR_TEETH;
        public static final double BIG_GEAR_DEG_PER_SMALL_REV = 360.0 / GEAR_RATIO;
        public static final double TICKS_PER_BIG_GEAR_DEGREE = TICKS_PER_SMALL_REV / BIG_GEAR_DEG_PER_SMALL_REV; // ≈3.247
    }

    public Params PARAMS = new Params();

    // ==================== HARDWARE ====================
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final Servo turretAngle;          // still servo for shooter angle
    public final DcMotorEx turretMotor;      // ← NEW: DC motor instead of servo for yaw

    public final Limelight3A limelight;
    public final FtcDashboard dashboard;
    public final Telemetry telemetry;
    public final MecanumDrive drive;

    // ==================== DISTANCE MEASUREMENT ====================
    public final double ATHeight = 29.5;
    public final double LimelightHeight = 13.5;
    public final double LimelightAngle = 20;
    public double disToAprilTag = 0;
    public double ATAngle = 0;
    public boolean tagFound = false;

    // ==================== TIMERS ====================
    public final ElapsedTime timer = new ElapsedTime();
    public final ElapsedTime pidTimer = new ElapsedTime();

    // ==================== STATE VARIABLES ====================
    // Flywheel state
    public double currentRPMLeft = 0.0;
    public double currentRPMRight = 0.0;
    public double targetRPM = 3000.0;
    public boolean flywheelUpToSpeed = false;

    // ← FIXED: missing declaration added here
    private double speedCheckTimer = 0.0;

    // Turret angle state (up/down – servo)
    public double turretAnglePos = 0.5;
    public int turretAngleCommand = 0;
    private static final double TURRET_ANGLE_STEP = 0.009;
    public boolean autoAngleEnabled = false;

    // Turret aiming state (yaw – now motor)
    public double targetAngle = 0;           // compensation offset
    public boolean adjustAiming = false;
    private boolean hasAligned = false;

    // Control flags
    public boolean shootingEnabled = false;
    public boolean autoRPMEnabled = false;
    public boolean trackingMode = false;
    public boolean continuousTracking = true;

    // Tracking state
    public double errorAngleDeg = 0.0;

    // ==================== CONSTRUCTOR ====================
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
        this.telemetry = telemetry;

        this.drive = new MecanumDrive(hardwareMap, initialPose);

        leftFlywheel  = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        turretAngle   = hardwareMap.get(Servo.class,     "shooterAngle");
        turretMotor   = hardwareMap.get(DcMotorEx.class, "turretMotor");  // ← your motor name here

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dashboard = FtcDashboard.getInstance();

        // Flywheel config
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        PARAMS.kF = -1 * batteryVoltage + 23.7;

        leftFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF * 0.98);
        rightFlywheel.setVelocityPIDFCoefficients(PARAMS.kP * 1.05, PARAMS.kI, PARAMS.kD, PARAMS.kF);

        // Turret motor config
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // turretMotor.setDirection(DcMotor.Direction.REVERSE);   // ← UNCOMMENT & CHANGE if direction is wrong

        // Initialize servos
        turretAngle.setPosition(turretAnglePos);

        limelight.setPollRateHz(100);
        limelight.start();

        pidTimer.reset();
    }

    // ==================== PUBLIC API ====================
    public void setShootingEnabled(boolean enabled) { this.shootingEnabled = enabled; }
    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }
    public double getCurrentRPMLeft() { return currentRPMLeft; }
    public double getCurrentRPMRight() { return currentRPMRight; }
    public double getTargetRPM() { return targetRPM; }
    public boolean isUpToSpeed() { return flywheelUpToSpeed; }
    public void setAutoRPMEnabled(boolean enabled) { this.autoRPMEnabled = enabled; }
    public double getDistanceToTarget() { return disToAprilTag; }
    public boolean isTagFound() { return tagFound; }

    public void setTrackingMode(boolean enabled) {
        this.trackingMode = enabled;
        if (enabled) {
            hasAligned = false;
            adjustAiming = true;
        } else {
            adjustAiming = false;
            hasAligned = false;
        }
    }

    public void setContinuousTracking(boolean enabled) { this.continuousTracking = enabled; }
    public double getTrackingError() { return errorAngleDeg; }
    public void setTurretAngleCommand(int cmd) { this.turretAngleCommand = cmd; }
    public void setAutoAngleEnabled(boolean enabled) { this.autoAngleEnabled = enabled; }
    public void setTurretAnglePosition(double pos) {
        this.turretAnglePos = clamper(pos, 0.0, 1.0);
        turretAngle.setPosition(turretAnglePos);
    }
    public boolean isAligned() { return hasAligned; }
    public double getTurretAnglePosition() { return turretAnglePos; }

    public MecanumDrive getDrive() { return drive; }
    public void setDrivePowers(PoseVelocity2d powers) { drive.setDrivePowers(powers); }
    public PoseVelocity2d updatePoseEstimate() { return drive.updatePoseEstimate(); }
    public Pose2d getPose() { return drive.localizer.getPose(); }
    public void setPose(Pose2d pose) { drive.localizer.setPose(pose); }

    // ==================== MAIN UPDATE ====================
    public void update() {
        updateVisionTracking();
        if (trackingMode) {
            updateTurretAiming();
        }
        if (tagFound) {
            if (autoRPMEnabled) calcTargetRPM();
            if (autoAngleEnabled) calcTurretAngle();
        }
        updateTurretAngle();
        pidUpdate();
        drive.updatePoseEstimate();
        sendTelemetry();
    }

    public void update(double forwardInput, double strafeInput, double rotationInput) {
        updateVisionTracking();
        if (trackingMode) {
            updateTurretAiming();
        }
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardInput, strafeInput),
                rotationInput
        ));
        if (tagFound) {
            if (autoRPMEnabled) calcTargetRPM();
            if (autoAngleEnabled) calcTurretAngle();
        }
        updateTurretAngle();
        pidUpdate();
        drive.updatePoseEstimate();
        sendTelemetry();
    }

    // ==================== PRIVATE METHODS ====================
    private void pidUpdate() {
        if (pidTimer.seconds() < Params.PID_INTERVAL) return;
        pidTimer.reset();

        double targetVelocity = (targetRPM / 60.0) * Params.TICKS_PER_REV;
        double leftVelocity = leftFlywheel.getVelocity();
        double rightVelocity = rightFlywheel.getVelocity();

        currentRPMLeft  = (leftVelocity  / Params.TICKS_PER_REV) * 60.0;
        currentRPMRight = (rightVelocity / Params.TICKS_PER_REV) * 60.0;

        double errorLeft  = Math.abs(targetRPM - currentRPMLeft);
        double errorRight = Math.abs(targetRPM - currentRPMRight);

        if (errorLeft < PARAMS.toleranceRPM && errorRight < PARAMS.toleranceRPM) {
            if (timer.seconds() - speedCheckTimer > 0.1) {
                flywheelUpToSpeed = true;
            }
        } else {
            speedCheckTimer = timer.seconds();
            flywheelUpToSpeed = false;
        }

        if (shootingEnabled) {
            leftFlywheel.setVelocity(targetVelocity);
            rightFlywheel.setVelocity(targetVelocity);
        } else {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
        }
    }

    private void sendTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left RPM", currentRPMLeft);
        packet.put("Right RPM", currentRPMRight);
        packet.put("Target RPM", targetRPM);
        packet.put("Up to Speed", flywheelUpToSpeed);
        packet.put("Shooting Enabled", shootingEnabled);
        packet.put("Tag Found", tagFound);
        packet.put("Distance to Target", disToAprilTag);
        packet.put("Tracking Error (deg)", errorAngleDeg);
        packet.put("Turret Motor Power", turretMotor.getPower());
        packet.put("Turret Ticks", turretMotor.getCurrentPosition());
        packet.put("LL TX (deg)", errorAngleDeg);
        packet.put("LL TY (deg)", ATAngle);
        packet.put("Has Aligned", hasAligned);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Left RPM", "%.0f", currentRPMLeft);
        telemetry.addData("Right RPM", "%.0f", currentRPMRight);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Up to Speed", flywheelUpToSpeed);
        telemetry.addData("Distance", "%.1f in", disToAprilTag);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Tracking Error", "%.1f°", errorAngleDeg);
        telemetry.addData("Turret Power", "%.2f", turretMotor.getPower());
        telemetry.addData("Turret Ticks", turretMotor.getCurrentPosition());
        telemetry.addData("LL TX", "%.2f°", errorAngleDeg);
        telemetry.addData("LL TY", "%.2f°", ATAngle);
        telemetry.addData("Has Aligned", hasAligned);
    }

    private void updateVisionTracking() {
        tagFound = false;
        errorAngleDeg = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                if (fid.getFiducialId() == PARAMS.TARGET_TAG_ID) {
                    tagFound = true;
                    ATAngle = fid.getTargetYDegrees();
                    errorAngleDeg = fid.getTargetXDegrees();
                    measureDistance();
                    break;
                }
            }
        }
    }

    private void updateTurretAiming() {
        if (!tagFound) {
            turretMotor.setPower(0);
            return;
        }

        if (!continuousTracking && hasAligned) {
            turretMotor.setPower(0);
            return;
        }

        double errorDeg = errorAngleDeg - targetAngle;

        if (Math.abs(errorDeg) < PARAMS.DEADZONE_DEG) {
            turretMotor.setPower(0);
            hasAligned = true;
            return;
        }

        hasAligned = false;

        double power = -errorDeg * PARAMS.KP_TURRET_VEL;   // ← negative because Limelight tx sign convention

        power = Math.max(-PARAMS.MAX_TURRET_POWER, Math.min(PARAMS.MAX_TURRET_POWER, power));

        if (Math.abs(power) > 0.001 && Math.abs(power) < PARAMS.MIN_TURRET_POWER) {
            power = Math.signum(power) * PARAMS.MIN_TURRET_POWER;
        }

        turretMotor.setPower(power);
    }

    private void measureDistance() {
        if (tagFound) {
            disToAprilTag = (ATHeight - LimelightHeight) / Math.tan((ATAngle + LimelightAngle) * (Math.PI / 180));
        }
    }

    private void calcTargetRPM() {
        double x = disToAprilTag;
        if (tagFound) {
            targetRPM = 0.00284 * x * x * x - 0.343 * x * x + 23.8 * x + 2022;
            targetRPM = clamper(targetRPM, 2300, 3180);
        }
    }

    private void calcTurretAngle() {
        double x = disToAprilTag;
        if (tagFound) {
            double shooterAngleSetting;
            if (x < 75) {
                shooterAngleSetting = -0.0000046 * x * x * x + 0.00108 * x * x - 0.0885 * x + 2.54;
            } else {
                shooterAngleSetting = 0.0;
            }
            turretAnglePos = clamper(shooterAngleSetting, 0.0, 1.0);
        }
    }

    private void updateTurretAngle() {
        if (!autoAngleEnabled) {
            if (turretAngleCommand > 0) {
                turretAnglePos += TURRET_ANGLE_STEP;
            } else if (turretAngleCommand < 0) {
                turretAnglePos -= TURRET_ANGLE_STEP;
            }
            turretAnglePos = clamper(turretAnglePos, 0.0, 1.0);
        }
        turretAngle.setPosition(turretAnglePos);
    }

    // ==================== HELPER METHODS ====================
    private static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
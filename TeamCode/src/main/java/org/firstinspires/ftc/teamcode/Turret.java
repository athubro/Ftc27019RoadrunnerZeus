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
 * Turret subsystem - non-blocking tracking, manual control works correctly.
 * Public API kept the same as before.
 */
public final class Turret {

    // ==================== PARAMETERS ====================
    public class Params {
        public static final double PID_INTERVAL = 0.1;

        // PID Coefficients
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

        // Robot Heading Control
        public static final double KP_HEADING = 0.015;
        public static final double MIN_HEADING_POWER = 0.03;
        public static final double MAX_HEADING_POWER = 0.25;
    }

    public Params PARAMS = new Params();

    // ==================== HARDWARE ====================
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final Servo turretAngle;       // Servo controlling turret up/down angle
    public final Limelight3A limelight;
    public final FtcDashboard dashboard;
    public final Telemetry telemetry;
    public final MecanumDrive drive;      // Integrated drive system

    // ==================== DISTANCE MEASUREMENT ====================
    public final double ATHeight = 29.5; // AprilTag height in inches
    public final double LimelightHeight = 11.75; // Limelight height in inches
    public final double LimelightAngle = 22.77; // degrees from horizontal

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

    // Turret angle state
    public double turretAnglePos = 0.5;   // Servo position (0.0 to 1.0)
    public int turretAngleCommand = 0;    // Manual control: -1 down, 0 stop, +1 up
    private static final double TURRET_ANGLE_STEP = 0.009;
    public boolean autoAngleEnabled = false; // Automatically calculate angle from distance

    // Control flags
    public boolean shootingEnabled = false;
    public boolean autoRPMEnabled = false; // Automatically calculate RPM from distance
    public boolean trackingMode = false; // Auto-track target with robot heading

    // Heading control
    public double headingCorrection = 0.0; // Output for robot heading adjustment
    public double errorAngleDeg = 0.0;
    private boolean hasAligned = false;   // Track if robot has aligned once

    // Speed check
    private double speedCheckTimer = 0.0;

    // ==================== CONSTRUCTOR ====================
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
        this.telemetry = telemetry;

        // Initialize MecanumDrive first
        this.drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize hardware
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "left motor");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "right motor");
        turretAngle = hardwareMap.get(Servo.class, "TurretAngle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dashboard = FtcDashboard.getInstance();

        // Configure flywheel motors
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients based on battery voltage
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        PARAMS.kF = -1 * batteryVoltage + 23.7;

        // Set individual PID coefficients (slight tuning difference for balance)
        leftFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF * 0.98);
        rightFlywheel.setVelocityPIDFCoefficients(PARAMS.kP * 1.05, PARAMS.kI, PARAMS.kD, PARAMS.kF);

        // Initialize turret angle to middle position
        turretAngle.setPosition(turretAnglePos);

        // Start limelight polling
        limelight.setPollRateHz(100);
        limelight.start();

        // Initialize timers
        pidTimer.reset();
    }

    // ==================== PUBLIC API ====================

    /**
     * Enable or disable the flywheel shooting
     */
    public void setShootingEnabled(boolean enabled) {
        this.shootingEnabled = enabled;
    }

    /**
     * Set the target RPM for the flywheel
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /**
     * Get the current flywheel RPM
     */
    public double getCurrentRPMLeft() {
        return currentRPMLeft;
    }

    /**
     * Get the current right flywheel RPM
     */
    public double getCurrentRPMRight() {
        return currentRPMRight;
    }

    /**
     * Get the target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Check if flywheel is up to speed
     */
    public boolean isUpToSpeed() {
        return flywheelUpToSpeed;
    }

    /**
     * Enable or disable automatic RPM calculation based on distance
     */
    public void setAutoRPMEnabled(boolean enabled) {
        this.autoRPMEnabled = enabled;
    }

    /**
     * Get the calculated distance to AprilTag
     */
    public double getDistanceToTarget() {
        return disToAprilTag;
    }

    /**
     * Check if AprilTag target is found
     */
    public boolean isTagFound() {
        return tagFound;
    }

    /**
     * Enable or disable tracking mode (robot heading adjustment)
     */
    public void setTrackingMode(boolean enabled) {
        this.trackingMode = enabled;
        // Reset alignment flag when tracking mode is toggled
        if (enabled) {
            hasAligned = false;  // Allow new alignment when tracking is re-enabled
        }
    }

    /**
     * Get the heading correction value for robot movement
     * Positive = turn right, Negative = turn left
     */
    public double getHeadingCorrection() {
        return headingCorrection;
    }

    /**
     * Get the current tracking error in degrees
     */
    public double getTrackingError() {
        return errorAngleDeg;
    }

    /**
     * Set turret angle command for manual control
     * @param cmd -1 for down, 0 for stop, +1 for up
     */
    public void setTurretAngleCommand(int cmd) {
        this.turretAngleCommand = cmd;
    }

    /**
     * Enable or disable automatic angle calculation based on distance
     */
    public void setAutoAngleEnabled(boolean enabled) {
        this.autoAngleEnabled = enabled;
    }

    /**
     * Manually set turret angle position
     * @param pos Position from 0.0 to 1.0
     */
    public void setTurretAnglePosition(double pos) {
        this.turretAnglePos = clamper(pos, 0.0, 1.0);
        turretAngle.setPosition(turretAnglePos);
    }

    /**
     * Check if robot has aligned to target
     */
    public boolean isAligned() {
        return hasAligned;
    }

    /**
     * Get current turret angle position
     */
    public double getTurretAnglePosition() {
        return turretAnglePos;
    }

    /**
     * Get the MecanumDrive instance for advanced control
     */
    public MecanumDrive getDrive() {
        return drive;
    }

    /**
     * Set drive powers directly (bypasses tracking)
     * @param powers PoseVelocity2d containing linear and angular velocities
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        drive.setDrivePowers(powers);
    }

    /**
     * Update pose estimate from drive localizer
     * @return Current robot velocity
     */
    public PoseVelocity2d updatePoseEstimate() {
        return drive.updatePoseEstimate();
    }

    /**
     * Get current robot pose
     */
    public Pose2d getPose() {
        return drive.localizer.getPose();
    }

    /**
     * Set robot pose (for localization)
     */
    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    // ==================== MAIN UPDATE ====================

    /**
     * Main update loop - call this once per loop iteration
     * Now also updates drive systems
     */
    public void update() {
        // Update vision tracking and distance measurement
        updateVisionTracking();

        // Calculate heading correction if tracking mode enabled
        if (trackingMode) {
            updateHeadingControl();
        } else {
            headingCorrection = 0.0;
        }

        // Calculate target RPM and angle based on distance if auto mode enabled
        if (tagFound) {
            if (autoRPMEnabled) {
                calcTargetRPM();
            }
            if (autoAngleEnabled) {
                calcTurretAngle();
            }
        }

        // Update turret angle (manual or auto)
        updateTurretAngle();

        // Update PID control
        pidUpdate();

        // Update drive pose estimation
        drive.updatePoseEstimate();

        // Send telemetry
        sendTelemetry();
    }

    /**
     * Update with driver inputs - handles both driving and turret control
     * @param forwardInput Driver forward/back input (left stick Y)
     * @param strafeInput Driver strafe input (left stick X)
     * @param rotationInput Driver rotation input (right stick X)
     */
    public void update(double forwardInput, double strafeInput, double rotationInput) {
        // Update vision tracking and distance measurement
        updateVisionTracking();

        // Calculate heading correction if tracking mode enabled
        if (trackingMode) {
            updateHeadingControl();
        } else {
            headingCorrection = 0.0;
        }

        // Determine final rotation: use tracking correction if active, otherwise use driver input
        double finalRotation = rotationInput;
        if (trackingMode && tagFound) {
            finalRotation = headingCorrection;
        }

        // Apply drive powers
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardInput, strafeInput),
                finalRotation
        ));

        // Calculate target RPM and angle based on distance if auto mode enabled
        if (tagFound) {
            if (autoRPMEnabled) {
                calcTargetRPM();
            }
            if (autoAngleEnabled) {
                calcTurretAngle();
            }
        }

        // Update turret angle (manual or auto)
        updateTurretAngle();

        // Update PID control
        pidUpdate();

        // Update drive pose estimation
        drive.updatePoseEstimate();

        // Send telemetry
        sendTelemetry();
    }

    // ==================== PRIVATE METHODS ====================

    /**
     * PID update for flywheel velocity control
     */
    private void pidUpdate() {
        if (pidTimer.seconds() < Params.PID_INTERVAL) return;
        pidTimer.reset();

        // Calculate target velocity in ticks per second
        double targetVelocity = (targetRPM / 60.0) * Params.TICKS_PER_REV;

        // Get current velocities
        double leftVelocity = leftFlywheel.getVelocity();
        double rightVelocity = rightFlywheel.getVelocity();

        // Convert to RPM for display/tolerance checking
        currentRPMLeft = (leftVelocity / Params.TICKS_PER_REV) * 60.0;
        currentRPMRight = (rightVelocity / Params.TICKS_PER_REV) * 60.0;

        // Check if both flywheels are up to speed
        double errorLeft = Math.abs(targetRPM - currentRPMLeft);
        double errorRight = Math.abs(targetRPM - currentRPMRight);

        if (errorLeft < PARAMS.toleranceRPM && errorRight < PARAMS.toleranceRPM) {
            if (timer.seconds() - speedCheckTimer > 0.1) {
                flywheelUpToSpeed = true;
            }
        } else {
            speedCheckTimer = timer.seconds();
            flywheelUpToSpeed = false;
        }

        // Apply motor velocity if shooting is enabled
        if (shootingEnabled) {
            leftFlywheel.setVelocity(targetVelocity);
            rightFlywheel.setVelocity(targetVelocity);
        } else {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
        }
    }

    /**
     * Send telemetry data to dashboard
     */
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
        packet.put("Heading Correction", headingCorrection);
        packet.put("Turret Angle Pos", turretAnglePos);

        dashboard.sendTelemetryPacket(packet);

        // Also send to driver station telemetry
        telemetry.addData("Left RPM", "%.0f", currentRPMLeft);
        telemetry.addData("Right RPM", "%.0f", currentRPMRight);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Up to Speed", flywheelUpToSpeed);
        telemetry.addData("Distance", "%.1f in", disToAprilTag);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Tracking Error", "%.1f°", errorAngleDeg);
        telemetry.addData("Turret Angle", "%.2f", turretAnglePos);
    }

    /**
     * Update vision tracking and measure distance to AprilTag
     */
    private void updateVisionTracking() {
        tagFound = false;
        errorAngleDeg = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                if (fid.getFiducialId() == PARAMS.TARGET_TAG_ID) {
                    tagFound = true;
                    ATAngle = fid.getTargetYDegrees();
                    errorAngleDeg = fid.getTargetXDegrees(); // Horizontal offset from center
                    measureDistance();
                    break;
                }
            }
        }
    }

    /**
     * Calculate heading correction for robot to align with target
     * Aligns once, then stops tracking until tracking mode is toggled again
     */
    private void updateHeadingControl() {
        if (!tagFound) {
            headingCorrection = 0.0;
            return;
        }

        // If already aligned once, stop tracking
        if (hasAligned) {
            headingCorrection = 0.0;
            return;
        }

        // Check if within tolerance
        if (Math.abs(errorAngleDeg) <= Params.TOLERANCE_DEG) {
            headingCorrection = 0.0;
            hasAligned = true;  // Mark as aligned - stop tracking
            return;
        }

        // Calculate proportional correction
        double proportionalPower = Params.KP_HEADING * Math.abs(errorAngleDeg);
        proportionalPower = clamper(proportionalPower, Params.MIN_HEADING_POWER, Params.MAX_HEADING_POWER);

        // Positive error = target is to the right, robot should turn right (positive correction)
        // Negative error = target is to the left, robot should turn left (negative correction)
        headingCorrection = (errorAngleDeg > 0) ? proportionalPower : -proportionalPower;
    }

    /**
     * Measure distance to AprilTag using limelight angle
     */
    private void measureDistance() {
        if (tagFound) {
            disToAprilTag = (ATHeight - LimelightHeight) /
                    Math.tan((ATAngle + LimelightAngle) * (Math.PI / 180));
        }
    }

    /**
     * Calculate target RPM based on distance to AprilTag
     * Uses polynomial formula from old robot (Jan 4 formula)
     */
    private void calcTargetRPM() {
        double x = disToAprilTag;

        if (tagFound) {
            // Cubic polynomial formula: 0.00284*x³ - 0.343*x² + 23.8*x + 2022
            targetRPM = 0.00284 * x * x * x - 0.343 * x * x + 23.8 * x + 2022;

            // Clamp RPM to safe operating range
            targetRPM = clamper(targetRPM, 2300, 3180);
        }
    }

    /**
     * Calculate turret angle based on distance to AprilTag
     * Uses polynomial formula from old robot (Jan 4 formula)
     */
    private void calcTurretAngle() {
        double x = disToAprilTag;

        if (tagFound) {
            double shooterAngleSetting;

            // Calculate angle based on distance
            if (x < 75) {
                // Cubic polynomial: -0.0000046*x³ + 0.00108*x² - 0.0885*x + 2.54
                shooterAngleSetting = -0.0000046 * x * x * x + 0.00108 * x * x - 0.0885 * x + 2.54;
            } else {
                // Beyond 75 inches, use minimum angle
                shooterAngleSetting = 0.0;
            }

            // Clamp to valid servo range
            turretAnglePos = clamper(shooterAngleSetting, 0.0, 1.0);
        }
    }

    /**
     * Update turret angle servo position (manual or auto)
     */
    private void updateTurretAngle() {
        // If not in auto mode, apply manual commands
        if (!autoAngleEnabled) {
            if (turretAngleCommand > 0) {
                turretAnglePos += TURRET_ANGLE_STEP;
            } else if (turretAngleCommand < 0) {
                turretAnglePos -= TURRET_ANGLE_STEP;
            }

            turretAnglePos = clamper(turretAnglePos, 0.0, 1.0);
        }

        // Apply position to servo
        turretAngle.setPosition(turretAnglePos);
    }

    // ==================== HELPER METHODS ====================

    /**
     * Clamp a value between min and max
     */
    private static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
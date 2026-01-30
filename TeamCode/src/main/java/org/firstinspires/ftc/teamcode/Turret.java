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

        // Turret Yaw Control (servo aiming)
        public static final double KP_TURRET = 0.01; // Reduced from 0.015 for smoother tracking
        public static final double MIN_TURRET_POWER = 0.03;
        public static final double BASE_TURRET_POWER = 0.25;
        public double aimTolerance = 1.0; // Increased deadband to prevent jitter

        // Turret Gear Calculations:
        // Small gear: 285 ticks per 360° = 39 teeth
        // Big gear: 160 teeth
        // Gear ratio: 160/39 = 4.1026:1
        // Big gear rotation per small gear rev: 360° / 4.1026 = 87.75°
        // Ticks per degree (big gear): 285 / 87.75 = 3.247 ticks/deg
        public static final double SMALL_GEAR_TEETH = 39.0;
        public static final double BIG_GEAR_TEETH = 160.0;
        public static final double TICKS_PER_SMALL_REV = 285.0;
        public static final double GEAR_RATIO = BIG_GEAR_TEETH / SMALL_GEAR_TEETH; // 4.1026
        public static final double BIG_GEAR_DEG_PER_SMALL_REV = 360.0 / GEAR_RATIO; // 87.75°
        public static final double TICKS_PER_BIG_GEAR_DEGREE = TICKS_PER_SMALL_REV / BIG_GEAR_DEG_PER_SMALL_REV; // 3.247

        // Servo position per degree (assuming 180° total servo range = 1.0 position range)
        public double posPerDegree = 1.0 / 180.0; // 0.00556 position units per degree

        // Smooth tracking parameters
        public double maxServoChange = 0.05; // Maximum servo position change per update (prevents large jumps)
        public double servoSmoothingFactor = 0.3; // 0.0 = no smoothing, 1.0 = full smoothing

        // OLD: Robot Heading Control (COMMENTED OUT - NOW USING TURRET BASE)
        // public static final double KP_HEADING = 0.015;
        // public static final double MIN_HEADING_POWER = 0.03;
        // public static final double MAX_HEADING_POWER = 0.25;
    }

    public Params PARAMS = new Params();

    // ==================== HARDWARE ====================
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final Servo turretAngle;       // Servo controlling turret up/down angle
    public final Servo turretAim;         // Servo for turret yaw (left/right aiming)
    public final Limelight3A limelight;
    public final FtcDashboard dashboard;
    public final Telemetry telemetry;
    public final MecanumDrive drive;      // Integrated drive system

    // ==================== DISTANCE MEASUREMENT ====================
    public final double ATHeight = 29.5; // AprilTag height in inches
    public final double LimelightHeight = 13.5; // Limelight height in inches
    public final double LimelightAngle = 23.2; // degrees from horizontal
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

    // Turret angle state (up/down)
    public double turretAnglePos = 0.5;   // Servo position (0.0 to 1.0)
    public int turretAngleCommand = 0;    // Manual control: -1 down, 0 stop, +1 up
    private static final double TURRET_ANGLE_STEP = 0.009;
    public boolean autoAngleEnabled = false; // Automatically calculate angle from distance

    // Turret aiming state (left/right yaw)
    public double turretAimPos = 0.5;     // Servo position for yaw (0.0 to 1.0)
    public double targetTurretPos = 0.5;  // Target position for smoothing
    public double lastTurretPos = 0.5;    // Last position for rate limiting
    public double targetAngle = 0;        // Target angle offset for compensation
    public boolean adjustAiming = false;  // Flag to adjust aiming once

    // Control flags
    public boolean shootingEnabled = false;
    public boolean autoRPMEnabled = false; // Automatically calculate RPM from distance
    public boolean trackingMode = false; // Auto-track target with turret servo
    public boolean continuousTracking = true; // NEW: Enable continuous tracking (not just once)

    // Tracking state
    public double errorAngleDeg = 0.0;
    private boolean hasAligned = false;   // Track if turret has aligned once

    // OLD: Heading control (COMMENTED OUT - NOW USING TURRET BASE)
    // public double headingCorrection = 0.0; // Output for robot heading adjustment
    // private boolean hasAligned = false;

    // Speed check
    private double speedCheckTimer = 0.0;

    // ==================== CONSTRUCTOR ====================
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose) {
        this.telemetry = telemetry;

        // Initialize MecanumDrive first
        this.drive = new MecanumDrive(hardwareMap, initialPose);

        // Initialize hardware
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        turretAngle = hardwareMap.get(Servo.class, "shooterAngle");
        turretAim = hardwareMap.get(Servo.class, "turretAim"); // NEW: Turret yaw servo
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

        // Initialize servos to middle position
        turretAngle.setPosition(turretAnglePos);
        turretAim.setPosition(turretAimPos);
        lastTurretPos = turretAimPos; // Initialize last position

        // Start limelight polling
        limelight.setPollRateHz(100);
        limelight.start();

        // Initialize timers
        pidTimer.reset();
    }

    // ==================== PUBLIC API ====================

    public void setShootingEnabled(boolean enabled) {
        this.shootingEnabled = enabled;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getCurrentRPMLeft() {
        return currentRPMLeft;
    }

    public double getCurrentRPMRight() {
        return currentRPMRight;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isUpToSpeed() {
        return flywheelUpToSpeed;
    }

    public void setAutoRPMEnabled(boolean enabled) {
        this.autoRPMEnabled = enabled;
    }

    public double getDistanceToTarget() {
        return disToAprilTag;
    }

    public boolean isTagFound() {
        return tagFound;
    }

    public void setTrackingMode(boolean enabled) {
        this.trackingMode = enabled;
        if (enabled) {
            hasAligned = false;
            adjustAiming = true;  // Enable aiming adjustment when tracking starts
        } else {
            adjustAiming = false;
            hasAligned = false;
        }
    }

    public void setContinuousTracking(boolean enabled) {
        this.continuousTracking = enabled;
    }

    public double getTrackingError() {
        return errorAngleDeg;
    }

    public void setTurretAngleCommand(int cmd) {
        this.turretAngleCommand = cmd;
    }

    public void setAutoAngleEnabled(boolean enabled) {
        this.autoAngleEnabled = enabled;
    }

    public void setTurretAnglePosition(double pos) {
        this.turretAnglePos = clamper(pos, 0.0, 1.0);
        turretAngle.setPosition(turretAnglePos);
    }

    public boolean isAligned() {
        return hasAligned;
    }

    public double getTurretAnglePosition() {
        return turretAnglePos;
    }

    public double getTurretAimPosition() {
        return turretAimPos;
    }

    public void setTurretAimPosition(double pos) {
        this.turretAimPos = clamper(pos, 0.0, 1.0);
        turretAim.setPosition(turretAimPos);
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        drive.setDrivePowers(powers);
    }

    public PoseVelocity2d updatePoseEstimate() {
        return drive.updatePoseEstimate();
    }

    public Pose2d getPose() {
        return drive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    // ==================== MAIN UPDATE ====================

    public void update() {
        updateVisionTracking();

        // NEW: Update turret aiming instead of robot heading
        if (trackingMode) {
            updateTurretAiming();
        }

        // OLD: Robot heading control (COMMENTED OUT)
        // if (trackingMode) {
        //     updateHeadingControl();
        // } else {
        //     headingCorrection = 0.0;
        // }

        if (tagFound) {
            if (autoRPMEnabled) {
                calcTargetRPM();
            }
            if (autoAngleEnabled) {
                calcTurretAngle();
            }
        }

        updateTurretAngle();
        pidUpdate();
        drive.updatePoseEstimate();
        sendTelemetry();
    }

    public void update(double forwardInput, double strafeInput, double rotationInput) {
        updateVisionTracking();

        // NEW: Update turret aiming instead of robot heading
        if (trackingMode) {
            updateTurretAiming();
        }

        // OLD: Robot heading control (COMMENTED OUT)
        // if (trackingMode) {
        //     updateHeadingControl();
        // } else {
        //     headingCorrection = 0.0;
        // }

        // double finalRotation = rotationInput;
        // if (trackingMode && tagFound && !hasAligned) {
        //     finalRotation = headingCorrection;
        // }

        // NEW: Just pass through rotation - no auto heading adjustment
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardInput, strafeInput),
                rotationInput  // Direct passthrough - turret aims instead
        ));

        if (tagFound) {
            if (autoRPMEnabled) {
                calcTargetRPM();
            }
            if (autoAngleEnabled) {
                calcTurretAngle();
            }
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

        currentRPMLeft = (leftVelocity / Params.TICKS_PER_REV) * 60.0;
        currentRPMRight = (rightVelocity / Params.TICKS_PER_REV) * 60.0;

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
        packet.put("Turret Aim Pos", turretAimPos);
        packet.put("Turret Angle Pos", turretAnglePos);
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
        telemetry.addData("Turret Aim", "%.2f", turretAimPos);
        telemetry.addData("Turret Angle", "%.2f", turretAnglePos);
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
                    errorAngleDeg = fid.getTargetXDegrees(); // Horizontal offset from center
                    measureDistance();
                    break;
                }
            }
        }
    }

    /**
     * NEW: Update turret aiming servo to track target - CONTINUOUS SMOOTH TRACKING
     * Anti-jitter features:
     * 1. Deadband tolerance (1.0°) - ignores small errors
     * 2. Exponential smoothing - gradual position changes
     * 3. Rate limiting - prevents sudden jumps
     */
    private void updateTurretAiming() {
        if (!tagFound) {
            return;
        }

        // If not continuous tracking and already aligned, stop
        if (!continuousTracking && hasAligned) {
            return;
        }

        // Calculate error angle (apply compensation offset if needed)
        double adjustedError = errorAngleDeg - targetAngle;

        // DEADBAND: Ignore small errors to prevent jitter
        if (Math.abs(adjustedError) < PARAMS.aimTolerance) {
            hasAligned = true;
            return;
        } else {
            hasAligned = false; // Keep tracking if error exceeds tolerance
        }

        // Calculate desired position change
        // Negative because servo direction is opposite to error
        double desiredPosChange = -adjustedError * PARAMS.posPerDegree;
        double desiredPos = turretAimPos + desiredPosChange;

        // Clamp to valid servo range
        desiredPos = clamper(desiredPos, 0.0, 1.0);

        // EXPONENTIAL SMOOTHING: Gradually approach target position
        // Lower smoothingFactor = faster response, higher = smoother but slower
        targetTurretPos = turretAimPos + (desiredPos - turretAimPos) * (1.0 - PARAMS.servoSmoothingFactor);

        // RATE LIMITING: Prevent sudden large jumps
        double positionChange = targetTurretPos - lastTurretPos;
        if (Math.abs(positionChange) > PARAMS.maxServoChange) {
            positionChange = Math.signum(positionChange) * PARAMS.maxServoChange;
        }

        turretAimPos = lastTurretPos + positionChange;
        turretAimPos = clamper(turretAimPos, 0.0, 1.0);

        // Apply to servo
        turretAim.setPosition(turretAimPos);

        // Update last position for next iteration
        lastTurretPos = turretAimPos;
    }

    // OLD: Robot heading control (COMMENTED OUT - KEPT FOR REFERENCE)
    /*
    private void updateHeadingControl() {
        if (!tagFound) {
            headingCorrection = 0.0;
            return;
        }

        if (hasAligned) {
            headingCorrection = 0.0;
            return;
        }

        if (Math.abs(errorAngleDeg) <= Params.TOLERANCE_DEG) {
            headingCorrection = 0.0;
            hasAligned = true;
            return;
        }

        double proportionalPower = PARAMS.KP_HEADING * errorAngleDeg;

        if (proportionalPower > 0) {
            headingCorrection = clamper(proportionalPower, Params.MIN_HEADING_POWER, Params.MAX_HEADING_POWER);
        } else {
            headingCorrection = clamper(proportionalPower, -Params.MAX_HEADING_POWER, -Params.MIN_HEADING_POWER);
        }
    }
    */

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
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.Target;

/**
 * Turret subsystem - with moveable turret base for aiming
 */
public final class Turret {

    // ==================== PARAMETERS ====================
    public class Params {
        public static final double PID_INTERVAL = 0.1; // Flywheel PID
        public double kP = 50.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 10.0; // Flywheel motor
        public static final double TICKS_PER_REV = 28.0;
        public double toleranceRPM = 350.0; // Vision
        public int TARGET_TAG_ID = 20;
        public static final double TOLERANCE_DEG = 4.0; // Turret motor settings for RUN_TO_POSITION
        public static final double TURRET_MOTOR_POWER = 0.6; // Power for RUN_TO_POSITION mode
        public static final double TURRET_POSITION_TOLERANCE_DEG = 1.4; // Position tolerance in degrees
        // Turret motor PIDF coefficients (for built-in position controller)
        // Lower P reduces oscillation, higher D adds damping
        public double turretKP = 7.0; // Proportional gain (default is often 10)
        public double turretKI = 0.0; // Integral gain
        public double turretKD = 0.0; // Derivative gain (adds damping)
        public double turretKF = 50.0; // Feedforward gain
        // Gear ratio
        public static final double SMALL_GEAR_TEETH = 39.0;
        public static final double BIG_GEAR_TEETH = 160.0;
        public static final double TICKS_PER_SMALL_REV = 285.0;
        public static final double GEAR_RATIO = BIG_GEAR_TEETH / SMALL_GEAR_TEETH;
        public static final double BIG_GEAR_DEG_PER_SMALL_REV = 360.0 / GEAR_RATIO;
        public static final double TICKS_PER_BIG_GEAR_DEGREE = TICKS_PER_SMALL_REV / BIG_GEAR_DEG_PER_SMALL_REV;
        // Soft limits
        public static final double TURRET_MIN_DEG = -90.0;
        public static final double TURRET_MAX_DEG = +90.0;
        // Legacy
        public double posPerDegree = 1.0 / 180.0;
        public double maxServoChange = 0.05;
        public double servoSmoothingFactor = 0.3;
    }
    public Params PARAMS = new Params();

    // ==================== HARDWARE ====================
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final Servo turretAngle;
    public final Servo rgbIndicator;
    public final DcMotorEx turretMotor;
    public final Limelight3A limelight;
    public final FtcDashboard dashboard;
    public final Telemetry telemetry;
    public  MecanumDrive drive;
    public boolean adjustmentFlag;

    // ==================== DISTANCE MEASUREMENT ====================
    public final double ATHeight = 29.5;
    public final double LimelightHeight = 13.5;
    public final double LimelightAngle = 13.6;
    public double disToAprilTag = 0;
    public double ATAngle = 0;
    public boolean tagFound = false;
    //change with alliance and april tag
    public double LLFarZoneOffset = 2;
    public double velocityCorFactor= 7;
    public double angleCorrFactor =0.5;

    // ==================== TIMERS ====================
    public final ElapsedTime timer = new ElapsedTime();
    public final ElapsedTime pidTimer = new ElapsedTime();

    // ==================== STATE VARIABLES ====================
    public double currentRPMLeft = 0.0;
    public double currentRPMRight = 0.0;
    public double targetRPM = 3000.0;
    public boolean flywheelUpToSpeed = false;
    public double speedCheckTimer = 0.0;
    public double turretAnglePos = 0.5;
    public int turretAngleCommand = 0;
    public static final double TURRET_ANGLE_STEP = 0.009;
    public boolean autoAngleEnabled = false;
    public double errorDeg = 0;
    public double targetAngle = 0;
    public boolean adjustAiming = false;
    public boolean hasAligned = false;
    public boolean shootingEnabled = false;
    public boolean autoRPMEnabled = false;
    public boolean trackingMode = false;
    public boolean continuousTracking = true;
    public double errorAngleDeg = 0.0;
    public double smoothedErrorDeg = 0.0;
    public boolean useOdometryTracking = false;
    public final Vector2d targetPos = new Vector2d(-53, -60); // Turret target position (in ticks)
    public int turretTargetPosition = 0;

    // Motiff detection (added back from old code)
    public String[] motiff = {"N", "N", "N"};

    // ==================== CONSTRUCTOR ====================
    public Turret(HardwareMap hardwareMap, MecanumDrive myDrive, Telemetry telemetry, Pose2d initialPose) {
        this.telemetry = telemetry;
        this.drive =myDrive;
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        turretAngle = hardwareMap.get(Servo.class, "shooterAngle");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        rgbIndicator = hardwareMap.get(Servo.class, "rgbLight");


        dashboard = FtcDashboard.getInstance();
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
        leftFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        PARAMS.kF = -2.21 * batteryVoltage + 42.9;
        leftFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF);
        rightFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set PIDF coefficients to reduce oscillation
        turretMotor.setPositionPIDFCoefficients(PARAMS.turretKP);
        // Note: setVelocityPIDFCoefficients can also be set if needed for smoother motion
        // turretMotor.setVelocityPIDFCoefficients(PARAMS.turretKP, PARAMS.turretKI, PARAMS.turretKD, PARAMS.turretKF);
        // Set initial target position (current position after reset = 0)
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
        turretAngle.setPosition(turretAnglePos);
        limelight.setPollRateHz(100);
        limelight.start();
        pidTimer.reset();


    }

    // ==================== PUBLIC API ====================



    public void setFlywheelPID() {
        leftFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF);
        rightFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF);
    }
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
            adjustAiming = true;
        } else {
            adjustAiming = false;
            hasAligned = false;
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
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
        return 0.5;
    }

    public void setTurretAimPosition(double pos) {
        /* no-op */
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
        drive.updatePoseEstimate();
        return drive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    public void setUseOdometryTracking(boolean enabled) {
        this.useOdometryTracking = enabled;
        adjustmentFlag = true;
    }

    // Methods to tune turret PIDF at runtime
    public void setTurretPIDFCoefficients(double p, double i, double d, double f) {
        PARAMS.turretKP = p;
        PARAMS.turretKI = i;
        PARAMS.turretKD = d;
        PARAMS.turretKF = f;
        turretMotor.setPositionPIDFCoefficients(PARAMS.turretKP);
        // Uncomment if using velocity PIDF:
        // turretMotor.setVelocityPIDFCoefficients(PARAMS.turretKP, PARAMS.turretKI, PARAMS.turretKD, PARAMS.turretKF);
    }

    public void setTurretPositionP(double p) {
        PARAMS.turretKP = p;
        turretMotor.setPositionPIDFCoefficients(PARAMS.turretKP);
    }

    // Motiff methods (added)
    public void updateMotiff() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                if (fid.getFiducialId() == 21) {
                    motiff[0] = "G";
                    motiff[1] = "P";
                    motiff[2] = "P";
                    break;
                }
                if (fid.getFiducialId() == 22) {
                    motiff[0] = "P";
                    motiff[1] = "G";
                    motiff[2] = "P";
                    break;
                }
                if (fid.getFiducialId() == 23) {
                    motiff[0] = "P";
                    motiff[1] = "P";
                    motiff[2] = "G";
                    break;
                }
            }
        }
    }

    public String[] getMotiff() {
        return motiff;
    }

    // ==================== MAIN UPDATE ====================
    public void update() {
        if (useOdometryTracking) {
            updateOdomTracking();
        } else {
            updateVisionTracking();
        }
        if (trackingMode) {
            if (adjustmentFlag) {
                updateTurretAiming();
            }
        }
        if (useOdometryTracking || trackingMode) {
            if (autoRPMEnabled) calcTargetRPM();
            if (autoAngleEnabled) calcTurretAngle();
        }
        updateTurretAngle();
        pidUpdate();
        drive.updatePoseEstimate();
        sendTelemetry();
    }

    public void update(double forwardInput, double strafeInput, double rotationInput) {
        if (useOdometryTracking) {
            updateOdomTracking();
        } else {
            updateVisionTracking();
        }
        if (trackingMode) {
            updateTurretAiming();
        }
        //drive.setDrivePowers(new PoseVelocity2d(
        //        new Vector2d(forwardInput, strafeInput),
        //        rotationInput
        //));
        if (useOdometryTracking || trackingMode) {
            if (autoRPMEnabled) calcTargetRPM();
            if (autoAngleEnabled) calcTurretAngle();
        }
        updateTurretAngle();
        pidUpdate();
        drive.updatePoseEstimate();
        sendTelemetry();
    }

    // ==================== public METHODS ====================
    public void pidUpdate() {
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

    public void sendTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left RPM", currentRPMLeft);
        packet.put("Right RPM", currentRPMRight);
        packet.put("Target RPM", targetRPM);
        packet.put("Up to Speed", flywheelUpToSpeed);
        packet.put("Shooting Enabled", shootingEnabled);
        packet.put("Tag Found", tagFound);
        packet.put("Distance to Target", disToAprilTag);
        packet.put("Tracking Error (deg)", errorAngleDeg);
        packet.put("Smoothed Error", smoothedErrorDeg);
        packet.put("Turret Target Pos", turretTargetPosition);
        packet.put("Turret Current Pos", turretMotor.getCurrentPosition());
        packet.put("Turret Deg", turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
        packet.put("Turret Target Deg", turretTargetPosition / PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
        packet.put("Odom Active", useOdometryTracking);
        packet.put("Has Aligned", hasAligned);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Left RPM", "%.0f", currentRPMLeft);
        telemetry.addData("Right RPM", "%.0f", currentRPMRight);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Up to Speed", flywheelUpToSpeed);
        telemetry.addData("Distance", "%.1f in", disToAprilTag);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Tracking Error", "%.1f°", errorAngleDeg);
        telemetry.addData("Smoothed Error", "%.1f°", smoothedErrorDeg);
        telemetry.addData("Turret Pos", "%d / %d", turretMotor.getCurrentPosition(), turretTargetPosition);
        telemetry.addData("Turret Deg", "%.1f° / %.1f°", turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE, turretTargetPosition / PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
        telemetry.addData("Odom Mode", useOdometryTracking);
        telemetry.addData("Has Aligned", hasAligned);
    }

    public void updateVisionTracking() {
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
                    //????????================================================================================
                    if (disToAprilTag > 100) {
                        targetAngle = LLFarZoneOffset;
                    } else {
                        targetAngle = 0;
                    }
                    //==============================================================
                    break;
                }
            }
        }
    }

    public void updateOdomTracking() {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        Vector2d robotPos = pose.position;
        double robotHeading = pose.heading.toDouble();
        Vector2d toTarget = targetPos.minus(robotPos);
        disToAprilTag = toTarget.norm();
        double absAngleToTarget = Math.atan2(toTarget.y, toTarget.x);
        double relativeAngleRad = absAngleToTarget - robotHeading;
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);
        double currentTurretDeg = turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;
        errorAngleDeg = currentTurretDeg - relativeAngleDeg; // positive = need to turn right
        tagFound = true;
        ATAngle = 0.0;
    }

    public void updateTurretAiming() {
        if (!tagFound) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            return;
        }
        if (!continuousTracking && hasAligned) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            return;
        }
        errorDeg = errorAngleDeg - targetAngle;
        // Low-pass filter for smooth response
        smoothedErrorDeg = 0.75 * smoothedErrorDeg + 0.25 * errorDeg;
        // Check if aligned
        if (Math.abs(smoothedErrorDeg) < PARAMS.TURRET_POSITION_TOLERANCE_DEG) {
            hasAligned = true;
            //turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            return;
        }

        if (errorDeg < PARAMS.TURRET_POSITION_TOLERANCE_DEG) {
            turretMotor.setPower(0);
            if (adjustmentFlag) {
                fineAdjustment();
            }
        }else{
            turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
        }





        hasAligned = false;
        // Calculate target position based on error
        double currentDeg = turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;
        double desiredDeg = currentDeg - smoothedErrorDeg;
        desiredDeg= normalizeAngleDegrees(desiredDeg);
        // Apply soft limits
        desiredDeg = clamper(desiredDeg, PARAMS.TURRET_MIN_DEG, PARAMS.TURRET_MAX_DEG);
        // Convert to ticks and set target position
        turretTargetPosition = (int)(desiredDeg * PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
        turretMotor.setTargetPosition(turretTargetPosition);
    }

    public void measureDistance() {
        if (tagFound) {
            disToAprilTag = (ATHeight - LimelightHeight) / Math.tan((ATAngle + LimelightAngle) * (Math.PI / 180));
        }
    }
    public void fineAdjustment(){

        updateVisionTracking();
        adjustmentFlag = false;
        if (Math.abs(errorDeg) > 0.5) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .turn(Math.toRadians(-errorDeg))
                            .build()
            );
        }
    }

    public void calcTargetRPM() {
        double turretAngle= turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;
        double velocityX=drive.localizer.update().linearVel.x;
        double velocityY=drive.localizer.update().linearVel.y;
        double velocityTowardGoal = velocityX*Math.cos(turretAngle*Math.PI/180)+velocityY*Math.sin(turretAngle*Math.PI/180);
        double velocityParallelGoal = velocityY*Math.cos(turretAngle*Math.PI/180)+velocityX*Math.sin(turretAngle*Math.PI/180); //moving toward left is positive
        targetAngle=-velocityParallelGoal*angleCorrFactor;
        double x = disToAprilTag;
        if (tagFound) {
            if (x<95) {
                targetRPM = 12.6 * x + 1586 - velocityCorFactor * velocityTowardGoal;///1586
            } else {
                targetRPM = 3170;
            }
            targetRPM = clamper(targetRPM, 1586, 4180);
        }
    }

    public void calcTurretAngle() {
        double x = disToAprilTag;
        if (tagFound) {
            double shooterAngleSetting;
            if (x < 95) {
                shooterAngleSetting = 1.76*0.001*x-0.0829;
            } else {
                shooterAngleSetting = 0.75;
            }
            turretAnglePos = clamper(shooterAngleSetting, 0.0, 1.0);
        }
    }

    public void updateTurretAngle() {
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


    public void manualTurretAngle(double degrees) {
        double desiredDeg;
        desiredDeg = clamper(degrees, PARAMS.TURRET_MIN_DEG, PARAMS.TURRET_MAX_DEG);

        // Convert to ticks and set target position
        turretTargetPosition = (int)(desiredDeg * PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
        turretMotor.setTargetPosition(turretTargetPosition);
    }


    public static double normalizeAngleDegrees(double angleDeg) {
        while (angleDeg > 180) {
            angleDeg -= 360;
        }
        while (angleDeg < -180) {
            angleDeg += 360;
        }
        return angleDeg;
    }

    // ==================== HELPER METHODS ====================
    public static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
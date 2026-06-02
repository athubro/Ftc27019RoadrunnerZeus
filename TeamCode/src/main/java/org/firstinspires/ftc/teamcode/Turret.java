package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Turret subsystem - with moveable turret base for aiming
 */
public final class Turret {

    // ==================== PARAMETERS ====================
    public class Params {
        public static final double PID_INTERVAL = 0.1; // Flywheel PID
        public double kP =150;//60, 15.929    ===========  99? //200
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 15.0; // Flywheel motor =========== 15?
        public static final double TICKS_PER_REV = 28.0;
        public double toleranceRPM = 350.0; // Vision
        public int TARGET_TAG_ID = 20;
        public static final double TOLERANCE_DEG = 4.0; // Turret motor settings for RUN_TO_POSITION
        public static final double TURRET_MOTOR_POWER = 1; // Power for RUN_TO_POSITION mode
        public static final double TURRET_POSITION_TOLERANCE_DEG = 2; // Position tolerance in degrees
        // Turret motor PIDF coefficients (for built-in position controller)
        // Lower P reduces oscillation, higher D adds damping
        public double turretKP = 10.0; // Proportional gain (default is often 10)
        public double turretKI = 0; // Integral gain
        public double turretKD = 0; // Derivative gain (adds damping)
        public double turretKF = 20; // Feedforward gain
        public double turretVelocity =700; // TurretMotor's max speed
        // Gear ratio
        public static final double SMALL_GEAR_TEETH = 39.0;
        public static final double BIG_GEAR_TEETH = 160.0;
        public static final double TICKS_PER_SMALL_REV = 540.0;   //285.0 for hex motor
        public static final double GEAR_RATIO = BIG_GEAR_TEETH / SMALL_GEAR_TEETH;
        public static final double BIG_GEAR_DEG_PER_SMALL_REV = 360.0 / GEAR_RATIO;
        public static final double TICKS_PER_BIG_GEAR_DEGREE = TICKS_PER_SMALL_REV / BIG_GEAR_DEG_PER_SMALL_REV;
        // Soft limits
        public static final double TURRET_MIN_DEG = -85.0;
        public static final double TURRET_MAX_DEG = +85.0;
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
    public boolean actionFlagForTurning=false;
    public boolean fineAdjustmentFlag=false;
    // ==================== DISTANCE MEASUREMENT ====================
    public final double ATHeight = 29.5;
    public final double LimelightHeight = 15.5;
    public final double LimelightAngle = 20.8;
    public double disToAprilTag = 0;
    public double ATAngle = 0;
    public boolean tagFound = false;
    //change with alliance and april tag
    public double LLFarZoneOffset = 3.5; //3
    public double velocityCorFactor= 7;
    public double angleCorrFactor =0.5;

    // ==================== TIMERS ====================
    public final ElapsedTime timer = new ElapsedTime();
    public final ElapsedTime pidTimer = new ElapsedTime();
    private double fineAdjustingTimer =0;
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
    public boolean oneTimeAdjust =true;
    public boolean continuousTracking = true;
    public double errorAngleDeg = 0.0;
    public double smoothedErrorDeg = 0.0;
    private double tagTimer=0;
    private double tagChangeDelay=0.5;

    public double TurretDesiredDeg=0;
    public boolean useOdometryTracking = false;
    public  Vector2d targetPos = new Vector2d(-58, -65); // Turret target position (in ticks)
    public int turretTargetPosition = 0;
    public double previousDesiredDeg=0;
    // Motiff detection (added back from old code)
    public String[] motiff = {"N", "N", "N"};
    public boolean teleOpOnly=true;
    private double [] disToTagList ={0, 0, 0 ,0 ,0};

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
        PARAMS.kF = -0.702 * batteryVoltage + 24.3;
        leftFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF);
        rightFlywheel.setVelocityPIDFCoefficients(PARAMS.kP, PARAMS.kI, PARAMS.kD, PARAMS.kF);
        //turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set PIDF coefficients to reduce oscillation


        //turretMotor.setPositionPIDFCoefficients(PARAMS.turretKP);

        // Set initial target position (current position after reset = 0)
        //turretMotor.setTargetPosition(0);
        turretMotor.resetDeviceConfigurationForOpMode();
        //turret.set
        //turret.setTargetPosition(turret.getCurrentPosition());
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidTurret0 = new PIDFCoefficients(10, 0, 0, 10);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurret0);
        turretMotor.setVelocity(100);
        turretMotor.setTargetPosition(0);
        //PIDFCoefficients pidTurret = new PIDFCoefficients(PARAMS.turretKP, PARAMS.turretKI, PARAMS.turretKD, PARAMS.turretKF);
        //turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurret);
        //turretMotor.setVelocity(PARAMS.turretVelocity);
        //
        //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
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
            oneTimeAdjust=true;
        } else {
            adjustAiming = false;
            hasAligned = false;
            //turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
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
        if (!enabled){
            fineAdjustmentFlag=true;
        }
        //adjustmentFlag = true;
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
            fineAdjustmentFlag=false;
            //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
            updateOdomTracking();
        } else {
            //fineAdjustmentFlag=true;
            updateVisionTracking();
        }
        if (trackingMode) {
            if (useOdometryTracking) {
                rgbIndicator.setPosition(0.388);
            } else {
                rgbIndicator.setPosition(1);
            }

            if (continuousTracking){
                updateTurretAiming();
            } else {
                oneTimeAdjustTurretAiming();
            }

            //updateTurretAiming();
        } else{
            previousDesiredDeg=200;
            fineAdjustmentFlag=false;
            //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
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
        if (Math.abs(turretMotor.getCurrentPosition()-turretTargetPosition)<20){
            //turretMotor.setPower(0);
        }
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
        boolean postiveTag=false;
       // boolean previousTagResult=tagFound;

        //tagFound = false;
        //errorAngleDeg = 0.0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                if (fid.getFiducialId() == PARAMS.TARGET_TAG_ID) {
                    postiveTag=true;
                    tagFound = true;
                    ATAngle = fid.getTargetYDegrees();

                    errorAngleDeg = fid.getTargetXDegrees();
                    measureDistance();
                    //????????================================================================================
                    if (disToAprilTag > 90 && drive.localizer.getPose().position.x >20) {
                        targetAngle = LLFarZoneOffset;
                    } else {
                        targetAngle = 0;
                    }
                    //==============================================================
                    break;
                }
            }
        }
        if (!postiveTag){
            if (timer.seconds()>tagTimer+tagChangeDelay) {
                tagFound=false;
                errorAngleDeg=0;
            }
        } else{
            tagTimer=timer.seconds();
        }
    }

    public void updateOdomTracking() {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        Vector2d robotPos = pose.position;
        double robotHeading = pose.heading.toDouble();
        Vector2d toTarget = targetPos.minus(robotPos);
        disToAprilTag = toTarget.norm()-10;
        double absAngleToTarget = Math.atan2(toTarget.y, toTarget.x);
        double relativeAngleRad = absAngleToTarget - robotHeading;
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);
        double currentTurretDeg = turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;
        errorAngleDeg = currentTurretDeg - relativeAngleDeg; // positive = need to turn right
        //tagFound = true;
        ATAngle = 0.0;
    }
    public void oneTimeAdjustTurretAiming(){

        if (!useOdometryTracking && !tagFound) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition()); //?/
            //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
            if (!useOdometryTracking){
                previousDesiredDeg=200;
            }
            return;
        }

        errorDeg = errorAngleDeg - targetAngle;
        // Low-pass filter for smooth response
        //smoothedErrorDeg = 0.75 * smoothedErrorDeg + 0.25 * errorDeg;
        // Check if aligned
        if (Math.abs(errorDeg) < PARAMS.TURRET_POSITION_TOLERANCE_DEG) {
            hasAligned = true;
            //turretMotor.setTargetPosition(turretMotor.getCurrentPosition());//?/
            //turretMotor.setPower(0);//?/
            //return;//?/
            //turretMotor.getCurrent(CurrentUnit.AMPS)
            return;
        } else{
            hasAligned = false;
        }


        // Calculate target position based on error
        double currentDeg = turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;

        //double desiredDeg = currentDeg - smoothedErrorDeg;

        double desiredDeg = currentDeg - errorDeg;
        desiredDeg= normalizeAngleDegrees(desiredDeg);
        // Apply soft limits
        desiredDeg = clamper(desiredDeg, PARAMS.TURRET_MIN_DEG, PARAMS.TURRET_MAX_DEG);
        //TurretDesiredDeg=desiredDeg;
        if (useOdometryTracking){ //useOdometryTracking
            // Convert to ticks and set target position
            if (Math.abs(desiredDeg-previousDesiredDeg)>4){
                turretTargetPosition = (int)(desiredDeg * PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
                turretMotor.setTargetPosition(turretTargetPosition);
                //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);//?/
                previousDesiredDeg=desiredDeg;
            }
        } else{
            if (Math.abs(desiredDeg-previousDesiredDeg)>PARAMS.TURRET_POSITION_TOLERANCE_DEG && oneTimeAdjust) {
                turretTargetPosition = (int) (desiredDeg * PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
                turretMotor.setTargetPosition(turretTargetPosition);
                oneTimeAdjust=false;
                // turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);//?/
                previousDesiredDeg = desiredDeg;
            }
        }

    }
    public void updateTurretAiming() {

        if (actionFlagForTurning && !useOdometryTracking) {
            if (timer.seconds()> fineAdjustingTimer+1 && timer.seconds()<fineAdjustingTimer+3 ){
                fineAdjustingTimer=-1;
                //fineAdjustmentFlag=false;
                fineAdjustment();
            }
            if (timer.seconds()>fineAdjustingTimer+4){
                fineAdjustmentFlag=true;
            }

        }


        if (!tagFound) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition()); //?/
            //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
            if (!useOdometryTracking){
                previousDesiredDeg=200;
            }


        }
        /*
        if (!continuousTracking && hasAligned) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            return;
        }


        if ( hasAligned) {
            //turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            return;
        }

         */
        errorDeg = errorAngleDeg - targetAngle;
        // Low-pass filter for smooth response

        // Check if aligned
        if (Math.abs(errorDeg) < PARAMS.TURRET_POSITION_TOLERANCE_DEG*2) {
            hasAligned = true;
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());//?/
            //turretMotor.setPower(0);//?/
            //return;//?/
            //turretMotor.getCurrent(CurrentUnit.AMPS)
        } else{
            hasAligned = false;
        }

        if (actionFlagForTurning && fineAdjustmentFlag && hasAligned) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            //turretMotor.setPower(0);
            fineAdjustingTimer=timer.seconds();
            return;
        }else if (hasAligned) {
            return;
            //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
        }

        // Calculate target position based on error
        double currentDeg = turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;

        //double desiredDeg = currentDeg - smoothedErrorDeg;

        double desiredDeg = currentDeg - errorDeg;
        desiredDeg= normalizeAngleDegrees(desiredDeg);
        // Apply soft limits
        desiredDeg = clamper(desiredDeg, PARAMS.TURRET_MIN_DEG, PARAMS.TURRET_MAX_DEG);
        //TurretDesiredDeg=desiredDeg;
        if (useOdometryTracking){ //useOdometryTracking
            // Convert to ticks and set target position
            if (Math.abs(desiredDeg-previousDesiredDeg)>5){
                turretTargetPosition = (int)(desiredDeg * PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
                turretMotor.setTargetPosition(turretTargetPosition);
                //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);//?/
                previousDesiredDeg=desiredDeg;
            }
        } else{
            if (Math.abs(desiredDeg-previousDesiredDeg)>PARAMS.TURRET_POSITION_TOLERANCE_DEG*2) {
                turretTargetPosition = (int) (desiredDeg * PARAMS.TICKS_PER_BIG_GEAR_DEGREE);
                turretMotor.setTargetPosition(turretTargetPosition);
               // turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);//?/
                previousDesiredDeg = desiredDeg;
            }
        }



    }

    public void measureDistance() {
        if (tagFound) {
            disToAprilTag = (ATHeight - LimelightHeight) / Math.tan((ATAngle + LimelightAngle) * (Math.PI / 180));
            //57 = 14/math.tan(-7+x)*(pi/180)
            //math.tan(-7+x)*(pi/180) = 14/57
            //

        }
    }
    public void fineAdjustment(){

        updateVisionTracking();
        fineAdjustmentFlag = false;

        if (actionFlagForTurning&&teleOpOnly){
            errorDeg = errorAngleDeg - targetAngle;
            if (Math.abs(errorDeg) > 0.5) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .turn(Math.toRadians(-errorDeg))
                                .build()
                );
            }
        }

    }

    public void calcTargetRPM() {
        double turretAngle= turretMotor.getCurrentPosition() / PARAMS.TICKS_PER_BIG_GEAR_DEGREE;
        double velocityX=drive.localizer.update().linearVel.x;
        double velocityY=drive.localizer.update().linearVel.y;
        double velocityTowardGoal = velocityX*Math.cos(turretAngle*Math.PI/180)+velocityY*Math.sin(turretAngle*Math.PI/180);
        double velocityParallelGoal = velocityY*Math.cos(turretAngle*Math.PI/180)+velocityX*Math.sin(turretAngle*Math.PI/180); //moving toward left is positive
        targetAngle=-velocityParallelGoal*angleCorrFactor;
        double x=0;
        for (int i =0; i< disToTagList.length-1; i++){
            x+= disToTagList[i];
            disToTagList[i]=disToTagList[i+1];
        }
        disToTagList[disToTagList.length-1]=disToAprilTag;
        x+=disToAprilTag;
        x=x/disToTagList.length;
        if (tagFound) {
           // targetRPM = 11.6 * x + 1650 - velocityCorFactor * velocityTowardGoal;

            if (x<80 && drive.localizer.getPose().position.x<30) {
               // targetRPM = 11.6 * x + 1650 - velocityCorFactor * velocityTowardGoal;
                targetRPM = 10.4 * x + 1860 - velocityCorFactor * velocityTowardGoal; //1910
            } else {
              //  targetRPM = 11.6 * x + 1720 - velocityCorFactor * velocityTowardGoal;
                targetRPM = 11 * x + 2150 - velocityCorFactor * velocityTowardGoal;

            }


            targetRPM = clamper(targetRPM, 1586, 5000);
        }
    }

    public void calcTurretAngle() {
        double x = disToAprilTag;
        if (tagFound) {
            double shooterAngleSetting;
            if (x < 85) {
                //shooterAngleSetting = 1.76*0.001*x-0.0829;
                shooterAngleSetting = -1.25 + 0.0681*x - 0.000781*x*x + 0.00000293*x*x*x;
            } else {
                shooterAngleSetting = 0.85;
               // shooterAngleSetting = -1.25 + 0.0681*x - 0.000781*x*x + 0.00000293*x*x*x;

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
        if (currentRPMLeft> targetRPM&& targetRPM>2800){
            turretAngle.setPosition(turretAnglePos*(1+2*(currentRPMLeft-targetRPM)/targetRPM));
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


    public void resetTurretEncoder() {
        turretMotor.resetDeviceConfigurationForOpMode();
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //turretMotor.setPower(PARAMS.TURRET_MOTOR_POWER);
    }

    public void updateTurretVelocity (double vol){
        PARAMS.turretVelocity=vol;
        turretMotor.setVelocity(PARAMS.turretVelocity);
    }

    public void updateTurretPID(){
        PIDFCoefficients pidTurret = new PIDFCoefficients(PARAMS.turretKP, PARAMS.turretKI, PARAMS.turretKD, PARAMS.turretKF);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurret);

        turretMotor.setVelocity(PARAMS.turretVelocity);
        turretMotor.setTargetPosition(0);
    }
    // ==================== HELPER METHODS ====================
    public static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
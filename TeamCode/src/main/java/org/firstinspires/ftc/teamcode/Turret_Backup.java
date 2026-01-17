package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret subsystem - non-blocking tracking, manual control works correctly.
 * Public API kept the same as before.
 */
public final class Turret_Backup {
    public class Params {
        public static final double PID_INTERVAL = 0.1;

        public double kP = 0.0005;
        public double kI = 0.002;
        public double kD = 0.00001;

        public double toleranceRPM = 80.0;

        public static final double TICKS_PER_REV = 28.0;

        public int TARGET_TAG_ID = 20;
        public static final double TOLERANCE_DEG = 1.0;
        public static final double BASE_TURRET_POWER = 0.2;
        public static final double MIN_TURRET_POWER = 0.08;
        public static final double KP_TURRET = 0.02;
        public static final double SEARCH_POWER = 0.08;
        public static final long TRACK_SLEEP_MS = 20L;
    }
    //public int targetID=20;
    public  Params PARAMS = new Params();

    // Hardware
    public final CRServo turret;          // continuous servo for yaw control
    public final Servo turretAngle;       // regular servo controlling turret angle
    public final Limelight3A limelight;

    public double disToAprilTag = 0;

    public double ATAngle = 0;

    //constants for measuring distance

    public final double  ATHeight = 29.5; //inches
    public final double LimelightHeight = 11.75; //inches

    public final double LimelightAngle = 22.77; //degrees fron horizontal position

    public final double targetSpeedLinearSplit = 46;
    public final DcMotor leftMotor;       // shooter left
    public final DcMotor rightMotor;      // shooter right
    public final FtcDashboard dashboard;
    public Servo rgbIndicator;

    // External telemetry (supplied by caller)
    public final Telemetry telemetry;

    // State (shooter PID)
    public final ElapsedTime timer = new ElapsedTime();
    public final ElapsedTime pidTimer = new ElapsedTime();

    public double lastTime = 0.0;
    public int lastPositionLeft = 0;
    public int lastPositionRight = 0;

    public double integralLeft = 0.0;
    public double lastErrorLeft = 0.0;
    public double outputPowerLeft = 0.0;
    public double currentRPMLeft = 0.0;

    public double integralRight = 0.0;
    public double lastErrorRight = 0.0;
    public double outputPowerRight = 0.0;
    public double currentRPMRight = 0.0;

    public boolean shooterUpToSpeed = false;

    // Turret tracking & control state
    public boolean trackingMode = false;   // whether limelight auto-tracks

    public boolean tagFound = false;
    public double lastDirection = 1.0;    // last search direction when tag lost

    // Turret angle state (for Servo position)
    public double turretAnglePos = 0.5;   // start at middle position
    private static final double TURRET_ANGLE_STEP = 0.009;

    public double targetAngle = 0;

    // External inputs (set by caller)
    public boolean shootingEnabled = false;
    public double manualTurretInput = 0.0; // store joystick value; update() applies it
    public int turretAngleCommand = 0;     // -1 left, 0 stop, 1 right (previously D-PAD)
    public double targetRPM = 3000.0;      // can be changed via setter
    double lastShotTimer =0;
    public double lastLeftRPM = 0;
    public double lastRightRPM = 0;

    public double shotDetectLastTime = 0;
    public boolean shotDetected = false;

    public final double RPMDerivativeThreshold = 3000;
    public double leftDerivative=0;
    public double rightDerivative=0;
    private double speedCheckTimer =0;
    public boolean autoSpinUp = false;
    public boolean constantRPM =false;

    public String[] motiff = {"N", "N", "N"};
    // Constructor
    public Turret_Backup(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        turret = hardwareMap.get(CRServo.class, "turret");
        turretAngle = hardwareMap.get(Servo.class, "TurretAngle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftMotor = hardwareMap.get(DcMotor.class, "left motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right motor");

        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");


        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        // start limelight polling immediately
        limelight.setPollRateHz(100);
        limelight.start();

        // initialize timers / positions
        lastPositionLeft = leftMotor.getCurrentPosition();
        lastPositionRight = rightMotor.getCurrentPosition();
        lastTime = timer.seconds();
        pidTimer.reset();
    }

    // External setters (API preserved)
    public void setShootingEnabled(boolean enabled) {
        this.shootingEnabled = enabled;
    }

    // store manual turret input; update() will apply it when not tracking
    public void setManualTurretPower(double inputPower) {
        this.manualTurretInput = inputPower;
    }

    /**
     * turretAngleCommand: -1 left, 0 stop, +1 right
     */
    public void setTurretAngleCommand(int cmd) {
        this.turretAngleCommand = cmd;
    }

    public void setTrackingMode(boolean on) {
        this.trackingMode = on;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getCurrentRPMLeft() { return currentRPMLeft; }
    public double getCurrentRPMRight() { return currentRPMRight; }
    public double getTargetRPM() { return targetRPM; }

    public void color (double rgb) {
        rgbIndicator.setPosition(rgb);
    }

    // -------------------------
    // Main update (call once per loop)
    // -------------------------
    public void update() {

        updateTurretControl();
        //updateMotiff();
      //  setShootingEnabled(tagFound);
        // update shooter PID (non-blocking)
        calcTargetAngleSpeed();
        pidUpdate();
        shotDetection();
        // update turret yaw (non-blocking; either tracking or manual)

        //updateTurretControl();

        // update turret angle servo (non-blocking)
        updateTurretAngle();
        if (autoSpinUp){
            if (tagFound) {
                setShootingEnabled(true);
            } else {
                setShootingEnabled(false);
            }
        }
        // telemetry
        sendTelemetry();
        // no sleeps here — caller loop remains responsive
    }
    /*
    private void pidUpdate() {
        if (pidTimer.seconds() < Params.PID_INTERVAL) return;
        pidTimer.reset();

        int currentPositionL = leftMotor.getCurrentPosition();
        int currentPositionR = rightMotor.getCurrentPosition();
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0.0) deltaTime = 1e-6;

        // Left
        int deltaTicksLeft = currentPositionL - lastPositionLeft;
        double revsPerSecLeft = (deltaTicksLeft / Params.TICKS_PER_REV) / deltaTime;
        currentRPMLeft = revsPerSecLeft * 60.0;
        double errorLeft = targetRPM - currentRPMLeft;
        if (Math.abs(errorLeft) < PARAMS.toleranceRPM/2) {
            errorLeft = 0;
        }
        integralLeft += errorLeft * deltaTime;
        double derivativeLeft = (errorLeft - lastErrorLeft) / deltaTime;
        double pidOutputLeft = (PARAMS.kP * errorLeft) + (PARAMS.kI * integralLeft) + (PARAMS.kD * derivativeLeft);
        pidOutputLeft = clamper(pidOutputLeft, 0.0, 1.0);
        lastErrorLeft = errorLeft;
        lastPositionLeft = currentPositionL;

        // Right
        int deltaTicksRight = currentPositionR - lastPositionRight;
        double revsPerSecRight = (deltaTicksRight / Params.TICKS_PER_REV) / deltaTime;
        currentRPMRight = revsPerSecRight * 60.0;
        double errorRight = targetRPM - currentRPMRight;
        if (Math.abs(errorRight) < PARAMS.toleranceRPM/2) {
            errorRight = 0;
            color(0.611);
        } else if (Math.abs(errorRight) < 150) {
            color(0.333);
        } else {
            color(0.27);
        }
        integralRight += errorRight * deltaTime;
        double derivativeRight = (errorRight - lastErrorRight) / deltaTime;
        double pidOutputRight = (PARAMS.kP * errorRight) + (PARAMS.kI * integralRight) + (PARAMS.kD * derivativeRight);
        pidOutputRight = clamper(pidOutputRight, 0.0, 1.0);
        lastErrorRight = errorRight;
        lastPositionRight = currentPositionR;

        lastTime = currentTime;

        // Apply shooter motor power if shooting is enabled
        if (shootingEnabled) {
            leftMotor.setPower(pidOutputLeft);
            rightMotor.setPower(pidOutputRight);
            outputPowerLeft = pidOutputLeft;
            outputPowerRight = pidOutputRight;
        } else {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            integralLeft = 0.0;
            integralRight = 0.0;
            outputPowerLeft = 0.0;
            outputPowerRight = 0.0;
        }
    }
    */
    private void pidUpdate() {
        if (pidTimer.seconds() < Params.PID_INTERVAL) return;
        pidTimer.reset();

        int currentPositionL = leftMotor.getCurrentPosition();
        int currentPositionR = rightMotor.getCurrentPosition();
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0.0) deltaTime = 1e-6;

        // --- Left Motor ---
        int deltaTicksLeft = currentPositionL - lastPositionLeft;
        double revsPerSecLeft = (deltaTicksLeft / Params.TICKS_PER_REV) / deltaTime;
        currentRPMLeft = revsPerSecLeft * 60.0;

        double errorLeft = targetRPM - currentRPMLeft;
        double offsetLeft;
        double offsetRight;
        offsetLeft = (targetRPM + 47.6) / 5952.0;
        //offsetLeft=offsetLeft/2;
        double pidOutputLeft;
        pidOutputLeft = offsetLeft;

        if (Math.abs(errorLeft) < PARAMS.toleranceRPM ) {
            //if (shooterUpToSpeed) integralLeft=0;

            //pidOutputLeft = offsetLeft;
        } else {
            integralLeft += errorLeft * deltaTime;
            double derivativeLeft = (errorLeft - lastErrorLeft) / deltaTime;
            pidOutputLeft = offsetLeft + (PARAMS.kP * errorLeft) + (PARAMS.kI * integralLeft) + (PARAMS.kD * derivativeLeft);
            pidOutputLeft = clamper(pidOutputLeft, 0.0, 1.0);
        }

        lastErrorLeft = errorLeft;
        lastPositionLeft = currentPositionL;

        // --- Right Motor ---
        int deltaTicksRight = currentPositionR - lastPositionRight;
        double revsPerSecRight = (deltaTicksRight / Params.TICKS_PER_REV) / deltaTime;
        currentRPMRight = revsPerSecRight * 60.0;

        double errorRight = targetRPM - currentRPMRight;
        offsetRight = (targetRPM + 47.6) / 5952.0;
        //offsetRight=offsetRight/2;
        double pidOutputRight;
        pidOutputRight = offsetRight;
        if (Math.abs(errorRight) < PARAMS.toleranceRPM ) {
            //if (shooterUpToSpeed) integralRight=0;
            //pidOutputRight = offsetRight;
           // color(0.611);

        } else if (Math.abs(errorRight) < 150) {
            integralRight += errorRight * deltaTime;
            double derivativeRight = (errorRight - lastErrorRight) / deltaTime;
            pidOutputRight = offsetRight + (PARAMS.kP * errorRight) + (PARAMS.kI * integralRight) + (PARAMS.kD * derivativeRight);
            pidOutputRight = clamper(pidOutputRight, 0.0, 1.0);
            color(0.333);

        } else {
            integralRight += errorRight * deltaTime;
            double derivativeRight = (errorRight - lastErrorRight) / deltaTime;
            pidOutputRight = offsetRight + (PARAMS.kP * errorRight) + (PARAMS.kI * integralRight) + (PARAMS.kD * derivativeRight);
            pidOutputRight = clamper(pidOutputRight, 0.0, 1.0);
            color(0.27);

        }
        if (Math.abs(errorRight) < PARAMS.toleranceRPM*1 && Math.abs(errorLeft) < PARAMS.toleranceRPM*1){
            if (timer.seconds()-speedCheckTimer>0.15){
                shooterUpToSpeed=true;
                color(0.611);
            }
        } else {
            speedCheckTimer=timer.seconds();
            shooterUpToSpeed=false;
        }
        lastErrorRight = errorRight;
        lastPositionRight = currentPositionR;
        lastTime = currentTime;

        // --- Apply Shooter Motor Power ---
        if (shootingEnabled) {
            leftMotor.setPower(pidOutputLeft);
            rightMotor.setPower(pidOutputRight);
            outputPowerLeft = pidOutputLeft;
            outputPowerRight = pidOutputRight;
            //outputPowerLeft = offsetLeft;
            //outputPowerRight = offsetRight;
        } else {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            integralLeft = 0.0;
            integralRight = 0.0;
            outputPowerLeft = 0.0;
            outputPowerRight = 0.0;
        }
    }
    public void shotDetection(){

        double deltaTime = 0;
        double deltaLeftRPM = 0;
        double deltaRightRPM = 0;
        integralRight = 0;
        integralLeft = 0;
        if (shotDetectLastTime==0) shotDetectLastTime=timer.time();
        deltaTime = timer.time() - shotDetectLastTime;

        if (deltaTime < Params.PID_INTERVAL) return;
        shotDetectLastTime = timer.time();

        if ( !(lastLeftRPM == 0 || lastRightRPM == 0))
        {
            deltaLeftRPM = currentRPMLeft - lastLeftRPM;
            deltaRightRPM = currentRPMRight - lastRightRPM;



            leftDerivative = deltaLeftRPM/deltaTime;
            rightDerivative = deltaRightRPM/deltaTime;



            if (leftDerivative < -RPMDerivativeThreshold && rightDerivative < - RPMDerivativeThreshold) {
                shotDetected = true;
                lastShotTimer = timer.time();
                integralLeft=0;
                integralRight=0;
            } else {

                if (timer.time() - lastShotTimer > 1 && shotDetected) {
                    shotDetected = false;
                }

            }
        }
        lastLeftRPM=currentRPMLeft;
        lastRightRPM=currentRPMRight;
    }

    public void updateMotiff () {
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
    // Non-blocking turret yaw control: tracking or manual
    public void updateTurretControl() {
        double turretPower = 0.0;
         tagFound = false;
        double errorAngleDeg = 0.0;

        if (trackingMode) {
            // read limelight result (non-blocking)
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == PARAMS.TARGET_TAG_ID) {

                        tagFound = true;
                        ATAngle = fid.getTargetYDegrees();
                        measureDis();
                      //  errorAngleDeg = fid.getTargetXDegrees() - targetAngle;
                        //---new---
                        if (disToAprilTag > 80 && PARAMS.TARGET_TAG_ID == 20) {
                            errorAngleDeg = fid.getTargetXDegrees() - (targetAngle+3);
                        } else if (disToAprilTag > 80 && PARAMS.TARGET_TAG_ID == 24) {
                            errorAngleDeg = fid.getTargetXDegrees() - (targetAngle-3);
                        } else {
                            errorAngleDeg = fid.getTargetXDegrees() - targetAngle;
                        }



                        break;
                    }
                }
            }

            if (tagFound) {
                if (Math.abs(errorAngleDeg) > Params.TOLERANCE_DEG) {
                    double proportionalPower = Params.KP_TURRET * Math.abs(errorAngleDeg);
                    proportionalPower = clamper(proportionalPower, Params.MIN_TURRET_POWER, Params.BASE_TURRET_POWER);
                    turretPower = (errorAngleDeg > 0) ? -proportionalPower : proportionalPower;
                    lastDirection = (errorAngleDeg > 0) ? -1.0 : 1.0;
                } else {
                    turretPower = 0.0;
                }
            } else {
                // simple search motion (non-blocking)
                turretPower = -lastDirection * Params.SEARCH_POWER;
            }
        } else {
            // manual control — direct feedthrough of the stored joystick value
            turretPower = manualTurretInput;
        }

        // finally apply power to turret (CRServo)
        turret.setPower(turretPower);

        telemetryData.tagFound = tagFound;
        telemetryData.errorAngleDeg = errorAngleDeg;
        telemetryData.turretPower = turretPower;
    }

    public void measureDis() {
        if (tagFound) {
            disToAprilTag = (ATHeight - LimelightHeight) / Math.tan((ATAngle + LimelightAngle) * (Math.PI / 180));
        }
    }


    public void calcTargetAngleSpeed() {
        double x = disToAprilTag;
        double shooterAngleSetting;
        double shooterAngle;
        if (tagFound) {
            if (x<82) {
                shooterAngle = 12.1 + (0.544 * x) - (0.00386 * (x * x));
                shooterAngleSetting = (shooterAngle-31.3)/(-12.3);
            } else{
                shooterAngleSetting =0;
            }

            turretAnglePos = clamper(shooterAngleSetting, 0.0, 1.0);

            if (!constantRPM){
                if (disToAprilTag < targetSpeedLinearSplit) {
                    //targetRPM = (disToAprilTag * -11.9) + 3166; //used to be 3266->3166
                    targetRPM = (disToAprilTag * -7) + 3016;
                } else {
                    targetRPM = (disToAprilTag * 17.9) + 1990; //change from 1997 to 1950
                }
            }

        }

    }

    private void updateTurretAngle() {
        // turretAngleCommand changes the stored position incrementally
        if (turretAngleCommand > 0) {
            turretAnglePos += TURRET_ANGLE_STEP;
        } else if (turretAngleCommand < 0) {
            turretAnglePos -= TURRET_ANGLE_STEP;
        }

        turretAnglePos = clamper(turretAnglePos, 0.0, 1.0);
        turretAngle.setPosition(turretAnglePos);

        telemetryData.turretAnglePower = turretAnglePos;
    }

    private void sendTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left RPM", currentRPMLeft);
        packet.put("Right RPM", currentRPMRight);
        packet.put("Target RPM", targetRPM);
        dashboard.sendTelemetryPacket(packet);
        /*
        telemetry.addLine("=== SHOOTER PID ===");
        telemetry.addData("Left Motor RPM", "%.1f", currentRPMLeft);
        telemetry.addData("Right Motor RPM", "%.1f", currentRPMRight);
        telemetry.addData("Left Motor power", "%.2f", leftMotor.getPower());
        telemetry.addData("Right Motor power", "%.2f", rightMotor.getPower());
        telemetry.addData("Left ticks", leftMotor.getCurrentPosition());
        telemetry.addData("Right ticks", rightMotor.getCurrentPosition());
        telemetry.addData("Target RPM", "%.1f", targetRPM);

        telemetry.addLine("=== TURRET TRACKING ===");
        telemetry.addData("Tracking Mode", trackingMode ? "ON" : "OFF");
        telemetry.addData("Tag Visible", telemetryData.tagFound);
        telemetry.addData("Error Angle (deg)", "%.2f", telemetryData.errorAngleDeg);
        telemetry.addData("Turret Power", "%.3f", telemetryData.turretPower);

        telemetry.addLine("=== TURRET ANGLE ===");
        telemetry.addData("Turret Angle Position", "%.3f", telemetryData.turretAnglePower);

        telemetry.update();

         */
    }

    // -------------------------
    // Helpers
    // -------------------------
    private static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static class TelemetryData {
        boolean tagFound = false;
        double errorAngleDeg = 0.0;
        double turretPower = 0.0;
        double turretAnglePower = 0.0;
    }
    public final TelemetryData telemetryData = new TelemetryData();
}

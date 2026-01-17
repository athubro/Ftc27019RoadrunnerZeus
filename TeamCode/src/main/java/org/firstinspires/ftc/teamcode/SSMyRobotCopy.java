package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

/**
 * Reusable Action library for FTC autonomous programs
 * Contains all common robot actions that can be used across different autonomous routines
 */
public class SSMyRobotCopy {

    private final MecanumDrive drive;
    private final TurretCopy turret;
    private final StorageWLoaderCopy storage;
    private final LinearOpMode opMode;

    public SSMyRobotCopy(MecanumDrive drive, TurretCopy turret, StorageWLoaderCopy storage, LinearOpMode opMode) {
        this.drive = drive;
        this.turret = turret;
        this.storage = storage;
        this.opMode = opMode;
        this.turret.autoSpinUp = true;

    }

    // ================================================================
    // TRAJECTORY ACTIONS
    // ================================================================

    /**
     * Build and run a trajectory with heading change
     */
    public class BuildAndRunTrajectory implements Action {
        private final Vector2d targetPos;
        private final double targetHeading;
        private final double tangent;
        private final boolean useSplineToLinearHeading;
        private Action builtAction = null;

        public BuildAndRunTrajectory(Vector2d targetPos, double targetHeading, double tangent) {
            this.targetPos = targetPos;
            this.targetHeading = targetHeading;
            this.tangent = tangent;
            this.useSplineToLinearHeading = true;
        }

        public BuildAndRunTrajectory(Vector2d targetPos, double tangent) {
            this.targetPos = targetPos;
            this.targetHeading = 0;
            this.tangent = tangent;
            this.useSplineToLinearHeading = false;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (builtAction == null) {
                if (useSplineToLinearHeading) {
                    builtAction = drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(new Pose2d(targetPos, targetHeading), tangent)
                            .build();
                } else {
                    builtAction = drive.actionBuilder(drive.localizer.getPose())
                            .splineToConstantHeading(targetPos, tangent)
                            .build();
                }
            }

            // Update subsystems while moving
            turret.update();
            storage.update();
            storage.loadingUpdate();

            return builtAction.run(packet);
        }
    }

    public Action moveToWithHeading(Vector2d targetPos, double targetHeading, double tangent) {
        return new BuildAndRunTrajectory(targetPos, targetHeading, tangent);
    }


    public Action moveToConstantHeading(Vector2d targetPos, double tangent) {
        return new BuildAndRunTrajectory(targetPos, tangent);
    }

    // ================================================================
    // VISION ACTIONS
    // ================================================================

    /**
     * Detect AprilTag motif and update pattern array
     * Looks for tags 21, 22, 23 and sets pattern accordingly
     */
    public class DetectMotif implements Action {
        private final String[] patternArray;
        private boolean initialized = false;

        public DetectMotif(String[] patternArray) {
            this.patternArray = patternArray;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) initialized = true;

            ElapsedTime timer = new ElapsedTime();
            int tagId = -1;

            while (timer.seconds() < 4.0 && tagId == -1 && opMode.opModeIsActive()) {
                // Update subsystems
                turret.update();
                storage.update();
                storage.loadingUpdate();

                LLResult result = turret.limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                        int id = fid.getFiducialId();
                        if (id == 21) {
                            tagId = 21;
                            patternArray[0] = "G";
                            patternArray[1] = "P";
                            patternArray[2] = "P";
                            break;
                        } else if (id == 22) {
                            tagId = 22;
                            patternArray[0] = "P";
                            patternArray[1] = "G";
                            patternArray[2] = "P";
                            break;
                        } else if (id == 23) {
                            tagId = 23;
                            patternArray[0] = "P";
                            patternArray[1] = "P";
                            patternArray[2] = "G";
                            break;
                        }
                    }
                }
                opMode.sleep(50);
            }

            return false;
        }
    }

    public Action detectMotif(String[] patternArray) {
        return new DetectMotif(patternArray);
    }

    // ================================================================
    // TURRET TRACKING ACTIONS
    // ================================================================

    /**
     * Turn on AprilTag tracking and enable shooter
     */
    public class TurnOnTracking implements Action {
        private final double targetAngle;
        private final double waitTime;

        public TurnOnTracking(double targetAngle, double waitTime) {
            this.targetAngle = targetAngle;
            this.waitTime = waitTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turret.setTrackingMode(true);
            turret.setShootingEnabled(true);
            turret.targetAngle = targetAngle;

            // Wait for turret to find target and calculate distance/angle
            ElapsedTime timer = new ElapsedTime();
            while (timer.seconds() < waitTime && !turret.tagFound && opMode.opModeIsActive()) {
                turret.update();
                storage.update();
                storage.loadingUpdate();
                opMode.sleep(50);
            }

            // Once tag is found, calculate distance and set proper RPM/angle
            if (turret.tagFound) {
                turret.measureDis();
                turret.calcTargetAngleSpeed();
            }

            return false;
        }
    }

    public Action turnOnTracking(double targetAngle, double waitTime) {
        return new TurnOnTracking(targetAngle, waitTime);
    }

    public Action turnOnTracking() {
        return new TurnOnTracking(0.0, 2.0);
    }
    public class ReverseTransfer implements Action {
        ElapsedTime generalTimer = new ElapsedTime();
        double timeCap;
        double startTime;

        public ReverseTransfer () {
            timeCap = 1.5;
            generalTimer.reset();
            startTime = generalTimer.seconds();
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            startTime = generalTimer.seconds();
            storage.transferPower(-1);
            pack.put("timer reading ", generalTimer.seconds());
            pack.put("starting time",startTime);

            while (generalTimer.seconds()<startTime+timeCap){

            }
            storage.transferPower(0);
            return false;





        }
    }

    public Action reverseTransfer() {
        return new ReverseTransfer();
    }

    /**
     * Turn off tracking and disable shooter
     */
    public class TurnOffTracking implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            turret.setTrackingMode(false);
            turret.setShootingEnabled(false);
            return false;
        }
    }

    public Action turnOffTracking() {
        return new TurnOffTracking();
    }

    // ================================================================
    // SHOOTING ACTIONS
    // ================================================================

    /**
     * Shoot a specific pattern of balls
     */
    public class ShootPattern implements Action {
        private final String[] pattern;
        private int currentBallIndex = 0;
        private boolean initialized = false;
        private ElapsedTime shootTimer = new ElapsedTime();
        private boolean loadingStarted = false;
        private boolean waitingForShot = false;
        private boolean waitingForReset = false;
        private final double timeoutSeconds;

        public ShootPattern(String[] pattern, double timeoutSeconds) {
            this.pattern = pattern;
            this.timeoutSeconds = timeoutSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                currentBallIndex = 0;
                shootTimer.reset();
                // Ensure shooter stays enabled for entire pattern
                turret.setShootingEnabled(true);
                packet.put("ShootPattern: Status", "Initialized - Starting pattern");
            }

            // CRITICAL: Always update subsystems to keep PID running
            turret.update();
            storage.update();
            storage.loadingUpdate();

            // Add color sensor telemetry

            // Add current ball status
            packet.put("ShootPattern: Current Ball", currentBallIndex + 1);
            packet.put("ShootPattern: Target Color", currentBallIndex < 3 ? pattern[currentBallIndex] : "Complete");
            packet.put("ShootPattern: Timer", String.format("%.2f", shootTimer.seconds()));

            // If we've shot all balls in the pattern, we're done
            if (currentBallIndex >= 3) {
                packet.put("ShootPattern: Status", "COMPLETE - All balls shot");
                return false;
            }

            String targetColor = pattern[currentBallIndex];

            // If we're waiting for kicker reset after a shot
            if (waitingForReset) {
                packet.put("ShootPattern: Status", "Waiting for kicker reset");
                packet.put("ShootPattern: Storage Flag", storage.flag);
                // Wait until storage is ready again
                if (storage.flag) {
                    packet.put("ShootPattern: Status", "Kicker reset complete - Moving to next ball");
                    waitingForReset = false;
                    loadingStarted = false;
                    waitingForShot = false;
                    currentBallIndex++;
                    shootTimer.reset();
                }
                return true;
            }

            // Start loading the ball (only call once)
            if (!loadingStarted && storage.flag && turret.shooterUpToSpeed) {
                packet.put("ShootPattern: Status", "Loading ball: " + targetColor);
                storage.loadBall(targetColor);
                loadingStarted = true;
                shootTimer.reset();
            } else if (!loadingStarted) {
                // Waiting for conditions to start loading
                String waitingReason = "";
                if (!storage.flag) {
                    waitingReason = "Storage not ready";
                } else if (!turret.shooterUpToSpeed) {
                    waitingReason = "Shooter not up to speed";
                }
                packet.put("ShootPattern: Status", "Waiting to load: " + waitingReason);
                packet.put("ShootPattern: Shooter Speed OK", turret.shooterUpToSpeed);
                packet.put("ShootPattern: Storage Ready", storage.flag);
            }

            // Wait for ball to finish loading while continuously updating
            if (loadingStarted && !waitingForShot) {
                packet.put("ShootPattern: Status", "Ball loading in progress");
                if (storage.flag) {
                    // Ball loading complete and kickers are reset
                    packet.put("ShootPattern: Status", "Ball loaded - Ready to shoot");
                    waitingForShot = true;
                    shootTimer.reset();
                }
                return true;
            }

            // If we're waiting for a shot, check if it happened
            if (waitingForShot) {
                packet.put("ShootPattern: Status", "Waiting for shot detection");
                packet.put("ShootPattern: Shot Detected", turret.shotDetected);

                // Wait for shot detection or timeout
                if (turret.shotDetected) {
                    // Shot detected! Wait for kicker reset before next ball
                    packet.put("ShootPattern: Status", "SHOT SUCCESSFUL - Ball " + (currentBallIndex + 1) + " fired");
                    waitingForShot = false;
                    waitingForReset = true;
                    shootTimer.reset();
                } else if (shootTimer.seconds() > timeoutSeconds) {
                    // Timeout - move to next ball anyway
                    packet.put("ShootPattern: Status", "TIMEOUT - No shot detected after " + timeoutSeconds + "s, moving to next");
                    packet.put("ShootPattern: Warning", "Ball " + (currentBallIndex + 1) + " timed out!");
                    waitingForShot = false;
                    waitingForReset = true;
                    shootTimer.reset();
                }
            }

            return true; // Keep running until all balls are shot
        }
    }

    public Action shootPattern(String[] pattern, double timeoutSeconds) {
        return new ShootPattern(pattern, timeoutSeconds);
    }

    public Action shootPattern(String[] pattern) {
        return new ShootPattern(pattern, 5.0);
    }

    /**
     * Shoot a single ball of specific color
     */
    public Action shootSingleBall(String color) {
        return new ShootPattern(new String[]{color}, 5.0);
    }

    // ================================================================
    // INTAKE ACTIONS
    // ================================================================

    /**
     * Start intake at specified power
     */
    public class StartIntake implements Action {
        private final double power;

        public StartIntake(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            storage.setIntakePower(power);
            return false;
        }
    }

    public Action startIntake(double power) {
        return new StartIntake(power);
    }

    public Action startIntake() {
        return new StartIntake(1.0);
    }

    /**
     * Stop intake
     */
    public class StopIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            storage.setIntakePower(0);
            return false;
        }
    }

    public Action stopIntake() {
        return new StopIntake();
    }

    /**
     * Reverse intake (outtake)
     */
    public Action reverseIntake(double power) {
        return new StartIntake(-power);
    }

    public Action reverseIntake() {
        return new StartIntake(-1.0);
    }

    // ================================================================
    // GATE ACTIONS
    // ================================================================

    public class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            storage.openGate();
            return false;
        }
    }

    public Action openGate() {
        return new OpenGate();
    }

    public class CloseGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            storage.closeGate();
            return false;
        }
    }

    public Action closeGate() {
        return new CloseGate();
    }

    // ================================================================
    // UTILITY ACTIONS
    // ================================================================

    /**
     * Wait for a specified duration
     */
    public class Wait implements Action {
        private final double seconds;
        private ElapsedTime timer = null;

        public Wait(double seconds) {
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            // Update subsystems while waiting
            turret.update();
            storage.update();
            storage.loadingUpdate();

            return timer.seconds() < seconds;
        }
    }

    public Action wait(double seconds) {
        return new Wait(seconds);
    }

    /**
     * Update subsystems continuously (useful for parallel actions)
     */
    public class UpdateSubsystems implements Action {
        private final double duration;
        private ElapsedTime timer = null;

        public UpdateSubsystems(double duration) {
            this.duration = duration;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            turret.update();
            storage.update();
            storage.loadingUpdate();

            if (duration <= 0) {
                return true; // Run forever
            }

            return timer.seconds() < duration;
        }
    }

    public Action updateSubsystems(double duration) {
        return new UpdateSubsystems(duration);
    }

    public Action updateSubsystems() {
        return new UpdateSubsystems(-1); // Run forever
    }
}
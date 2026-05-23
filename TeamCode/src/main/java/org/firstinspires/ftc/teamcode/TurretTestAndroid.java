package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "TurretAndroidTest", group = "TeleOp")
public class TurretTestAndroid extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx turret = null;
    //private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        turret  = hardwareMap.get(DcMotorEx.class, "turretMotor");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //turret.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);
        turret.resetDeviceConfigurationForOpMode();
        //turret.set
        //turret.setTargetPosition(turret.getCurrentPosition());
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //turret.setPower(1);
        double kP=10;
        double kI=10;
        double kD=0.;
        double kF=10;
        double vol=100;
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set PIDF coefficients to reduce oscillation
        PIDFCoefficients pidTurret = new PIDFCoefficients(kP, kI, kD, kF);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurret);
        turret.setVelocity(vol);
        //turret.setPositionPIDFCoefficients(PARAMS.turretKP);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftPower;
            //double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            if (gamepad1.dpadUpWasPressed()) {
                kP+=1;
            }
            if (gamepad1.dpadDownWasPressed()) {
                kP-=1;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                kF+=1;
            }
            if (gamepad1.dpadRightWasPressed()) {
                kF-=1;
            }
            if (gamepad2.dpadUpWasPressed()) {
                kI+=1;
            }
            if (gamepad2.dpadDownWasPressed()) {
                kI-=1;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                kD+=1;
            }
            if (gamepad2.dpadRightWasPressed()) {
                kD-=1;
            }

            if (gamepad1.leftBumperWasPressed()){
                vol+=10;
            }
            if (gamepad1.rightBumperWasPressed()){
                vol-=10;
            }
            pidTurret = new PIDFCoefficients(kP, kI, kD, kF);
            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidTurret);
            turret.setVelocity(vol);
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            if (gamepad1.aWasReleased()){
                turret.setTargetPosition(200);

            }

            if (gamepad1.bWasReleased()){
                turret.setTargetPosition(-200);

            }

            if (gamepad1.xWasReleased()){
                turret.setPower(0);

            }
            if (gamepad1.yWasReleased()){
                turret.setPower(1);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("kP, kI, kD, kF ", " (%.2f), (%.2f), (%.2f), (%.2f)", kP,kI,kD,kF);
            telemetry.addData("velocity ", " (%.2f)", vol);
            telemetry.addData("Current Position" , turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
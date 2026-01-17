package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "GoingSomewhere", group = "Autonomous")
public class GoingSomewhere extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors (all lowercase names)
        DcMotorEx armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        DcMotorEx slidemotor = hardwareMap.get(DcMotorEx.class, "slidemotor");
        DcMotorEx slide2motor = hardwareMap.get(DcMotorEx.class, "slide2motor");

        // Reverse second slide
        slide2motor.setDirection(DcMotor.Direction.REVERSE);

        // ARM ACTION (negative is up)
        Action armUp = new Action() {
            boolean started = false;
            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                if (!started) {
                    armmotor.setTargetPosition(-1500);
                    armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armmotor.setPower(0.6);
                    started = true;
                }
                return armmotor.isBusy();
            }
        };

        // SLIDES ACTION (move together)
        Action slidesOut = new Action() {
            boolean started = false;
            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                if (!started) {
                    slidemotor.setTargetPosition(-1500);
                    slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidemotor.setPower(0.5);

                    slide2motor.setTargetPosition(-1500);
                    slide2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide2motor.setPower(0.5);
                    started = true;
                }
                return slidemotor.isBusy() || slide2motor.isBusy();
            }
        };

        // MOVE TO TARGET
        Vector2d targetPos = new Vector2d(40, 36);
        double targetHeading = 45; // radians

        Action moveToTarget = drive.actionBuilder(startPose)
                .splineTo(targetPos, targetHeading)
                .build();

        // SEQUENTIAL ACTIONS
        List<Action> actions = Arrays.asList(
                //armUp,
                //slidesOut,
                moveToTarget
        );

        waitForStart();
        if (isStopRequested()) return;

        for (Action a : actions) {
            Actions.runBlocking(a);
        }

        // Stop all drive motors to make sure robot stops exactly at the target
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);
    }
}

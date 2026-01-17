package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TurretServoTest", group = "TeleOp")
public class TurretServoTest extends LinearOpMode {


    public Servo turretAim;
    public Servo turretVect;
    public double servoPos = 0.1;
    public double vertPos =0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        turretAim = hardwareMap.get(Servo.class, "turret");
        turretVect = hardwareMap.get(Servo.class, "TurretAngle");

        waitForStart();


        while (opModeIsActive()) {

            servoPos -= gamepad1.right_stick_x * 0.001;
            servoPos = clamper(servoPos, 0, 1);
            turretAim.setPosition(servoPos);

            vertPos -= gamepad2.right_stick_x* 0.001;
            vertPos = clamper(vertPos, 0, 1);
            turretVect.setPosition(vertPos);

            // --- Driver Station Telemetry ---
            telemetry.addData("servo postion", servoPos);
            telemetry.addData("vertical postion", vertPos);
            telemetry.update();


        }
    }

    private static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

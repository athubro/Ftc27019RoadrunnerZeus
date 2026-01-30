package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem - controls intake motor and gate servo
 */
public final class Intake {

    // ==================== HARDWARE ====================
    public final DcMotorEx intakeMotor;
    public Servo gate, middleCompartment, topCompartment;
    public final Telemetry telemetry;

    // ==================== CONSTANTS ====================
    private static final double GATE_OPEN = 0.0;
    private static final double GATE_CLOSED = 1.0;

    // ==================== STATE ====================
    private double intakePower = 0.0;
    private boolean gateOpen = false;

    // ==================== CONSTRUCTOR ====================
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        middleCompartment = hardwareMap.get(Servo.class, "middleCompartment");

        topCompartment = hardwareMap.get(Servo.class, "backCompartment");
        middleCompartment.setDirection(Servo.Direction.REVERSE);


        // Set motor to coast when power is 0
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize gate to closed position
        gate.setPosition(GATE_CLOSED);
    }

    // ==================== PUBLIC API ====================

    /**
     * Set intake motor power
      power Motor power from -1.0 to 1.0 (typically gamepad trigger value)
     */
    public void setIntakePower(double power) {
        this.intakePower = power;
        intakeMotor.setPower(power);
    }

    /**
     * Open the gate (position 0)
     */
    public void openGate() {
        gateOpen = true;
        gate.setPosition(GATE_OPEN);
    }

    /**
     * Close the gate (position 1)
     */
    public void closeGate() {
        gateOpen = false;
        gate.setPosition(GATE_CLOSED);
    }

    /**
     * Set gate position directly
     *open true to open, false to close
     */
    public void setGate(boolean open) {
        if (open) {
            openGate();
        } else {
            closeGate();
        }
    }

    /**
     * Toggle gate between open and closed
     */
    public void toggleGate() {
        if (gateOpen) {
            closeGate();
        } else {
            openGate();
        }
    }

    //0 position is normal pass through positions
    //1 is stored position
    public void storeMiddle(){
        middleCompartment.setPosition(1);
    }
    public void storeTop(){
        topCompartment.setPosition(1);
    }
    public void resetMiddle(){
        middleCompartment.setPosition(0);
    }
    public void resetTop(){
        topCompartment.setPosition(0);
    }
    public void resetAll(){
        middleCompartment.setPosition(0);
        topCompartment.setPosition(0);
    }



    /**
     * Get current intake power
     */
    public double getIntakePower() {
        return intakePower;
    }

    /**
     * Check if gate is open
     */
    public boolean isGateOpen() {
        return gateOpen;
    }

    /**
     * Get current gate position
     */
    public double getGatePosition() {
        return gate.getPosition();
    }

    /**
     * Stop intake motor
     */
    public void stop() {
        setIntakePower(0.0);
    }
}
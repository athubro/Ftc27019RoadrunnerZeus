package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Intake subsystem - controls intake motor and gate servo
 */
public final class Intake {
    //==================== TIMER =============
    private ElapsedTime generalTimer = new ElapsedTime();

    //==================== SORTING ===========

    public String[] storage = {"N", "N", "N"};

    boolean ballsStored = false;
//---------------front sensor -----------------
    double frontColorHuePurpleBall = 227;
    double frontColorHueGreenBall = 158;
    double frontColorHueUpperBound = 6;
    double frontColorHueLowerBound = -6;
    double frontColorDis = 3.2;
    //-----------middle sensor ------------
    double middleColorHuePurpleBall = 181;
    double middleColorHueGreenBall = 152;

    double middleColorHueUpperBound = 15;
    double middleColorHueLowerBound = -15;
    double middleColorDis = 12;
    //------------- back sensor -------------
    double backColorHuePurpleBall = 195;
    double backColorHueGreenBall = 160;
    double backColorHueUpperBound = 10;
    double backColorHueLowerBound = -10;
    double backColorDis = 5.5;

    // ==================== HARDWARE ====================
    public final DcMotorEx intakeMotor;
    public Servo gate, middleCompartment, topCompartment;
    public final Telemetry telemetry;


    //-------------Color Sensor
    private ColorSensor middleColor_REV_ColorRangeSensor;
    private ColorSensor frontColor_REV_ColorRangeSensor;
    private ColorSensor backColor_REV_ColorRangeSensor;
    private ColorSensor backColor;
    private ColorSensor middleColor;
    private ColorSensor frontColor;

    NormalizedRGBA normalizedBackColor;
    NormalizedRGBA normalizedMiddleColor;
    NormalizedRGBA normalizedFrontColor;
    int myFrontColor;
    int myMiddleColor;
    int myBackColor;



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



        middleColor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "middleColor");
        frontColor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "frontColor");
        backColor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "backColor");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");
        middleColor = hardwareMap.get(ColorSensor.class, "middleColor");
        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");

        // Set motor to coast when power is 0
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize gate to closed position
        gate.setPosition(GATE_CLOSED);

        resetAll();
    }

    // ==================== PUBLIC API ====================


    //======================MAIN UPDATE ====================

    public void storageUpdate () {
        normalizedBackColor = ((NormalizedColorSensor) backColor).getNormalizedColors();
        normalizedMiddleColor = ((NormalizedColorSensor) middleColor).getNormalizedColors();
        normalizedFrontColor = ((NormalizedColorSensor) frontColor).getNormalizedColors();
        myFrontColor = normalizedFrontColor.toColor();
        myMiddleColor = normalizedMiddleColor.toColor();
        myBackColor = normalizedBackColor.toColor();
        // bottom (front of robot) is 0
        storage[0] = detectColor(((DistanceSensor) frontColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM), JavaUtil.colorToHue(myFrontColor), "front");
        storage[1] = detectColor(((DistanceSensor) middleColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM), JavaUtil.colorToHue(myMiddleColor), "middle");
        storage[2] = detectColor(((DistanceSensor) backColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM), JavaUtil.colorToHue(myBackColor), "back");

        //telemetry.addData("storage",storage);
       // telemetry.update();
    }

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

    private String detectColor (double dis, double hue, String slot) {
        double disCheck = 10;
        double hueCheckPurpleBall = 150;
        double hueCheckGreenBall = 150;

        double hueUpperBound = 5;
        double hueLowerBound = -5;
        if (slot.equals("back")) {
            disCheck = backColorDis;
            hueCheckPurpleBall = backColorHuePurpleBall;
            hueCheckGreenBall = backColorHueGreenBall;

            hueUpperBound = backColorHueUpperBound;
            hueUpperBound = backColorHueLowerBound;

        } else if (slot.equals("middle")) {
            disCheck = middleColorDis;
            hueCheckPurpleBall = middleColorHuePurpleBall;
            hueCheckGreenBall = middleColorHueGreenBall;
            hueUpperBound = middleColorHueUpperBound;
            hueUpperBound = middleColorHueLowerBound;

        } else if (slot.equals("front")) {
            disCheck = frontColorDis;
            hueCheckPurpleBall = frontColorHuePurpleBall;
            hueCheckGreenBall = frontColorHueGreenBall;
            hueUpperBound = frontColorHueUpperBound;
            hueUpperBound = frontColorHueLowerBound;

        }

        if (dis < disCheck && hue < hueCheckPurpleBall + hueUpperBound && hue > hueCheckPurpleBall + hueLowerBound) {
            return "P";
        } else if (dis < disCheck && hue < hueCheckGreenBall + hueUpperBound && hue > hueCheckGreenBall + hueLowerBound) {
            return "G";
        } else if (dis < disCheck) {
            return "U";
        } else {
            return "N";
        }
    }
    public void storeBalls (String[] target, String[] currentArray) {
        if (!ballsStored) {

        }
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
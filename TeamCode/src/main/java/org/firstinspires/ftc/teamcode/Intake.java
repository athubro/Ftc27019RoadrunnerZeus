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

import java.util.Arrays;

/**
 * Intake subsystem - controls intake motor and gate servo
 */
public final class Intake {
    //==================== TIMER =============
    private ElapsedTime generalTimer = new ElapsedTime();

    //==================== SORTING ===========

    public String[] storage = {"N", "N", "N"};

    public boolean finishedStoring = true;

    public double ballCount = 0;

    boolean ballsStored = false;

    //B is both storage reset, T is top one reset, M is middle reset, and N is no step
    String firstStep = "N";
    String secondStep = "N";


    //---------------front sensor -----------------
    double frontColorHuePurpleBall = 227;
    double frontColorHueGreenBall = 158;
    double frontColorHueUpperBound = 20;
    double frontColorHueLowerBound = -20;
    double frontColorDis = 3.2;
    //-----------middle sensor ------------
    double middleColorHuePurpleBall = 176;
    double middleColorHueGreenBall = 152;

    double middleColorHueUpperBound = 15;
    double middleColorHueLowerBound = -15;
    double middleColorDis = 12;
    //------------- back sensor -------------
    double backColorHuePurpleBall = 200;
    double backColorHueGreenBall = 160;
    double backColorHueUpperBound = 10;
    double backColorHueLowerBound = -10;
    double backColorDis = 5.5;



    public double frontHue;
    public double frontDis;
    public double middleHue;
    public double middleDis;
    public double backHue;
    public double backDis;

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


        ((NormalizedColorSensor) middleColor_REV_ColorRangeSensor).setGain(80);
        ((NormalizedColorSensor) frontColor_REV_ColorRangeSensor).setGain(70);
        ((NormalizedColorSensor) backColor_REV_ColorRangeSensor).setGain(80);

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

        frontHue = JavaUtil.colorToHue(myFrontColor);
        frontDis = ((DistanceSensor) frontColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM);
        middleHue = JavaUtil.colorToHue(myMiddleColor);
        middleDis = ((DistanceSensor) middleColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM);;
        backHue = JavaUtil.colorToHue(myBackColor);
        backDis = ((DistanceSensor) backColor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM);;
        // bottom (front of robot) is 0
        storage[2] = detectColor(frontDis, frontHue , "front");
        storage[1] = detectColor(middleDis,middleHue, "middle");
        storage[0] = detectColor(backDis, backHue, "back");
        countBalls();

        //telemetry.addData("storage",storage);
        // telemetry.update();
    }

    public void generalTimerReset() {
        generalTimer.reset();
    }

    public void countBalls() {
        ballCount=0;
        for (int i = 0; i < 3; i++) {
            if (!storage[i].equals("N")) {
                ballCount++;
            }
        }
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
            hueLowerBound = backColorHueLowerBound;

        } else if (slot.equals("middle")) {
            disCheck = middleColorDis;
            hueCheckPurpleBall = middleColorHuePurpleBall;
            hueCheckGreenBall = middleColorHueGreenBall;
            hueUpperBound = middleColorHueUpperBound;
            hueLowerBound = middleColorHueLowerBound;

        } else if (slot.equals("front")) {
            disCheck = frontColorDis;
            hueCheckPurpleBall = frontColorHuePurpleBall;
            hueCheckGreenBall = frontColorHueGreenBall;
            hueUpperBound = frontColorHueUpperBound;
            hueLowerBound = frontColorHueLowerBound;

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
    public void storeBalls (String[] target) {
        String[] output = {"N", "N", "N"};
        //to shoot (first ball to shoot);
        String firstBall = "N";
        if (!ballsStored) {
            if ((storage[0].equals(storage[1]) && storage[0].equals(storage[2]))) {
                firstStep = "N";
                secondStep = "N";
                ballsStored = false;
                finishedStoring = true;
            } else {
                //======check if store first ball works
                output[0] = storage[1];
                output[1] = storage[2];
                output[2] = storage[0];
                if (Arrays.equals(output, target)) {
                    storeTop();
                    firstStep = "T";
                    ballsStored = true;
                    finishedStoring = true;

                } else { //----------storing first ball doesnt work, checking if storing only second ball works
                    output[0] = storage[0];
                    output[1] = storage[2];
                    output[2] = storage[1];
                    if (Arrays.equals(output, target)) {
                        storeMiddle();
                        firstStep = "M";
                        secondStep = "N";
                        ballsStored = true;
                        finishedStoring = true;
                    } else { //========finally, checks if moving both balls to the back works


                        output[0] = storage[2];
                        output[1] = storage[0];
                        output[2] = storage[1];
                        if (Arrays.equals(output, target)) {
                            storeTop();
                            storeMiddle();
                            firstStep = "B";
                            secondStep = "N";
                            ballsStored = true;
                            finishedStoring = true;


                        } else { // checks if you can store first 2 balls, then put second back in before first
//========finally, checks if moving both balls to the back works
                            output[0] = storage[2];
                            output[1] = storage[1];
                            output[2] = storage[0];
                            if (Arrays.equals(output, target)) {
                                storeTop();
                                storeMiddle();
                                firstStep = "M";
                                secondStep = "T";
                                ballsStored = true;
                                finishedStoring = true;
                            } else { //======================IF NOTHING WORKSSSSS====
                                firstStep = "N";
                                secondStep = "N";
                                ballsStored = false;
                                finishedStoring = true;
                            }

                        }
                    }
                }

            }





        }
    }


    public void executeNextStep() {
        if (!firstStep.equals("N")) {
            if (firstStep.equals("T")) {
                resetTop();

            } else if (firstStep.equals("M")) {
                resetMiddle();

            } else if (firstStep.equals("B")) {
                resetAll();

            }
            firstStep = "N";
        } else {
            if (!secondStep.equals("N")) {
                if (secondStep.equals("T")) {
                    resetTop();

                } else if (secondStep.equals("M")) {
                    resetMiddle();

                } else if (secondStep.equals("B")) {
                    resetAll();

                }
                secondStep = "N";
                ballsStored = false;
            }
            ballsStored = false;
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
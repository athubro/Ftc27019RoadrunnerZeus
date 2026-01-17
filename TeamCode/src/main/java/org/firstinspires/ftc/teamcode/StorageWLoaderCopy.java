package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class StorageWLoaderCopy {
    public static class Params {
        public static double backDisCheck = 7;
        public static double frontDisCheck = 3.55; //3.7--->3.4
        public static double middleDisCheck = 4.6;//change from 5 to 4.5 ->4.6
        public static double backGreenRatio = 3.0;
        public static double middleGreenRatio = 2.7;
        public static double frontGreenRatio = 3.75;
        //public static double boundaryGreen = 0.05;
        //--------------------------------------------------------front boundaries------------------------------
        public static double Front_GreenRedRatio_UpperBound_PurpleBall = 0.05; //0.055-->0.03
        //public static double boundaryUPPurple = 0.03;
        public static double Front_GreenRedRatio_LowerBound_PurpleBall = -0.03;//0.015

        public static double Front_BlueRedRatio_UpperBound_PurpleBall = 0.04;

        public static double Front_BlueRedRatio_LowerBound_PurpleBall = -0.04; //-0.02

        public static double Front_GreenRedRatio_UpperBound_GreenBall = 0.05;
        //public static double boundaryUPPurple = 0.03;
        public static double Front_GreenRedRatio_LowerBound_GreenBall = -0.05;

        public static double Front_BlueRedRatio_UpperBound_GreenBall = 0.05;

        public static double Front_BlueRedRatio_LowerBound_GreenBall = -0.05;

        //---------------------------------------------------middle Boundaries----------------------------------
        public static double Middle_GreenRedRatio_UpperBound_PurpleBall = 0.03;
        //public static double boundaryUPPurple = 0.03;
        public static double Middle_GreenRedRatio_LowerBound_PurpleBall = -0.03;

        public static double Middle_BlueRedRatio_UpperBound_PurpleBall = 0.04;

        public static double Middle_BlueRedRatio_LowerBound_PurpleBall = -0.04;

        public static double Middle_GreenRedRatio_UpperBound_GreenBall = 0.05;
        //public static double boundaryUPPurple = 0.03;
        public static double Middle_GreenRedRatio_LowerBound_GreenBall = -0.05;

        public static double Middle_BlueRedRatio_UpperBound_GreenBall = 0.05;

        public static double Middle_BlueRedRatio_LowerBound_GreenBall = -0.05;

        //---------------------------------------------------------Back Boundaries------------------------------------

        public static double Back_GreenRedRatio_UpperBound_PurpleBall = 0.03;
        //public static double boundaryUPPurple = 0.03;
        public static double Back_GreenRedRatio_LowerBound_PurpleBall = -0.03;

        public static double Back_BlueRedRatio_UpperBound_PurpleBall = 0.04;

        public static double Back_BlueRedRatio_LowerBound_PurpleBall = -0.04;

        public static double Back_GreenRedRatio_UpperBound_GreenBall = 0.05;
        //public static double boundaryUPPurple = 0.03;
        public static double Back_GreenRedRatio_LowerBound_GreenBall = -0.05;

        public static double Back_BlueRedRatio_UpperBound_GreenBall =0.05;

        public static double Back_BlueRedRatio_LowerBound_GreenBall = -0.05;

        //-----------------------------------------------------------------------------------------------------
        public static double boundaryLOWPurple = 0.04;
        public static double blueRedRatioGreen = 2.35;
        public static double blueRedRatioPurple = 2.02;
        public static double greenRedRatioPurple = 1.1;

    }

    private static double midPos = 0.6;
    private static double shortDelay = 0.03; //s
    private static double longDelay =1; //s
    private static double gateMidPOs =0.6;

    private static double gateDelay = 0.1 ; //s
    private static double frontGateDelay = 0.5;

    public boolean kick2StepFlag=false  ;
    public double redReading1 = 0;
    public double greenReading1 = 0;
    public double blueReading1=0;
    public double redReading2 = 0;
    public double greenReading2 = 0;
    public double blueReading2=0;
    public double redReading3 = 0;
    public double greenReading3 = 0;
    public double blueReading3=0;

    public boolean flagLoadingMiddle = false;
    public boolean flagLoadingBack=false;
    public boolean flagLoadingFront = false;

    private String[] firstColorReadings = {"N","N","N"}; // same formate as ball array index 0 is front
    private String[] secondColorReadings = {"N","N","N"};
    private String[] thirdColorReadings = {"N","N","N"};




    public static StorageWLoaderCopy.Params PARAMS = new StorageWLoaderCopy.Params();
    private Servo frontKick, middleKick, backKick, gate;
    private CRServo transferServo;

    public Servo rgbIndicator;
    private TurretCopy myTurret;

    private ColorSensor frontColorSensor, middleColorSensor, backColorSensor;
    private DistanceSensor frontDisSensor, middleDisSensor, backDisSensor;
    private NormalizedRGBA normalizedColors;
    private DcMotor activeIntake;
    boolean flag = true; //loader is available to load balls, when it is true;

    public int count;

    private boolean intakeOn = false;
    private boolean intakeInward = false; //false is outward, true is inward;

    public String kickTarget="";
    public double timeRN = 0;

    public double frontPos = 0;
    public double middlePos = 0;
    public double backPos = 0;

    public double dis =0;

    public boolean kickUp = false;
    private double gateNextTrigger = -1;
    private double gateNextPos=0;
    private double frontLoaderNextTrigger = -1;
    private double frontLoaderNextPos=0;
    private double midLoaderNextTrigger=-1;
    private double midLoaderNextPos=0;
    private double backLoaderNextTrigger=-1;
    private double backLoaderNextPos=0;

    public double changeFlagTrigger=-1;
    private boolean nextFlagValue=true;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    public ElapsedTime generalTimer = new ElapsedTime();
    private double waitForLoaderResetTimer =0;
    private final double resetDelay =0.8; //seconds to wait after reset the kicker;
    private boolean resetting=false;
    public String[] ballArray = {"N", "N", "N"};
    public String frontBall = "N";
    public String middleBall = "N";
    public String backBall = "N";
    // 3-ball cases (only move the kicker at the target ball)
    private void kickFront3()  {
        double t=generalTimer.seconds();
        gateNextPos=1;
        gateNextTrigger= t;
        frontLoaderNextTrigger=t+frontGateDelay;
        kickFront();
    }

    private void kickFront3w2Step() {kickFront3();}
    //private void kickFront3w2Step()  { gate.setPosition(1); sleep(gateDelay);frontKick.setPosition(midPos); sleep(longDelay);        kickFront();  }

    private void kickMiddle3() {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        midLoaderNextTrigger=t+gateDelay;
        kickMiddle();
    }

    private void kickMiddle3w2Step(){kickMiddle3();}
    //private void kickMiddle3w2Step() {gate.setPosition(gateMidPOs); sleep(gateDelay); middleKick.setPosition(midPos);  sleep(longDelay);         kickMiddle(); }

    private void kickBack3()   {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        backLoaderNextTrigger=t+gateDelay;
        kickBack();
    }

    private void kickBack3w2Step (){kickBack3();}
    //private void kickBack3w2Step ()   {gate.setPosition(gateMidPOs); sleep(gateDelay); backKick.setPosition(midPos); sleep(longDelay); kickBack(); }

    // 2-ball targeted kicks
    private void kickBack2Front()   {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        backLoaderNextTrigger=t+gateDelay;
        kickBack();
        midLoaderNextTrigger=t+gateDelay+shortDelay;
        kickMiddle();

    }

    private void kickBack2Front_2Step() {kickBack2Front();}
    //private void kickBack2Front_2Step()   { gate.setPosition(gateMidPOs); sleep(gateDelay); backKick.setPosition(midPos); sleep(shortDelay); middleKick.setPosition(midPos);   sleep(longDelay);kickBack(); sleep(shortDelay); kickMiddle();}
    private void kickBack2Middle()  {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        backLoaderNextTrigger=t+gateDelay;
        kickBack();
        frontLoaderNextTrigger=t+gateDelay+shortDelay;
        kickFront();
    }

    private void kickBack2Middle_2Step(){ kickBack2Middle() ;}
    //private void kickBack2Middle_2Step()  {gate.setPosition(gateMidPOs); sleep(gateDelay); backKick.setPosition(midPos); sleep(shortDelay); frontKick.setPosition(midPos);      sleep(longDelay); kickBack(); sleep(shortDelay); kickFront();}
    private void kickMiddle2Front() {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        midLoaderNextTrigger=t+gateDelay;
        kickMiddle();
        backLoaderNextTrigger=t+gateDelay+shortDelay;
        kickBack();
    }

    public void transferPower(double power) {
        transferServo.setPower(power);
    }
    private void kickMiddle2Front_2Step(){kickMiddle2Front();}
    //private void kickMiddle2Front_2Step() { gate.setPosition(gateMidPOs); sleep(gateDelay); middleKick.setPosition(midPos); sleep(shortDelay); backKick.setPosition(midPos);    sleep(longDelay); kickMiddle(); sleep(shortDelay); kickBack();}
    private void kickMiddle2Back()  {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        midLoaderNextTrigger=t+gateDelay;
        kickMiddle();
        frontLoaderNextTrigger=t+gateDelay+shortDelay;
        kickFront();
    }

    private void kickMiddle2Back_2Step(){kickMiddle2Back();}
    //private void kickMiddle2Back_2Step()  { gate.setPosition(gateMidPOs); sleep(gateDelay); middleKick.setPosition(midPos); sleep(shortDelay); frontKick.setPosition(midPos);    sleep(longDelay); kickMiddle(); sleep(shortDelay); kickFront();}
    private void kickFront2Back()   {
        double t=generalTimer.seconds();
        gateNextPos=1;
        gateNextTrigger= t;
        frontLoaderNextTrigger=t+frontGateDelay;
        kickFront();
        midLoaderNextTrigger=t+frontGateDelay+shortDelay*4;
        kickMiddle();
    }
    private void kickFront2Back_2Step(){kickFront2Back();}
    //private void kickFront2Back_2Step()   { gate.setPosition(1); sleep(gateDelay); frontKick.setPosition(midPos); sleep(shortDelay); middleKick.setPosition(midPos);   sleep(longDelay); kickFront(); sleep(shortDelay); kickMiddle();}
    private void kickFront2Middle() {
        double t=generalTimer.seconds();
        gateNextPos=1;
        gateNextTrigger= t;
        frontLoaderNextTrigger=t+frontGateDelay;
        kickFront();
        backLoaderNextTrigger=t+frontGateDelay+shortDelay*4;
        kickBack();
    }
    private void kickFront2Middle_2Step(){kickFront2Middle() ;}
    //private void kickFront2Middle_2Step() { gate.setPosition(1); sleep(gateDelay);frontKick.setPosition(midPos); sleep(shortDelay); backKick.setPosition(midPos);       sleep(longDelay);kickFront(); sleep(shortDelay); kickBack();}

    // 1-ball cases (have to actuate all downstream kickers)
    private void kickFront1()  {
        double t=generalTimer.seconds();
        gateNextPos=1;
        gateNextTrigger= t;
        frontLoaderNextTrigger=t+frontGateDelay;
        kickFront();
        midLoaderNextTrigger=t+frontGateDelay+shortDelay*4;
        kickMiddle();
        backLoaderNextTrigger=t+frontGateDelay+5*shortDelay;
        kickBack();

    }
    private void kickFront1_2Step(){kickFront1();}
    //private void kickFront1_2Step()  {gate.setPosition(1); sleep(gateDelay);  frontKick.setPosition(midPos); sleep(shortDelay); backKick.setPosition(midPos); sleep(shortDelay); middleKick.setPosition(midPos);    sleep(longDelay);  kickFront(); sleep(shortDelay); kickMiddle(); sleep(shortDelay); kickBack();  }
    private void kickMiddle1() {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        midLoaderNextTrigger=t+gateDelay;
        kickMiddle();
        backLoaderNextTrigger=t+gateDelay+shortDelay;
        kickBack();
        frontLoaderNextTrigger=t+gateDelay+2*shortDelay;
        kickFront();
    }

    private void kickMiddle1_2Step(){kickMiddle1();}
    //private void kickMiddle1_2Step() { gate.setPosition(gateMidPOs); sleep(gateDelay);  middleKick.setPosition(midPos); sleep(shortDelay); backKick.setPosition(midPos); sleep(shortDelay); frontKick.setPosition(midPos);   sleep(longDelay);  kickMiddle(); sleep(shortDelay);kickBack(); sleep(shortDelay); kickFront();}
    private void kickBack1()   {
        double t=generalTimer.seconds();
        gateNextPos=gateMidPOs;
        gateNextTrigger= t;
        backLoaderNextTrigger=t+gateDelay;
        kickBack();
        midLoaderNextTrigger=t+gateDelay+shortDelay;
        kickMiddle();
        frontLoaderNextTrigger=t+gateDelay+2*shortDelay;
        kickFront();
    }
    private void kickBack1_2Step(){kickBack1();}
    //private void kickBack1_2Step()   { gate.setPosition(gateMidPOs); sleep(gateDelay); backKick.setPosition(midPos); sleep(shortDelay); middleKick.setPosition(midPos); sleep(shortDelay); frontKick.setPosition(midPos);    sleep(longDelay); kickBack(); sleep(shortDelay); kickMiddle(); sleep(shortDelay); kickFront(); }


    public StorageWLoaderCopy(HardwareMap hardwareMap, TurretCopy turret) {

        myTurret=turret;
        frontKick = hardwareMap.get(Servo.class, "frontKick");
        middleKick = hardwareMap.get(Servo.class, "middleKick");
        backKick = hardwareMap.get(Servo.class, "backKick");
        gate = hardwareMap.get(Servo.class, "gate");
        transferServo = hardwareMap.get(CRServo.class, "transfer");
        activeIntake = hardwareMap.get(DcMotor.class, "activeIntake");

        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");
        // backKick : deepest slot
        // middleKick : 2nd deepest slot
        //frontKick : closest to intake slot
        middleKick.setDirection(Servo.Direction.REVERSE);

        frontKick.setPosition(0);
        middleKick.setPosition(0);
        backKick.setPosition(0);

        frontColorSensor = hardwareMap.get(ColorSensor.class, "frontColor");
        middleColorSensor = hardwareMap.get(ColorSensor.class, "middleColor");
        backColorSensor = hardwareMap.get(ColorSensor.class, "backColor");

        frontDisSensor = hardwareMap.get(DistanceSensor.class, "frontColor");
        middleDisSensor = hardwareMap.get(DistanceSensor.class, "middleColor");
        backDisSensor = hardwareMap.get(DistanceSensor.class, "backColor");

        ((NormalizedColorSensor) frontColorSensor).setGain(82);
        ((NormalizedColorSensor) middleColorSensor).setGain(82);
        ((NormalizedColorSensor) backColorSensor).setGain(82);
    }
    public String determineColor(String color, String index) {
        int arrayindex=-1;
        if (index.equals("front")) {
            arrayindex = 0;
        } else if (index.equals("middle")) {
            arrayindex = 1;

        } else if (index.equals("back")) {
            arrayindex = 2;

        }

        firstColorReadings[arrayindex] = secondColorReadings[arrayindex];
        secondColorReadings[arrayindex] = thirdColorReadings[arrayindex];
        thirdColorReadings[arrayindex] = color;
        if (firstColorReadings[arrayindex].equals(secondColorReadings[arrayindex]) && secondColorReadings[arrayindex].equals(thirdColorReadings[arrayindex])) {
            return color;
        } else {
            return ballArray[arrayindex];
        }
    }
    // Update the color in each slot
    public void update() {
        String f = ballArray[0] = determineColor(detectColor(frontColorSensor, frontDisSensor, "front"), "front");
        String m = ballArray[1] = determineColor(detectColor(middleColorSensor, middleDisSensor, "middle"), "middle");

        String b = ballArray[2] = determineColor(detectColor(backColorSensor, backDisSensor, "back"), "back");

        dis = frontDisSensor.getDistance(DistanceUnit.CM);
        // Count how many balls are currently in storage
        count = 0;
        if (f.equals("N")) count++;
        if (m.equals("N")) count++; //count is how many empty spaces
        if (b.equals("N")) count++;
        timeRN=timer.time();
        double timeCap = 2.5;
        frontPos = frontKick.getPosition();
        middlePos = middleKick.getPosition();
        backPos = backKick.getPosition();

        if (!flag) {
            if (pastTime(timeCap)) {
                resetKick();
            }

        }

    }



    public void loadingUpdate() {
        if (gateNextTrigger>0 &&generalTimer.seconds()>gateNextTrigger){
            gate.setPosition(gateNextPos);
            gateNextTrigger=-1;
        }

        if (frontLoaderNextTrigger>0 &&  generalTimer.seconds()>frontLoaderNextTrigger){
            frontKick.setPosition(frontLoaderNextPos);
            frontLoaderNextTrigger=-1;
        }
        if (midLoaderNextTrigger>0 &&  generalTimer.seconds()>midLoaderNextTrigger){
            middleKick.setPosition(midLoaderNextPos);
            midLoaderNextTrigger=-1;
        }

        if (backLoaderNextTrigger>0 &&  generalTimer.seconds()>backLoaderNextTrigger){
            backKick.setPosition(backLoaderNextPos);
            backLoaderNextTrigger=-1;
        }

        if (changeFlagTrigger>0 &&  generalTimer.seconds()>changeFlagTrigger){
            flag=nextFlagValue;
            changeFlagTrigger=-1;
        }
        autoLoad();
        resetAfterLoad();
    }

    public void color (double rgb) {
        rgbIndicator.setPosition(rgb);
    }

    private String detectBall (DistanceSensor distanceSensor, String slot) {
        String ball = "";
        double distance = distanceSensor.getDistance(DistanceUnit.CM);


        if (slot.equals("back")) {
            if (distance < 7) {
                ball = "Y";
            } else {
                ball = "N";
            }
        } else if (slot.equals("middle")) {
            if (distance < 5) {
                ball = "Y";
            } else {
                ball = "N";
            }

        } else if (slot.equals("front")) {
            if (distance < 3) {
                ball = "Y";
            } else {
                ball = "N";
            }

        }
        return ball;

    }
    public void resetAfterLoad(){
        if (myTurret.shotDetected && kickUp){
            resetKickers();
            myTurret.setShootingEnabled(false);
            waitForLoaderResetTimer=generalTimer.seconds();
        }
        if (resetting&&(generalTimer.seconds()-waitForLoaderResetTimer>resetDelay)){
            flag=true;
            resetting=false;
            //kickUp=false;
        }
    }
    public void shotDetectReset() {
        if (kickUp && myTurret.shotDetected) {
            resetKick();
            myTurret.setShootingEnabled(false);
            myTurret.shotDetected = false;
        }
    }
    public boolean isKickUp (String target) {
        double floor = 0.9;
        if (target.equals("front")) {
            if (frontKick.getPosition() > floor) {
                return true;
            } else {
                return false;
            }
        } else if (target.equals("middle")) {
            if (middleKick.getPosition() > floor) {
                return true;
            } else {
                return false;
            }

        } else if (target.equals("back")) {
            if (backKick.getPosition() > floor) {
                return true;
            } else {
                return false;
            }

        } else {
            return false;
        }

    }

    public boolean pastTime (double seconds) {
        if (seconds < timer.time() && changeFlagTrigger <0) {
            return true;
        } else {
            return false;

        }
    }

    public void setIntakePower(double inputPower) {
        if (inputPower > 0.1) {
            gate.setPosition(0);
            activeIntake.setPower(inputPower);
            intakeOn = true;
            intakeInward = true;


            transferServo.setPower(1);
        } else if (inputPower < -0.1){
            activeIntake.setPower(inputPower);
            transferServo.setPower(0);
            intakeOn = true;
            intakeInward = false;
        }
        else {
            if(intakeOn && intakeInward) {
                gate.setPosition(0.6);
            }
            intakeOn=false;
            transferServo.setPower(0);
            activeIntake.setPower(0);
        }
    }
    public void openGate(){
        gate.setPosition(0);
    }
    public void closeGate(){
        gate.setPosition(0.5);
    }
    private String detectColor(ColorSensor colorSensor, DistanceSensor distanceSensor, String slot) {
        normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        float R = normalizedColors.red;
        float G = normalizedColors.green;
        float B = normalizedColors.blue;

        double DisCheck = 3.7, greenRatio = 4.22;
        double GR_LBound_P = 0.03;
        double GR_UBound_P = 0.03;
        double BR_LBound_P = 0.03;
        double BR_UBound_P = 0.03;

        double GR_LBound_G = 0.03;
        double GR_UBound_G = 0.03;
        double BR_LBound_G = 0.03;
        double BR_UBound_G = 0.03;

        if (slot.equals("back")) { DisCheck = PARAMS.backDisCheck; greenRatio = PARAMS.backGreenRatio; }
        if (slot.equals("back")) {
            GR_LBound_P = PARAMS.Back_GreenRedRatio_LowerBound_PurpleBall;
            GR_UBound_P = PARAMS.Back_GreenRedRatio_UpperBound_PurpleBall;
            BR_LBound_P = PARAMS.Back_BlueRedRatio_LowerBound_PurpleBall;
            BR_UBound_P = PARAMS.Back_BlueRedRatio_UpperBound_PurpleBall;

            GR_LBound_G = PARAMS.Back_GreenRedRatio_LowerBound_GreenBall;
            GR_UBound_G = PARAMS.Back_GreenRedRatio_UpperBound_GreenBall;
            BR_LBound_G = PARAMS.Back_BlueRedRatio_LowerBound_GreenBall;
            BR_UBound_G = PARAMS.Back_BlueRedRatio_UpperBound_GreenBall;
            redReading3=R;
            blueReading3=B;
            greenReading3=G;
        }
        //---------------------------------------------------------------------------------------------------------
        if (slot.equals("middle")) { DisCheck = PARAMS.middleDisCheck; greenRatio = PARAMS.middleGreenRatio; }
        if (slot.equals("middle")) {
            GR_LBound_P = PARAMS.Middle_GreenRedRatio_LowerBound_PurpleBall;
            GR_UBound_P = PARAMS.Middle_GreenRedRatio_UpperBound_PurpleBall;
            BR_LBound_P = PARAMS.Middle_BlueRedRatio_LowerBound_PurpleBall;
            BR_UBound_P = PARAMS.Middle_BlueRedRatio_UpperBound_PurpleBall;

            GR_LBound_G = PARAMS.Middle_GreenRedRatio_LowerBound_GreenBall;
            GR_UBound_G = PARAMS.Middle_GreenRedRatio_UpperBound_GreenBall;
            BR_LBound_G = PARAMS.Middle_BlueRedRatio_LowerBound_GreenBall;
            BR_UBound_G = PARAMS.Middle_BlueRedRatio_UpperBound_GreenBall;
            redReading2=R;
            blueReading2=B;
            greenReading2=G;
        }
        //---------------------------
        if (slot.equals("front")) { DisCheck = PARAMS.frontDisCheck; greenRatio = PARAMS.frontGreenRatio;}
        if (slot.equals("front")) {
            GR_LBound_P = PARAMS.Front_GreenRedRatio_LowerBound_PurpleBall;
            GR_UBound_P = PARAMS.Front_GreenRedRatio_UpperBound_PurpleBall;
            BR_LBound_P = PARAMS.Front_BlueRedRatio_LowerBound_PurpleBall;
            BR_UBound_P = PARAMS.Front_BlueRedRatio_UpperBound_PurpleBall;

            GR_LBound_G = PARAMS.Front_GreenRedRatio_LowerBound_GreenBall;
            GR_UBound_G = PARAMS.Front_GreenRedRatio_UpperBound_GreenBall;
            BR_LBound_G = PARAMS.Front_BlueRedRatio_LowerBound_GreenBall;
            BR_UBound_G = PARAMS.Front_BlueRedRatio_UpperBound_GreenBall;
            redReading1=R;
            blueReading1=B;
            greenReading1=G;
        }
        //-------------------------------

        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        if (G > (PARAMS.greenRedRatioPurple * R) + GR_LBound_P && G < (PARAMS.greenRedRatioPurple * R) + GR_UBound_P && B > (PARAMS.blueRedRatioPurple * R) + BR_LBound_P && B < (PARAMS.blueRedRatioPurple * R) +BR_UBound_P && distance < DisCheck)
            return "P";
        else if (G > (greenRatio * R) + GR_LBound_G && G < (greenRatio * R) + GR_UBound_G && B > (PARAMS.blueRedRatioGreen * R) + BR_LBound_G && B < (PARAMS.blueRedRatioGreen * R) + BR_UBound_G && distance < DisCheck)
            return "G";
        else
            return "N";
    }

    // Kickers
    public void kickFront() {
        kickUp = true;
        frontLoaderNextPos=1;
        //frontKick.setPosition(1);
    }
    public void kickMiddle() {  kickUp = true; midLoaderNextPos=1; }
    public void kickBack() {  kickUp = true; backLoaderNextPos=1; }
    public void resetKick() {
        kickUp = false;
        double t =generalTimer.seconds();
        backLoaderNextTrigger=t;
        backLoaderNextPos=0;
        midLoaderNextTrigger=t+0.067;
        midLoaderNextPos=0;
        frontLoaderNextTrigger=t+0.067*2;
        frontLoaderNextPos=0;
        gateNextTrigger=frontLoaderNextTrigger;
        gateNextPos=gateMidPOs;
        changeFlagTrigger=gateNextTrigger+0.3 ;
        nextFlagValue=true;
    }
    public void resetKickers() {
        resetting = true;
        kickUp = false;
        double t =generalTimer.seconds();
        backLoaderNextTrigger=t;
        backLoaderNextPos=0;
        midLoaderNextTrigger=t+0.03;
        midLoaderNextPos=0;
        frontLoaderNextTrigger=t+0.03*2;
        frontLoaderNextPos=0;
        gateNextTrigger=frontLoaderNextTrigger;
        gateNextPos=gateMidPOs;

    } // new reset function, created by FX

    //private void sleep(int ms) { sleepTimer.reset(); while(sleepTimer.milliseconds() < ms) {} }
    double currentTime = 0;
    private void timeReset() {timer.reset();}



    // Automated loading for green ball
    public void loadGreen() { loadBall("G"); }



    // Automated loading for purple ball
    public void loadPurple() { loadBall("P"); }

    public void loadBack(){
        if (flag){
            update();
            String f = ballArray[0]; // front
            String m = ballArray[1]; // middle
            String b = ballArray[2]; // back
            if (!b.equals("N")){
                flag =false;
                if (count == 0) {
                    kickTarget ="back";
                    timeReset();
                    kickBack3();

                } else if (count == 1) {
                    if (m.equals("N")) {
                        kickTarget ="back";
                        timeReset();
                        kickBack2Front();    // front + back
                    }
                    else {
                        timeReset();
                        kickTarget ="back";
                        kickBack2Middle();  // front + middle
                    }
                } else if (count == 2) {
                    kickTarget ="back";
                    timeReset();
                    kickBack1();

                }
            } else{
                flag=true;
            }

        }
    }

    public void loadMiddle(){
        if (flag){
            update();
            String f = ballArray[0]; // front
            String m = ballArray[1]; // middle
            String b = ballArray[2]; // back
            if (!m.equals("N")){
                flag =false;
                if (count == 0) {
                    kickTarget ="middle";
                    timeReset();
                    kickMiddle3();
                } else if (count == 1) {
                    if (f.equals("N")) {
                        kickTarget ="middle";
                        timeReset();
                        kickMiddle2Back();   // middle + back

                    }
                    else {
                        kickTarget ="middle";
                        timeReset();
                        kickMiddle2Front();
                    }     //middle + front
                } else if (count == 2) {
                    kickTarget ="middle";
                    timeReset();
                    kickMiddle1();

                }
            } else{
                flag=true;
            }

        }
    }

    public void loadFront(){
        if (flag){
            update();
            String f = ballArray[0]; // front
            String m = ballArray[1]; // middle
            String b = ballArray[2]; // back
            if (!f.equals("N")){
                flag =false;
                if (count == 0) {
                    kickTarget ="front";
                    timeReset();
                    kickFront3();

                } else if (count == 1) {
                    if (b.equals("N")) {
                        kickTarget ="front";
                        timeReset();
                        kickFront2Middle();   // back + middle

                    }
                    else {
                        kickTarget ="front";
                        timeReset();
                        kickFront2Back();    // back + front

                    }
                } else if (count == 2) {
                    kickTarget ="front";
                    timeReset();
                    kickFront1();

                }
            } else{
                flag=true;
            }

        }
    }


    public void loadBall(String targetColor) { // "G" or "P" //changed to middle first, then back, front last.
        if (flag) {
            update(); // Refresh sensor data

            String f = ballArray[0]; // front
            String m = ballArray[1]; // middle
            String b = ballArray[2]; // back
            flag = false;
            // Count how many balls are currently in storage

            if (targetColor.equals("P")) {
                color(0.722);
            } else if (targetColor.equals("G")) {
                color(0.5);
            }
            // Find the FIRST (closest to intake) ball that matches the target color
            if (b.equals(targetColor)) {
                // Target ball is in BACK

                if (count == 0) {
                    kickTarget ="back";
                    timeReset();
                    kickBack3();

                } else if (count == 1) {
                    if (m.equals("N")) {
                        kickTarget ="back";
                        timeReset();
                        kickBack2Front();    // front + back
                    }
                    else {
                        timeReset();
                        kickTarget ="back";
                        kickBack2Middle();  // front + middle
                    }
                } else if (count == 2) {
                    kickTarget ="back";
                    timeReset();
                    kickBack1();

                }

            } else if (m.equals(targetColor)) {
                // Target ball is in MIDDLE
                if (count == 0) {
                    kickTarget ="middle";
                    timeReset();
                    kickMiddle3();
                } else if (count == 1) {
                    if (f.equals("N")) {
                        kickTarget ="middle";
                        timeReset();
                        kickMiddle2Back();   // middle + back

                    }
                    else {
                        kickTarget ="middle";
                        timeReset();
                        kickMiddle2Front();
                    }     //middle + front
                } else if (count == 2) {
                    kickTarget ="middle";
                    timeReset();
                    kickMiddle1();

                }
            } else if (f.equals(targetColor)) {
                // Target ball is in FRONT
                if (count == 0) {
                    kickTarget ="front";
                    timeReset();
                    kickFront3();

                } else if (count == 1) {
                    if (b.equals("N")) {
                        kickTarget ="front";
                        timeReset();
                        kickFront2Middle();   // back + middle

                    }
                    else {
                        kickTarget ="front";
                        timeReset();
                        kickFront2Back();    // back + front

                    }
                } else if (count == 2) {
                    kickTarget ="front";
                    timeReset();
                    kickFront1();

                }
            } else {
                flag = true;
            }
        }
        // If no matching ball found → do nothing

        // Always reset kickers and refresh state
        //update();
    }


    private void loadBallw2StepKick(String targetColor) { // "G" or "P"
        if (flag) {
            update(); // Refresh sensor data

            String f = ballArray[0]; // front
            String m = ballArray[1]; // middle
            String b = ballArray[2]; // back
            flag = false;
            // Count how many balls are currently in storage

            if (targetColor.equals("P")) {
                // color(0.722);
            } else if (targetColor.equals("G")) {
                //  color(0.5);
            }
            // Find the FIRST (closest to intake) ball that matches the target color
            if (b.equals(targetColor)) {
                // Target ball is in FRONT
                if (count == 0) {
                    kickTarget ="back";
                    timeReset();
                    if (kick2StepFlag){ kickBack3w2Step();}
                    else {kickBack3();}

                } else if (count == 1) {
                    if (m.equals("N")) {
                        kickTarget ="back";
                        timeReset();
                        if (kick2StepFlag) { kickBack2Front_2Step();}
                        else {
                            kickBack2Front();
                        }// front + back
                    }
                    else {
                        timeReset();
                        kickTarget ="back";
                        if (kick2StepFlag) {kickBack2Middle_2Step();}
                        else {kickBack2Middle();}  // front + middle
                    }
                } else if (count == 2) {
                    kickTarget ="back";
                    timeReset();
                    if (kick2StepFlag) {kickBack1_2Step();}
                    else {kickBack1();}

                }
            } else if (m.equals(targetColor)) {
                // Target ball is in MIDDLE
                if (count == 0) {
                    kickTarget ="middle";
                    timeReset();
                    if (kick2StepFlag) {kickMiddle3w2Step();}
                    else {kickMiddle3();}

                } else if (count == 1) {
                    if (f.equals("N")) {
                        kickTarget ="middle";
                        timeReset();
                        if (kick2StepFlag) {kickMiddle2Back_2Step();}
                        else {kickMiddle2Back();  } // middle + back

                    }
                    else {
                        kickTarget ="middle";
                        timeReset();
                        if (kick2StepFlag) {kickMiddle2Front_2Step();}
                        else {kickMiddle2Front();}
                    }     //middle + front
                } else if (count == 2) {
                    kickTarget ="middle";
                    timeReset();
                    if (kick2StepFlag) {kickMiddle1_2Step();}
                    else {kickMiddle1();}

                }
            } else if (f.equals(targetColor)) {
                // Target ball is in BACK
                if (count == 0) {
                    kickTarget ="front";
                    timeReset();
                    if (kick2StepFlag) {kickFront3w2Step();}
                    else {kickFront3();}

                } else if (count == 1) {
                    if (b.equals("N")) {
                        kickTarget ="front";
                        timeReset();
                        if (kick2StepFlag) {kickFront2Middle_2Step();}
                        else {kickFront2Middle(); }  // back + middle

                    }
                    else {
                        kickTarget ="front";
                        timeReset();
                        if (kick2StepFlag) {kickFront2Back_2Step();}
                        else {kickFront2Back();}    // back + front

                    }
                } else if (count == 2) {
                    kickTarget ="front";
                    timeReset();
                    if (kick2StepFlag) {kickFront1_2Step();}
                    else {kickFront1();}

                }
            }
        }





        // If no matching ball found → do nothing

        // Always reset kickers and refresh state
        //update();
    }

    public void autoLoad(){
        boolean turretReady = myTurret.tagFound;
        if (turretReady && flag && myTurret.shooterUpToSpeed){
            if (flagLoadingBack){
                loadBack();
                flagLoadingBack=false;
            } else {
                if (flagLoadingMiddle){
                    loadMiddle();
                    flagLoadingMiddle=false;
                } else{
                    if (flagLoadingFront){
                        loadFront();
                        flagLoadingFront=false;
                    }
                }
            }
        }
    }
    public  void loadAll(){
        flagLoadingBack=true;
        flagLoadingMiddle=true;
        flagLoadingFront=true;
    }

    /*
    public void rapidFire() {
        boolean stat=false;
        boolean stat2=false;
        loadBack();
        stat=waitForReset();
        if (stat){
            loadMiddle();
            stat2=waitForReset();
            if (stat2){
                loadFront();
            }
        }
    }
    */

    /*
    public boolean waitForReset() {
        double startTime = generalTimer.time();
        boolean timeOut = false;

        while (!timeOut && !flag && !myTurret.shotDetected) {
            update();
            myTurret.update();
            if (generalTimer.time() - startTime > 3) {
                timeOut=true;
            }
        }
        shotDetectReset();
        sleep(400);

        return (!timeOut);
    }
    */

    // Count balls in storage
    public int ballCount() {
        int count = 0;
        if (!frontBall.equals("N")) count++;
        if (!middleBall.equals("N")) count++;
        if (!backBall.equals("N")) count++;

        return count;
    }

    public class LoadGreenAction implements Action {
        public boolean run(@NonNull TelemetryPacket pack){
            loadBall("G");
            return true;
        }


    }

    public Action loadGreenAction(){
        return new LoadGreenAction();
    }
}

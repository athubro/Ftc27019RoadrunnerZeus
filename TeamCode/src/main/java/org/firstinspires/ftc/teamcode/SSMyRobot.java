
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


public class SSMyRobot  {

    private Turret turretSystem;
    private MecanumDrive drive;

    private Intake intake;

    //private Storage kickers;  // intake-free storage class

    private Pose2d pose ;
    //private boolean waitingForConfig = true;
    private double speedRatio = 0.4;

    public boolean doneShooting = true;
    public ElapsedTime generalTimer = new ElapsedTime();
    public ElapsedTime intakeTime = new ElapsedTime();
    private HardwareMap myHardwareMap;
    public boolean updateFlag = false;
    private double previousTime =0;
    public final FtcDashboard dashboard;

    public SSMyRobot (HardwareMap hardwareMap, MecanumDrive myDrive, Intake myIntake, Turret turret, Pose2d newPos) {
        intake = myIntake;

        drive = myDrive;
        turretSystem=turret;

        myHardwareMap=hardwareMap;
        pose=newPos;
        generalTimer.reset();
        intakeTime.reset();
        dashboard = FtcDashboard.getInstance();

    }

    //=========================================functions =====================================
    public class IntakePower implements Action {
        private double power;
        public IntakePower (double Power) {
            power = Power;
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            if (power>1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
            }
            intake.setIntakePower(power);
            return false;
        }
    }

    public Action intakePower(double Power) {
        return new IntakePower(Power);
    }



    public class StoreBalls implements Action {
        private String[] target;
        public StoreBalls (String[] Target) {
            target = Target;
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            intake.storageUpdate();
            intake.storeBalls(target);
            if (intake.ballsStored) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action storeBalls(String[] Target) {
        return new StoreBalls(Target);
    }




    public class ExecuteNextStep implements Action{
        double timeCap;

        public ExecuteNextStep () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            intake.storageUpdate();
            intake.executeNextStep();

            //if ((!intake.firstStep.equals("N")) && (!intake.secondStep.equals("N"))) {

            //    return true;
            //} else {
                return false;
            //}
        }
    }

    public Action executeNextStep() {
        return new ExecuteNextStep();
    }



    public class WaitSorting implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            intake.storageUpdate();
            if (intake.firstStep.equals("N") && intake.firstStep.equals("N")) {
                return false;
            }
            //intake.ballCount == 0 ||
            if ( intakeTime.seconds() > 1.3) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action waitSorting(){
        return new WaitSorting();
    }


    public class CloseGate implements Action {

        public boolean run(@NonNull TelemetryPacket pack) {
            intake.closeGate();
            return false;
        }
    }

    public Action closeGate() {
        return new CloseGate();
    }





    public class OpenGate implements Action {

        public boolean run(@NonNull TelemetryPacket pack) {
            intake.openGate();
            return false;
        }
    }

    public Action openGate() {
        return new OpenGate();
    }


    public class FireBalls implements Action {

        public boolean run(@NonNull TelemetryPacket pack) {
            if (turretSystem.flywheelUpToSpeed){
                intake.openGate();
                intake.setIntakePower(1);
                return false;
            } else {
                return true;
            }


        }
    }

    public Action fireBalls() {
        return new FireBalls();
    }



    //=======================================================================================================

/*
    public class ReverseTransfer implements Action {
        double timeCap;
        double startTime;

        public ReverseTransfer () {
            //timeCap = 0.5;
            //generalTimer.reset();
            //startTime = generalTimer.seconds();
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            //startTime = generalTimer.seconds();
            kickers.transferPower(-1);
            //pack.put("timer reading ", generalTimer.seconds());
            //pack.put("starting time",startTime);
            //turretSystem.update();
            //kickers.update();
            //kickers.loadingUpdate();
            return false;

      }
    }

    public Action reverseTransfer() {
        return new ReverseTransfer();
    }
*/





    public class TurnOnTracking implements Action {
        double timeCap;

        public TurnOnTracking () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {

            turretSystem.setTrackingMode(true);
            turretSystem.autoAngleEnabled = true;
            turretSystem.autoRPMEnabled = true;
            //   turretSystem.aimingMode = true;
            //turretSystem.update();
            intake.storageUpdate();  // senses colors and updates slot states
            //  kickers.loadingUpdate();
            turretSystem.update();
            //turretSystem.turretSetPos();
            if (turretSystem.tagFound) {
                turretSystem.setShootingEnabled(true);
                return false;
            } else {
                return true;
            }

        }
    }


    public class TurnOnTrackingOnly implements Action {
        double timeCap;

        public TurnOnTrackingOnly () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {

            turretSystem.setTrackingMode(true);
            // turretSystem.aimingMode = true;
            turretSystem.setShootingEnabled(true);
            return false;


        }
    }
    public Action turnOnTracking() {
        return new TurnOnTracking();
    }

    public Action turnOnTrackingOnly() {
        return new TurnOnTrackingOnly();
    }


    public class WaitForTracking implements Action{
        double timeCap;

        public WaitForTracking () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {


            if (turretSystem.tagFound) {

                return false;
            } else {
                return true;
            }
        }
    }

    public Action waitForTracking() {
        return new WaitForTracking();
    }

    public class TurnOffTracking implements Action {
        double timeCap;

        public TurnOffTracking () {
            timeCap = 5;

        }
        public boolean run(@NonNull TelemetryPacket pack) {

            turretSystem.setTrackingMode(false);
            turretSystem.setShootingEnabled(false);
            turretSystem.update();

            return false;


        }
    }

    public class UpdateRobot implements Action{
        public boolean run(@NonNull TelemetryPacket pack){
            TelemetryPacket packet = new TelemetryPacket();
            turretSystem.update();
            intake.storageUpdate();  // senses colors and updates slot states

            pack.put("tag found",turretSystem.tagFound);
            pack.put("shooting enabled",turretSystem.shootingEnabled);
            pack.put("tracking mode",turretSystem.trackingMode);
            packet.put("time gap", generalTimer.seconds()-previousTime);
            //previousTime=generalTimer.seconds();

            if (updateFlag){
                dashboard.sendTelemetryPacket(packet);
                return true;
            } else {
                return false;
            }

        }
    }



    public Action updateRobot (){
        return new UpdateRobot();
    }
    public Action turnOffTracking() {
        return new TurnOffTracking();
    }

    public class TurnOnUpdate implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            updateFlag=true;
            return false;
        }
    }

    public Action turnOnUpdate(){
        return new TurnOnUpdate();
    }

    public class TurnOffUpdate implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            updateFlag=false;
            return false;
        }
    }

    public Action turnOffUpdate(){
        return new TurnOffUpdate();
    }

    public class UpdateTracking implements Action {
        double timeCap;

        public UpdateTracking () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {

            turretSystem.setTrackingMode(true);
            turretSystem.update();
            intake.storageUpdate();  // senses colors and updates slot states

            turretSystem.update();
            if (turretSystem.tagFound) {
                return true;
            } else {
                return false;
            }







        }
    }

    public class WaitEmptyStorage implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            intake.storageUpdate();

            //intake.ballCount == 0 ||
            if ( intakeTime.seconds() > 1.3) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action waitEmptyStorage(){
        return new WaitEmptyStorage();
    }


    public class ResetIntakeTimer implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            intakeTime.reset();
            return false;

        }
    }

    public Action resetIntakeTimer(){
        return new ResetIntakeTimer();
    }


    public class WaitFullStorage implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            intake.storageUpdate();
            if (intake.ballCount == 3 || intakeTime.seconds() > 1.5) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action waitFullStorage(){
        return new WaitFullStorage();
    }


    public class DetectMotiff implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.updateMotiff();
            //if (intake.ballCount == 3 || intakeTime.seconds() > 1.5) {
                    return false;

        }
    }

    public Action dectMotiff(){
        return new DetectMotiff();
    }


    public class WaitSpinUp implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.update();
            if (turretSystem.flywheelUpToSpeed) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action waitSpinUp(){
        return new WaitSpinUp();
    }


    public class AimInit implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            //turretSystem.turretManualPos(0.5);

            return false;

        }
    }

    public Action aimInit(){
        return new AimInit();
    }




    public class LookRed implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            //turretSystem.turretManualPos(0.4);


            return false;



        }
    }

    public Action lookRed(){
        return new LookRed();
    }

    public class LookMotifBlue implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            // turretSystem.turretManualPos(0);
            return false;
        }
    }
    public Action lookMotifBlue(){
        return new LookMotifBlue();
    }

    public class LookMotifRed implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            //           turretSystem.turretManualPos(1);
            return false;
        }
    }
    public Action lookMotifRed(){
        return new LookMotifRed();
    }
    public class LookBlue implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            //      turretSystem.turretManualPos(0.6);


            return false;



        }
    }

    public Action lookBlue(){
        return new LookBlue();
    }


    public class SetColor implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            //  turretSystem.color(0.49);


            return false;



        }
    }

    public Action setColor(){
        return new SetColor();
    }

    public class Load3 implements Action {
        double timeCap;

        public Load3 () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {


            return false;




        }
    }
    public class AfterLoad implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){

            // if () {
            return false;
            //} else {
            //     return true;
            //}
        }
    }

    public Action afterLoad (){
        return new AfterLoad();
    }







    public Action load3() {
        return new Load3();
    }



    public class MotiffUpdate implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            //  turretSystem.updateMotiff();
            intake.storageUpdate();   // senses colors and updates slot states
            turretSystem.update();

            //if (turretSystem.motiff[0].equals("N")) {
            //    kickers.color(0.722); //show purple
            //    return true;
            // } else {
            //       kickers.color(0.5); //show green
            return false;
            // }
        }
    }

    public Action motiffUpdate() {
        return  new MotiffUpdate();
    }



    public class MotiffCheck implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            //turretSystem.updateMotiff();
            // kickers.update();  // senses colors and updates slot states
            // kickers.loadingUpdate();
            turretSystem.update();
            //  pack.put("motiff 0", turretSystem.motiff[0]);
            // pack.put("motiff 1", turretSystem.motiff[1]);
            // pack.put("motiff 3", turretSystem.motiff[2]);
            //  if (turretSystem.motiff[0].equals("N")) {
            //     kickers.color(0.722); //show purple

            //   } else {
            //     kickers.color(0.5); //show green

            //  }
            return false;
        }
    }

    public Action motiffCheck() {
        return  new MotiffCheck();
    }






    public class CalcShotVariables implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;


            if (turretSystem.tagFound) return false;
            else return true;
        }
    }



    public Action calcShotVariables() {
        return new CalcShotVariables();
    }



    public class TrackingPRM implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){

            return false;
        }
    }

    public Action trackingPRM (){
        return new TrackingPRM();

    }
    public class ShooterSpinUp implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.shootingEnabled = true;
            return false;
        }
    }

    public Action shooterSpinUp() {
        return new ShooterSpinUp();
    }

    public class ShooterSpinUpWAprilTag implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            if (turretSystem.tagFound){
                turretSystem.shootingEnabled = true;
            } else {
                turretSystem.shootingEnabled = false;
            }

            if (turretSystem.trackingMode) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action shooterSpinUpWAprilTag() {
        return new ShooterSpinUpWAprilTag();
    }


    public class SetTargetRPM implements Action {
        double rpm;

        public SetTargetRPM (double RPM) {
            rpm = RPM;
        }

        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.targetRPM = rpm;
            return false;
        }
    }

    public Action setTargetRPM(double RPM) {
        return new SetTargetRPM(RPM);
    }



    public class SetTurretAngle implements Action {
        double degrees;

        public SetTurretAngle (double Degrees) {
            degrees = Degrees;
        }

        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.manualTurretAngle(degrees);
            return false;
        }
    }

    public Action setTurretAnlge(double Degrees) {
        return new SetTurretAngle(Degrees);
    }




    public class SetShooterAngle implements Action {
        double pos;

        public SetShooterAngle (double Pos) {
            pos = Pos;
        }

        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.turretAngleCommand = 0;

            turretSystem.turretAngle.setPosition(pos);
            turretSystem.updateTurretAngle();
            return false;
        }
    }

    public Action setShooterAngle(double Pos) {
        return new SetShooterAngle(Pos);
    }







    public class CalcRPMAndAngle implements Action {
        double rpm;


        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.updateVisionTracking();
            turretSystem.calcTargetRPM();
            turretSystem.calcTurretAngle();
            turretSystem.updateTurretAngle();
            return false;
        }
    }

    public Action calcRPMAndAngle() {
        return new CalcRPMAndAngle();
    }

    public class ContCalcRPMAndAngle implements Action {
        double rpm;


        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.updateVisionTracking();
            turretSystem.calcTargetRPM();
            turretSystem.calcTurretAngle();
            turretSystem.updateTurretAngle();
            if (turretSystem.trackingMode) {
                return false;
            } else {
                return true;
            }

        }
    }

    public Action contCalcRPMAndAngle() {
        return new ContCalcRPMAndAngle();
    }

    public class ShooterStop implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.shootingEnabled = false;
            turretSystem.update();
            return false;
        }
    }

    public Action shooterStop() {
        return new ShooterStop();
    }











    /*
    public void rapidFire(){
        int count = kickers.ballCount();
        kickers.update();
        for (int i = 0; i < 3; i++) {
            if (kickers.ballArray[i] == "P") {
                kickers.loadPurple();
                while (!kickers.flag && turretSystem.shotDetected) {
                    kickers.update();
                    turretSystem.update();

                }
                shotDetectReset();
                sleep(400);

            } else if (kickers.ballArray[i] == "G") {
                kickers.loadGreen();
                kickers.loadPurple();
                while (!kickers.flag && turretSystem.shotDetected) {
                    kickers.update();
                    turretSystem.update();

                }
                shotDetectReset();
                sleep(400);

            } else {

            }
        }
    }
    */


}

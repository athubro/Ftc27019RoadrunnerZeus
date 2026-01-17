package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


public class SSMyRobot  {

    private Turret turretSystem;
    private MecanumDrive drive;
    //private Storage kickers;  // intake-free storage class
    private StorageWLoader kickers;
    private Pose2d pose ;
    //private boolean waitingForConfig = true;
    private double speedRatio = 0.4;

    public boolean doneShooting = true;
    public ElapsedTime generalTimer = new ElapsedTime();
    public ElapsedTime transferTime = new ElapsedTime();
    private HardwareMap myHardwareMap;
    public boolean updateFlag = false;
    private double previousTime =0;
    public final FtcDashboard dashboard;

    public SSMyRobot (HardwareMap hardwareMap, MecanumDrive myDrive, StorageWLoader storage, Turret turret, Pose2d newPos) {
        kickers=storage;
        drive = myDrive;
        turretSystem=turret;
        turretSystem.autoSpinUp=true;
        myHardwareMap=hardwareMap;
        pose=newPos;
        generalTimer.reset();
        dashboard = FtcDashboard.getInstance();
    }


    public class Intake implements Action {
        private double power;
        public Intake (double Power) {
            power = Power;
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            if (power>1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
            }
            kickers.setIntakePower(power);
            return false;
        }
    }

    public Action intake(double Power) {
        return new Intake(Power);
    }



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


    public class ReverseTransferStop implements Action {
        double timeCap;
        double startTime;

        public ReverseTransferStop () {
            //timeCap = 0.5;
            //generalTimer.reset();
            //startTime = generalTimer.seconds();
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            //startTime = generalTimer.seconds();
            kickers.transferPower(0);
            //pack.put("timer reading ", generalTimer.seconds());
            //pack.put("starting time",startTime);
            //turretSystem.update();
            //kickers.update();
            //kickers.loadingUpdate();
            return false;




        }
    }

    public Action reverseTransferStop() {
        return new ReverseTransferStop();
    }



    public class TurnOnTracking implements Action {
        double timeCap;

        public TurnOnTracking () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {

            turretSystem.setTrackingMode(true);
            turretSystem.aimingMode = true;
            //turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
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
            turretSystem.aimingMode = true;
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
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            pack.put("tag found",turretSystem.tagFound);
            pack.put("shooting enabled",turretSystem.shootingEnabled);
            pack.put("tracking mode",turretSystem.trackingMode);
            packet.put("time gap", generalTimer.seconds()-previousTime);
            previousTime=generalTimer.seconds();

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
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
            if (turretSystem.tagFound) {
                return true;
            } else {
                return false;
            }







        }
    }




    public class LockOn implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
          //  turretSystem.turretSetPos();
          //  if (turretSystem.tagFound) {
            //    turretSystem.turretSetPos();
                return false;
           // } else {
            //    return true;
          //  }

        }
    }

    public Action lockOn(){
        return new LockOn();
    }

    public class AimInit implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.turretManualPos(0.5);


            return false;



        }
    }

    public Action aimInit(){
        return new AimInit();
    }


    public class LookRed implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.turretManualPos(0.4);


                return false;



        }
    }

    public Action lookRed(){
        return new LookRed();
    }

    public class LookMotifBlue implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.turretManualPos(0.1);
            return false;
        }
    }
    public Action lookMotifBlue(){
        return new LookMotifBlue();
    }

    public class LookMotifRed implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.turretManualPos(0.9);
            return false;
        }
    }
    public Action lookMotifRed(){
        return new LookMotifRed();
    }
    public class LookBlue implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.turretManualPos(0.6);


            return false;



        }
    }

    public Action lookBlue(){
        return new LookBlue();
    }

    public class SetColor implements  Action{
        public boolean run(@NonNull TelemetryPacket pack){
            turretSystem.color(0.49);


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
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
           kickers.loadAll();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
           return false;




        }
    }
    public class AfterLoad implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){
            //double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;
            boolean finished = false;

            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();

            if (kickers.kickUp && turretSystem.shotDetected) {
                kickers.resetKick();
                turretSystem.shotDetected = false;
            }
            finished=kickers.flag || kickers.pastAutoTime;
            if (finished) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action afterLoad (){
        return new AfterLoad();
    }

    public class AfterLoadNTer implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){
            //double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;
            boolean finished = false;

            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();

            if (kickers.kickUp && turretSystem.shotDetected) {
                kickers.resetKick();
                turretSystem.shotDetected = false;
            }
            finished=kickers.flag;
            if (finished) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action afterLoadNTer (){
        return new AfterLoadNTer();
    }
    public class LastAfterLoad implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){
            //double startingTime= generalTimer.seconds();
            //double timeOut= 5; // 5 sec to time out;
            //boolean finished = false;

            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
            if (kickers.flag|| kickers.pastAutoTime) {
                return false;
            }
            else {
                if ((kickers.kickUp && turretSystem.shotDetected)|| kickers.pastAutoTime) {
                    kickers.resetKick();
                    turretSystem.shotDetected = false;
                    return false;
                }
                return true;
            }

        }
    }

    public Action lastAfterLoad (){
        return new LastAfterLoad();
    }

    public class LastAfterLoadNTer implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){
            //double startingTime= generalTimer.seconds();
            //double timeOut= 5; // 5 sec to time out;
            //boolean finished = false;

            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
            if (kickers.flag) {
                return false;
            }
            else {
                if ((kickers.kickUp && turretSystem.shotDetected)) {
                    kickers.resetKick();
                    turretSystem.shotDetected = false;
                    return false;
                }
                return true;
            }

        }
    }

    public Action lastAfterLoadNTer (){
        return new LastAfterLoadNTer();
    }
    public Action load3() {
        return new Load3();
    }

    public class AfterLoad3 implements Action {
        double timeCap;

        public AfterLoad3 () {
            timeCap = 5;
        }
        public boolean run(@NonNull TelemetryPacket pack) {
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
            pack.put("flag", kickers.flag);
            pack.put("front flag", kickers.flagLoadingFront);
            pack.put("mid flag", kickers.flagLoadingMiddle);
            pack.put("back flag", kickers.flagLoadingBack);
            pack.put("front color", kickers.ballArray[0]);
            pack.put("mid color", kickers.ballArray[1]);
            pack.put("back color", kickers.ballArray[2]);
            pack.put("motiff", turretSystem.motiff);
            if (kickers.flag && (! kickers.flagLoadingFront) && (!kickers.flagLoadingMiddle) && (!kickers.flagLoadingBack)){
                return false;
            } else {
                return true;
            }





        }
    }




    public Action afterLoad3() {
        return new AfterLoad3();
    }


    public class MotiffUpdate implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.updateMotiff();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
            pack.put("motiff 0", turretSystem.motiff[0]);
            pack.put("motiff 1", turretSystem.motiff[1]);
            pack.put("motiff 3", turretSystem.motiff[2]);
            if (turretSystem.motiff[0].equals("N")) {
                kickers.color(0.722); //show purple
                return true;
            } else {
                kickers.color(0.5); //show green
                return false;
            }
        }
    }

    public Action motiffUpdate() {
        return  new MotiffUpdate();
    }

    public class MotiffCheck implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            turretSystem.updateMotiff();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();
            pack.put("motiff 0", turretSystem.motiff[0]);
            pack.put("motiff 1", turretSystem.motiff[1]);
            pack.put("motiff 3", turretSystem.motiff[2]);
            if (turretSystem.motiff[0].equals("N")) {
                kickers.color(0.722); //show purple

            } else {
                kickers.color(0.5); //show green

            }
            return false;
        }
    }

    public Action motiffCheck() {
        return  new MotiffCheck();
    }

    public class LoadMotiff implements Action {

        public boolean run(@NonNull TelemetryPacket pack) {
            boolean finished = false;
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            turretSystem.update();

            for (int i = 0; i < 3; i++) {
                kickers.update();
                turretSystem.update();
                if (turretSystem.motiff[i].equals("P")) {
                    loadPurpleAction();
                } else {
                    loadGreenAction();
                }

            }
            return false;

        }
    }

    public Action loadMotiff() {
        return  new LoadMotiff();
    }




    public class CalcShotVariables implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;


            turretSystem.updateTurretControl();
            turretSystem.measureDis();
            turretSystem.calcTargetAngleSpeed();
            if (turretSystem.tagFound) return false;
            else return true;
        }
    }



    public Action calcShotVariables() {
        return new CalcShotVariables();
    }

    public class ConstantRPM implements Action{
        public ConstantRPM(double rpm){
            turretSystem.setTargetRPM(rpm);
        }
        public  boolean run(@NonNull TelemetryPacket pack){
            turretSystem.constantRPM=true;
            return false;
        }
    }

    public Action constantPRM (double rpm){
        return new ConstantRPM( rpm);

    }

    public class TrackingPRM implements Action{
        public  boolean run(@NonNull TelemetryPacket pack){
            turretSystem.constantRPM=false;
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



    public class LoadGreenAction implements Action {
        public boolean run(@NonNull TelemetryPacket pack){

            turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            if (turretSystem.shooterUpToSpeed){
                kickers.loadGreen();
            }else{
                return true;
            }

            return false;

            /*
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;
            boolean finished =false;
            boolean shotFlag=false;
            boolean storageFlag=false;

            turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();

            shotDetectReset();
            kickers.loadGreen();
            while (!finished && generalTimer.seconds()<startingTime+timeOut){
                turretSystem.update();
                kickers.update();  // senses colors and updates slot states
                kickers.loadingUpdate();

                shotFlag=shotFlag || turretSystem.shotDetected;
                if (shotFlag){
                    storageFlag=storageFlag || kickers.flag;
                }
                finished= shotFlag && storageFlag;
                shotDetectReset();

            }

            if (finished) return true;
            else return false;
            */
        }


    }

    public Action loadGreenAction() {
        return new LoadGreenAction();

    }





    public class LoadPurpleAction implements Action {
        public boolean run(@NonNull TelemetryPacket pack){

            turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();
            if (turretSystem.shooterUpToSpeed){
                kickers.loadPurple();
            }else{
                return true;
            }

            return false;


            /*
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;
            boolean finished =false;
            boolean shotFlag=false;
            boolean storageFlag=false;

            turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();

            shotDetectReset();
            kickers.loadPurple();
            while (!finished && generalTimer.seconds()<startingTime+timeOut){
                turretSystem.update();
                kickers.update();  // senses colors and updates slot states
                kickers.loadingUpdate();

                shotFlag=shotFlag || turretSystem.shotDetected;
                if (shotFlag){
                    storageFlag=storageFlag || kickers.flag;
                }
                finished= shotFlag && storageFlag;
                shotDetectReset();

            }

            if (finished) return true;
            else return false;
*/
        }


    }

    public Action loadPurpleAction() {
        return new LoadPurpleAction();

    }


    public void shotDetectReset() {
        if (kickers.kickUp && turretSystem.shotDetected) {
            kickers.resetKick();
            turretSystem.shotDetected = false;
        }
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

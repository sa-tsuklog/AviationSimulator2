/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package aviationsimulator2;

import java.awt.geom.Point2D;

/**
 *
 * @author sa
 */
public class AviationModel {
    final double RHO = 1.29;
    
    double kgWeight;
    double nmInertia;
    WingModel mainWing;
    double mMainWingPos;
    WingModel elevator;
    double mElevatorPos;
    double extraDrag;
    RocketMoterModel moter;
    double secTimeStep;
    double radDirection;
    Controller controller;
    
    Point2D mPos;
    Point2D mpsSpeed;
    Point2D nSpeedFrameForce;
    double radpsRotationSpeed;
    
    enum ControllerType{
        NO_CONTROL,
        PITCH_BASED,
        DLC_GROUND_SKIMMING,
    }
    class Controller{
        int state;
        ControllerType controllerType;
        
        public Controller(ControllerType controllerType){
            state = 0;
            this.controllerType = controllerType;
        }
        public int getState(){
            return state;
        }
        public void update(){
            if(controllerType == ControllerType.PITCH_BASED){
                double gain=100;
                double dumperGain = 10;
                double targetDirection = -Math.PI/64;
                
                if(state == 0){
                    mainWing.setEnabled(false);
                    elevator.setRadOffsetAngle(0);
                    
                    if(targetDirection<radDirection){
                        state = 1;
                    }
                }else{
                    mainWing.setEnabled(true);
                    double speed = pointAbs(mpsSpeed);

                    double angle = -gain/speed/speed *(targetDirection-radDirection);
                    angle += dumperGain/speed/speed * radpsRotationSpeed;

                    elevator.setRadOffsetAngle(angle);
                }
            }else if(controllerType == ControllerType.DLC_GROUND_SKIMMING){
                double gain = 100;
                double dumperGain = 10;
                
                double heightGain = 100;
                double heightDumperGain = 10;
                double targetHeight = 1;
                double targetDirection = 0;
                double radMainWingLimit = 10.0/180*Math.PI;
                
                if(state == 0){
                    mainWing.setEnabled(false);
                    elevator.setRadOffsetAngle(0);
                    
                    if(targetDirection<radDirection){
                        state = 1;
                    }
                }else if(state == 1){
                    double speed = pointAbs(mpsSpeed);
                    double angle = -gain/speed/speed *(-0.15+targetDirection-radDirection);
                    angle += dumperGain/speed/speed * radpsRotationSpeed;
                    elevator.setRadOffsetAngle(angle);
                    if(mPos.getY()<targetHeight*2){
                        state = 2;
                    }
                }else{
                    mainWing.setEnabled(true);
                    double speed = pointAbs(mpsSpeed);

                    double angle = -gain/speed/speed *(targetDirection-radDirection);
                    angle += dumperGain/speed/speed * radpsRotationSpeed;

                    double mAngle = heightGain/speed/speed * (targetHeight - mPos.getY());
                    mAngle -= heightDumperGain/speed/speed * mpsSpeed.getY();
                    if(mAngle<0){
                        mAngle=0;
                    }else if(radMainWingLimit<mAngle){
                        mAngle=radMainWingLimit;
                    }
                    
                    System.out.println(mAngle*180/Math.PI);
                    
                    elevator.setRadOffsetAngle(angle);
                    mainWing.setRadOffsetAngle(mAngle);
                }
            }else{//NO_CONTROL
                //do nothing.
            }
        }
    }
    
    
    public AviationModel(double secTimeStep) {
        kgWeight = 0.5;
        nmInertia = 0.1;
        mainWing = new WingModel("naca4412.txt",0.1, 0);
        mMainWingPos = 0.0;
        elevator = new WingModel("naca0012.txt", 0.04, 0.0, 10);
        mElevatorPos = -0.8;
        extraDrag = 0.0056;
        moter = new RocketMoterModel(secTimeStep, 9, 1.5, 4.5, 0.01);
        controller = new Controller(ControllerType.PITCH_BASED);
        //controller = new Controller(ControllerType.NO_CONTROL);
        this.secTimeStep = secTimeStep;
        radDirection = -60*Math.PI/180;
        
        mPos=new Point2D.Double(0,0.1);
        mpsSpeed = new Point2D.Double(0,0);
        radpsRotationSpeed = 0;
    }
    public AviationModel(double kgWeight, double nmInertia, WingModel mainWing, double mMainWingPos, WingModel elevator, 
            double mElevatorPos, double extraDrag, RocketMoterModel moter, double secTimeStep, double radDirection,AviationModel.ControllerType controllerType){
        this.kgWeight = kgWeight;
        this.nmInertia = nmInertia;
        this.mainWing = mainWing;
        this.mMainWingPos = mMainWingPos;
        this.elevator = elevator;
        this.mElevatorPos = mElevatorPos;
        this.extraDrag=extraDrag;
        this.moter = moter;
        this.secTimeStep = secTimeStep;
        this.radDirection = radDirection;
        controller = new Controller(controllerType);
        
        mPos = new Point2D.Double(0,0.1);
        mpsSpeed = new Point2D.Double(0,0);
        radpsRotationSpeed = 0;
    }
    public void updateDynamics(double secTime){
        if(mPos.getY()<0.0){
            //do nothing
        }else{
            moter.update();
            controller.update();
        
            Point2D bodyFrameForce = getBodyFrameForce();
            Point2D earthFrameForce = getEarthFrameForce();
            nSpeedFrameForce = getSpeedFrameForce();
            
            bodyFrameForce = pointRotate(bodyFrameForce, radDirection);
            Point2D nSpeedFrameForceInFrame = pointRotate(nSpeedFrameForce, speedToAngle(mpsSpeed));
            
            double mpspsAccelX;
            double mpspsAccelY;
            double mPosX = mPos.getX();
            double mPosY = mPos.getY();
            double mpsSpeedX = mpsSpeed.getX();
            double mpsSpeedY = mpsSpeed.getY();
            
            mpspsAccelX = (earthFrameForce.getX()+bodyFrameForce.getX()+nSpeedFrameForceInFrame.getX())/(kgWeight+moter.getKgCurrentWeight());
            mpspsAccelY = (earthFrameForce.getY()+bodyFrameForce.getY()+nSpeedFrameForceInFrame.getY())/(kgWeight+moter.getKgCurrentWeight());
            
            
            
            mPosX = mPosX + mpsSpeedX*secTimeStep + 1.0/2*mpspsAccelX*secTimeStep*secTimeStep;
            mPosY = mPosY + mpsSpeedY*secTimeStep + 1.0/2*mpspsAccelY*secTimeStep*secTimeStep;
            
            mpsSpeedX = mpsSpeedX + mpspsAccelX*secTimeStep;
            mpsSpeedY = mpsSpeedY + mpspsAccelY*secTimeStep;
            
            mPos = new Point2D.Double(mPosX,mPosY);
            mpsSpeed = new Point2D.Double(mpsSpeedX,mpsSpeedY);
            
            double rotationForce = getRotationForce();
            
            radpsRotationSpeed = radpsRotationSpeed + rotationForce*secTimeStep/nmInertia;
            radDirection = radDirection + radpsRotationSpeed*secTimeStep + 1.0/2*rotationForce*secTimeStep*secTimeStep/nmInertia;
        }
    }
    private Point2D getBodyFrameForce(){
        return new Point2D.Double(moter.getN_CurrentForce(),0);
    }
    private Point2D getSpeedFrameForce(){
        double mpsTotalSpeed = pointAbs(mpsSpeed);
        double radSpeedDirection = speedToAngle(mpsSpeed);
        double radAoa = radDirection - radSpeedDirection;
        
        Point2D nMainWingForce = mainWing.getN_Force(mpsTotalSpeed, -radAoa);
        Point2D nElevatorForce = elevator.getN_Force(mpsTotalSpeed, -radAoa);
        Point2D nExtraDragForce = new Point2D.Double(-1/2.0*extraDrag*mpsTotalSpeed*mpsTotalSpeed*RHO,0);
        
        Point2D totalForce = pointAdd(pointAdd(nMainWingForce, nElevatorForce),nExtraDragForce);
        
        return totalForce;
    }
    private Point2D getEarthFrameForce(){
        double xForce = 0;
        double yForce = -9.8*(kgWeight+moter.getKgCurrentWeight());
        
        return new Point2D.Double(xForce,yForce);
    }
    private double getRotationForce(){
        double mpsTotalSpeed = pointAbs(mpsSpeed);
        double radSpeedDirection = speedToAngle(mpsSpeed);
        double radAoa = radDirection - radSpeedDirection;
        
        Point2D nMainWingForce = mainWing.getN_Force(mpsTotalSpeed, radAoa);
        Point2D nElevatorForce = elevator.getN_Force(mpsTotalSpeed, radAoa);
        
        
        
        double rotationForce = mMainWingPos*(-nMainWingForce.getX()*Math.sin(radAoa)+nMainWingForce.getY()*Math.cos(radAoa))
                + mElevatorPos*(-nElevatorForce.getX()*Math.sin(radAoa)+nElevatorForce.getY()*Math.cos(radAoa));
        
        return rotationForce;
    }
    
    
    public Point2D getM_Pos(){
        return mPos;
    }
    public double getRadDirection(){
        return radDirection;
    }
    public Point2D getMpsSpeed(){
        return mpsSpeed;
    }
    public double getRadAoa(){
        double speedAngle = speedToAngle(mpsSpeed);
        return speedAngle - radDirection;
    }
    public double getLbyD(){
        if(nSpeedFrameForce!=null){
            return -nSpeedFrameForce.getY()/nSpeedFrameForce.getX();
        }else{
            return 0;
        }
    }
    
    
    private double speedToAngle(Point2D speed){
        return -Math.atan2(speed.getY(), speed.getX());
    }
    private double pointAbs(Point2D p){
        return Math.sqrt(p.getX()*p.getX()+p.getY()*p.getY());
    }
    private Point2D pointAdd(Point2D p1, Point2D p2){
        return new Point2D.Double(p1.getX()+p2.getX(),p1.getY()+p2.getY());
    }
    private Point2D pointRotate(Point2D p,double radAngle){
        double x = p.getX()*Math.cos(radAngle)+p.getY()*Math.sin(radAngle);
        double y = -p.getX()*Math.sin(radAngle)+p.getY()*Math.cos(radAngle);
        return new Point2D.Double(x,y);
    }
    
}

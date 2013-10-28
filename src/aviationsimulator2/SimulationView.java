package aviationsimulator2;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.geom.Point2D;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JPanel;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author sa
 */

public class SimulationView extends JPanel{
    double[][] planePlot;
    double meterPlaneSize;
    boolean alwaysDrawPlaneOnCenter;
    double meterPerGrid;
    
    int screenWidth;
    int screenHeight;
    int pixcelsPerGrid;
    int screenShiftX;
    int screenShiftY;
    
    double secTime;
    double secTimeStep;
    int repaintDecimation;
    AviationModel model;
    
    boolean fastForward;
    
    RepaintTimer repaintTimer;
    public SimulationView() {
        super();
        
        meterPlaneSize = 0.6;
        alwaysDrawPlaneOnCenter = true;
        meterPerGrid = 5;
        
        secTimeStep = 0.001;
        repaintDecimation = 10;
        
        model = new AviationModel(secTimeStep);
        
        secTime = 0;
        initPlanePlot();
        
        fastForward = false;
        
        repaintTimer=null;
        //repaintTimer = new RepaintTimer(this);
        //repaintTimer.start();
    }
    public void startSimulation(){
        this.model = new AviationModel(0.001);
        if(repaintTimer==null){
            repaintTimer = new RepaintTimer(this);
            repaintTimer.start();
        }
    }
    public void startSimulation(double kgWeight, double nmInertia, WingModel mainWing, double mMainWingPos, WingModel elevator, double mElevatorPos, double extraDrag, RocketMoterModel moter, double secTimeStep, double radDirection,AviationModel.ControllerType controllerType){
        
        this.model = new AviationModel(kgWeight, nmInertia, mainWing, mMainWingPos, elevator, mElevatorPos, extraDrag, moter, secTimeStep, radDirection, controllerType);
        //this.model = new AviationModel(0.001);
        if(repaintTimer==null){
            repaintTimer = new RepaintTimer(this);
            repaintTimer.start();
        }
    }
    
    public void setFastForward(boolean fastForward){
        this.fastForward = fastForward;
    }
    
    class RepaintTimer extends Thread{
        SimulationView parent;
        public RepaintTimer(SimulationView parent){
            this.parent = parent;
        }
        @Override
        public void run(){
            long l=0;
            while(true){
                secTime=l*secTimeStep;
                model.updateDynamics(secTime);
                
                l++;
                if(l%repaintDecimation==0){
                    parent.repaint();
                }
                if(!fastForward){
                    try {
                        RepaintTimer.sleep((int)(1000*secTimeStep));
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                }
                
            }
        }
    }
    
    private void initPlanePlot(){
        planePlot = new double[2][16];
        //body
        planePlot[0][0] = 0;
        planePlot[1][0] = 0;
        planePlot[0][1] = -0.05;
        planePlot[1][1] = -0.05;
        planePlot[0][2] = -1;
        planePlot[1][2] = -0.05;
        planePlot[0][3] = -1;
        planePlot[1][3] = 0.05;
        planePlot[0][4] = -0.05;
        planePlot[1][4] = 0.05;
        planePlot[0][5] = 0;
        planePlot[1][5] = 0;
        planePlot[0][6] = -0.05;
        planePlot[1][6] = -0.05;
        //lower wing
        planePlot[0][7] = -0.8;
        planePlot[1][7] = -0.05;
        planePlot[0][8] = -0.85;
        planePlot[1][8] = -0.1;
        planePlot[0][9] = -1.0;
        planePlot[1][9] = -0.1;
        //center wing
        planePlot[0][10] = -1.0;
        planePlot[1][10] = 0.0;
        planePlot[0][11] = -0.8;
        planePlot[1][11] = 0.0;
        planePlot[0][12] = -1.0;
        planePlot[1][12] = 0.0;
        //upper wing
        planePlot[0][13] = -1.0;
        planePlot[1][13] = 0.1;
        planePlot[0][14] = -0.85;
        planePlot[1][14] = 0.1;
        planePlot[0][15] = -0.8;
        planePlot[1][15] = 0.05;

        for (int i = 0; i < planePlot[0].length; i++) {
            planePlot[0][i] += 0.5;
        }
    }
    
    @Override
    public void paint(Graphics g){
        super.paint(g);
        updateScreenParameter(g);
        
        drawGrid(g,model.getM_Pos());
        drawPlane(g, model.getM_Pos(), model.getRadDirection(), model.moter.getBoostPhase());
        drawStat(g);
    }
    
    private void drawStat(Graphics g){
        g.setColor(Color.BLACK);
        g.drawString("x:\t"+String.format("%.3f", model.getM_Pos().getX()), 10, 20);
        g.drawString("y:\t"+String.format("%.3f", model.getM_Pos().getY()), 10, 35);
        g.drawString("speed:\t"+String.format("%.3f", Math.sqrt(model.getMpsSpeed().getX()*model.getMpsSpeed().getX()+model.getMpsSpeed().getY()*model.getMpsSpeed().getY())), 10, 50);
        g.drawString("aoa[deg]:\t"+String.format("%.3f", model.getRadAoa()*180/Math.PI), 10, 65);
        g.drawString("state:\t"+model.controller.getState(), 10, 80);
        g.drawString("pitch:\t"+String.format("%.3f", -model.getRadDirection()*180/Math.PI), 10, 95);
        g.drawString("L/D:\t"+String.format("%.3f", model.getLbyD()), 10, 110);
    }
    
    private void drawPlane(Graphics g,Point2D mPos,double radDirection,int boostPhase){
        if(alwaysDrawPlaneOnCenter){
            Point2D p1,p2;
            mPos = new Point2D.Double(0,0);

            g.setColor(Color.black);

            for (int i = 0; i < planePlot[0].length-1; i++) {

                p1 = new Point2D.Double(planePlot[0][i],planePlot[1][i]);
                p2 = new Point2D.Double(planePlot[0][i+1],planePlot[1][i+1]);
                p1 = mPositionToScreenPosition(g, p1, radDirection, meterPlaneSize, mPos);
                p2 = mPositionToScreenPosition(g, p2, radDirection, meterPlaneSize, mPos);

                g.drawLine((int)(p1.getX()),
                        (int)(p1.getY()), 
                        (int)(p2.getX()),
                        (int)(p2.getY()));
            }

            if(boostPhase == 2){
                g.setColor(Color.red);
                p1 = new Point2D.Double(-0.5,0);
                p2 = new Point2D.Double(-0.9,0);
                p1 = mPositionToScreenPosition(g, p1, radDirection, meterPlaneSize, mPos);
                p2 = mPositionToScreenPosition(g, p2, radDirection, meterPlaneSize, mPos);

                g.setColor(Color.red);
                g.drawLine((int)(p1.getX()),
                           (int)(p1.getY()),
                           (int)(p2.getX()),
                           (int)(p2.getY())
                        );
            }
            g.setColor(Color.black);
        }else{
            Point2D p1,p2;

            g.setColor(Color.black);

            for (int i = 0; i < planePlot[0].length-1; i++) {

                p1 = new Point2D.Double(planePlot[0][i],planePlot[1][i]);
                p2 = new Point2D.Double(planePlot[0][i+1],planePlot[1][i+1]);
                p1 = mPositionToScreenPosition(g, p1, radDirection, meterPlaneSize, mPos);
                p2 = mPositionToScreenPosition(g, p2, radDirection, meterPlaneSize, mPos);

                g.drawLine((int)(p1.getX()),
                        (int)(p1.getY()), 
                        (int)(p2.getX()),
                        (int)(p2.getY()));
            }

            if(boostPhase == 2){
                g.setColor(Color.red);
                p1 = new Point2D.Double(-0.5,0);
                p2 = new Point2D.Double(-0.9,0);
                p1 = mPositionToScreenPosition(g, p1, radDirection, meterPlaneSize, mPos);
                p2 = mPositionToScreenPosition(g, p2, radDirection, meterPlaneSize, mPos);

                g.setColor(Color.red);
                g.drawLine((int)(p1.getX()),
                           (int)(p1.getY()),
                           (int)(p2.getX()),
                           (int)(p2.getY())
                        );
            }
            g.setColor(Color.black);
        }
    }
    

    
    private void drawGrid(Graphics g,Point2D mPos){      
        
        if(alwaysDrawPlaneOnCenter){
            int localScreenShiftX = (int)(mPos.getX()*pixcelsPerGrid/meterPerGrid);
            int localScreenShiftY = (int)(mPos.getY()*pixcelsPerGrid/meterPerGrid);
            
            g.setColor(Color.BLACK);
            g.drawLine(0, localScreenShiftY, screenWidth, localScreenShiftY);
            g.drawLine(localScreenShiftX , 0, localScreenShiftX, screenHeight);

            g.setColor(Color.lightGray);

            for (int i = localScreenShiftX+pixcelsPerGrid; i < screenWidth; i+=pixcelsPerGrid) {
                //tate right side
                g.drawLine(i, 0, i, screenHeight);
            }
            for (int i = localScreenShiftX-pixcelsPerGrid; 0 < i ; i-=pixcelsPerGrid) {
                //tate right side
                g.drawLine(i, 0, i, screenHeight);
            }
            for (int i = localScreenShiftY+pixcelsPerGrid; i < screenHeight; i+=pixcelsPerGrid) {
                //yoko down side
                g.drawLine(0, i, screenWidth, i);
            }
            for (int i = localScreenShiftY-pixcelsPerGrid; 0 < i ; i-=pixcelsPerGrid) {
                //yoko up side
                g.drawLine(0, i, screenWidth, i);
            }
        }else{
            g.setColor(Color.BLACK);
            g.drawLine(0, screenShiftY, screenWidth, screenShiftY);
            g.drawLine(screenShiftX , 0, screenShiftX, screenHeight);

            g.setColor(Color.lightGray);

            for (int i = screenShiftX+pixcelsPerGrid; i < screenWidth; i+=pixcelsPerGrid) {
                //tate right side
                g.drawLine(i, 0, i, screenHeight);
            }
            for (int i = screenShiftX-pixcelsPerGrid; 0 < i ; i-=pixcelsPerGrid) {
                //tate right side
                g.drawLine(i, 0, i, screenHeight);
            }
            for (int i = screenShiftY+pixcelsPerGrid; i < screenHeight; i+=pixcelsPerGrid) {
                //yoko down side
                g.drawLine(0, i, screenWidth, i);
            }
            for (int i = screenShiftY-pixcelsPerGrid; 0 < i ; i-=pixcelsPerGrid) {
                //yoko up side
                g.drawLine(0, i, screenWidth, i);
            }
        }
    }
    
    private void updateScreenParameter(Graphics g){
        screenWidth = g.getClipBounds().width;
        screenHeight= g.getClipBounds().height;
        pixcelsPerGrid = screenWidth/10;
        if(alwaysDrawPlaneOnCenter){
            screenShiftX = screenWidth/2;
            screenShiftY = screenHeight/2;
        }else{
            screenShiftX = screenWidth*9/10;
            screenShiftY = screenHeight*9/10;
        }
    }
    
    private Point2D shift(Point2D plot, Point2D pos){
        double x,y;
        x=plot.getX()+pos.getX();
        y=plot.getY()+pos.getY();
        return new Point2D.Double(x,y);
    }
    
    private Point2D scale(Point2D pos, double scale){
        double x,y;
        x=pos.getX()*scale;
        y=pos.getY()*scale;
        
        return new Point2D.Double(x,y);
    }
    
    private Point2D rotate(double x,double y,double rad){
        return new Point2D.Double(Math.cos(rad)*x+Math.sin(rad)*y,
                -Math.sin(rad)*x+Math.cos(rad)*y);
    }
    private Point2D rotate(Point2D plot,double rad){
        return new Point2D.Double(Math.cos(rad)*plot.getX()+Math.sin(rad)*plot.getY(),
                -Math.sin(rad)*plot.getX()+Math.cos(rad)*plot.getY());
    }
    
    private Point2D mPositionToScreenPosition(Graphics g,Point2D pos){
        double x,y;
        
        double scale = pixcelsPerGrid/meterPerGrid;
        
        x = screenShiftX - pos.getX()*scale;
        y = screenShiftY -pos.getY()*scale;
        
        return new Point2D.Double(x,y);
    }
    
    private Point2D mPositionToScreenPosition(Graphics g,Point2D plot,double radDirection,double mScale,Point2D mPos){
        Point2D p;
        
        p = rotate(plot, radDirection);
        p = scale(p, mScale);
        p = shift(p, mPos);
        p = mPositionToScreenPosition(g, p);

        return p;
    }
    
}

package aviationsimulator2;

import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author sa
 */
public class WingModel {
    static final double RHO = 1.225;
    static final double GRAVITY = 9.8;
    
    double m2Size;
    double radOffsetAngle;
    boolean enabled;
    double[] cl;
    double[] cd;
    
    double[] controlDelayRingBuf;
    int delayPointer;
    
    public WingModel(double m2Size,double radAngleOffset){
        this(m2Size,radAngleOffset,1);
    }
    public WingModel(double m2Size,double radAngleOffset,int msDelay){
        this("clcd/naca0012.txt",m2Size,radAngleOffset,msDelay);
    }
    public WingModel(String filename,double m2Size,double radAngleOffset){
        this(filename,m2Size,radAngleOffset,1);
    }
    public WingModel(String filename,double m2Size,double radAngleOffset,int msDelay){
        this.m2Size = m2Size;
        this.radOffsetAngle = radAngleOffset;
        enabled = true;
        
        cl = new double[361];
        cd = new double[361];
        
        controlDelayRingBuf = new double[msDelay];
        delayPointer=0;
        
        
        try {
            BufferedReader br = new BufferedReader(new FileReader("clcd/"+filename));
            String line;
            for (int i = 0; i < 5; i++) {
                br.readLine();
            }
            
            for (int i = 0; i < cl.length; i++) {
                line = br.readLine();
                String[] splited = line.split("\t");
                
                cl[i] = Double.parseDouble(splited[1]);
                cd[i] = Double.parseDouble(splited[2]);
            }
            
            br.close();
        } catch (Exception e) {
            e.printStackTrace();
            for (int i = 0; i < cl.length; i++) {
                cl[i] = 0.0;
                cd[i] = 0.0;
            }
        }
    }
    
    public Point2D getN_Force(double mpsTotalSpeed,double radAoa){
        if(!enabled){
            return new Point2D.Double(0,0);
        }else{
            double nLift;
            double nDrag;

            nLift = 1.0/2* RHO* (mpsTotalSpeed*mpsTotalSpeed)*m2Size*getCl(radAoa+radOffsetAngle);
            nDrag = 1.0/2* RHO* (mpsTotalSpeed*mpsTotalSpeed)*m2Size*getCd(radAoa+radOffsetAngle);

            return new Point2D.Double(-nDrag,nLift);
        }
    }
    
    public void setRadOffsetAngle(double radOffsetAngle){
        this.radOffsetAngle=controlDelayRingBuf[delayPointer];
        controlDelayRingBuf[delayPointer]=radOffsetAngle;
        
        delayPointer = (delayPointer+1)%controlDelayRingBuf.length;
        
    }
    
    public void setEnabled(boolean enabled){
        this.enabled=enabled;
    }
    
    private double getCl(double radAoa){
        while(radAoa<=-Math.PI){
            radAoa+=2*Math.PI;
        }
        while(Math.PI<=radAoa){
            radAoa -= 2*Math.PI;
        }
        double degAoa = radAoa*180/Math.PI;
        int ceil = (int)Math.ceil(degAoa);
        int floor = (int)Math.floor(degAoa);
        double cll=0;
        double clh=0;
        try{
            cll = cl[floor+180];
            clh = cl[ceil+180];
        }catch(Exception e){
            System.out.println(radAoa);
        }
            
        
        double mix = degAoa-floor;
        
        return cll*(1.0-mix)+clh*(mix);
    }
    private double getCd(double radAoa){
        while(radAoa<=-Math.PI){
            radAoa+=2*Math.PI;
        }
        while(Math.PI<=radAoa){
            radAoa -= 2*Math.PI;
        }
        double degAoa = radAoa*180/Math.PI;
        int ceil = (int)Math.ceil(degAoa);
        int floor = (int)Math.floor(degAoa);
        
        double cdl = cd[floor+180];
        double cdh = cd[ceil+180];
        
        double mix = degAoa-floor;
        
        return cdl*(1.0-mix)+cdh*(mix);
    }
}

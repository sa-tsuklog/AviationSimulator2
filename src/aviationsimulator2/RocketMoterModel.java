/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author sa
 */
public class RocketMoterModel {
    static final double W_RHO = 1000; //kg/m^3
    
    double timeStep;
    
    double atmPressure;
    double atmInitialPressue;
    double kgWaterWeight;
    double lTankSize;
    double kgInitialWaterWeight;
    double mNozzleDiameter;
    double m2NozzleArea;

    public RocketMoterModel(double timeStep,double atmInitialPressure,double kgWaterWeight,double lTankSize,double mNozzleDiameter) {
        this.timeStep = timeStep;
        this.atmPressure = this.atmInitialPressue =atmInitialPressure;
        this.kgWaterWeight = kgWaterWeight;
        this.lTankSize = lTankSize;
        this.mNozzleDiameter = mNozzleDiameter;
        this.m2NozzleArea = Math.PI*(mNozzleDiameter/2*mNozzleDiameter/2);
        
        this.kgInitialWaterWeight = kgWaterWeight;
    }
    
    public void update(){
        if(0<kgWaterWeight){
            double kgWaterEmitted;
            double mpsWaterEmittionSpeed = Math.sqrt(getN_CurrentForce()/W_RHO/m2NozzleArea);
            kgWaterEmitted = W_RHO*(m2NozzleArea) * mpsWaterEmittionSpeed * timeStep;
            
            this.kgWaterWeight -= kgWaterEmitted;
            this.atmPressure = atmInitialPressue*(lTankSize-kgInitialWaterWeight)/(lTankSize-kgWaterWeight);
            
            //System.out.println("pressure:"+String.format("%.2f", atmPressure)+", weight:"+String.format("%.3f", kgWaterWeight));
        }else{
            //do nothing.
        }
    }
    
    public double getN_CurrentForce(){
        if(0<kgWaterWeight){
            return 2*(m2NozzleArea) * (atmPressure-1.0)*100000;
        }else{
            return 0;
        }
    }
    public double getAtmCurrentPressure(){
        return atmPressure;
    }
    public double getKgCurrentWeight(){
        return kgWaterWeight;
    }
    public void reset(){
        kgWaterWeight = kgInitialWaterWeight;
        atmPressure = atmInitialPressue;
    }
}

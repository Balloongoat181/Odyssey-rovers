package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelFinder {
    static DistValMapValue [] aimValues = {
            new DistValMapValue(20, 1070),
            new DistValMapValue(30, 1200),
            new DistValMapValue(40, 1230),
            new DistValMapValue(50, 1250),
            new DistValMapValue(120, 1500)
    };
    public static double getVel(double distince){
        if (distince < aimValues[0].d){
            return aimValues[0].s;
        }
        if ((distince > aimValues[aimValues.length-1].d)){
            return aimValues[aimValues.length-1].s;
        }
        double lowerDistince = 0;
        double lowerSpeed = 0;
        double higherDistince = 0;
        double higherSpeed= 0;
        for (int i = 0;i< aimValues.length-1; i++){
            lowerDistince = aimValues [i].d;
            lowerSpeed = aimValues [i].s;
            higherDistince = aimValues [i+1].d;
            higherSpeed = aimValues [i+1].s;
            if (distince < higherDistince){
                return lowerSpeed + (distince-lowerDistince)*(higherSpeed-lowerSpeed)/(higherDistince-lowerDistince);
            }
        }
        return aimValues[0].s;
    }
}

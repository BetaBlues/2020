package  frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Piston{
// make abstract after we're done switching to subsystems

    DoubleSolenoid piston;
    boolean pistonForward;
    String pistonName; 

    public Piston (int forwardChannel, int reverseChannel, String name){
        piston = new DoubleSolenoid(forwardChannel,reverseChannel);
        pistonForward = true;
        pistonName = name;

        
    }
    public void toggle (){
        
       if (pistonForward){
        piston.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putString(pistonName, "Piston in");
        pistonForward = false;

       }else{
        piston.set(DoubleSolenoid.Value.kForward);
        SmartDashboard.putString(pistonName, "Piston out");
        pistonForward = true;

       }
    
    }

    

}
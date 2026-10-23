package frc.robot.Sales_because_if_6328_does_if_it_must_be_good;

import org.wpilib.math.kinematics.ChassisVelocities;

public class SalesAssociate {
    private static boolean doWeCareAboutCustomerService = false;
    public static void careAboutCustomerService(boolean yeOrNo){
        doWeCareAboutCustomerService = yeOrNo;
    }

    public static boolean getCareAboutCustomerService(){
        return doWeCareAboutCustomerService;
    }

    public static void greetFuel(){
        if (doWeCareAboutCustomerService)
            System.out.println("Hello fuel, We'd love to see you enter our hopper and create a mutual partnership");
    }

    public static void sayByeToFuel(){
        if (doWeCareAboutCustomerService)
            System.out.println("Goodbye fuel, we hope you had a great time");
    }

    public static void driving(ChassisVelocities robotSpeed){
        if (doWeCareAboutCustomerService){
            if (robotSpeed.vx > 0.5)
                System.out.println("NYOOOOOOOM, WE GOT THE ZOOMIES");
            else
                System.out.println("nyoom");
        }
    }

    public static void persuasion(){
        if (doWeCareAboutCustomerService)
          System.out.println("Hey, I see you don't want to leave us, but I must insist that you move on. We just think that you aren't a fit for our company. I'm sorry but you have to go.");
    }

}

package frc.robot;

public class RobotState {


    private static RobotState mInstance;

    public static RobotState getInstance(){
        if(mInstance == null){
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private RobotState(){
    }

    
}

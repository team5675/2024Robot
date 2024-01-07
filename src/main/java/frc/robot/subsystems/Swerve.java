public class Swerve {
    
    public static Swerve instance;


    public static Swerve getInstance() {
        if (instance == null) 
            instance = new Swerve();
        
        return instance;
    }
}

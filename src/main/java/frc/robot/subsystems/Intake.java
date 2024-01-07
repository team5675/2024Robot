public class Intake {
    
    public static Intake instance;


    public Intake getInstance() {
        if (instance == null) 
            instance = new Intake();

        return instance;
    }
}

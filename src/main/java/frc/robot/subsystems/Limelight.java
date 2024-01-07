public class Limelight {
    

    public static Limelight instance;


    public Limelight getInstance() {
        if(instance == null)
            instance = new Limelight();

        return instance;
    }
}

public class Launcher {
    
    public static Launcher instance;


    public Launcher getInstance() {
        if (instance == null) 
            instance = new Launcher();

        return instance;
    }
}

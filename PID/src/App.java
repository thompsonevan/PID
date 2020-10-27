public class App {
    public static void main(String[] args){
        double target = 1000;
        double currentPos = 10;

        PID pid = new PID(.06, .25, .01);

        boolean running = true;
    
        while(running) {
            //double error = target - currentPos;
            
            double change = pid.calculate(currentPos, target);
            
            currentPos = currentPos + change;

            System.out.println("The number is: " + currentPos);
        }

    }
}

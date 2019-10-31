import lejos.nxt.Button;

public class HelloWorld {
    /**
     * @param args uebergebene Argumente
     */
    public static void main(String[] args) {
        System.out.println("Hello World!");
        Button.waitForAnyPress();
    }
}
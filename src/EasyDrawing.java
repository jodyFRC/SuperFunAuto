import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

/**
 * Created by Jody on 10/2/2018.
 */
public class EasyDrawing extends Canvas {
    static ArrayList<Rectangle> rectangles = new ArrayList<>();

    public static Canvas beginDrawing() {
        JFrame frame = new JFrame();
        Canvas canvas = new EasyDrawing();
        canvas.setSize(1200, 1200);
        frame.add(canvas);
        frame.pack();
        frame.setVisible(true);
        canvas.setIgnoreRepaint(true);
        return canvas;
    }

    public static void addRectangle(Rectangle rectangle) {
        rectangles.add(rectangle);
    }

    public void paint(Graphics g) {
        for (int x = 0; x <= 1000; x = x + 100) {
            g.drawLine(x, 0, x, 1000);
        }
        for (int y = 0; y <= 1000; y = y + 100) {
            g.drawLine(0, y, 1000, y);
        }

        for (Rectangle rect : rectangles) {
            g.drawRect(rect.x, rect.y, rect.width, rect.height);
        }
    }
}

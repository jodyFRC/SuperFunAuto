import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by Jody on 10/2/2018.
 */
public class EasyDrawing extends Canvas {
    static ArrayList<Rectangle> rectangles = new ArrayList<>();
    static private Canvas canvas;

    public static Canvas beginDrawing() {
        JFrame frame = new JFrame();
        canvas = new EasyDrawing();
        canvas.setSize(2000, 1200);
        frame.add(canvas);
        frame.pack();
        frame.setVisible(true);
        canvas.setIgnoreRepaint(true);
        return canvas;
    }

    public static void addRectangle(Rectangle rectangle) {
        rectangles.add(rectangle);
    }

    public static void addRobotPose(double x, double y) {
        y = y + 1;
        int p_x = (int) (x * 100);
        int p_y = (int) (y * 100);
        addRectangle(new Rectangle(p_x, p_y, 1, 1));
        canvas.repaint();
    }

    public void paint(Graphics g) {
        try {
            File pathToFile = new File("F:\\Downloads\\3iwa9dircqd01.png");
            Image image = ImageIO.read(pathToFile);
            g.drawImage(image, 0, 0, null);
        } catch (IOException ex) {
            ex.printStackTrace();
        }

        g.setColor(Color.black);
        for (int x = 0; x <= 1000; x = x + 100) {
            g.drawLine(x + 300, 300, x + 300, 1000);
        }
        for (int y = 0; y <= 1000; y = y + 100) {
            g.drawLine(300, y + 300, 1300, y + 300);
        }
        g.setColor(Color.blue);
        for (Rectangle rect : rectangles) {
            g.drawRect(rect.x + 300, rect.y + 200, rect.width, rect.height);
        }
    }
}

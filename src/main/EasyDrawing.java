package main;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by Jody on 10/2/2018.
 */

class RobotStance {
    Rectangle rectangle;
    Color color;

    public RobotStance(Rectangle _rectangle, Color _color) {
        rectangle = _rectangle;
        color = _color;
    }
}

public class EasyDrawing extends JPanel {
    static public String debug_info = "";
    static ArrayList<RobotStance> rectangles = new ArrayList<>();
    static private JPanel canvas;

    public static JPanel beginDrawing() {
        JFrame frame = new JFrame();
        canvas = new EasyDrawing();
        frame.add(canvas);
        frame.pack();
        frame.setVisible(true);
        canvas.setIgnoreRepaint(true);
        canvas.setDoubleBuffered(true);

        frame.setSize(2000, 1200);

        return canvas;
    }

    public static void addRectangle(RobotStance stance) {
        rectangles.add(stance);
    }

    public static void addRobotPose(double x, double y, Color color) {
        int p_x = (int) (x * 10);
        int p_y = (int) (y * 10);
        addRectangle(new RobotStance(new Rectangle(p_x, 100 + p_y, 1, 1), color));
        canvas.repaint();
    }

    public void paint(Graphics g) {
        g.clearRect(0, 0, 1000, 1000);
        try {
            File pathToFile = new File("F:\\Downloads\\3iwa9dircqd01.png");
            Image image = ImageIO.read(pathToFile);
            //g.drawImage(image, 0, 0, null);
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
        for (RobotStance stance : rectangles) {
            Rectangle rect = stance.rectangle;
            g.setColor(stance.color);
            g.drawRect(rect.x + 300, rect.y + 200, rect.width, rect.height);
        }
        g.drawString(debug_info, 200, 200);
    }
}

package comm;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.swing.JComponent;
import javax.swing.JFrame;

public class BotClientMap {
	public double gridSize;
	public Pose startPose;
	public ArrayList<Wall> walls;
	
	public BotClientMap() {
		walls = new ArrayList<BotClientMap.Wall>();
	}
	
	public void load(String s) {
		String[] parts = s.split(":");
		int i = 0;
		this.gridSize = parseGridSize(parts[i++]);
		this.startPose = parsePose(parts[i++]);
		
		for (; i < parts.length; i++) {
			walls.add(parseWall(parts[i]));
		}
	}
	
	private double parseGridSize(String s) {
		return Double.valueOf(s);
	}
	
	private Pose parsePose(String s) {
		String[] parts = s.split(",");
		return new Pose(Double.valueOf(parts[0]), Double.valueOf(parts[1]), Double.valueOf(parts[2]));
	}
	
	private Wall parseWall(String s) {
		String[] parts = s.split(",");
		
		Point start = new Point(Double.valueOf(parts[0]), Double.valueOf(parts[1]));
		Point end = new Point(Double.valueOf(parts[2]), Double.valueOf(parts[3]));
		Wall.WallType type = Wall.WallType.values()[Wall.WallTypeShort.valueOf(parts[4]).ordinal()];
		
		return new Wall(start, end, type);
	}
	
	public static class Point {
		public final double x;
		public final double y;
		
		public Point (double x, double y) {
			this.x = x;
			this.y = y;
		}
		
		@Override
		public String toString() {
			return String.format("(%.2f, %.2f)", x, y);
		}
	}
	
	public static class Pose extends Point{
		public final double theta;
		public final double prob;
		
		public Pose(double x, double y, double theta) {
			super(x, y);
			this.theta = theta;
			this.prob = 0;
		}
		
		public Pose(double x, double y, double theta, double prob) {
			super(x, y);
			this.theta = theta;
			this.prob = prob;
		}
		
		@Override
		public String toString() {
			return String.format("(%.2f, %.2f, %.2f)", x, y, theta);
		}
	}
	
	public static class Wall {
		enum WallTypeShort {N, O, S, R};
		public enum WallType {NORMAL, OPPONENT, SILO, REACTOR};
		
		public final WallType type;
		public final Point start;
		public final Point end;
		
		public Wall(Point start, Point end, WallType type) {
			this.start = start;
			this.end = end;
			this.type = type;
		}
		

		@Override
		public String toString() {
			return String.format("Wall: %s\t[%s - %s]", type, start, end);
		}
	}
	
	@Override
	public String toString() {
		String mapString = String.format("Grid Size: %.2f\n", gridSize);
		mapString += "Pose: " + this.startPose.toString();
		for (Wall w : walls)
			mapString += "\n" + w.toString();
		return mapString;
	}
	
	public static BotClientMap getDefaultMap() {
		String mapString = "22.0:2.5,0.7,3:";
		mapString += "0,0,0,2,N:0,2,1,2,N:1,2,2,3,N:2,3,3,3,N:3,3,3,2,N:3,2,2,1,N:";
		mapString += "3,3,4,3,N:4,3,4,1,N:4,1,3,0,N:3,0,0,0,N:";
		BotClientMap m = new BotClientMap();
		m.load(mapString);
		return m;
	}
	
	public void drawMap() {
		JFrame jf = new JFrame();
		jf.setContentPane(new WallPainter());
		jf.setSize(500,500);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		jf.setBackground(Color.white);
		jf.setVisible(true);
	}
	
	private class WallPainter extends JComponent {
		@Override
		public void paint(Graphics g) {
			super.paint(g);
			int size = 50;
			g.translate(1*size, 8*size);
		    ((Graphics2D)g).setStroke(new BasicStroke(3));

			Color[] colors = new Color[] {Color.black, Color.yellow, new Color(255,0,255), Color.green};
			for (Wall w : walls) {
				g.setColor(colors[w.type.ordinal()]);
				g.drawLine(size * (int)w.start.x, size * -(int)w.start.y, size * (int)w.end.x, size * -(int)w.end.y);
			}
			
			g.fillOval((int) (size*startPose.x) - (int)(size/4.0),(int) (-size*startPose.y) - (int)(size/4.0), (int)(size/2.0), (int)(size/2.0));
			
			double DX = Math.cos(startPose.theta) * size/2.0;
			double DY = -Math.sin(startPose.theta) * size/2.0;
			g.drawLine((int) (size*startPose.x), (int) (-size*startPose.y), (int) (size*startPose.x + DX), (int) (-size*startPose.y + DY));
		}
	}	
	
	public static void main(String[] args) {
		BotClientMap map = getDefaultMap();
		map.drawMap();
	}
}
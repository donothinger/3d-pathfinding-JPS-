package com.donothinger.dto;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Scanner;
import java.util.Set;

import lombok.Data;

@Data
public class Map {
    private Set<Point> obstacleSet;
    private Integer height;
    private Integer width;
    private Integer depth;

    public Boolean inBounds(Point point) {
        return 0 <= point.x && point.x < this.height &&
                0 <= point.y && point.y < this.width &&
                0 <= point.z && point.z < this.depth;
    }

    public Boolean traversable(Point point) {
        return this.obstacleSet.contains(point);
    }

    public Boolean walkable(Point point) {
        return this.inBounds(point) && this.traversable(point);
    }

    public List<Point> getNeighbors(Point point) {
        Integer x = point.x;
        Integer y = point.y;
        Integer z = point.z;

        List<Point> neighbors = new ArrayList<>();
        List<Integer> coord_step = new ArrayList<>(Arrays.asList(-1, 0, 1));

        for (Integer dx : coord_step) {
            for (Integer dy : coord_step) {
                for (Integer dz : coord_step) {
                    Point next_point = new Point(x + dx, y + dy, z + dz);
                    if (this.walkable(next_point)) {
                        neighbors.add(next_point);
                    }
                }
            }
        }
        return neighbors;
    }

    public List<Point> getSuccessors(Point point, Point goalPoint) {
        return this.getNeighbors(point);
    }

    public Double computeCost(Point point1, Point point2) {
        return Math.sqrt((point1.x - point2.x) * (point1.x - point2.x) +
                (point1.y - point2.y) * (point1.y - point2.y) +
                (point1.z - point2.z) * (point1.z - point2.z));
    }

    public Map(String filePath) {
        Scanner scanner;
        try {
            scanner = new Scanner(new File(filePath));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return;
        }
        String line = scanner.nextLine();
        String[] parts = line.split(" ");
        Integer height = Integer.parseInt(parts[0]);
        Integer width = Integer.parseInt(parts[1]);
        Integer depth = Integer.parseInt(parts[2]);
        Set<Point> obstacleSet = new HashSet<Point>();
        while (scanner.hasNextLine()) {
            line = scanner.nextLine();
            parts = line.split(" ");
            if (parts.length < 3) {
                System.out.println("Некорректная строка для координат: " + line);
                continue;
            }
            Integer x = Integer.parseInt(parts[0]);
            Integer y = Integer.parseInt(parts[1]);
            Integer z = Integer.parseInt(parts[2]);
            Point newPoint = new Point(x, y, z);
            obstacleSet.add(newPoint);
        }
        this.height = height;
        this.width = width;
        this.depth = depth;
        this.obstacleSet = obstacleSet;
        scanner.close();
    }
}

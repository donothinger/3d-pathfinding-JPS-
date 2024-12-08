package com.donothinger.dto;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import lombok.Data;

@Data
public class Map {
    private Integer[][][] cells;
    private Integer height;
    private Integer width;
    private Integer depth;

    public Boolean inBounds(Point point) {
        return 0 <= point.x && point.x < this.height &&
                0 <= point.y && point.y < this.width &&
                0 <= point.z && point.z < this.depth;
    }

    public Boolean traversable(Point point) {
        return this.cells[point.x][point.y][point.z] != 1;
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

    public Map readFromFile(String filePath) {
        Scanner scanner;
        try {
            scanner = new Scanner(new File(filePath));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return null;
        }
        Integer height = scanner.nextInt(); 
        Integer width = scanner.nextInt();
        Integer depth = scanner.nextInt();
        Integer[][][] cells = new Integer[height][width][depth];
        while (scanner.hasNextInt()) {
            Integer x = scanner.nextInt();
            Integer y = scanner.nextInt();
            Integer z = scanner.nextInt();
            cells[x - 1][y - 1][z - 1] = 1;
        }
        scanner.close();
        return new Map(cells);

    }
    public Map(Integer[][][] new_cells) {
        this.cells = new_cells;
        this.height = new_cells.length;
        this.width = new_cells[0].length;
        this.depth = new_cells[0][0].length;
    }
}

package com.donothinger.heuristics;

import com.donothinger.dto.Point;

public interface Heuristic {
    public Double calculate(Point point1, Point point2);
}
package com.donothinger.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.EqualsAndHashCode;

@AllArgsConstructor
@Data
@EqualsAndHashCode(of = "point")
public class Node {
    public Point point;
    public Double g;
    public Double h;
    public Double f;
    public Node parent;

    public Node(Point point, Double g, Double h, Node parent) {
        this.point = point;
        this.g = g;
        this.h = h;
        this.f = g + h;
        this.parent = parent;
    }
}

package com.donothinger.astar;

import java.util.List;

import com.donothinger.dto.Map;
import com.donothinger.dto.Node;
import com.donothinger.dto.Point;
import com.donothinger.heuristics.Heuristic;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class Astar {
    public Map taskMap;
    public Point startPoint;
    public Point goalPoint;
    public Node startNode;
    public Node goalNode;
    public Heuristic heuristic;
    public SearchTree searchTree;

    public Node run() {
        Integer steps = 0;
        this.searchTree.addToOpen(this.startNode);

        while (!this.searchTree.openIsEmpty()) {
            steps += 1;
            Node currentNode = this.searchTree.getBestNodeFromOpen();
            this.searchTree.addToClosed(currentNode);
            System.out.println(steps);
            if (currentNode.getPoint() == this.goalNode.getPoint()) {
                System.out.println("During the search, the following number of OPEN dublicates was encountered: " +
                        String.valueOf(this.searchTree.getEncOpenDuplicates()));
                return currentNode;
            }

            List<Point> successors = this.taskMap.getSuccessors(currentNode.getPoint(), goalPoint);

            for (Point coord : successors) {
                Node newNode = new Node(coord,
                        currentNode.getG() + this.taskMap.computeCost(currentNode.getPoint(), coord),
                        this.heuristic.calculate(coord, this.goalPoint), currentNode);
                if (!this.searchTree.wasOpened(newNode)) {
                    this.searchTree.addToOpen(newNode);
                } else {
                    if (!this.searchTree.wasExpanded(newNode)) {
                        Node oldNode = this.searchTree.findNode(coord);
                        if (oldNode.getF() > newNode.getF()) {
                            oldNode.setG(newNode.getG());
                            oldNode.setF(newNode.getF());
                            oldNode.setParent(newNode.getParent());
                        }
                    }
                }
            }
            ;

        }
        return null;
    }

    public Astar(Map taskMap, Point starPoint, Point goalPoint, Heuristic heuristic, SearchTree searchTree) {
        this.taskMap = taskMap;
        this.startPoint = starPoint;
        this.goalPoint = goalPoint;
        this.heuristic = heuristic;
        this.searchTree = searchTree;

        this.startNode = new Node(startPoint, 0.0, heuristic.calculate(starPoint, goalPoint), null);
        this.goalNode = new Node(goalPoint, 0.0, 0.0, null);
    }

    public Astar(Map taskMap, Node startNode, Node goalNode, Heuristic heuristic, SearchTree searchTree) {
        this.taskMap = taskMap;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.heuristic = heuristic;
        this.searchTree = searchTree;

        this.startPoint = startNode.getPoint();
        this.goalPoint = goalNode.getPoint();
    }

}

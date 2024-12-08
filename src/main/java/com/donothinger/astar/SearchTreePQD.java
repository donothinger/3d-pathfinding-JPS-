package com.donothinger.astar;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Set;

import com.donothinger.dto.Node;
import com.donothinger.dto.Point;

import lombok.Data;

@Data
public class SearchTreePQD {
    private PriorityQueue<Node> openList = new PriorityQueue<>(Comparator.comparingDouble(Node::getF));
    private Set<Node> openedSet;
    private Set<Node> closedSet;
    private Integer encOpenDuplicates = 0;

    public Boolean openIsEmpty() {
        return openList.isEmpty();
    }

    public void addToOpen(Node node) {
        for (Node existingNode : openList) {
            if (existingNode.equals(node)) {
                if(existingNode.getG() > node.getG()) {
                    existingNode.setG(node.getG());
                    existingNode.setF(node.getF());
                    existingNode.setParent(node.getParent());
                }
                return ;
            }
        }

        openList.add(node);
    }

    public Node getBestNodeFromOpen() {
        if(this.openIsEmpty()) {
            return null;
        }
        Node bestNode = openList.remove();
        if(this.wasExpanded(bestNode)) {
            this.encOpenDuplicates += 1;
            return this.getBestNodeFromOpen();
        }

        return bestNode;
    }

    public Boolean wasExpanded(Node node) {
        return closedSet.contains(node);
    }

    public void addToClosed(Node node) {
        closedSet.add(node);
    }

    public Node findNode(Point point) {
        for(Node node: openList) {
            if(point.equals(node.getPoint())){
                return node;
            }
        }
        throw new IllegalArgumentException();
    }
}

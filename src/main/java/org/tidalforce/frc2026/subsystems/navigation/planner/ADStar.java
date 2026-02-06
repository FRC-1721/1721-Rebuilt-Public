package org.tidalforce.frc2026.subsystems.navigation.planner;

import org.tidalforce.frc2026.subsystems.navigation.grid.*;
import java.util.*;

public class ADStar {
    private final OccupancyGrid grid;
    private final Costmap costmap;
    private final Map<String, Node> nodes = new HashMap<>();
    private final PriorityQueue<Node> open =
        new PriorityQueue<>(Comparator.comparingDouble(this::key));

    private Node start, goal;

    public ADStar(OccupancyGrid grid) {
        this.grid = grid;
        this.costmap = new Costmap(grid);
    }

    private String id(int x, int y) { return x + "," + y; }

    private Node node(int x, int y) {
        return nodes.computeIfAbsent(id(x, y), k -> new Node(x, y));
    }

    private double heuristic(Node a, Node b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    private double key(Node n) {
        return Math.min(n.g, n.rhs) + heuristic(n, start);
    }

    public void initialize(int sx, int sy, int gx, int gy) {
        nodes.clear();
        open.clear();

        start = node(sx, sy);
        goal = node(gx, gy);

        goal.rhs = 0;
        open.add(goal);
    }

    private List<Node> neighbors(Node n) {
        List<Node> list = new ArrayList<>();
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                // no diagonal clipping
                if (dx != 0 && dy != 0) {
                    if (grid.isBlocked(n.x + dx, n.y) ||
                        grid.isBlocked(n.x, n.y + dy))
                        continue;
                }

                int nx = n.x + dx;
                int ny = n.y + dy;
                if (!grid.isBlocked(nx, ny)) list.add(node(nx, ny));
            }
        }
        return list;
    }

    private double cost(Node a, Node b) {
        return Math.hypot(a.x - b.x, a.y - b.y) + costmap.cost(b.x, b.y);
    }

    private void update(Node u) {
        if (!u.equals(goal)) {
            u.rhs = Double.POSITIVE_INFINITY;
            for (Node s : neighbors(u))
                u.rhs = Math.min(u.rhs, s.g + cost(u, s));
        }
        open.remove(u);
        if (u.g != u.rhs) open.add(u);
    }

    public void compute() {
        while (!open.isEmpty() && open.peek().g != open.peek().rhs) {
            Node u = open.poll();
            if (u.g > u.rhs) {
                u.g = u.rhs;
                for (Node s : neighbors(u)) update(s);
            } else {
                u.g = Double.POSITIVE_INFINITY;
                update(u);
                for (Node s : neighbors(u)) update(s);
            }
        }
    }

    public List<Node> extractPath() {
        List<Node> path = new ArrayList<>();
        Node cur = start;
        path.add(cur);

        while (!cur.equals(goal)) {
            Node best = null;
            double bestCost = Double.POSITIVE_INFINITY;

            for (Node s : neighbors(cur)) {
                double c = cost(cur, s) + s.g;
                if (c < bestCost) {
                    bestCost = c;
                    best = s;
                }
            }

            if (best == null) break;
            cur = best;
            path.add(cur);
        }
        return path;
    }
}

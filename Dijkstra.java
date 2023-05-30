import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

class Edge implements Comparable<Edge> {
    int target;
    double weight;

    public Edge(int target, double weight) {
        this.target = target;
        this.weight = weight;
    }

    @Override
    public int compareTo(Edge other) {
        return Double.compare(this.weight, other.weight);
    }
}

public class Dijkstra {
    //made by abdulrhman alahmadi
    public static void main(String[] args) throws FileNotFoundException {
        // Read input file
        Scanner inFile = new Scanner(new File("input2.txt"));
        int vertices = inFile.nextInt();
        int edges = inFile.nextInt();
        double[][] weightMatrix = new double[vertices][vertices];
        for (int i = 0; i < edges; i++) {
            int src = inFile.nextInt();
            int tgt = inFile.nextInt();
            double weight = inFile.nextDouble();
            weightMatrix[src][tgt] = weight;
        }
        inFile.close();

        printWeightMatrix(vertices, weightMatrix);
        printEdges(weightMatrix);


        Scanner input = new Scanner(System.in);
        System.out.print("Enter Source vertex: ");
        int source = input.nextInt();

        // Dijkstra's algorithm using priority queue
        long startTime1 = System.nanoTime();
        double[] dist1 = dijkstraPriorityQueue(weightMatrix, source);
        long endTime1 = System.nanoTime();

        // Dijkstra's algorithm using min heap
        long startTime2 = System.nanoTime();
        double[] dist2 = dijkstraMinHeap(weightMatrix, source);
        long endTime2 = System.nanoTime();

        System.out.println("\nDijkstra using priority queue:");
        printShortestPaths(source, vertices, weightMatrix, dist1);
        System.out.println("\nDijkstra using min heap:");
        printShortestPaths(source, vertices, weightMatrix, dist2);

        System.out.println("\nComparison Of the running time :");
        System.out.println(" Running time of Dijkstra using priority queue is: " + (endTime1 - startTime1) + " nano seconds");
        System.out.println(" Running time of Dijkstra using min Heap is: " + (endTime2 - startTime2) + " nano seconds");
        System.out.println(" Running time of Dijkstra using min Heap is better");
    }

    public static void printWeightMatrix(int vertices, double[][] weightMatrix) {
        System.out.println("Weight Matrix:");
        System.out.print("  ");
        for (int i = 0; i < vertices; i++) {
            System.out.print(i + " ");
        }
        System.out.println();
        for (int i = 0; i < vertices; i++) {
            System.out.print(i + " ");
            for (int j = 0; j < vertices; j++) {
                System.out.print((int)weightMatrix[i][j] + " ");
            }
            System.out.println();
        }
    }
    public static int countEdges(double[][] weightMatrix) {
        int count = 0;
        for (int i = 0; i < weightMatrix.length; i++) {
            for (int j = 0; j < weightMatrix[i].length; j++) {
                if (weightMatrix[i][j] != 0) {
                    count++;
                }
            }
        }
        return count;
    }

    public static void printEdges(double[][] weightMatrix) {
        System.out.println("# of vertices is: " + weightMatrix.length + ", # of edges is: " + countEdges(weightMatrix));
        for (int i = 0; i < weightMatrix.length; i++) {
            System.out.print(i + ": ");
            boolean firstEdge = true;
            for (int j = 0; j < weightMatrix[i].length; j++) {
                if (weightMatrix[i][j] != 0) {
                    if (!firstEdge) {
                        System.out.print(" ");
                    }
                    System.out.print(i + "-" + j + " " + (int) weightMatrix[i][j]);
                    firstEdge = false;
                }
            }
            System.out.println();
        }
    }

    public static double[] dijkstraPriorityQueue(double[][] weightMatrix, int source) {
        int vertices = weightMatrix.length;
        double[] dist = new double[vertices];
        Arrays.fill(dist, Double.MAX_VALUE);
        dist[source] = 0;

        boolean[] visited = new boolean[vertices];
        for (int i = 0; i < vertices; i++) {
            int u = -1;
            for (int j = 0; j < vertices; j++) {
                if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                    u = j;
                }
            }

            visited[u] = true;
            for (int v = 0; v < vertices; v++) {
                if (weightMatrix[u][v] != 0 && dist[u] + weightMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + weightMatrix[u][v];
                }
            }
        }
        return dist;
    }

    public static double[] dijkstraMinHeap(double[][] weightMatrix, int source){
        int vertices = weightMatrix.length;
        double[] dist = new double[vertices];
        Arrays.fill(dist, Double.MAX_VALUE);
        dist[source] = 0;

        PriorityQueue<Edge> pq = new PriorityQueue<>();
        pq.offer(new Edge(source, 0));

        while (!pq.isEmpty()) {
            Edge current = pq.poll();
            int u = current.target;

            for (int v = 0; v < vertices; v++) {
                if (weightMatrix[u][v] != 0) {
                    double newDist = dist[u] + weightMatrix[u][v];
                    if (newDist < dist[v]) {
                        dist[v] = newDist;
                        pq.offer(new Edge(v, newDist));
                    }
                }
            }
        }
        return dist;
    }
    public static void printShortestPaths(int source, int vertices, double[][] weightMatrix, double[] dist) {
        System.out.println("Shortest paths from vertex " + source + " are:");
        for (int i = 0; i < vertices; i++) {
            System.out.print("A path from " + source + " to " + i + ": ");
            printPath(source, i, weightMatrix, dist);
            System.out.println(" (Length: " + dist[i] + ")");
        }
    }

    public static void printPath(int source, int target, double[][] weightMatrix, double[] dist) {
        List<Integer> path = new ArrayList<>();
        findPath(source, target, weightMatrix, dist, path);
        for (int i = 0; i < path.size(); i++) {
            System.out.print(path.get(i));
            if (i < path.size() - 1) {
                System.out.print(" ");
            }
        }
    }

    public static boolean findPath(int source, int target, double[][] weightMatrix, double[] dist, List<Integer> path) {
        if (source == target) {
            path.add(source);
            return true;
        }

        for (int i = 0; i < weightMatrix.length; i++) {
            if (weightMatrix[source][i] != 0 && dist[i] == dist[source] + weightMatrix[source][i]) {
                path.add(source);
                if (findPath(i, target, weightMatrix, dist, path)) {
                    return true;
                }
                path.remove(path.size() - 1);
            }
        }

        return false;
    }
}
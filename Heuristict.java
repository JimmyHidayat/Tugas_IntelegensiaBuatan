import java.util.*;

class Graph {
    private int V; // Jumlah simpul (vertices) dalam graf
    private LinkedList<GraphNode>[] adj; // Daftar adjacency untuk setiap simpul

    public Graph(int v) {
        V = v;
        adj = new LinkedList[v];
        for (int i = 0; i < v; i++) {
            adj[i] = new LinkedList<>(); // Inisialisasi daftar adjacency untuk setiap simpul
        }
    }

    public void addEdge(int src, int dest, int weight) {
        adj[src].add(new GraphNode(dest, weight)); // Menambahkan tepian dari simpul src ke simpul dest dengan bobot tertentu
    }

    public LinkedList<GraphNode> getNeighbors(int v) {
        return adj[v]; // Mengembalikan daftar adjacency dari simpul v
    }
}

class GraphNode {
    int vertex; // Nomor simpul
    int weight; // Bobot atau jarak dari simpul saat ini ke simpul tujuan

    public GraphNode(int v, int w) {
        vertex = v;
        weight = w;
    }
}

class AStarSearch {
    public static List<Integer> astarSearch(Graph graph, int start, int goal) {
        PriorityQueue<AStarNode> openSet = new PriorityQueue<>();
        openSet.add(new AStarNode(start, 0, heuristic(start, goal), null)); // Menambahkan simpul awal ke himpunan terbuka dengan gScore awal, fScore, dan tanpa simpul induk

        Map<Integer, AStarNode> visited = new HashMap<>(); // Untuk melacak simpul yang telah dikunjungi

        while (!openSet.isEmpty()) {
            AStarNode current = openSet.poll(); // Mengambil simpul dengan fScore terendah dari himpunan terbuka

            if (current.vertex == goal) {
                // Jika simpul tujuan ditemukan, rekonstruksi dan kembalikan jalur
                return reconstructPath(current);
            }

            if (visited.containsKey(current.vertex)) {
                continue;
            }

            visited.put(current.vertex, current); // Menandai simpul saat ini sebagai sudah dikunjungi

            for (GraphNode neighbor : graph.getNeighbors(current.vertex)) {
                int neighborVertex = neighbor.vertex;
                int tentativeGScore = current.gScore + neighbor.weight; // Biaya aktual untuk mencapai tetangga ini
                AStarNode neighborNode = new AStarNode(neighborVertex, tentativeGScore, heuristic(neighborVertex, goal), current);

                openSet.add(neighborNode); // Menambahkan tetangga ke himpunan terbuka
            }
        }

        return new ArrayList<>(); // Jika tidak ada jalur yang ditemukan, kembalikan daftar kosong
    }

    private static List<Integer> reconstructPath(AStarNode node) {
        List<Integer> path = new ArrayList<>();
        while (node != null) {
            path.add(node.vertex); // Menambahkan simpul ke jalur
            node = node.parent; // Pindah ke simpul sebelumnya dalam jalur
        }
        Collections.reverse(path); // Membalikkan jalur karena jalur direkonstruksi dari simpul tujuan ke simpul awal
        return path;
    }

    private static int heuristic(int current, int goal) {
        // Anda dapat mendefinisikan fungsi heuristik sesuai dengan kebutuhan, contoh: Manhattan distance
        return Math.abs(current - goal);
    }

    static class AStarNode implements Comparable<AStarNode> {
        int vertex; // Nomor simpul
        int gScore; // Biaya aktual dari simpul awal ke simpul ini
        int fScore; // gScore ditambah heuristic
        AStarNode parent; // Simpul sebelumnya dalam jalur

        public AStarNode(int vertex, int gScore, int fScore, AStarNode parent) {
            this.vertex = vertex;
            this.gScore = gScore;
            this.fScore = fScore;
            this.parent = parent;
        }

        @Override
        public int compareTo(AStarNode other) {
            return Integer.compare(this.fScore, other.fScore); // Membandingkan simpul berdasarkan fScore
        }
    }
}

class GreedyBFSSearch {
    public static List<Integer> greedyBFS(Graph graph, int start, int goal) {
        PriorityQueue<GreedyBFSNode> openSet = new PriorityQueue<>();
        openSet.add(new GreedyBFSNode(start, heuristic(start, goal))); // Menambahkan simpul awal ke himpunan terbuka dengan heuristic

        Map<Integer, GreedyBFSNode> visited = new HashMap<>(); // Untuk melacak simpul yang telah dikunjungi

        while (!openSet.isEmpty()) {
            GreedyBFSNode current = openSet.poll(); // Mengambil simpul dengan heuristic terendah dari himpunan terbuka

            if (current.vertex == goal) {
                // Jika simpul tujuan ditemukan, rekonstruksi dan kembalikan jalur
                return reconstructPath(current);
            }

            if (visited.containsKey(current.vertex)) {
                continue;
            }

            visited.put(current.vertex, current); // Menandai simpul saat ini sebagai sudah dikunjungi

            for (GraphNode neighbor : graph.getNeighbors(current.vertex)) {
                int neighborVertex = neighbor.vertex;
                GreedyBFSNode neighborNode = new GreedyBFSNode(neighborVertex, heuristic(neighborVertex, goal));

                openSet.add(neighborNode); // Menambahkan tetangga ke himpunan terbuka
            }
        }

        return new ArrayList<>(); // Jika tidak ada jalur yang ditemukan, kembalikan daftar kosong
    }

    private static List<Integer> reconstructPath(GreedyBFSNode node) {
        List<Integer> path = new ArrayList<>();
        while (node != null) {
            path.add(node.vertex); // Menambahkan simpul ke jalur
            node = node.parent; // Pindah ke simpul sebelumnya dalam jalur
        }
        Collections.reverse(path); // Membalikkan jalur karena jalur direkonstruksi dari simpul tujuan ke simpul awal
        return path;
    }

    private static int heuristic(int current, int goal) {
        // Anda dapat mendefinisikan fungsi heuristik sesuai dengan kebutuhan, contoh: Manhattan distance
        return Math.abs(current - goal);
    }

    static class GreedyBFSNode implements Comparable<GreedyBFSNode> {
        int vertex; // Nomor simpul
        int fScore; // Heuristic
        GreedyBFSNode parent; // Simpul sebelumnya dalam jalur

        public GreedyBFSNode(int vertex, int fScore) {
            this.vertex = vertex;
            this.fScore = fScore;
        }

        @Override
        public int compareTo(GreedyBFSNode other) {
            return Integer.compare(this.fScore, other.fScore); // Membandingkan simpul berdasarkan heuristic (fScore)
        }
    }
}

public class Heuristict {
    public static void main(String[] args) {
        Graph graph = new Graph(6);

        // Menambahkan tepian ke graf
        graph.addEdge(0, 1, 2);
        graph.addEdge(0, 2, 4);
        graph.addEdge(1, 3, 7);
        graph.addEdge(2, 4, 3);
        graph.addEdge(3, 5, 1);
        graph.addEdge(4, 5, 5);

        int start = 0;
        int goal = 5;

        // Menggunakan A* Search untuk mencari jalur terpendek dari simpul awal ke simpul tujuan
        List<Integer> aStarPath = AStarSearch.astarSearch(graph, start, goal);
        System.out.println("A* Path: " + aStarPath);

        // Menggunakan Greedy Best-First Search untuk mencari jalur terpendek dari simpul awal ke simpul tujuan
        List<Integer> greedyBFSPath = GreedyBFSSearch.greedyBFS(graph, start, goal);
        System.out.println("Greedy BFS Path: " + greedyBFSPath);
    }
}

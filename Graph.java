package graphs;
import java.lang.Exception;
import java.util.*;
import java.io.*;

public class Graph {
    static class Edge{
        int weight,to_vertex;
        Edge(int tv){
            this.to_vertex=tv;
        }
        Edge(int tv,int w){
            this.to_vertex=tv;
            this.weight=w;
        }
    }

    // function to add an edge in adjacency list
    static void addEdge(ArrayList<ArrayList<Edge>>adj,int from,int to,int weight){
        ArrayList<Edge>ls;
        Edge edge=new Edge(to,weight);
        ls=adj.get(from);
        ls.add(edge);
    }
    // ---------------------------------------------------------------------------------------------------------
    
    // Breadth First Search for a graph (covered for disconnected graph as well)
    static void bfs(ArrayList<ArrayList<Edge>>adj,int n,int e){

        boolean visited[]=new boolean[n];
        System.out.println("BFS Traversal of Graph");
        
        // for loop to consider for disconnected graph also
        for(int i=0;i<n;i++){
            
            if(!visited[i]){
                // bfs
                Queue<Integer>q=new LinkedList<>();
                q.add(i);
                visited[i]=true;
                StringBuilder sb=new StringBuilder();

                while(!q.isEmpty()){
                    int edge=q.poll();
                    sb.append(" -> "+edge);
                    for(Edge adj_edge:adj.get(edge)){
                        if(!visited[adj_edge.to_vertex]){
                            q.add(adj_edge.to_vertex);
                            visited[adj_edge.to_vertex]=true;
                        }
                    }
                }

                System.out.println(sb.toString());
            }
        }
    }
    // ---------------------------------------------------------------------------------------------------------

    // Depth first search for a graph (covered for disconnected graph as well)
    static void dfsUtil(ArrayList<ArrayList<Edge>>adj,boolean visited[],StringBuilder sb,int from){
        sb.append(" -> "+from);
        visited[from]=true;

        for(Edge edge:adj.get(from)){
            if(!visited[edge.to_vertex]){
                dfsUtil(adj, visited, sb, edge.to_vertex);
            }
        }
    }
    static void dfs(ArrayList<ArrayList<Edge>>adj,int n,int e){
        boolean visited[]=new boolean[n];

        for(int i=0;i<n;i++){
            if(!visited[i]){
                StringBuilder sb=new StringBuilder();
                dfsUtil(adj,visited,sb,0);
                System.out.println(sb.toString());
            }
        }
    }
    // ---------------------------------------------------------------------------------------------------------

    // Pair class to keep track of parent of a vertex during traversal
    static class Pair{
        int v,prev;
        Pair(int v,int prev){
            this.v=v;
            this.prev=prev;
        }
    }
    // ---------------------------------------------------------------------------------------------------------

    // checking if undirected graph is bipartite or not, using DFS
    static boolean isBipartitieUsingDFS(ArrayList<ArrayList<Integer>> adj, int src, int prevColor,int color[]){
        color[src]=prevColor;

        for(int adj_vertex:adj.get(src)){
            if(color[adj_vertex]==0){
                if(!isBipartitieUsingDFS(adj, adj_vertex, prevColor==1?2:1 , color)){
                    return false;
                }
            }else if(color[adj_vertex]==color[src]){
                return false;
            }
        }
        return true;
    }
    static void checkBipartiteDFS(ArrayList<ArrayList<Integer>> adj, int n, int e){
        int color[]=new int[n];

        for(int i=0;i<n;i++){
            if(color[i]==0){
                if(!isBipartitieUsingDFS(adj, i, 1, color)){
                    System.out.println("Not Bipartite");
                    return;
                }
            }
        }
        System.out.println("Yes Graph is Bipartite");
    }
    // ---------------------------------------------------------------------------------------------------------

    // checking if undirected graph is bipartite or not, using BFS
    static boolean isBipartitieUsingBFS(ArrayList<ArrayList<Integer>> adj, int src, int color[]){

        Queue<Integer>q=new LinkedList<>();

        q.add(src);
        color[src]=1;

        while(!q.isEmpty()){
            int parent=q.poll();

            for(int adj_vertex:adj.get(parent)){
                if(color[adj_vertex]==0){
                    q.add(adj_vertex);
                    color[adj_vertex]=color[parent]==1?2:1;
                }else if(color[adj_vertex]==color[parent]){
                    return false;
                }
            }
        }

        return true;
    }
    static void checkBipartiteBFS(ArrayList<ArrayList<Integer>> adj,int n,int e){
        int visited[]=new int[n];

        for(int i=0;i<n;i++){
            if(visited[i]==0){
                if(!isBipartitieUsingBFS(adj, i, visited)){
                    System.out.println("Not bipartite");
                    return;
                }
            }
        }
        System.out.println("Yes Graph is Bipartite");
    }
    // ---------------------------------------------------------------------------------------------------------

    //cycle detection in undirected graph using DFS
    static boolean isCycelExistsDFS(ArrayList<ArrayList<Integer>> adj, Pair src,boolean visited[]){
        visited[src.v]=true;
        
        for(int adj_vertex:adj.get(src.v)){
            if(!visited[adj_vertex]){
                if(isCycelExistsDFS(adj, new Pair(adj_vertex,src.v), visited)){
                    return true;
                }
            }else if(adj_vertex!=src.prev){
                return true;
            }
        }
        return false;
    }
    static void detectCycelUsingDFS(ArrayList<ArrayList<Integer>> adj, int n, int e){

        boolean visited[]=new boolean[n];

        for(int i=0;i<n;i++){
            if(!visited[i]){
                if(isCycelExistsDFS(adj,new Pair(i,-1),visited)){
                    System.out.println("Cycle Exist");
                    return;
                }
            }
        }

    } 
    // ---------------------------------------------------------------------------------------------------------

    // cycle detection in undirected graph using BFS
    static boolean cycleExistBFS(ArrayList<ArrayList<Integer>>adj,boolean visited[],int src){
        boolean cycle = false;
        Queue<Pair> q = new LinkedList<>();
        q.add(new Pair(src,-1));
        visited[src]=true;
        
        while(!q.isEmpty()){
            Pair head = q.poll();

            for(int adj_vertex:adj.get(head.v)){
                if(visited[adj_vertex] && adj_vertex!=head.prev){
                    return true;
                }
                if(!visited[adj_vertex]){
                    q.add(new Pair(adj_vertex,head.v));
                    visited[adj_vertex]=true;
                }
            }
        }
        return cycle;
    }
    static void detectCycleUsingBFS(ArrayList<ArrayList<Integer>>adj,int n,int e){

        boolean visited[] = new boolean[n];

        for(int i = 0 ; i < n ; i++ ){
            if(!visited[i]){
                if(cycleExistBFS(adj, visited, i)) {
                    System.out.println("Cycle exists");
                    return;
                }
            }
        }
        System.out.println("No cycle exist");
    }
    // ---------------------------------------------------------------------------------------------------------

    // cycle detection in a directed graph
    static boolean cycleExistInDirectedGraphDFS(int src,ArrayList<ArrayList<Integer>> adj, boolean dfs[],boolean visited[]){
        visited[src]=true;
        dfs[src]=true;
        
        for(int adj_vertex:adj.get(src)){
            if(!visited[adj_vertex]){
                if(cycleExistInDirectedGraphDFS(adj_vertex,adj,dfs,visited)){
                    return true;
                }
            }else if(dfs[adj_vertex]){
                return true;
            }
        }
        dfs[src]=false;
        return false;
    }
    static void detectCycleInDirectedGraph(ArrayList<ArrayList<Integer>> adj, int n, int e){
        boolean visited[]=new boolean[n];
        boolean dfsVisited[]=new boolean[n];
        
        for(int i=0;i<n;i++){
            if(!visited[i]){
                if(cycleExistInDirectedGraphDFS(i,adj,dfsVisited,visited)){
                    System.out.println("Cycle Exists in Directed Graph");
                    return;
                }
            }
        }

        System.out.println("Cycle doesn't exists in Directed Graph");
    }
    // ---------------------------------------------------------------------------------------------------------

    //cycle detection in a directed graph using BFS (Kahn's Algorithm)
    static void detectCycleInDirectedGraphBFS(ArrayList<ArrayList<Integer>> adj, int n,int e){
        int indegree[]=new int[n];
        Queue<Integer>q=new LinkedList<>();
        
        for(int i=0;i<n;i++){
            for(int adj_vertex:adj.get(i)){
                indegree[adj_vertex]++;
            }
        }

        for(int i=0;i<n;i++){
            if(indegree[i]==0){
                q.add(i);
            }
        }
        
        int k=0;
        while(!q.isEmpty()){
            int head=q.poll();
            
            k++;
            
            for(int adj_vertex:adj.get(head)){
                indegree[adj_vertex]--;
                if(indegree[adj_vertex]==0){
                    q.add(adj_vertex);
                }
            }
        }
        
        if(k!=n){
            System.out.println("Cycle Exists");
        }else{
            System.out.println("Cycle doesn't Exists");
        }
    }
    // ---------------------------------------------------------------------------------------------------------

    // shortest path in undirected graph with unit wieght
    static void shortestPath_inUndirectedGraph_withUnitWeight_BFS(ArrayList<ArrayList<Integer>>adj,int src,int n){

        Queue<Integer>q=new LinkedList<>();
        int distance[]=new int[n];
        Arrays.fill(distance,Integer.MAX_VALUE);

        q.add(src);
        distance[src]=0;

        while(!q.isEmpty()){
            int vertex=q.poll();

            for(int adj_vertex:adj.get(vertex)){
                if(distance[vertex]+1 <distance[adj_vertex]){
                    q.add(adj_vertex);
                    distance[adj_vertex]=distance[vertex]+1;
                }
            }
        }

        for(int i=0;i<n;i++){
            System.out.println("Distance from "+src+" to "+i+" = "+distance[i]);
        }

    }
    // ---------------------------------------------------------------------------------------------------------

    // Dijkstra's Algorithm
    // shortest path in undirected weighted graph ; T.C.-> O((N+E)logN) using PriorityQueue(min Heap)
    static void shortestPath_undirectedWeightedGraph(ArrayList<ArrayList<Edge>> adj,int src,int n){

        PriorityQueue<Edge>pq = new PriorityQueue<>((e1,e2)->(e1.weight-e2.weight));
        int distance[]=new int[n];
        Arrays.fill(distance,Integer.MAX_VALUE);
        distance[src]=0;
        pq.add(new Edge(src, distance[src]));

        while(!pq.isEmpty()){
            int vertex=pq.poll().to_vertex;

            for(Edge adj_Edge:adj.get(vertex)){
                if(distance[vertex] + adj_Edge.weight < distance[adj_Edge.to_vertex] ){
                    distance[adj_Edge.to_vertex]=distance[vertex]+adj_Edge.weight;
                    pq.add(new Edge(adj_Edge.to_vertex, distance[adj_Edge.to_vertex]));
                }
            }
        }

        for(int i=0;i<n;i++){
            System.out.println("Distance of "+i+ " from "+ src + " = "+distance[i]);
        }
    }
    // ---------------------------------------------------------------------------------------------------------

    // ****************** Minimum Spanning Tree **************************

    // Prim's Algorithm
    static void mstUsingPrimsNaive(ArrayList<ArrayList<Edge>> adj, int n){

        boolean mst[]=new boolean[n];

        int keys[]=new int[n];
        Arrays.fill(keys,Integer.MAX_VALUE);
        keys[0]=0;

        int parent[]=new int[n];
        Arrays.fill(parent,-1);


        int edgesCnt=0;


        // O(n*(n+e)) ~ O(n^2)
        while(edgesCnt < n-1){
            int mini=Integer.MAX_VALUE,u=0,v,w;

            // O(n)
            for(int i=0;i<n;i++){
                if(!mst[i] && keys[i]<mini){
                    mini=keys[i];
                    u=i;
                }
            }

            mst[u]=true;

            // O(n+e)
            for(Edge adj_edge:adj.get(u)){
                v=adj_edge.to_vertex;
                w=adj_edge.weight;
                if(!mst[v] && w<keys[v]){
                    keys[v]=w;  
                    parent[v]=u;
                }
            }
            edgesCnt++;
        }

        System.out.println("Prim's MST");
        for(int i=0;i<n;i++){
            System.out.println(parent[i]+" -> "+ i);
        }

    }
    
    static void mstUsingPrimsOptimized(ArrayList<ArrayList<Edge>> adj, int n){
        boolean mst[]=new boolean[n];
        int keys[]=new int[n];
        int parent[]=new int[n];
        Arrays.fill(keys, Integer.MAX_VALUE);
        Arrays.fill(parent,-1);

        PriorityQueue<Edge>pq=new PriorityQueue<>((e1,e2)->(e1.weight-e2.weight));
        pq.add(new Edge(0, 0));
        keys[0]=0;

        int edgesCnt=0,u,v,w;

        // O(nlogn)
        while(edgesCnt<n-1){

            u=pq.poll().to_vertex;
            // if previously visited then continue
            if(mst[u]) continue;
            mst[u]=true;

            for(Edge adj_edge:adj.get(u)){
                v=adj_edge.to_vertex;
                w=adj_edge.weight;
                if(!mst[v] && w<keys[v]){
                    keys[v]=w;
                    parent[v]=u;
                    pq.add(new Edge(v,w));
                }
            }

            edgesCnt++;
        }

        System.out.println("Prims MST using Min Heap");
        for(int i=0;i<n;i++){
            System.out.println(parent[i]+" -> "+i);
        }
    }


    // ****************** Directed Acyclic Graph *************************
    
    // topological sort(DAG) using DFS
    static void topologicalSortDFS(ArrayList<ArrayList<Integer>> adj, int src, boolean visited[],Stack<Integer> st){
        visited[src]=true;

        for(int adj_vertex:adj.get(src)){
            if(!visited[adj_vertex]){
                topologicalSortDFS(adj, adj_vertex, visited, st);
            }
        }
        st.push(src);
    }
    static void topologicalSortUsingDFS(ArrayList<ArrayList<Integer>> adj, int n, int e){
        boolean visited[]=new boolean[n];
        Stack<Integer>st=new Stack<>();

        for(int i=0;i<n;i++){
            if(!visited[i]){
                topologicalSortDFS(adj, i, visited, st);
            }
        }

        while(!st.isEmpty()){
            System.out.print(st.pop()+" ");
        }
        System.out.println();
    }
    // ---------------------------------------------------------------------------------------------------------

    // topological sort(DAG) using BFS(Kahn's Algorithm)
    static void topologicalSortUsingBFS(ArrayList<ArrayList<Integer>> adj, int n,int e){
        int indegree[]=new int[n];
        Queue<Integer>q=new LinkedList<>();
        
        for(int i=0;i<n;i++){
            for(int adj_vertex:adj.get(i)){
                indegree[adj_vertex]++;
            }
        }

        for(int i=0;i<n;i++){
            if(indegree[i]==0){
                q.add(i);
            }
        }
        int order[]=new int[n];
        int k=0;
        while(!q.isEmpty()){
            int head=q.poll();
            order[k++]=head;
            for(int adj_vertex:adj.get(head)){
                indegree[adj_vertex]--;
                if(indegree[adj_vertex]==0){
                    q.add(adj_vertex);
                }
            }
        }
        System.out.print("\nTopological Order is : ");
        for(int i:order){
            System.out.print(i+" ");
        }
        System.out.println();
    }
   // ---------------------------------------------------------------------------------------------------------
   
    // shortest path in DAG
    static void topologicalSortUtilForShortestPath(ArrayList<ArrayList<Edge>> adj,int src,boolean visited[],Stack<Integer> st){
        visited[src]=true;

        for(Edge adj_edge:adj.get(src)){
            if(!visited[adj_edge.to_vertex]){
                topologicalSortUtilForShortestPath(adj, adj_edge.to_vertex, visited, st);
            }
        }
        st.push(src);
    }
    static void shortestPath_inDAG(ArrayList<ArrayList<Edge>>adj,int src,int n){
        //step 1 find topological ordering
        Stack<Integer>st=new Stack<>();
        boolean visited[]=new boolean[n];

        for(int i=0;i<n;i++){
            if(!visited[i]){
                topologicalSortUtilForShortestPath(adj, i, visited, st);
            }
        }

        // step 2 find shortest path
        int distance[]=new int[n];
        Arrays.fill(distance,Integer.MAX_VALUE);
        distance[src]=0;

        while(!st.isEmpty()){
            int vertex = st.pop();
            // if vertex is reached previously via some definite path
            if(distance[vertex]!=Integer.MAX_VALUE){

                for(Edge adj_Edge:adj.get(vertex)){
                    if(distance[vertex]+adj_Edge.weight < distance[adj_Edge.to_vertex]){
                        distance[adj_Edge.to_vertex]=distance[vertex]+adj_Edge.weight;
                    }
                }
            }
        }
        
        for(int i=0;i<n;i++){
            System.out.println("Distance from "+src+" to "+i+" = "+distance[i]);
        }
    }
    

    // Kruskal's Algorithm for MST in undirected graph

    // Disjoint Set for cycle detection in undirected graph 
    // returns parent of given node
    // O(logv)
    static int find(int u,int parent[]){
        if(u==parent[u]) return u;

        return parent[u]=find(parent[u],parent);
    }
    
    // joins two separate components into one component
    // O(logv)
    static void union(int u,int v,int parent[],int rank[]){
        u=find(u, parent);
        v=find(v,parent);

        if(rank[u]<rank[v]){
            parent[u]=v;
        }else if(rank[v]<rank[u]){
            parent[v]=u;
        }else{
            parent[v]=u;
            rank[u]++;
        }
    }
    
    // Utility class for sorting edges based on weights
    static class EdgePair implements Comparable<EdgePair>{
        private int u,v,w;
        EdgePair(int u,int v,int w){
            this.u=u;
            this.v=v;
            this.w=w;
        }

        @Override
        public int compareTo(EdgePair e2){
            return this.w-e2.w;
        }
    }

    // Minimum Spanning Tree Using Kruskal's Algorithm (GREEDY) 
    // T.C.: O(eloge + elogv)
    static void kruskalMST(ArrayList<ArrayList<Edge>> adj,int n,int e){
        int parent[]=new int[n];
        int rank[]=new int[n];

        for(int i=0;i<n;i++){
            rank[i]=0;
            parent[i]=i;
        }
        ArrayList<EdgePair> sortedEdges = new ArrayList<>();

        // O(v+e)
        for(int i=0;i<n;i++){
            ArrayList<Edge>adj_edges=adj.get(i);

            for(Edge edge:adj_edges){
                sortedEdges.add(new EdgePair(i,edge.to_vertex,edge.weight));
            }
        }

        int u,v,w,cost=0;
        System.out.println("\tKruskals MST\n  Edge\t Weight");
        for(EdgePair edgePair:sortedEdges){
            u=edgePair.u;
            v=edgePair.v;
            w=edgePair.w;
            if(find(u,parent)!=find(v,parent)){
                System.out.println(u+" -> "+v+" :"+w);
                cost+=w;
                union(u,v,parent,rank);
            }
        }
        System.out.println("Cost of MST : "+cost);

    }
    
    public static void main(String args[]){        

        try{

            BufferedReader br=new BufferedReader(new InputStreamReader(System.in));

            int n=Integer.parseInt(br.readLine().trim());
            int e=Integer.parseInt(br.readLine().trim());

            ArrayList<ArrayList<Edge>>adj=new ArrayList<>();

            boolean isDirectedGraph=false;

            for(int i=0;i<n;i++){
                adj.add(new ArrayList<Edge>());
            }
            
            int from,to,weight=0;
            for(int i=0;i<e;i++){
                String ip[]=br.readLine().trim().split(" ");
                from=Integer.parseInt(ip[0]);
                to=Integer.parseInt(ip[1]);
                weight=ip.length==3?Integer.parseInt(ip[2]):0;
                addEdge(adj,from,to,weight);
                if(!isDirectedGraph){
                    addEdge(adj, to, from, weight);
                }
            }


            for(;;){
                System.out.println("*Graph Implementation in Java*");
                System.out.print("Press:-\n1. for BFS traversal\n2.for DFS traversal\n");
                int choice=Integer.parseInt(br.readLine().trim());
                switch (choice) {
                    case 1:
                        bfs(adj,n,e);
                        break;
                    case 2:
                        dfs(adj,n,e);
                        break;
                    case 3:
                        return;
                    default:
                        break;
                }
            }

        }catch(Exception e){
            System.out.println(e.getMessage());
        }
    }
    
}

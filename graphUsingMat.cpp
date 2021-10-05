#include<bits/stdc++.h>
using namespace std;

class Graph{
    int **mat;
    int V;
public:
    Graph(int);
    void create();
    void BFS();
    void DFS(int);
    void prims_MST();
};

Graph::Graph(int n){
    this->V=n;
}
void Graph::create(){
    int n=this->V;
    mat=new int*[n];
    int mini=INT_MAX;
    int temp[n][n]={
        {0,25,0,0,0,5,0},
        {25,0,12,0,0,0,10},
        {0,12,0,8,0,0,0},
        {0,0,8,0,16,0,14},
        {0,0,0,16,0,20,18},
        {5,0,0,0,20,0,0},
        {0,10,0,14,18,0,0}
    };
    // int m=INT_MAX;
    // int temp[7][7]={
    //     {m,25,m,m,m,5,m},
    //     {25,m,12,m,m,m,10},
    //     {m,12,m,m,m,m,m},
    //     {m,m,m,m,16,m,14},
    //     {m,m,m,16,m,20,18},
    //     {5,m,m,m,20,m,m},
    //     {m,10,m,14,18,m,m}
    // };
    for(int i=0;i<n;i++){
        mat[i]=new int[n];
        for(int j=0;j<n;j++){
            // cout<<" Is there Edge between vertices ("<<i+1<<", "<<j+1<<") : ";
            // cin>>mat[i][j];
            mat[i][j]=temp[i][j];
        }
    }
}
void Graph::BFS(){
    int n=this->V,vertex=-1;
    vector<bool>vis(n,false);
    vector<int>res;
    queue<int>q;
    cout<<"Select a starting vertex ( 1, "<<n<<") to begin BFS : ";
    cin>>vertex;
    vertex--;
    res.push_back(vertex);
    q.push(vertex);
    vis[vertex]=true;
    while(!q.empty()){
        vertex=q.front();
        q.pop();
        for(int i=0;i<n;i++){
            if(mat[vertex][i]!=0 && vis[i]==false){
                res.push_back(i);
                q.push(i);
                vis[i]=true;
            }
        }
    }
    cout<<"\tBFS for the given graph is : ";
    for(int i=0;i<n;i++){
        cout<<res.at(i)+1<<" ";
    }
    cout<<endl;
}
void Graph::DFS(int vertex){
    static int n=this->V;
    static vector<bool>vis(n,false);
    
    if(vis[vertex]==false){
        cout<<vertex+1<<" ";
        vis[vertex]=true;
        for(int i=0;i<n;i++){
            if(mat[vertex][i]!=0 && vis[i]==false){
                DFS(i);
            }
        }
    }
}
int min_weight_adjVertex(int keys[],bool visited[],int n){
    int mini=INT_MAX,min_index=-1;
    for(int i=0;i<n;i++){
        if(visited[i]==false && keys[i]<mini){
            mini=keys[i];
            min_index=i;
        }
    }
    return min_index;
}


void Graph::prims_MST(){
    // int min=INT_MAX;
    // int n=this->V;
    // int v1,v2;
    // //obtaining the minimum edge
    // for(int i=0;i<n;i++){
    //     for(int j=0;j<n;j++){
    //         if(mat[i][j]<min){
    //             min=mat[i][j];
    //             v1=i+1;
    //             v2=j+1;
    //         }
    //     }
    // }
    // cout<<v2<<" , "<<v2<<endl;
    // vector<pair<int,int>>edges;
    // int nearest[n+1];

    // edges.push_back(make_pair(v1,v2));
    // nearest[v1]=nearest[v2]=0;
    // //updating nearest elements
    // for(int i=1;i<=n;i++){
    //     if(nearest[i]!=0){
    //         if(mat[i][v1-1]<mat[i][v2-1]){
    //             nearest[i]=v1;
    //         }else{
    //             nearest[i]=v2;
    //         }
    //     }
    // }
    // for(int i=1;i<n-1;i++){
    //     min=INT_MAX;
    //     for(int j=1;j<=n;j++){
    //         if(nearest[j]!=0 && mat[j-1][nearest[j]-1]<min){
    //             min=mat[j-1][nearest[j]-1];
    //             v1=j;
    //             v2=nearest[j];
    //         }
    //     }
    //     //inserting minimum edge
    //     edges.push_back(make_pair(v1,v2));
    //     //marking as visited
    //     nearest[v1]=0;
    //     //updating nearest
    //     for(int j=1;j<=n;j++){
    //         if(nearest[j]!=0){
    //             if(mat[j-1][v1-1]<mat[j-1][v2-1]){
    //                 nearest[j]=v1;
    //             }else{
    //                 nearest[j]=v2;
    //             }
    //         }
    //     }
    // }
    // cout<<"Prim's MST -> ";
    // for(pair<int,int> P:edges){
    //     cout<<"( "<<P.first<<", "<<P.second<<")"<<" ";
    // }cout<<endl;
    int n=this->V;
    int keys[n];//to store minimum weight edge in a cut of graph
    int parent[n];//store MST
    bool visited[n];//vertex included in MST or not

    for(int i=0;i<n;i++){
        visited[i]=false;
        keys[i]=INT_MAX;
    }
    keys[0]=0;//to select 0 as starting vertex
    parent[0]=-1;//since starting vertex has no parent
    for(int i=0;i<n-1;i++){
        //selecting minimum weighted edge
        int v=min_weight_adjVertex(keys,visited,n);
        //marking vertex as visited
        visited[v]=true;
        // cout<<v<<"*\n";
        //updating keys and parent of adjacent vertices of selected vertex
        for(int j=0;j<n;j++){
            if(mat[v][j] && visited[j]==false && mat[v][j]<keys[j]){
                keys[j]=mat[v][j];
                parent[j]=v;
            }
        }

    }
    cout<<"Edges \t Weight\n";
    int miniWeight=0;
    for(int i=1;i<n;i++){
        miniWeight+=keys[i];
        cout<<"("<<i+1<<", "<<parent[i]+1<<") -> "<<keys[i]<<endl;
    }
    cout<<"Total weight : "<<miniWeight<<endl;
}

int main(){
    int n;
    cout<<"Enter number of vertices : ";
    cin>>n;
    Graph g(n);
    g.create();
    g.BFS();
    cout<<"Enter a vertex to begin DFS : ";
    cin>>n;
    n--;
    cout<<"\tDFS of the given graph : ";
    g.DFS(n);
    cout<<endl;
    g.prims_MST();
    return 0;
}
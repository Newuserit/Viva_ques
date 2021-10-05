#include<iostream>
using namespace std;

class node{
    public:
    int key;
    int height;
    node *right,*left;
};
class AVLtree{
    node *root;
public:
    AVLtree(){
        root=NULL;
    }
    void create();
    node* insertRecursive(node*,int);
    void inorderRecursive(node*);
    int nodeHeight(node*);
    int balanceFactor(node*);
    node* LLrotation(node*);
    node* RRrotation(node*);
    node* LRrotation(node*);
    node* RLrotation(node*);
    node* getRoot(){return root;};
    node* deleteKey(node*,int);
    node* inorderPredecessor(node*);
    node* inorderSuccessor(node*);
    void delKey(int k){root=deleteKey(root,k);}
    void search(int);
};

int AVLtree::nodeHeight(node *r){
    if(r==NULL)return 0;
    int lh=r && r->left?r->left->height:0;
    int rh=r && r->right?r->right->height:0;
    return lh>rh?lh+1:rh+1;
}
int AVLtree::balanceFactor(node *r){
    int lh=r && r->left?r->left->height:0;
    int rh=r && r->right?r->right->height:0;
    return lh-rh;
}
node* AVLtree::LLrotation(node *p){
    node *pl,*plr;
    pl=p->left;//root's left
    plr=pl->right;//root's left's right
    pl->right=p;
    p->left=plr;

    p->height=nodeHeight(p);
    pl->height=nodeHeight(pl);

    //if rotation is performed around the root node
    if(root==p){
        root=pl;
    }
    return pl;
}
node* AVLtree::RRrotation(node *p){
    node *pr,*prl;
    pr=p->right;
    prl=pr->left;
    pr->left=p;
    p->right=prl;

    p->height=nodeHeight(p);
    pr->height=nodeHeight(pr);
    if(root==p){
        root=pr;
    }
    return pr;
}
node* AVLtree::LRrotation(node *p){
    node *pl,*plr;
    pl=p->left;
    plr=pl->right;
    //4 updations in links
    pl->right=plr->left;
    p->left=plr->right;
    plr->left=pl;
    plr->right=p;
    //update heights    
    plr->height=nodeHeight(plr);
    pl->height=nodeHeight(pl);
    p->height=nodeHeight(p);

    if(p==root){
        root=plr;
    }
    return plr;
}
node* AVLtree::RLrotation(node *p){
    node *pr,*prl;
    pr=p->right;
    prl=pr->left;

    p->right=prl->left;
    pr->left=prl->right;
    prl->left=p;
    prl->right=pr;

    if(p==root){
        root=prl;
    }
    return prl;
}
void AVLtree::create(){
    int n,val;
    cout<<"\tEnter no of keys to be inserted in AVL tree : ";
    cin>>n;
    for(int i=0;i<n;i++){
        cout<<"\tEnter key to be inserted : ";
        cin>>val;
        //iterative insertion
        // insert(val);
        //recursive insertion
        //root can change everytime a new element is inserted
        root=insertRecursive(root,val);
        
    }
}
node* AVLtree::insertRecursive(node *r,int k){
    //inserting elements
    if(r==NULL){
        node *temp=new node;
        temp->key=k;
        temp->height=1;
        temp->right=temp->left=NULL;
        if(root==NULL)root=temp;
        return temp;
    }

    if(k>r->key){
        r->right=insertRecursive(r->right,k);
    }else if(k<r->key){
        r->left=insertRecursive(r->left,k);
    }
    //updating height
    r->height=nodeHeight(r);
    //performing rotations
    if(balanceFactor(r)==2 && balanceFactor(r->left)==1){
        return LLrotation(r);
    }else if(balanceFactor(r)==2 && balanceFactor(r->left)==-1){
        return LRrotation(r);
    }else if(balanceFactor(r)==-2 && balanceFactor(r->right)==-1){
        return RRrotation(r);
    }else if(balanceFactor(r)==-2 && balanceFactor(r->right)==1 ){
        return RLrotation(r);
    }
    //returning updated node
    return r;
}
node* AVLtree::inorderPredecessor(node *p){
    while(p && p->right!=NULL){
        p=p->right;
    }
    return p;
}
node* AVLtree::inorderSuccessor(node *p){
    while(p && p->left!=NULL){
        p=p->left;
    }
    return p;
}
node* AVLtree::deleteKey(node *r,int k){

    if(r==NULL){
        cout<<"\tNot found\n";
        return NULL;
    }

    if(r->left==NULL && r->right==NULL && r->key==k){
        if(r==root)root=NULL;
        delete r;
        cout<<"\tDeleted key "<<k<<" from AVL tree.\n";    
        return NULL;
    }
    if(k<r->key){
        r->left=deleteKey(r->left,k);
    }else if(k>r->key){
        r->right=deleteKey(r->right,k);
    }else{
        
        node *q;
        if(nodeHeight(r->left)>nodeHeight(r->right)){
            q=inorderPredecessor(r->left);
            r->key=q->key;
            r->left=deleteKey(r->left,q->key);
        }else{
            q=inorderSuccessor(r->right);
            r->key=q->key;
            r->right=deleteKey(r->right,q->key);
        }
    }
    //updating height
    r->height=nodeHeight(r);
    //performing rotations
    if(balanceFactor(r)==2 && balanceFactor(r->left)==1){//L 1
        return LLrotation(r);
    }else if(balanceFactor(r)==2 && balanceFactor(r->left)==-1){//L -1
        return LRrotation(r);
    }else if(balanceFactor(r)==2 && balanceFactor(r->left)==0){//L 0
        return LLrotation(r);
    }else if(balanceFactor(r)==-2 && balanceFactor(r->right)==-1){//R -1
        return RRrotation(r);
    }else if(balanceFactor(r)==-2 && balanceFactor(r->right)==1 ){//R 1
        return RLrotation(r);
    }else if(balanceFactor(r)==-2 && balanceFactor(r->right)==0){//R 0
        return RRrotation(r);
    }
    //updating node
    return r;
}
void AVLtree::inorderRecursive(node *r){
    if(r==NULL)return;
    inorderRecursive(r->left);
    cout<<r->key<<" ";
    inorderRecursive(r->right);
}
void AVLtree::search(int k){
    if(root==NULL){
        cout<<"\tEmpty"<<endl;
        return;
    }
    node *t=root;
    while(t){
        if(t->key<k){
            t=t->right;
        }else if(t->key>k){
            t=t->left;
        }else{
            cout<<"\tFound key "<<k<<" in AVL tree."<<endl;
            return;
        }
    }
    cout<<"\tKey not found in AVL tree.\n";
}
int main(){
    AVLtree avl;
    node *t;
    int val,ch;
    for(;;){
        cout<<"Press\n1.to create\n2.to display inorder\n3.to insert\n4.to delete\n5.to search\n6.to exit\n";
        cin>>ch;
        switch (ch)
        {
        case 1:
            avl.create();
            break;
        case 2:
            cout<<"\tInorder traversal of AVL tree : ";
            avl.inorderRecursive(avl.getRoot());
            cout<<endl;
            break;
        case 3:
            cout<<"\tEnter a value to insert : ";
            cin>>val;
            
            avl.insertRecursive(avl.getRoot(),val);
            break;
        case 4:
            cout<<"\tEnter a value to delete : ";
            cin>>val;
            avl.delKey(val);
            break;
        case 5:
            cout<<"\tEnter a value to search : ";
            cin>>val;
            avl.search(val);
            cout<<endl;
            break;
        case 6:
            return 0;
        default:
            cout<<"\tInvalid choice\n";
            break;
        }
    }
    return 0;
}
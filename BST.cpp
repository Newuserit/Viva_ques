#include<iostream>
using namespace std;

class node{
    public:
    int key;
    node *left,*right;
};
class BST{
private:
    node *root;
public:
    BST(){root=NULL;}
    void create();
    void inorderRecusive(node*);
    void search(int);
    void insert(int);
    void searchRecursive(node*,int);
    node* insertRecursive(node*,int);
    node* deleteKey(node*,int);
    void delKey(int k){root=deleteKey(root,k);}
    node* inorderPredecessor(node*);
    node* inorderSuccessor(node*);
    int height(node*);
    node* getRoot(){return root;}
};
void BST::create(){
    int n,val;
    cout<<"\tEnter no of keys to be inserted in BST : ";
    cin>>n;
    for(int i=0;i<n;i++){
        cout<<"\tEnter key to be inserted : ";
        cin>>val;
        //iterative insertion
        insert(val);
        //recursive insertion
        // if(root==NULL){
        //     root=insertRecursive(root,val);
        // }else{
        //     insertRecursive(root,val);
        // }
    }
}
void BST::inorderRecusive(node *r){
    if(r==NULL)return;
    inorderRecusive(r->left);
    cout<<r->key<<" ";
    inorderRecusive(r->right);
}
void BST::insert(int k){
    node *r,*t,*p;
    if(root==NULL){
        root=new node;
        root->key=k;
        root->left=root->right=NULL;
        return;
    }
    t=root;

    while(t){
        r=t;
        if(t->key<k){
            t=t->right;
        }else if(t->key>k){
            t=t->left;
        }else{
            cout<<"\tKey already present\n";
            return;
        }
    }
    p=new node;
    p->key=k;
    p->left=p->right=NULL;
    if(r->key>k)r->left=p;
    else r->right=p;
}
node* BST::insertRecursive(node*r,int k){
    if(r==NULL){
        node *t=new node;
        t->key=k;
        t->left=t->right=NULL;
        return t;
    }
    if(k<r->key){
        r->left=insertRecursive(r->left,k);
    }else if(k>r->key){
        r->right=insertRecursive(r->right,k);
    }else{
        cout<<"\tKey already present\n";
    }
    return r;
}
void BST::search(int k){
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
            cout<<"\tFound key "<<k<<" in BST."<<endl;
            return;
        }
    }
    cout<<"\tKey not found in BST.\n";
}
void BST::searchRecursive(node *r,int k){
    if(r==NULL){
        cout<<"\tNot found\n";
        return;
    }
    if(k<r->key){
        searchRecursive(r->left,k);
    }else if(k>r->key){
        searchRecursive(r->right,k);
    }else{
        cout<<"\tFound key "<<k<<" in BST.\n";
    }
}
int BST::height(node *r){
    if(r==NULL)return 0;
    int left,right;
    left=height(r->left);
    right=height(r->right);
    return left>right?left+1:right+1;
}
node* BST::inorderPredecessor(node *p){
    while(p && p->right!=NULL){
        p=p->right;
    }
    return p;
}
node* BST::inorderSuccessor(node *p){
    while(p && p->left!=NULL){
        p=p->left;
    }
    return p;
}
node* BST::deleteKey(node *r,int k){
    if(r==NULL){
        cout<<"\tNot found\n";
        return r;
    }
    if(r->left==NULL && r->right==NULL && r->key==k){
        if(r==root)root=NULL;
        delete r;
        cout<<"\tDeleted key "<<k<<" from BST.\n";    
        return NULL;
    }

    if(k<r->key){
        r->left=deleteKey(r->left,k);
    }else if(k>r->key){
        r->right=deleteKey(r->right,k);
    }else{
        if(r->left==NULL){
            return r->right;
        }else if(r->right==NULL){
            return r->left;
        }
        node *q;    
        if(height(r->left)>height(r->right)){
            q=inorderPredecessor(r->left);
            r->key=q->key;
            r->left=deleteKey(r->left,q->key);
        }else{
            q=inorderSuccessor(r->right);
            r->key=q->key;
            r->right=deleteKey(r->right,q->key);
        }
    }
    return r;
}
int main(){
    BST bst;
    node *t;
    int val,ch;
    for(;;){
        cout<<"Press\n1.to create\n2.to display inorder\n3.to insert\n4.to insert recursively\n5.to search\n6.to search recursively\n7.to delete\n8.to exit\n";
        cin>>ch;
        switch (ch)
        {
        case 1:
            bst.create();
            break;
        case 2:
            cout<<"\tInorder traversal : ";
            bst.inorderRecusive(bst.getRoot());
            cout<<endl;
            break;
        case 3:
            cout<<"\tEnter a value to insert : ";
            cin>>val;
            bst.insert(val);
            break;
        case 4:
            cout<<"\tEnter a value to insert : ";
            cin>>val;
            bst.insertRecursive(bst.getRoot(),val);
            break;
        case 5:
            cout<<"\tEnter a value to search : ";
            cin>>val;
            bst.search(val);
            cout<<endl;
            break;
        case 6:
            cout<<"\tEnter a value to search : ";
            cin>>val;
            bst.searchRecursive(bst.getRoot(),val);
            cout<<endl;
            break;
        case 7:
            cout<<"\tEnter a value to delete : ";
            cin>>val;
            bst.delKey(val);            
            break;
        case 8:
            return 0;
        default:
            cout<<"\tInvalid choice\n";
            break;
        }
    }
    return 0;
}

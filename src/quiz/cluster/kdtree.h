/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	~KdTree()
	{
		delete root;
	}

	void insert_recursive(Node**node, std::vector<float> point, int id, uint32_t depth)
	{
		if(*node != NULL)
		{
			uint32_t dim = depth%point.size();
			if(point[dim] < (*node)->point[dim])
			{
				insert_recursive(&(*node)->left, point,id,depth+1);
			}
			else
			{
				insert_recursive(&(*node)->right, point,id,depth+1);
			}
			
		}
		else
		{
			*node = new Node(point,id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insert_recursive(&root, point, id, 0);
	}

	void search_recursive(Node**node, std::vector<float> target, float distanceTol, uint32_t depth,std::vector<int>& ids)
	{
		if(*node != NULL)
		{
			/*
			Conditions:
				if(target[0|1] <=>	node[0|1] +/- tolerance)
					if distance within tolerance
						add id to return list
				recursively search based on dimension
			*/
			if(
				((*node)->point[0] <= target[0] + distanceTol)
			&&	((*node)->point[0] >= target[0] - distanceTol)
			&&	((*node)->point[1] <= target[1] + distanceTol)
			&&	((*node)->point[1] >= target[1] - distanceTol)
			&&	((*node)->point[2] <= target[2] + distanceTol)
			&&	((*node)->point[2] >= target[2] - distanceTol)
			)
			{
				//d = sqrt(x^2 +y^2)
				//calculate distance to point
				float  d = std::sqrt(
						std::pow(((*node)->point[0] - target[0]),2) 
					+	std::pow(((*node)->point[1] - target[1]),2)
					+	std::pow(((*node)->point[2] - target[2]),2)
				);
				if(d <= distanceTol)
					ids.push_back((*node)->id);
			}

			uint32_t dim = depth%target.size();
			if((target[dim] -distanceTol) < (*node)->point[dim])
			{
				search_recursive(&((*node)->left), target, distanceTol, depth+1, ids);
			}
			if((target[dim] +distanceTol) > (*node)->point[dim])
			{
				search_recursive(&((*node)->right), target, distanceTol, depth+1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_recursive(&root, target, distanceTol, 0, ids);
		return ids;
	}
	

};





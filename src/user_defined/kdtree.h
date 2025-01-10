#ifndef __KDTREE__
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
  	// Recursive method to insert the point
	void insert_recursive(Node**node, std::vector<float> point, int id, uint32_t depth)
	{
      	// Check if Node is already has some Value
		if(*node != NULL)
		{
          	// Compare point value based on dimension
			uint32_t dim = depth%3;
			if(point[dim] < (*node)->point[dim])
			{
              	// Traverse left to the tree for point insertion
				insert_recursive(&(*node)->left, point,id,depth+1);
			}
			else
			{
              	// Traverse right to the tree for point insertion
				insert_recursive(&(*node)->right, point,id,depth+1);
			}
			
		}
		else
		{
            // Define new node and assign
			*node = new Node(point,id);
		}
	}
    // Insert point to the Kd tree 
	void insert(std::vector<float> point, int id)
	{
		insert_recursive(&root,point,id,0);
	}

	void search_recursive(Node**node, std::vector<float> target, float distanceTol, uint32_t depth,std::vector<int>& ids)
	{
		if(*node != NULL)
		{
			/*
			Conditions:
				if(target[0|1] <>	node[0|1] +/- tolerance)
					compute distance
					if distance within tolerance
						add id to return list
				recursively proceed based on dimension
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
				// Calculate distance to point
				float  d = std::sqrt(
						std::pow(((*node)->point[0] - target[0]),2) 
					+	std::pow(((*node)->point[1] - target[1]),2)
					+	std::pow(((*node)->point[2] - target[2]),2)
				);
                
                // If distance is within tolerance add id to vector holding clustering info 
				if(d <= distanceTol)
					ids.push_back((*node)->id);
			}

			uint32_t dim = depth%3;

            // Compare point value in node is beyond distance tolerance of target point ( less than check)
          	if((target[dim] -distanceTol) < (*node)->point[dim])
			{
                // Traverse left to the tree for point search
				search_recursive(&((*node)->left), target, distanceTol, depth+1, ids);
			}
            // Compare point value in node is beyond distance tolerance of target point( greater than check)
			if((target[dim] +distanceTol) > (*node)->point[dim])
			{
                // Traverse right to the tree for point search
				search_recursive(&((*node)->right), target, distanceTol, depth+1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_recursive(&root,target,distanceTol,0, ids);
		return ids;
	}
};
#endif

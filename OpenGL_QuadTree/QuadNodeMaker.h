#ifndef _QUADNODEMAKER_H
#define _QUADNODEMAKER_H

#include "GLIncludes.h"

#pragma region Quad_Tree and Node maker
/*

testing
struct QTNode
{
_Point_xy pos;
int data;

QTNode(_Point_xy _pos, int _data)
{
pos = _pos;
data = _data;
}
QTNode()
{
data = 0;
}
};
*/


//point getter from the vector data
class QT_PointGetter
{
public:
	virtual float get_X() const = 0;
	virtual float get_Y() const = 0;
	virtual ~QT_PointGetter() = default;

	virtual bool equals(const QT_PointGetter & point) const;

};

//setter for data point of the quadtree
class QT_data_point : public QT_PointGetter
{
private:
	float X;
	float Y;

public:
	//void set_X(const float& x) { X = x; }
	//void set_Y(const float& y) { Y = y; }

	QT_data_point::QT_data_point() : X(0.0f), Y(0.0f) {}
	QT_data_point::QT_data_point(const float& x, const float& y) : X(x), Y(y) {}

	float QT_data_point::get_X() const { return X; }
	float QT_data_point::get_Y() const { return Y; }
};


//box around the data points
struct QT_Data_Box
{
private:
	float mX1; //top section of box
	float mY1; //top section of box
	float mX2; //bottom section of box
	float mY2; //bottom section of box

public:
	void set_X1(const float &x1) { mX1 = x1; }
	void set_Y1(const float &y1) { mY1 = y1; }
	void set_X2(const float &x2) { mX2 = x2; }
	void set_Y2(const float &y2) { mY2 = y2; }
	virtual float get_X1()const;
	virtual float get_Y1()const;
	virtual float get_X2()const;
	virtual float get_Y2()const;

	bool intersects_p(const QT_PointGetter& point) const; // check if the points intersects

	bool intersects_b(const QT_Data_Box &box) const; // check if the box intersects with another box
	
													 
	//inital bounding box without a data points
	QT_Data_Box::QT_Data_Box() : mX1(0.0f), mY1(0.0f), mX2(0.0f), mY2(0.0f) {} 
	QT_Data_Box::QT_Data_Box(const float& X1, const float& Y1, const float& X2, const float& Y2) :
		mX1(X1), mY1(Y1), mX2(X2), mY2(Y2) {}

};

typedef std::vector<QT_PointGetter*> qtpoint_vec; //storage for the data point

//makes the node and the quadtree
class quadtree_maker
{
private:
	quadtree_maker * node_NW; //NorthWest Node
	quadtree_maker* node_NE; //NorthEast Node
	quadtree_maker* node_SW; //SouthWest Node
	quadtree_maker* node_SE; //SouthEast Node

	QT_Data_Box t_bounds; //bounds of the data_box

	qtpoint_vec pointData; //object of the storage of the data point

	void split_tree(); //splits tree

	void join(); //

public:
	//Constructor and destructor for the of a tree with no data
	quadtree_maker(const QT_Data_Box& bounds);
	~quadtree_maker();

	//Constructor and destructor of the quadtree
	quadtree_maker(const quadtree_maker& qtpoint);
	quadtree_maker& operator=(const quadtree_maker& qtpoint);

	//These two functions place or remove a data point node from the tree
	bool place(QT_PointGetter* point);
	bool remove(QT_PointGetter* point);

	//Query_b searchs the quad tree based on bounds and points being requested.
	//Query_p searchs the quad tree based on data point and points being requested.
	void query_b(const QT_Data_Box& bounds, qtpoint_vec& points) const;
	void query_p(const QT_data_point& point, qtpoint_vec& points) const;

	//Links the nodes and makes sub quad trees.
	void pull_b(const QT_Data_Box& bounds, qtpoint_vec& points);
	void pull_p(const QT_data_point& point, qtpoint_vec& points);

	//Clears the whole quad tree.
	void clear();

};


#pragma endregion

#pragma region QuadTree

//checks if two data points are equal. 
//returns true if they are.
//returns false if they are not.
bool operator==(const QT_PointGetter & p1, const QT_PointGetter & p2)
{
	return (p1.get_X() == p2.get_X() && p1.get_Y() == p2.get_Y());
}
//checks if two data points are not equal. 
//returns true if they are.
//returns false if they are not.
bool operator!=(const QT_PointGetter & p1, const QT_PointGetter & p2)
{
	return !(p1 == p2);
}
bool QT_PointGetter::equals(const QT_PointGetter & point) const
{
	return !(get_X() != point.get_X() || get_Y() != point.get_Y());
}

float QT_Data_Box::get_X1() const { return mX1; }
float QT_Data_Box::get_Y1() const { return mY1; }
float QT_Data_Box::get_X2() const { return mX2; }
float QT_Data_Box::get_Y2() const { return mY2; }

bool QT_Data_Box::intersects_p(const QT_PointGetter &point) const
{
	return !((point.get_X() < get_X1()) || (point.get_Y() < get_Y1()) ||
		(point.get_X() > get_X2()) || (point.get_Y() > get_Y2()));
}

bool QT_Data_Box::intersects_b(const QT_Data_Box &box) const
{
	return !((get_X1() > box.get_X2()) || (get_Y1() > box.get_Y2()) ||
		(get_X2() < box.get_X1()) || (get_Y2() < box.get_Y1()));
}

quadtree_maker::quadtree_maker(const QT_Data_Box& bounds)
	: node_NW{ nullptr }, node_NE{ nullptr },
	node_SW{ nullptr }, node_SE{ nullptr }, t_bounds{ bounds }
{}

quadtree_maker::~quadtree_maker()
{
	if (node_NW != nullptr)
	{
		delete node_NW;
		delete node_NE;
		delete node_SW;
		delete node_SE;

	}
}

quadtree_maker::quadtree_maker(const quadtree_maker& qtpoint) : quadtree_maker(qtpoint.t_bounds)
{
	this->operator=(qtpoint);
}

quadtree_maker & quadtree_maker::operator=(const quadtree_maker& qtpoint)
{
	t_bounds = qtpoint.t_bounds;
	if (node_NW != nullptr)
	{
		delete node_NW;
		delete node_NE;
		delete node_SW;
		delete node_SE;

		node_NW = nullptr;
		node_NE = nullptr;
		node_SW = nullptr;
		node_SE = nullptr;
	}
	if (qtpoint.node_NW != nullptr)
	{
		split_tree();

		*node_NW = *qtpoint.node_NW;
		*node_NE = *qtpoint.node_NE;
		*node_SW = *qtpoint.node_SW;
		*node_SE = *qtpoint.node_SE;
	}
	if (!qtpoint.pointData.empty())
	{
		pointData = qtpoint.pointData;
	}
	return *this;
}

void quadtree_maker::split_tree()
{
	if (node_NW == nullptr)
	{
		//upper bounds
		float X0 = t_bounds.get_X1();
		float Y0 = t_bounds.get_Y1();
		//lower bounds
		float X1 = t_bounds.get_X2();
		float Y1 = t_bounds.get_Y2();
		//the midpoints
		float x_mid = (X0 + X1) / 2.0f;
		float y_mid = (Y0 + Y1) / 2.0f;

		node_NW = new quadtree_maker(QT_Data_Box(X0, Y0, x_mid, y_mid));
		node_NE = new quadtree_maker(QT_Data_Box(x_mid, Y0, X1, y_mid));
		node_SW = new quadtree_maker(QT_Data_Box(X0, y_mid, x_mid, Y1));
		node_SE = new quadtree_maker(QT_Data_Box(x_mid, y_mid, X1, Y1));
		while (!pointData.empty())
		{
			if (node_NW->place(pointData.back()));
			else if (node_NE->place(pointData.back()));
			else if (node_SW->place(pointData.back()));
			else if (node_SE->place(pointData.back()));

			pointData.pop_back();

		}

	}

}

void quadtree_maker::join()
{
	if (node_NW != nullptr)
	{
		if ((node_NW->pointData.empty()) &&
			(node_NE->pointData.empty()) &&
			(node_SW->pointData.empty()) &&
			(node_SE->pointData.empty()) &&

			(node_NW->node_NW == nullptr) &&
			(node_NE->node_NW == nullptr) &&
			(node_SW->node_NW == nullptr) &&
			(node_SE->node_NW == nullptr))
		{

			delete node_NW;
			delete node_NE;
			delete node_SW;
			delete node_SE;

			node_NW = nullptr;
			node_NE = nullptr;
			node_SW = nullptr;
			node_SE = nullptr;
		}
	}
}

bool quadtree_maker::place(QT_PointGetter* point)
{
	if (t_bounds.intersects_p(*point))
	{
		if (node_NW != nullptr)
		{

			if (node_NW->place(point)) {}
			else if (node_NE->place(point)) {}
			else if (node_SW->place(point)) {}
			else if (node_SE->place(point)) {}
		}
		else
		{
			for (auto& e : pointData)
			{
				if (e == point) return false;
			}
			pointData.push_back(point);

			if (*pointData.front() != *point)
			{
				split_tree();
			}


		}
		return true;
	}
	return false;
}

bool quadtree_maker::remove(QT_PointGetter* point)
{
	if (t_bounds.intersects_p(*point))
	{
		if (node_NW != nullptr)
		{
			if (node_NW->remove(point) ||
				node_NE->remove(point) ||
				node_SW->remove(point) ||
				node_SE->remove(point))
			{
				join();
				return true;
			}

		}

		for (auto i = pointData.begin(); i != pointData.end(); i++)
		{
			if (*i == point)
			{
				pointData.erase(i);
				return true;
			}

		}

	}
	return false;
}

void quadtree_maker::query_p(const QT_data_point& point, qtpoint_vec& points) const
{
	if (t_bounds.intersects_p(*(QT_PointGetter*)&point))
	{
		if (node_NW != nullptr)
		{
			qtpoint_vec NW; node_NW->query_p(point, NW);
			qtpoint_vec NE; node_NE->query_p(point, NE);
			qtpoint_vec SW; node_SW->query_p(point, SW);
			qtpoint_vec SE; node_NW->query_p(point, SE);

			points.reserve(NW.size() + NE.size() + SW.size() + SE.size());
			points.insert(points.end(), NW.begin(), NW.end());
			points.insert(points.end(), NE.begin(), NE.end());
			points.insert(points.end(), SW.begin(), SW.end());
			points.insert(points.end(), SE.begin(), SE.end());
		}
		if (!pointData.empty())
		{
			if (*pointData.back() == *(QT_PointGetter*)&point)
			{
				points = pointData;
			}

		}

	}
}

void quadtree_maker::query_b(const QT_Data_Box& bounds, qtpoint_vec& points) const
{
	if (t_bounds.intersects_b(bounds))
	{
		if (node_NW != nullptr)
		{
			qtpoint_vec NW; node_NW->query_b(bounds, NW);
			qtpoint_vec NE; node_NE->query_b(bounds, NE);
			qtpoint_vec SW; node_SW->query_b(bounds, SW);
			qtpoint_vec SE; node_NW->query_b(bounds, SE);

			points.reserve(NW.size() + NE.size() + SW.size() + SE.size());
			points.insert(points.end(), NW.begin(), NW.end());
			points.insert(points.end(), NE.begin(), NE.end());
			points.insert(points.end(), SW.begin(), SW.end());
			points.insert(points.end(), SE.begin(), SE.end());
		}
		if (!pointData.empty())
		{
			if (bounds.intersects_p(*pointData.back()))
			{
				points = pointData;
			}

		}

	}

}

void quadtree_maker::pull_p(const QT_data_point& point, qtpoint_vec& points)
{
	if (t_bounds.intersects_p(*(QT_PointGetter*)&point))
	{
		if (node_NW != nullptr)
		{
			qtpoint_vec NW; node_NW->pull_p(point, NW);
			qtpoint_vec NE; node_NE->pull_p(point, NE);
			qtpoint_vec SW; node_SW->pull_p(point, SW);
			qtpoint_vec SE; node_NW->pull_p(point, SE);

			points.reserve(NW.size() + NE.size() + SW.size() + SE.size());
			points.insert(points.end(), NW.begin(), NW.end());
			points.insert(points.end(), NE.begin(), NE.end());
			points.insert(points.end(), SW.begin(), SW.end());
			points.insert(points.end(), SE.begin(), SE.end());

			join();
		}
		if (!pointData.empty())
		{
			if (*pointData.back() == *(QT_PointGetter*)&point)
			{
				points = pointData;
				pointData.clear();
			}

		}

	}
}

void quadtree_maker::pull_b(const QT_Data_Box& bounds, qtpoint_vec& points)
{
	if (t_bounds.intersects_b(bounds))
	{
		if (node_NW != nullptr)
		{
			qtpoint_vec NW; node_NW->pull_b(bounds, NW);
			qtpoint_vec NE; node_NE->pull_b(bounds, NE);
			qtpoint_vec SW; node_SW->pull_b(bounds, SW);
			qtpoint_vec SE; node_NW->pull_b(bounds, SE);

			points.reserve(NW.size() + NE.size() + SW.size() + SE.size());
			points.insert(points.end(), NW.begin(), NW.end());
			points.insert(points.end(), NE.begin(), NE.end());
			points.insert(points.end(), SW.begin(), SW.end());
			points.insert(points.end(), SE.begin(), SE.end());

			join();
		}
		if (!pointData.empty())
		{
			if (bounds.intersects_p(*pointData.back()))
			{
				points = pointData;
				pointData.clear();
			}

		}

	}
}

void quadtree_maker::clear()
{
	if (node_NW != nullptr)
	{
		delete node_NW;
		delete node_NE;
		delete node_SW;
		delete node_SE;

		node_NW = nullptr;
		node_NE = nullptr;
		node_SW = nullptr;
		node_SE = nullptr;
	}


}


#pragma endregion


#endif _QUADNODEMAKER_H
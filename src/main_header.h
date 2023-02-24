#include<math.h>
#include<cmath>
#include<iostream>
#include<fstream>
#include<typeinfo>
#include<vector>
#include<string>
#include "Eigen/Dense"

using std::cout;
using std::endl;
using std::string;
using std::system;
using std::ctype;
using std::vector;
using std::pair;

using namespace Eigen;

class point
{
	public:
		double x, y;
		
		point(double x_in, double y_in)
		{
			x = x_in;
			y = y_in;
		}

		point() { x = 0; y = 0; }

		~point() {};

		void print()
		{
			cout << "point: " << x << " " << y <<endl;
		}
};

class line
{
	public:
		point p1, p2;

		line(point p1_in, point p2_in)
		{
			p1 = p1_in;
			p2 = p2_in;
		}

		~line() {};

		void print()
		{
			cout << "Printing line:\n";
			cout << "p1: ";
			p1.print();
			cout << "p2: ";
			p2.print();
		}
			
};

class ped 
{
	public: 
		int id;
		// position and velocity 4 vec
		Vector4d y;
		Vector2d desired_loc;
		double desired_speed;
		double max_speed;
		double field_of_view;

		double current_speed;

		ped (int id_in,
			 Vector4d y_in,
			 Vector2d desired_loc_in,
			 double desired_speed_in,
			 double max_speed_in,
			 double field_of_view_in)
		{
			id = id_in;
			y = y_in;
			desired_loc = desired_loc_in;
			desired_speed = desired_speed_in;
			max_speed = max_speed_in;
			field_of_view = field_of_view_in;
			get_current_speed();
		}

		Vector2d get_pos()
		{
			Vector2d x(y[0],y[1]);
			return x;
		}

		point get_point()
		{
			return point(y[0],y[1]);
		}

		Vector2d get_vel()
		{
			Vector2d v(y[2],y[3]);
			return v;
		}

		void get_current_speed()
		{
			Vector2d v(y[2],y[3]);
			current_speed = v.norm();
		}

		void print()
		{
			cout << "Printing for pedestrian: " << id << endl;
			cout << "Position:\n" << y[0] << endl << y[1] << endl;
			cout << "Desired Location:\n" << desired_loc[0]
					<< endl << desired_loc[1] << endl;
			cout << "Velocity:\n" << y[2] << endl << y[3] << endl;
			get_current_speed();
			cout << "Current Speed: " << current_speed << endl;
			cout << "Desired Speed: " << desired_speed << endl;
			cout << "Max Speed: " << max_speed << endl;
			cout << "Field of View: " << field_of_view  << endl;
			cout << endl;
		}
};

Vector2d ped_vec_A2B(ped pedA, ped pedB)
{
	// returns the vector from A to B
	return pedB.get_pos() - pedA.get_pos();

}

double nearest_point(ped p, line l)
{
	// returns the distance to the nearest point of
	// line l to pedestrian p.
	//
	// algo from the following post on getting distance from
	// point to a finite line segment:
	// https://stackoverflow.com/questions/849211/shortest-distance-
	// between-a-point-and-a-line-segment
	
	point ped = p.get_point();
	point l1 = l.p1;
	point l2 = l.p2;

	double delta_x = l2.x - l1.x;
	double delta_y = l2.y - l1.y;

	double norm = delta_x * delta_x + delta_y * delta_y;
	double u = ((ped.x - l1.x) * delta_x + (ped.y - l1.y) * delta_y) / norm;

	if (u > 1)
	{
		u = 1;
	} else if (u < 0) {
		u = 0;
	}

	double x = l1.x + u * delta_x;
	double y = l1.y + u * delta_y;
    
	double dx = x - ped.x;
	double dy = y - ped.y;
		
	return sqrt(dx * dx + dy * dy);
}

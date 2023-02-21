#include<math.h>
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

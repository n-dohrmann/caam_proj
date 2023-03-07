#include<math.h>
#include<cmath>
#include<iostream>
#include<fstream>
#include<typeinfo>
#include<vector>
#include<string>
#include"Eigen/Dense"
#include<chrono>
#include<random>

using std::cout;
using std::endl;
using std::string;
using std::system;
using std::ctype;
using std::vector;
using std::reference_wrapper;
using std::pair;

using namespace Eigen;

// CONSTANTS
const double RELAX_TIME = 0.5E0;
const double dt = 2.5E-1;
const double stop_threshold = 0.5E0;
const double d_max = 5.0E0;
const double dist_threshold = 0.1E0;
const double num_angles = 100;
/* #define M_PI           3.14159265358979323846 */
//

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
	// in the simulation, each border will be a line (no non-linear boundaries)
	// so it might not be necessary to even have a separate class for them?
	// The objects themselves can be called borders, though
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

class trajectory
{
	public:
		vector<Vector2d> positions;
		vector<Vector2d> velocities;

		trajectory() {}

		void add_position(Vector2d p)
		{
			positions.push_back(p);
		}

		void add_velocity(Vector2d v)
		{
			velocities.push_back(v);
		}

		void to_json(int id, Vector2d desired_loc, string sim_name)
		{
			// writes each of the trajectory points to a
			// JSON FILE
			std::system(("mkdir -p " + sim_name).c_str());
			string file_name = "p" + std::to_string(id) + "_traj.json";
			file_name = "./" + sim_name + "/" + file_name;
			std::ofstream out {file_name};

			int n = (int)positions.size();
			int index = 0;

			// ID and step counts
			out << "{" << endl;
			out << "\"id\" : " << id << "," << endl;
			out << "\"steps\" : " << n << "," << endl;

			// desired location of the pedestrian.
			// to be used in de-bugging 
			// NEED TO GET THIS VECTOR FROM THE PEDESTRIAN!
			out << "\"desired_loc\" : \"" <<
				desired_loc[0] << " " << desired_loc[1]
				<< "\"," << endl;

			out << "\"positions\" : [" << endl;
			for (auto& p : positions)
			{
				if (index != n - 1) {
					out << "\"" <<  p[0] << " " << p[1] << "\"," << endl;
				} else {
					out << "\"" << p[0] << " " << p[1] << "\"" << endl;
				}
				++index;
			}
			out << "]," << endl;

			index = 0;

			out << "\"velocities\" : [" << endl;
			for (auto& v : velocities)
			{
				if (index != n - 1) {
					out << "\"" <<  v[0] << " " << v[1] << "\"," << endl;
				} else {
					out << "\"" << v[0] << " " << v[1] << "\"" << endl;
				}
				++index;
			}
			out << "]" << endl;

			out << "}" << endl;

			out.close();

		}

		~trajectory() {};
};

class ped
{
	public:
		int id;
		// position and velocity 4 vec
		Vector4d y;
		Vector2d desired_loc;
		double desired_speed;
		Vector2d desired_dir;
		Vector2d preferred_velocity;
		double max_speed;
		double field_of_view;

		double current_speed;
		// whether or not the pedestrian will follow
		// the BH model for updating their desired direction
		bool behavioral;
		trajectory traj;

		// whether or not a pedestrian will be stopped in place
		// (updated once they should be stopped)
		bool has_stopped = false;

		// attributes for the behavioral model
		Vector2d SF_desired_dir;

		ped (int id_in,
			 Vector4d y_in,
			 Vector2d desired_loc_in,
			 double desired_speed_in,
			 double max_speed_in,
			 double field_of_view_in,
			 bool behavioral_in)
		{
			id = id_in;
			y = y_in;
			desired_loc = desired_loc_in;
			desired_speed = desired_speed_in;
			max_speed = max_speed_in;
			field_of_view = field_of_view_in;
			behavioral = behavioral_in;
			update_current_speed();
			update_desired_dir();
			preferred_velocity = Vector2d(y[2], y[3]);
			traj.add_position(get_pos());
			traj.add_velocity(get_vel());
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

		Vector2d get_vel() const
		{
			Vector2d v(y[2],y[3]);
			return v;
		}

		void update_current_speed()
		{
			Vector2d v(y[2],y[3]);
			current_speed = v.norm();
		}

		void update_desired_dir()
		{
			Vector2d v = desired_loc - get_pos();
			desired_dir = v / v.norm();
		}

		void update_has_stopped()
		{
			if ( abs(y[0] - desired_loc[0]) <= stop_threshold )
			{
				has_stopped = true;
			}
		}

		void update()
		{
			update_current_speed();
			update_desired_dir();
			/* update_has_stopped(); */
		}

		void update(Vector2d dir)
		{
			update_current_speed();
			desired_dir = dir;
			Vector2d v = desired_loc - get_pos();
			SF_desired_dir = v / v.norm();
		}

		void print()
		{
			cout << "Printing for pedestrian: " << id << endl;
			cout << "Position:\n" << y[0] << endl << y[1] << endl;
			cout << "Desired Location:\n" << desired_loc[0]
					<< endl << desired_loc[1] << endl;
			cout << "Velocity:\n" << y[2] << endl << y[3] << endl;
			update_current_speed();
			cout << "Current Speed: " << current_speed << endl;
			cout << "Desired Speed: " << desired_speed << endl;
			cout << "Max Speed: " << max_speed << endl;
			cout << "Field of View: " << field_of_view  << endl;
			cout << endl;
		}

		// add to destructor if needed
		~ped() {};
};

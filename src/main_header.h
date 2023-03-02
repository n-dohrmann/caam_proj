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
using std::reference_wrapper;
using std::pair;

using namespace Eigen;

// CONSTANTS
const double RELAX_TIME = 0.5E0;
const double dt = 2.0E0;
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

class ped
{
	public:
		int id;
		// position and velocity 4 vec
		Vector4d y;
		Vector2d desired_loc;
		double desired_speed;
		Vector2d desired_dir;
		double max_speed;
		double field_of_view;

		double current_speed;
		// whether or not the pedestrian will follow
		// the BH model for updating their desired direction
		bool behavioral;

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
			update_desired_dir(behavioral);
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

		void update_desired_dir(bool bh)
		{
			if ( not bh ) {
				Vector2d v = desired_loc - get_pos();
				desired_dir = v / v.norm();
			} else {
				cout << "ERR: implement behavioral functionality!" << endl;
			}
		}

		void update()
		{
			update_current_speed();
			update_desired_dir(behavioral);
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
};

Vector2d ped_vec_A2B(ped pedA, ped pedB)
{
	// returns the vector from A to B
	return pedB.get_pos() - pedA.get_pos();
}

Vector2d nearest_point(ped p, line l)
{
	// returns the nearest point of
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
	/* double dx = x - ped.x; */
	/* double dy = y - ped.y; */

	/* return sqrt(dx * dx + dy * dy); */
	return Vector2d(x, y);
}

Vector2d drift_acc(ped& p)
{
	p.update();

	Vector2d difference = p.desired_speed * p.desired_dir - p.get_vel();

	return difference / RELAX_TIME;
}

double b_parameter(const ped& A, const ped& B)
{
	Vector2d rab = ped_vec_A2B(A, B);
	double sum = rab.norm();
	sum += (rab - B.get_vel().norm() * dt * B.desired_dir).norm();
	sum = sum * sum;
	sum -= pow(B.get_vel().norm() * dt , 2);
	return 0.5 * sqrt(sum);
}

Vector2d interpersonal_force(ped& A,
							 ped& B,
							 double V0,
							 double sigma)
{
	A.update();
	B.update();

	double b = b_parameter(A, B);
	double db_pre = 1 / (8 * b);
	Vector2d rab = ped_vec_A2B(A, B);

	// check sign!!! Should be negative since I defined rab to be the opposite
	// from Helbing and Molnar
	double pre_factor = -V0 * db_pre * exp(-b/sigma) / sigma;

	double rx = rab[0];
	double ry = rab[1];

	Vector2d q = rab - B.get_vel().norm() * dt * B.desired_dir;
	double qx = q[0];
	double qy = q[1];

	// directional changes
	double db_drx =   2 * rx
					+ 2 * (rx - qx)
					+ 2 * rx * pow(rx*rx + ry*ry, -0.5)
					  * pow((rx - qx)*(rx - qx) + (ry - qy)*(ry - qy), 0.5)
					+ pow(rx*rx + ry*ry, 0.5) * pow(2*(rx - qx), -0.5);

	double db_dry =   2 * ry
		            + 2 * (ry - qy)
					+ 2 * ry * pow(rx*rx + ry*ry, -0.5)
					  * pow((rx - qx)*(rx - qx) + (ry - qy)*(ry - qy), 0.5)
					+ pow(rx*rx + ry*ry, 0.5) * pow(2*(ry - qy), -0.5);


	Vector2d F(pre_factor * db_drx, pre_factor * db_dry);
	return F;
}

Vector2d border_force(ped& A,
                      line& border,
					  double U0,
					  double R)
{
	// check sign!!
	Vector2d nearest = nearest_point(A, border);
	double distance = nearest.norm();

	// points from nearest to the pedestrian A
	Vector2d border_2_A = A.get_pos() - nearest;

	return (U0 / R) * exp(-distance/R) * border_2_A;
}

double w_adjustment(Vector2d& e, Vector2d& f, double phi)
{
	double dot = e.dot(f);
	double norm_quant = f.norm() * cos(phi);

	if ( dot >= norm_quant ) {
		return 1.0;
	}
	return 0.5;
}

double g_parameter(ped& A, Vector2d& omega)
{
	double ratio = A.max_speed / omega.norm();
	if ( 1 <= ratio ) {
		return 1.0;
	}
	return ratio;
}

void integrator(vector<reference_wrapper<ped>> pedestrians,
				vector<reference_wrapper<line>> borders,
				bool Euler)
{
	// steps: Calculate all forces on every pedestrian, then with these forces
	// apply the chosen integration scheme on the 4-vector (called "y") for each
	// pedestrian.

	// make a vector that holds the forces on each pedestrian
	vector<reference_wrapper<Vector2d>> forces;

	for (ped& p : pedestrians) {
		forces
}

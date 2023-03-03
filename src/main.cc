#include "methods.h"

/* CAAM 282 Final Project Main File
 *
 * Author: Noah Dohrmann
 *
 * Date: Winter 2023
 *
 * Note: In order to run this, the user needs the "Eigen" header files
 * found in eigen/Eigen from:
 * http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
 */

void trial_simulation()
{
	// tester code for a first sample simulation
	int num_peds = 10;
	int time_steps = 100;
	string sim_name = "trial_sim";

	// trial parameters from Helbing and Molnar
	double V0 = 2.1;
	double U0 = 10;
	double sigma = 0.3;
	double R = 0.2;

	// integration
	bool Euler = true;

	// borders and border points
	double x_length = 50;
	double y_length = 20;
	vector<point> points;
	point p1(0,0);
	point p2(x_length, 0);
	point p3(0, y_length);
	point p4(x_length, y_length);

	point lower(0,0);
	point upper(x_length,y_length);

	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);
	points.push_back(p4);

	vector<line> borders = initialize_borders(points);

	bool long_hallway = false;
	bool behavioral = false;
	vector<ped> pedestrians = initialize_pedestrians(num_peds,
                                                   borders,
                                                   lower,
                                                   upper,
                                                   long_hallway,
                                                   behavioral);
	perform_simulation(pedestrians,
                     borders,
                     time_steps,
                     sim_name,
                     V0,
                     U0,
                     sigma,
                     R,
                     Euler);

	cout << "Done!" << endl;
}

int main(int argc, char** argv) {
	//
	/* Vector4d v(0,0,1,1); */
	/* Vector2d e(1,1); */

	/* int id = 1; */

	/* // create a new pedestrian p */
	/* ped p(id, v, e, 10, 10, 100, false); */

	/* /1* p.print(); *1/ */

	/* point p1(1,0); */
	/* point p2(0,1); */

	/* line l(p1, p2); */

	/* l.print(); */
	
	/* cout << endl; */

	/* Vector2d np = nearest_point(p, l); */

	/* cout << "The nearest point is: " << np << endl; */

	/* Vector2d dd = p.desired_dir; */
	/* cout << "desired dir\n" << dd << endl; */

	trajectory traj;
	Vector2d p1(1,0);
	Vector2d p2(0,1);
	Vector2d v1(0.5,0.5);
	Vector2d v2(0.714, 0.0);

	traj.add_position(p1);
	traj.add_position(p2);
	traj.add_velocity(v1);
	traj.add_velocity(v2);

	string name = "sim1";

	traj.to_json(1, name);

	return 0;

}

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

void trial_simulation(bool behavioral, bool doorway)
{
	// tester code for a first sample simulation
	int num_peds = 40;
	int time_steps = 225;
	string sim_name = "trial_sim";

	// trial parameters from Helbing and Molnar
	double V0 = 2.1;
	/* double V0 = 5.0; // higher custom value */
	double U0 = 10;
	/* double U0 = 50; */
	double sigma = 0.3;
	double R = 0.2; // check dependence on time scale

	// integration
	bool Euler = true;

	// borders and border points
	double x_length = 50;
	double y_length = 25;
	vector<point> points;
	point p1(0,-1);
	point p2(x_length, -1);
	point p3(0, y_length);
	point p4(x_length, y_length);

	point lower(0,0);
	point upper(x_length,y_length);

	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);
	points.push_back(p4);

	if ( doorway )
	{
		// initialize points for the walls to make a doorway
		point w1(25,25);
		points.push_back(w1);
		point w2(25,15);
		points.push_back(w2);

		point w3(25,10);
		points.push_back(w3);
		point w4(25,-1);
		points.push_back(w4);

	}

	vector<line> borders = initialize_borders(points);

	// trying w/o borders
	/* vector<line> borders; */

	bool long_hallway = true;
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

	trial_simulation(false, false);

	/* point b1(0,25); */
	/* point b2(50,25); */

	/* line l(b1,b2); */

	/* Vector2d nearest = nearest_point(p,l); */

	/* cout << "nearest\n" << nearest << endl; */

	/* Vector2d dd = vanilla_des_dir(p); */
	/* cout << "dd:\n" << dd << endl; */

	/* ped p(1,Vector4d(10,10,1,1),Vector2d(11,11),1.3,1.4,100,false); */
	/* ped p2(2,Vector4d(12,12,1,0),Vector2d(11,11),1.3,1.4,100,false); */

	/* vector<ped> pedestrians; */
	/* pedestrians.push_back(p); */
	/* pedestrians.push_back(p2); */

	/* line l = get_vision_line(p, 0); */

	/* // zeta testing */
	/* double z1 = zeta_function(pedestrians, 0, 0); */
	/* double z2 = zeta_function(pedestrians, 0, M_PI / 6); */

	/* cout << "this is z1: " << z1 << endl; */
	/* cout << "this is z2: " << z2 << endl; */

	return 0;

}

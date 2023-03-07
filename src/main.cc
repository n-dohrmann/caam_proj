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
	int num_peds = 30;
	int time_steps = 300;
	string sim_name = "trial_sim";

	// trial parameters from Helbing and Molnar
	double V0 = 2.1;
	/* double V0 = 5.0; // higher custom value */
	double U0 = 10;
	double sigma = 0.3;
	double R = 0.2;

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

	vector<line> borders = initialize_borders(points);

	// trying w/o borders
	/* vector<line> borders; */

	bool long_hallway = true;
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

	/* trial_simulation(); */

	vector<double> ls = linspace(0,10,11);

	for (auto& l : ls)
	{
		cout << l << endl;
	}


	return 0;

}

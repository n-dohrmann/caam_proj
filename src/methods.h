#include "class_constructions.h"

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

	Vector2d difference = p.desired_speed * p.desired_dir.normalized() 
		- p.get_vel();

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

	// simple versiion

	/* Vector2d rab = ped_vec_A2B(A, B); */
	/* double distance = rab.norm(); */

	/* return (-V0 / sigma) * exp(-distance/sigma) * rab; */

	// simple version end

	if ( isnan(b) ) {
		cout << "Err - have nan values! at b" << endl;
		std::_Exit(EXIT_FAILURE);
	}

	double db_pre = 1 / (8 * b);
	Vector2d rab = ped_vec_A2B(A, B);

	// check sign!!! Should be negative since I defined rab to be the opposite
	// from Helbing and Molnar
	double pre_factor = -V0 * db_pre * exp(-b/sigma) / sigma;

	double rx = rab[0];
	double ry = rab[1];

	if ( isnan(rx) or isnan(ry) ) {
		cout << "Err - have nan values! at rx ry" << endl;
		std::_Exit(EXIT_FAILURE);
	}

	Vector2d q = rab - B.get_vel().norm() * dt * B.desired_dir;
	double qx = q[0];
	double qy = q[1];

	if ( isnan(qx) or isnan(qy) ) {
		cout << "Err - have nan values! at qx qy" << endl;
		std::_Exit(EXIT_FAILURE);
	}

	// directional changes
	double db_drx =   2 * rx
                  + 2 * (rx - qx)
                  + 2 * rx / sqrt(rx*rx + ry*ry)
                  * sqrt(abs((rx - qx)*(rx - qx) + (ry - qy)*(ry - qy)))
                  + sqrt(rx*rx + ry*ry) / sqrt(abs(2*(rx - qx)));

	double db_dry =   2 * ry
                  + 2 * (ry - qy)
                  + 2 * ry / sqrt(rx*rx + ry*ry)
                  * sqrt(abs((rx - qx)*(rx - qx) + (ry - qy)*(ry - qy)))
                  + sqrt(rx*rx + ry*ry) / sqrt(abs(2*(ry - qy)));

	if (isnan(db_drx) or isnan(db_dry)) {
		cout << "Err - have nan values! at db drxy" << endl;
		std::_Exit(EXIT_FAILURE);
	}

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

double g_parameter(ped& A, Vector2d omega)
{
	double ratio = A.max_speed / omega.norm();
	if ( 1 <= ratio ) {
		return 1.0;
	}
	return ratio;
}

Vector2d total_force_calc(ped& p,
                      int index,
                      vector<ped> &pedestrians,
                      vector<line> &borders,
                      double V0,
                      double U0,
                      double sigma,
                      double R)
{
	// gives back a vector of the total *scaled* forces F on a single
	// pedestrian. In a professional implementation, the code should exploit
	// force symmetries where that can be done, but this implementation will
	// take the longer, more expensive route for simplicity.
	/* Vector2d force(0.0,0.0); */

	// add drift acceleration
	/* force = force + drift_acc(p); */

	Vector2d force = drift_acc(p);

	// get all inter-pedestrian forces
	for (unsigned long i = 0; i < pedestrians.size(); ++i)
	{
		if ( (unsigned long)index == i )
		{
			continue;
		}

		if ( pedestrians[i].has_stopped )
		{
			continue;
		}

		Vector2d temp_force = interpersonal_force(p, pedestrians[i], V0, sigma);
		force = force 
			    + w_adjustment(p.desired_dir, temp_force, p.field_of_view) 
				* temp_force;
	}

	// get all pedestrian-border forces
	for (unsigned long i = 0; i < borders.size(); ++i) {
		Vector2d temp_force = border_force(p, borders[i], U0, R);
		force = force 
			    + w_adjustment(p.desired_dir, temp_force, p.field_of_view) 
				* temp_force;
	}

	return force;
}

void integrator(vector<ped> &pedestrians,
				vector<line> &borders,
				double V0,
				double U0,
				double sigma,
				double R,
				bool Euler)
{
	// steps: Calculate all forces on every pedestrian, then with these forces
	// apply the chosen integration scheme on the 4-vector (called "y") for each
	// pedestrian.

	// make a vector that holds the forces on each pedestrian
	vector<Vector2d> forces;

	int index = 0;
	for (ped& p : pedestrians) {
		p.update();
		forces.push_back(total_force_calc(p, 
                                      index, 
                                      pedestrians, 
                                      borders,
                                      V0,
                                      U0,
                                      sigma,
                                      R));
		++index;
	}

	index = 0;
	// next, perform the integration on each pedestrian with the forces
	// that were just found.
	if ( Euler ) {
		for (ped& p : pedestrians) {

			if ( not p.has_stopped )
			{
				// update the position vector
				p.y[0] += p.y[2] * dt;
				p.y[1] += p.y[3] * dt;

				// make sure to enforce the speed limit
				double g = g_parameter(p, p.get_vel());
				p.y[2] += forces[index][0] * g * dt;
				p.y[3] += forces[index][1] * g * dt;

				if ( (p.y[3] / abs(p.y[2]) >= 10 ) )
				{
					p.y[3] = -0.5 * p.y[3];
				}
				++index;
			}

			// update the trajectory
			p.traj.add_position(p.get_pos());
			p.traj.add_velocity(p.get_vel());

		}
	} else {
		cout << "implement RK method!" << endl;
		std::_Exit(EXIT_FAILURE);
	}
}

vector<ped> initialize_pedestrians(int num_peds,
                                   vector<line> &borders,
                                   point lower_boundary,
                                   point upper_boundary,
                                   bool long_hallway,
								   bool behavioral)
{
	// the two kinds of simulations to run are the long hallway and the narrow
	// corridor
	vector<ped> pedestrians;

	if (num_peds % 2 != 0) {
		cout << "use an even number of pedestrians for convenience!" << endl;
		std::_Exit(EXIT_FAILURE);
	}

	// geometry stuff for the simulation arena
	double dx = upper_boundary.x - lower_boundary.x;
	double dy = upper_boundary.y - lower_boundary.y;

	// give a buffer so that  the pedestrians are not placed directly
	// on top of walls
	double density = num_peds  / (2 * (dy - 5));

	// random number generator
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);
	std::uniform_real_distribution<> uniform(0+1, dy-1);
	// distribution values from Moussaid
	std::normal_distribution<double> distribution (1.3,0.2);
	double gen_desired_speed;
	double px, py, vx, vy, des_x, des_y;
	double max_speed;
	// standardized value for the field of view
	double fov = 100;


	if ( long_hallway ) {
		for (int id = 0; id < num_peds; ++id)
		{
			gen_desired_speed = abs(distribution(generator));
			max_speed = 1.3 * gen_desired_speed;

			// alternatively place the people left and right
			px = (id % 2) * dx;
			py = 2 + (id*(0.9*dy)/num_peds);

			des_x = ((id+1) % 2) * dx;
			des_y = uniform(generator);
			des_y = uniform(generator);
			/* des_y = dy - (2 + py); */

			// add check that this is not too close to
			// another pedestrian's desire point!!!!!!
			/* des_y = py + 2 * pow(-1,id); */

			Vector2d desired_loc(des_x, des_y);
			
			// init velocity and desired direction
			Vector2d vel(des_x - px, des_y - py);
			Vector2d desired_dir = vel / vel.norm();
			vel = gen_desired_speed * vel / vel.norm();

			Vector4d y(px, py, vel[0], vel[1]);

			ped p(id,
			      y,
			      desired_loc,
			      gen_desired_speed,
			      max_speed,
			      fov,
			      behavioral);
			
			pedestrians.push_back(p);

		}

	} else {
		cout << "implement the narrow corridor" << endl;
		std::_Exit(EXIT_FAILURE);

	}

	return pedestrians;
}


vector<line> initialize_borders(vector<point> points)
{
	vector<line> borders;
	int num = (int)points.size();

	if (num % 2 != 0) {
		cout << "must use an even number of points!" << endl;
		std::_Exit(EXIT_FAILURE);
	}

	for (int i = 0; i < num - 1; i = i + 2) {
		borders.push_back(line(points[i],points[i+1]));
	}
	return borders;
}

void perform_simulation(
                vector<ped> &pedestrians,
                vector<line> &borders,
                int time_steps,
                string sim_name,
                double V0,
                double U0,
                double sigma,
                double R,
				bool Euler)
{
	// note that the borders should be already constructed prior to
	// calling this function
	//
	// pedestrians will come pre-initialized to this function with
	// initial positions, velocities, desirec locations, etc
	//
	
	// this might not be used, but it will be reported in the output.
	double total_time = time_steps * dt;

	for (int time = 0; time < time_steps; ++time)
	{
		// call integration - this should be all that's necessary?
		integrator(pedestrians,
				   borders,
				   V0,
				   U0,
				   sigma,
				   R,
				   Euler);
	}

	// write trajectories to file
	for (auto& p : pedestrians)
	{
		p.traj.to_json(p.id, p.desired_loc, sim_name);
	}

}

double cos_vec(Vector2d A, Vector2d B)
{
	// need a different name than the STL "cos"?
	return A.dot(B) / (A.norm() * B.norm());
}

vector<double> linspace(double lower, double upper, int n_steps)
{
	vector<double> lin;
	double size = (upper - lower) / double(n_steps-1);

	for (int i = 0; i < n_steps; ++i)
	{
		lin.push_back(lower + ((double)(i) * size));
	}
	return lin;
}

double get_theta(Vector2d v)
{
	double x = v[0];
	double y = v[1];
	double theta = atan( y / x );

	if ( (y < 0) and (x < 0) )
	{
		theta += M_PI;
	} else if ((y > 0) and (x < 0))
	{
		theta += M_PI;
	}
	return theta;
}

double zeta_function(vector<ped> &pedestrians,
                     int index,
                     double phi)
{
	// the distance for the pedestrian pedestrians[index] for
	// the angle phi. Called zeta function since that's the letter
	// I used for the function in the proposal. the actual name
	// varies by source in the literature....

	double zeta = d_max;

	// create a "line" for the line of sight coming from the pedestrian
	// for the chosen angle.

	// theta is the angle between ped velocity and x axis. This will
	// be used in the rotation matrix.
	double theta = get_theta(pedestrians[index].get_vel());

	Matrix2d rotation;

	rotation << cos(theta), -sin(theta),
			    sin(theta),  cos(theta);

	Vector2d dx(d_max*cos(theta), 0);
	Vector2d dy(0, d_max*sin(theta));

	dx = rotation * dx;
	dy = rotation * dy;

	Vector2d line_end = dx + dy;

	point p0 = pedestrians[index].get_point();
	point p1(line_end[0], line_end[1]);

	line vision_line(p0,p1);

	Vector2d ped_pos = pedestrians[index].get_pos();
	Vector2d nearest;
	double dist; // distance from other ped to line
	double ped_range; // distance from other ped to curr ped

	int curr = 0;
	for (ped& p : pedestrians)
	{
		if ( curr == index )
		{
			continue;
		}

		nearest = nearest_point(p, vision_line);
		dist = (ped_pos - nearest).norm();

		if ( dist < dist_threshold )
		{
			ped_range = (p.get_pos() - ped_pos).norm();
			if ( ped_range < zeta )
			{
				zeta = ped_range;
			}


		}
		++curr;
	}

	return zeta;
}





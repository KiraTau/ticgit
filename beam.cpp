#include <math.h>
#include <cmath>

/**
  Correct range: p_hit
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input r_max: maximum sensor range
  \input: sigma_hit: sensor standart deviation
  \return phit
*/
double pi = 3.1415926535897;

double p_hit( double r, double rs, double r_max, double sigma_hit ){
	double phit;
	// TODO
	//std::cout<<"r_max is "<<r_max<<std::endl;//r_max is 15
	if (r < r_max){//less than the true range
		double eta_hit = 1/(0.5 + 0.5 * erf((r_max-rs)/(sqrt(2)*sigma_hit)));//evaluate integral using error function
		phit = eta_hit * (1/sqrt(2*pi*pow(sigma_hit,2))) * exp(-0.5*pow((r-rs)/sigma_hit,2));
	}else{
		phit = 0;
	}  
	return phit;
}

/**
  Unexpected object: p_short
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input lambda_short: exponential parameter
  \return p_short
*/
double p_short( double r, double rs, double r_max, double lambda_short ){
	double pshort;
	// TODO
	if (r < rs){//less than the true range
		double eta_short = 1/(1-exp(-lambda_short * rs));
		pshort = eta_short * lambda_short * exp(-lambda_short * r);
	}else{
		pshort = 0;
	}
	return pshort;
}

/**
  Failure: p_max
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input r_max: maximum sensor range
  \return p_max
 */
double p_max( double r, double rs, double r_max ){
	double pmax;
	// TODO
	if (r = r_max){
		pmax = 1;
	}else{
		pmax = 0;
	}
	return pmax;
}  

/**
   Random measurement: p_rand
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input r_max: maximum sensor range
  \return p_rand
*/
double p_rand( double r, double rs, double r_max ){
	double prand;
	// TODO
	if (r < r_max){//less than the true range
		prand = 1/r_max;
	}else{
		prand = 0;
	}  
	return prand;
}

/**
   beam model
   Implement the beam model for a laser range finder. The function
   returns the probability P( z | s, m ), that is the probability that a 
   measurement z was caused by a robot in a state s and in a map m.
   \input Map* m:  The map of the environment the robot.
   \input Scan z: A vector of laser beams. Each beam has two members: range and 
                  bearing. You can access the range if the ith beam with
		  z[i].range and the bearing of the ith beam with z[i].bearing.
   \input State s: The state of the robot with respect to the world frame.
                   The x coordinate is given by s[0], the y coordinate by s[1]
		   and the angle theta by s[2].
   \input Pose laser_pose: The position and orientation of the laser with 
                           respect to the robot's coordinate frame. The x, y
			   and angle are given by coordinate is laser_pose[0]
			   laser_pose[1] and laser_pose[2] respectively.
   \input sigma_hit: The standard variation of errors of a beam
   \input lambda_short: The p_short exponential parameter
   \input r_max:  The laser maximum range (in meters)
   \input w_hit:  The coefficient of measurements
   \input w_short:The coefficient of measurements
   \input w_max:  The coefficient of measurements
   \input w_rand: The coefficient of random errors
   \return        The probability p( z | s, m )
*/
double beam_model( Map* map,
		   Scan& z,
		   State s, 
		   Pose laser_pose, 
		   double sigma_hit, 
		   double lambda_short,
		   double r_max, 
		   double w_hit,
		   double w_short,
		   double w_max,
		   double w_rand ){
  
	double p = 0;
	// TODO
	double x = s[0];
	double y = s[1];
	double theta = s[2];
	double xs = laser_pose[0];
	double ys = laser_pose[1];
	double thetas = laser_pose[2];
	double x_bs = x + xs * cos(theta) - ys*sin(theta);//x coordinate of base to sensor
	double y_bs = y + ys * cos(theta) + xs*sin(theta);//y coordinate of base to sensor
	double theta_bs = theta + thetas;//theta of base to sensor
	for (int i = 0; i < z.size(); i += 5){//take one measurement from every 5 measurements
		double bearing = z[i].bearing;
		double range = z[i].range;
		double rs = GetRangeFromMap(map, x_bs, y_bs, theta_bs + bearing, r_max);//true range of the beam
		double Phit = p_hit(range, rs, r_max, sigma_hit);
		double Pshort = p_short(range, rs, r_max, lambda_short);
		double Pmax = p_max(range, rs, r_max);
		double Prand = p_rand(range, rs, r_max);
		double p_i = w_hit*Phit+w_short*Pshort+w_max*Pmax+w_rand*Prand;
		p_i = log(p_i);
		p += p_i;
	}
	//std::cout<<"x is "<<x<<std::endl;
	//std::cout<<"y is "<<y<<std::endl;
	//std::cout<<"theta is "<<theta<<std::endl;
	//std::cout<<"xs is "<<xs<<std::endl;
	//std::cout<<"ys is "<<ys<<std::endl;
	//std::cout<<"thetas is "<<thetas<<std::endl;
	//std::cout<<"x_bs is "<<x_bs<<std::endl;
	//std::cout<<"y_bs is "<<y_bs<<std::endl;
	//std::cout<<"theta_bs is "<<theta_bs<<std::endl;
	p = exp(p);//get only one arrow
	return p;
}

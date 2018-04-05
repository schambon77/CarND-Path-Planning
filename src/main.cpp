#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<string> get_successor_states(double ref_vel, int lane, string state) {
	//KLN: keep lane no change
	//KLA: keep lane accelerate
	//KLD: keep lane decelerate
	//LCL: lane change left
	//LCR: lane change right
    vector<string> states;
    states.push_back("KLN");
    if (ref_vel < 49.5) {
        states.push_back("KLA");
    }
    if (ref_vel > 0) {
    	states.push_back("KLD");
    }
    if (lane > 0) {
    	states.push_back("LCL");
    }
    if (lane < 2) {
    	states.push_back("LCR");
    }
    return states;
}

double get_simple_prediction(double s, double vx, double vy, int number_timesteps) {
	double speed = sqrt(vx*vx + vy*vy);
	double predicted_s = s;
	predicted_s += ((double)number_timesteps*0.02*speed);
	return predicted_s;
}

void get_prediction(double car_vx, double car_vy, float car_s, float car_d,
		const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
		vector<double> &next_x_vals, vector<double> &next_y_vals) {

	double speed = sqrt(car_vx*car_vx + car_vy*car_vy);
	double s = car_s;
	vector<double> next_point;
	for (int i = 0; i < 50; i++) {
		next_point = getXY(s + i * 0.02 * speed, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);  //here we assume other vehicles go at constant speed without changing lanes
		next_x_vals.push_back(next_point[0]);
		next_y_vals.push_back(next_point[1]);
	}
}

void get_trajectory(double car_x, double car_y, double car_yaw, double end_path_s, const vector<double> &previous_path_x, const vector<double> &previous_path_y,
		const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, const int lane, const double ref_vel,
		vector<double> &next_x_vals, vector<double> &next_y_vals) {

  	//sparse vector x, y points
  	vector<double> ptsx;
  	vector<double> ptsy;

  	double ref_x = car_x;
  	double ref_y = car_y;
  	double ref_yaw = deg2rad(car_yaw);

  	int prev_size = previous_path_x.size();

  	if (prev_size < 2) {
  		double prev_car_x = car_x - cos(car_yaw);
  		double prev_car_y = car_y - sin(car_yaw);

  		ptsx.push_back(prev_car_x);
  		ptsx.push_back(car_x);

  		ptsy.push_back(prev_car_y);
  		ptsy.push_back(car_y);
  	}
  	else {
  		ref_x = previous_path_x[prev_size - 1];
  		ref_y = previous_path_y[prev_size - 1];

  		double ref_x_prev = previous_path_x[prev_size - 2];
  		double ref_y_prev = previous_path_y[prev_size - 2];
  		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

  		ptsx.push_back(ref_x_prev);
  		ptsx.push_back(ref_x);

  		ptsy.push_back(ref_y_prev);
  		ptsy.push_back(ref_y);
  	}

  	vector<double> next_wp0 = getXY(end_path_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  	vector<double> next_wp1 = getXY(end_path_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  	vector<double> next_wp2 = getXY(end_path_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  	ptsx.push_back(next_wp0[0]);
  	ptsx.push_back(next_wp1[0]);
  	ptsx.push_back(next_wp2[0]);

  	ptsy.push_back(next_wp0[1]);
  	ptsy.push_back(next_wp1[1]);
  	ptsy.push_back(next_wp2[1]);

  	//transform points into car frame of reference
  	for (int i = 0; i < ptsx.size(); i++) {
  		double shift_x = ptsx[i] - ref_x;
  		double shift_y = ptsy[i] - ref_y;
  		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
  		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  	}

  	tk::spline s;

  	for (int i = 0; i < ptsx.size(); i++) {
  		//cout << "Pt: " << i << ", x: " << ptsx[i] << ", y: " << ptsy[i] << endl;
  	}
  	s.set_points(ptsx, ptsy);

  	for (int i = 0; i < prev_size; i++) {
  		next_x_vals.push_back(previous_path_x[i]);
  		next_y_vals.push_back(previous_path_y[i]);
  	}

  	double target_x = 30.0;
  	double target_y = s(target_x);
  	double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

  	double x_add_on = 0.0;

  	for (int i = 1; i <= 50 - prev_size; i++) {
  		double N = (target_dist / (0.02*ref_vel/2.24));
  		double x_point = x_add_on + (target_x)/N;
  		double y_point = s(x_point);

  		x_add_on = x_point;

  		double x_ref = x_point;
  		double y_ref = y_point;

  		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
  		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

  		x_point += ref_x;
  		y_point += ref_y;

  		next_x_vals.push_back(x_point);
  		next_y_vals.push_back(y_point);
  	}
}

double get_collision_cost(const vector<double> &trajectory_x, const vector<double> &trajectory_y, const vector<double> &prediction_x, const vector<double> &prediction_y) {
	double cost = 0.0;
	double dist = 0.0;
	double dist_zero = 999999.0;
	for (int i = 0; i < prediction_x.size(); i++) {
		dist = distance(trajectory_x[i], trajectory_y[i], prediction_x[i], prediction_y[i]);
		//cout << "i: " << i << ", trajectory_x: " << trajectory_x[i] << ", trajectory_y: " << trajectory_y[i] << ", prediction_x: " << prediction_x[i] << ", prediction_y: " << prediction_y[i] << ", dist: " << dist << endl;
		//cout << "i: " << i << ", dist: " << dist;
		if (i == 0) {
			dist_zero = dist;
		}
		if (dist < 10.0) { //avoid trajectories less than x meters away from other vehicles
			cost = 1.0;
			//cout << "WATCH OUT: POTENTIAL COLLISION - dist zero: " << dist_zero << endl;
			break;
		}
	}
	//cout << endl;
	cout << "Collision cost: " << cost << endl;
	return cost;
}

double get_too_close_cost(const vector<double> &trajectory_x, const vector<double> &trajectory_y, const vector<double> &prediction_x, const vector<double> &prediction_y, const double ref_vel) {
	double cost = 0.0;
	double dist = 0.0;
	double safety_dist = ref_vel * 1.61 * 0.28 * 1.5;   //compute safety distance related to keeping x seconds interval from other car
	//cout << "Safety distance: " << safety_dist << endl;
	for (int i = 0; i < prediction_x.size(); i++) {
		dist = distance(trajectory_x[i], trajectory_y[i], prediction_x[i], prediction_y[i]);
		if (dist < safety_dist) { //avoid trajectories less than 10 meters away from other vehicles
			cost = (safety_dist - dist)/safety_dist;
			break;
		}
	}
	cout << "Too close cost: " << cost << endl;
	return cost;
}

double get_efficiency_cost(const double ref_vel) {
	double speed_limit = 49.5;
	double cost = 0.0;
	if (ref_vel > speed_limit) {
		cost = 1.0;
	}
	else {
		cost = (speed_limit - ref_vel)/speed_limit;
	}
	cout << "Efficiency cost: " << cost << endl;
	return cost;
}

double get_acceleration_cost(const vector<double> &trajectory_x, const vector<double> &trajectory_y, const double ref_vel) {
	double cost = 0.0;
	double dist = 0.0;
	double prev_vel = 0.0;
	double vel = 0.0;
	double acc_inst = 0.0;
	double acc_sum = 0.0;
	double acc_avg = 0.0;
	int count = 0;
	vector<double> accs_inst;
	for (int i = 0; i < trajectory_x.size() - 1; i++) {
		dist = distance(trajectory_x[i], trajectory_y[i],trajectory_x[i+1], trajectory_y[i+1]);
		vel = dist / 0.02;
		if (i > 0) {    //discard first computation of instantaneous acceleration
			acc_inst = (vel - prev_vel) / 0.02;
			accs_inst.push_back(acc_inst);
			acc_sum += acc_inst;
			//cout << "dist " << dist << " prev vel " << prev_vel << " vel " << vel << " acc inst " << acc_inst << " acc avg " << acc_avg << endl;
			if (count >= 9) {   // 10 * 0.02 second = 0.2 second
				acc_avg = acc_sum / 10.0;
				if (acc_avg > 10.0) {
					cost = 1.0;
				}
				acc_sum -= accs_inst[0];
				accs_inst.erase(accs_inst.begin());
			}
			else {
				count += 1;
			}
		}
		prev_vel = vel;
	}
	cout << "Acceleration cost: " << cost << endl;
	return cost;
}

double get_cost(const vector<double> &trajectory_x, const vector<double> &trajectory_y, const vector<double> &prediction_x, const vector<double> &prediction_y, const double ref_vel) {
	double cost = 0.0;
	cost += 100.0 * get_collision_cost(trajectory_x, trajectory_y, prediction_x, prediction_y);
	cost += 50.0 * get_too_close_cost(trajectory_x, trajectory_y, prediction_x, prediction_y, ref_vel);
	cost += 20.0 * get_acceleration_cost(trajectory_x, trajectory_y, ref_vel);
	cost += 1.0 * get_efficiency_cost(ref_vel);
	cout << "Total cost: " << cost << endl;
	return cost;
}

int get_closest_front_car_in_lane(const vector<vector<double>> sensor_fusion, int lane, double car_s) {
	int ret = -1;
	double dist_s = 999999.0;
  	for (int i = 0; i < sensor_fusion.size(); i++) {
  		float d = sensor_fusion[i][6];
  		if (d < (2+4*lane+2) && (d >= (2+4*lane-2))) {  //a car is in lane
  			double check_car_s = sensor_fusion[i][5];
  			//cout << "Car " << i << " is in lane " << lane << " at distance " << check_car_s - car_s << endl;
  			if (check_car_s >= car_s) {   //the car is in front
  				if (abs(check_car_s - car_s) < dist_s) {
  					ret = i;
  					dist_s = abs(check_car_s - car_s);
  				}
  			}
  		}
  	}
  	//cout << "Closest car in front: " << ret << " in lane " << lane << " at distance: " << dist_s << endl;
  	return ret;
}

vector<int> get_closest_cars_in_side_lane(const vector<vector<double>> &sensor_fusion, const int lane, const double car_s, const double ref_vel, bool is_left) {
	vector<int> ret;
	double dist_radius = ref_vel * 1.61 * 0.28 * 3.0;   //consider all vehicles within 3 seconds
  	//cout << "Distance radius: " << dist_radius << endl;
  	bool is_within_bounds = false;
	int lane_of_interest = 0;
  	if (is_left) {
  		if (lane > 0) {
  			lane_of_interest = lane - 1;
  			is_within_bounds = true;
  		}
  	}
  	else {
  		if (lane < 2) {
  			lane_of_interest = lane + 1;
  			is_within_bounds = true;
  		}
  	}
  	if (is_within_bounds) {
		for (int i = 0; i < sensor_fusion.size(); i++) {
			float d = sensor_fusion[i][6];
			if (d < (2+4*lane_of_interest+2) && (d> (2+4*lane_of_interest-2))) {  //a car is in lane of interest
				double check_car_s = sensor_fusion[i][5];
				double dist = abs(check_car_s - car_s);   //approximation, forget about d
				if (dist < dist_radius) {
					ret.push_back(i);
					//cout << "Vehicle " << i << " is in target lane " << lane_of_interest << " within distance " << dist << endl;
				}
			}
		}
  	}
  	return ret;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;  //lane id
  double ref_vel = 0.0;  //mph
  string state = "KL";

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	int prev_size = previous_path_x.size();

          	//cout << "Car x: " << car_x << endl;
          	//cout << "Car y: " << car_y << endl;
          	//cout << "Car s: " << car_s << endl;
          	//cout << "Car d: " << car_d << endl;
          	//cout << "Car yaw: " << car_yaw << endl;
          	//cout << "Car speed: " << car_speed << endl;
          	//cout << "Previous path size: " << prev_size << endl;
          	//cout << "End path s: " << end_path_s << endl;
          	//cout << "End path d: " << end_path_d << endl;

          	if (prev_size == 0) {
          		end_path_s = car_s;
          	}

          	//dump sensor fusion info for debug purposes
          	for (int i = 0; i < sensor_fusion.size(); i++) {
              	//cout << "Car " << i << " at s " << sensor_fusion[i][5] << "and d " << sensor_fusion[i][6] << endl;
          	}

          	vector<string> successor_states = get_successor_states(ref_vel, lane, state);

          	int front_car_id = get_closest_front_car_in_lane(sensor_fusion, lane, car_s);
          	vector<int> right_cars = get_closest_cars_in_side_lane(sensor_fusion, lane, car_s, ref_vel, false);
          	vector<int> left_cars = get_closest_cars_in_side_lane(sensor_fusion, lane, car_s, ref_vel, true);

          	double cost = 0.0;
          	double min_cost = 999999.0;
          	int min_cost_lane = lane;
          	double min_cost_ref_vel = ref_vel;
          	string next_state = "KLN";
          	for (int i = 0; i < successor_states.size(); i++) {
          		cout << "Checking potential next state : " << successor_states[i] << endl;
      			double tmp_ref_vel = ref_vel;
      			int tmp_lane = lane;
      			double tmp_car_vx = 0.0;
      			double tmp_car_vy = 0.0;
      			double tmp_car_s = 0.0;
      			double tmp_car_d = 0.0;
          		if (successor_states[i].compare("KLN") == 0 || successor_states[i].compare("KLA") == 0 || successor_states[i].compare("KLD") == 0) {
                  	if (successor_states[i].compare("KLD") == 0) {
                  		tmp_ref_vel -= 0.224;
                  	}
                  	else if (successor_states[i].compare("KLA") == 0) {
                  		tmp_ref_vel += 0.224;
                  	}
                  	vector<double> car_next_x_vals;
                  	vector<double> car_next_y_vals;
                  	vector<double> tmp_next_x_vals;
                  	vector<double> tmp_next_y_vals;
		          	get_trajectory(car_x, car_y, car_yaw, end_path_s, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, tmp_lane, tmp_ref_vel, car_next_x_vals, car_next_y_vals);
					if (front_car_id >= 0) {
						tmp_car_vx = sensor_fusion[front_car_id][3];
						tmp_car_vy = sensor_fusion[front_car_id][4];
						tmp_car_s = sensor_fusion[front_car_id][5];
						tmp_car_d = sensor_fusion[front_car_id][6];
						get_prediction(tmp_car_vx, tmp_car_vy, tmp_car_s, tmp_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y, tmp_next_x_vals, tmp_next_y_vals);
					}
					//cout << "Car " << front_car_id << endl;
					cost = get_cost(car_next_x_vals, car_next_y_vals, tmp_next_x_vals, tmp_next_y_vals, tmp_ref_vel);
          		}
          		else {
          			vector<int> side_cars;
          			if (successor_states[i].compare("LCL") == 0) {
          				tmp_lane -= 1;
          				side_cars = left_cars;
          			}
          			else if (successor_states[i].compare("LCR") == 0) {
          				tmp_lane += 1;
          				side_cars = right_cars;
          			}
                  	vector<double> car_next_x_vals;
                  	vector<double> car_next_y_vals;
                  	get_trajectory(car_x, car_y, car_yaw, end_path_s, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, tmp_lane, tmp_ref_vel, car_next_x_vals, car_next_y_vals);
          			if (side_cars.size() > 0) {   //if some cars are in side lane
              			double lane_change_max_cost = 0.0;
						for (int j = 0; j < side_cars.size(); j++) {
							vector<double> tmp_next_x_vals;
							vector<double> tmp_next_y_vals;
							tmp_car_vx = sensor_fusion[side_cars[j]][3];
							tmp_car_vy = sensor_fusion[side_cars[j]][4];
							tmp_car_s = sensor_fusion[side_cars[j]][5];
							tmp_car_d = sensor_fusion[side_cars[j]][6];
							get_prediction(tmp_car_vx, tmp_car_vy, tmp_car_s, tmp_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y, tmp_next_x_vals, tmp_next_y_vals);
							//cout << "Car " << side_cars[j] << endl;
							cost = get_cost(car_next_x_vals, car_next_y_vals, tmp_next_x_vals, tmp_next_y_vals, tmp_ref_vel);
							if (cost > lane_change_max_cost) {
								lane_change_max_cost = cost;
							}
						}
	          			cost = lane_change_max_cost;
          			}
          			else {    //no cars in side lane
						vector<double> tmp_next_x_vals;
						vector<double> tmp_next_y_vals;
						cost = get_cost(car_next_x_vals, car_next_y_vals, tmp_next_x_vals, tmp_next_y_vals, tmp_ref_vel);
          			}
          		}
				if (cost < min_cost) {
					next_state = successor_states[i];
					min_cost = cost;
					min_cost_ref_vel = tmp_ref_vel;
					min_cost_lane = tmp_lane;
				}
          	}

          	//execute next step
          	lane = min_cost_lane;
          	ref_vel = min_cost_ref_vel;
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	get_trajectory(car_x, car_y, car_yaw, end_path_s, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, lane, ref_vel, next_x_vals, next_y_vals);

          	cout << "Next state: " <<  next_state << " in lane: " << lane << " at speed: " << ref_vel << endl;

//          	bool too_close = false;
//
//          	for (int i = 0; i < sensor_fusion.size(); i++) {
//          		float d = sensor_fusion[i][6];
//          		if (d < (2+4*lane+2) && (d> (2+4*lane-2))) {  //a car is in lane
//          			double vx = sensor_fusion[i][3];
//          			double vy = sensor_fusion[i][4];
//          			double check_car_s = sensor_fusion[i][5];
//          			check_car_s = get_simple_prediction(check_car_s, vx, vy, prev_size);
//
//          			if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {
//          				too_close = true;
//
//          				if (lane > 0) {
//          					lane = 0;
//          				}
//          			}
//          		}
//          	}
//
//          	if (too_close) {
//          		ref_vel -= 0.224;
//          	}
//          	else if (ref_vel < 49.5) {
//          		ref_vel += 0.224;
//          	}
//
//          	get_trajectory(car_x, car_y, car_yaw, car_s, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, lane, ref_vel, next_x_vals, next_y_vals);

//          	//sparse vector x, y points
//          	vector<double> ptsx;
//          	vector<double> ptsy;
//
//          	double ref_x = car_x;
//          	double ref_y = car_y;
//          	double ref_yaw = deg2rad(car_yaw);
//
//          	if (prev_size < 2) {
//          		double prev_car_x = car_x - cos(car_yaw);  //TODO: check if it should be ref_yaw here
//          		double prev_car_y = car_y - sin(car_yaw);  //TODO: check if it should be ref_yaw here
//
//          		ptsx.push_back(prev_car_x);
//          		ptsx.push_back(car_x);
//
//          		ptsy.push_back(prev_car_y);
//          		ptsy.push_back(car_y);
//          	}
//          	else {
//          		ref_x = previous_path_x[prev_size - 1];
//          		ref_y = previous_path_y[prev_size - 1];
//
//          		double ref_x_prev = previous_path_x[prev_size - 2];
//          		double ref_y_prev = previous_path_y[prev_size - 2];
//          		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
//
//          		ptsx.push_back(ref_x_prev);
//          		ptsx.push_back(ref_x);
//
//          		ptsy.push_back(ref_y_prev);
//          		ptsy.push_back(ref_y);
//          	}
//
//          	vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
//          	vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
//          	vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
//
//          	ptsx.push_back(next_wp0[0]);
//          	ptsx.push_back(next_wp1[0]);
//          	ptsx.push_back(next_wp2[0]);
//
//          	ptsy.push_back(next_wp0[1]);
//          	ptsy.push_back(next_wp1[1]);
//          	ptsy.push_back(next_wp2[1]);
//
//          	//transform points into car frame of reference
//          	for (int i = 0; i < ptsx.size(); i++) {
//          		double shift_x = ptsx[i] - ref_x;
//          		double shift_y = ptsy[i] - ref_y;
//          		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
//          		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
//          	}
//
//          	tk::spline s;
//
//          	s.set_points(ptsx, ptsy);
//
//          	for (int i = 0; i < prev_size; i++) {
//          		next_x_vals.push_back(previous_path_x[i]);
//          		next_y_vals.push_back(previous_path_y[i]);
//          	}
//
//          	double target_x = 30.0;
//          	double target_y = s(target_x);
//          	double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
//
//          	double x_add_on = 0.0;
//
//          	for (int i = 1; i <= 50 - prev_size; i++) {
//          		double N = (target_dist / (0.02*ref_vel/2.24));
//          		double x_point = x_add_on + (target_x)/N;
//          		double y_point = s(x_point);
//
//          		x_add_on = x_point;
//
//          		double x_ref = x_point;
//          		double y_ref = y_point;
//
//          		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//          		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
//
//          		x_point += ref_x;
//          		y_point += ref_y;
//
//          		next_x_vals.push_back(x_point);
//          		next_y_vals.push_back(y_point);
//          	}

          	for (int i = 0; i < next_x_vals.size(); i++) {
          		//cout << "Next x and y - " << i << ": " << next_x_vals[i] << ", " << next_y_vals[i] << endl;
          	}

          	cout << " " << endl;

          	//END

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
      		//cout << "JSON msg back to simulator: " << msg << endl;

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

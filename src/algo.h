#ifndef ALGO_H
#define ALGO_H

#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;

// PRIMARY DATA STRUCTURES for the algorithm
struct worldData{
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

struct carData{
  // Main car's localization Data
  double x; 
  double y;
  double s;
  double d;
  double yaw; //unit: degrees
  double speed; //unit: miles/hr
  // Previous path data given to the Planner which weren't used
  vector<double> previous_x;
  vector<double> previous_y;
  // Previous path's end s and d values 
  double end_s;
  double end_d;
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  vector<vector<double>> sensor_fusion;
  
  double ACC_INC=.3;
  double ACC_DEC=.15;
  double MAX_SPEED=49;
  double MIN_VERT_DIST=20;
};

struct curve{
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> d;
};

double mph2mps(double miles_per_hr){return miles_per_hr*0.44704;}
double mps2mph(double m_per_sec){return m_per_sec/0.44704;}

int findLane(double d){
  int lane;
  if (d>0 && d<4){
    lane=0; //Fast lane
  } else if (d>4 && d<8){
    lane=1; //Middle lane
  } else {
    lane=2; //Slow lane
  }
  return lane;
}
curve generate_spline(worldData waypoints, carData ego, int final_lane, bool slow, bool traj_long){
  curve traj;
  curve spl;
  double dt=.02;
  vector<double> spl_x;
  vector<double> spl_y;
  double ref_x;
  double ref_y;
  double ref_yaw;
  double prev_x;
  double prev_y;
  double ref_speed;
  
  ///////////////////////////////////////////////////////////////////
  //First 2 points of spline
  if (ego.previous_x.size()<1){
    ref_x=ego.x;
    ref_y=ego.y;
    ref_yaw=ego.yaw;
    prev_x=ref_x-cos(ref_yaw);
    prev_y=ref_y-sin(ref_yaw);
    
    spl.x.push_back(prev_x);
    spl.x.push_back(ref_x);
    spl.y.push_back(prev_y);
    spl.y.push_back(ref_y);
    ref_speed=0;
  }
  else{
    prev_x=ego.previous_x.end()[-2];
    prev_y=ego.previous_y.end()[-2];
    ref_x=ego.previous_x.end()[-1];
    ref_y=ego.previous_y.end()[-1];
    spl.x.push_back(prev_x);
    spl.x.push_back(ref_x);
    spl.y.push_back(prev_y);
    spl.y.push_back(ref_y);
    ref_yaw=atan2(ref_y-prev_y,ref_x-prev_x);
    ref_speed=mps2mph(distance(ref_x, ref_y, prev_x, prev_y)/dt);
    //std::cout<<"ref_speed: "<<ref_speed<<std::endl;
  }

  //lane- 0,1,2
  //Remaining points of spline (max 30m travel in 1s at max speed)
  for (int ds=40;ds<100;ds+=20){
    vector<double> pt=getXY(ego.end_s+ds,2+4*final_lane,waypoints.s,waypoints.x,waypoints.y);
    spl.x.push_back(pt[0]);
    spl.y.push_back(pt[1]);
  }

  //Spline points in moving frame
  for (int i=0;i<spl.x.size();i++){
    double shift_x=spl.x[i]-ref_x;
    double shift_y=spl.y[i]-ref_y;
    spl.x[i]=shift_x*cos(-ref_yaw)-shift_y*sin(-ref_yaw);
    spl.y[i]=shift_x*sin(-ref_yaw)+shift_y*cos(-ref_yaw);
  }

  //PRINT SPLINE PTS
  //for (int i=0;i<spl.x.size();i++){
  //  std::cout<<"x:"<<spl.x[i]<<", y:"<<spl.y[i]<<std::endl;
  //}
  //std::cout<<"END SPLINE"<<std::endl;

  //Create spline
  tk::spline s;
  s.set_points(spl.x,spl.y);
  ///////////////////////////////////////////////////////////////////

  //If n_pts=50, we are predicting a trajectory upto 1s in future
  string state;
  int n_pts;
  if (traj_long){
    n_pts=150;
  } else {
    n_pts=50;
  }
  //Add old remaining trajectory
  for (int i=0;i<ego.previous_x.size();i++){
    traj.x.push_back(ego.previous_x[i]);
    traj.y.push_back(ego.previous_y[i]);
  }

  //Add new predicted spline
  double target_x=30;
  double target_y=s(target_x);
  double target_dist=sqrt(pow(target_y,2)+pow(target_x,2));
  
  double x_rel=0;
  for (int i=1;i<=n_pts-ego.previous_x.size();i++){
    //Check speed
    if (slow && ref_speed>30){
      ref_speed-=ego.ACC_DEC;
    }else if (ref_speed<ego.MAX_SPEED){
      ref_speed+=ego.ACC_INC;
    }

    //Update speed and point
    double time=target_dist/mph2mps(ref_speed);
    double iter=time/dt;
    double x_incr=target_x/iter;
    x_rel+=x_incr;
    double y_rel=s(x_rel);
    
    double rot_x=x_rel*cos(ref_yaw)-y_rel*sin(ref_yaw);
    double rot_y=x_rel*sin(ref_yaw)+y_rel*cos(ref_yaw);
    double shift_x=rot_x+ref_x;
    double shift_y=rot_y+ref_y;
    traj.x.push_back(shift_x);
    traj.y.push_back(shift_y);
    //std::cout<<"x:"<<shift_x<<", y:"<<shift_y<<std::endl;
  }
  //std::cout<<"END TRAJECTORY"<<std::endl;
  
  return traj;
}
curve generate_trajectory(string state, worldData waypoints, carData ego) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  curve trajectory;
  if (state.compare("KL") == 0) {
    //std::cout<<"s: "<<ego.end_d<<", Lane no.:"<<findLane(ego.end_d)<<std::endl;
    trajectory = generate_spline(waypoints, ego, findLane(ego.end_d), false, false);
  } else if (state.compare("LCL") == 0) {
    trajectory = generate_spline(waypoints, ego, findLane(ego.end_d)-1, false, true);
  } else if (state.compare("LCR") == 0) {
    trajectory = generate_spline(waypoints, ego, findLane(ego.end_d)+1, false, true);
  } else if (state.compare("SLOW") == 0) {
    trajectory = generate_spline(waypoints, ego, findLane(ego.end_d), true, false);
  }
  return trajectory;
}
bool checkCar(vector<curve> predictions, carData ego, string side){
  bool CarPresent=false;
  double ego_s=ego.end_s;
  double ego_d=ego.end_d;
  //std::cout<<"EGO DATA- s:"<<ego_s<<", d:"<<ego_d<<std::endl;
  for (int i=0; i<predictions.size();i++){
    double car_s=predictions[i].s.back();
    double car_d=predictions[i].d.back();
    //std::cout<<"OTHER CAR DATA- s:"<<car_s<<", d:"<<car_d<<std::endl;
    int car_lane=findLane(car_d);
    int ego_lane=findLane(ego_d);
    if (side.compare("Front")==0 && car_lane==ego_lane && (-ego.MIN_VERT_DIST/5)<(car_s-ego_s) && (car_s-ego_s)<(1.0*ego.MIN_VERT_DIST)){
      CarPresent=true;
    }
    if (side.compare("Left")==0 && (ego_lane-car_lane)==1 && (-ego.MIN_VERT_DIST/5)<(car_s-ego_s) && (car_s-ego_s)<(1.25*ego.MIN_VERT_DIST)){
      CarPresent=true;
    }
    if (side.compare("Right")==0 && (car_lane-ego_lane)==1 && (-ego.MIN_VERT_DIST/5)<(car_s-ego_s) && (car_s-ego_s)<(1.25*ego.MIN_VERT_DIST)){
      CarPresent=true;
    }
  }
  return CarPresent;
}
vector<string> select_states(vector<curve> predictions, carData ego) {
  vector<string> states;
  //If car in front, check on sides for lane change. Slow if no option.
  if (checkCar(predictions,ego,"Front")){
    //std::cout<<"CAR IN FRONT!!"<<std::endl;
    if (!(checkCar(predictions,ego,"Left")) && (findLane(ego.end_d)!=0)){
      states.push_back("LCL");
    }else if (!(checkCar(predictions,ego,"Right")) && (findLane(ego.end_d)!=2)){
      states.push_back("LCR");
    }else{
      states.push_back("SLOW");
    }
  } else {
    states.push_back("KL");
  }
  std::cout<<"STATE SELECTED: "<<states[0]<<std::endl;

  //If another car is about to collide. Emergency brake.
  //states.push_back("SOS")
  return states;
}
vector<curve> PredictionModule(carData ego){
  //Assumes Keep lane motion by other cars
  vector<curve> predictions;

  //std::cout<<"EGO DATA- x:"<<ego.x<<", y:"<<ego.y<<", speed:"<<ego.speed<<", s:"<<ego.s<<", d:"<<ego.d<<std::endl;
  for (int i=0;i<ego.sensor_fusion.size();i++){
    double car_x=ego.sensor_fusion[i][1];
    double car_y=ego.sensor_fusion[i][2];
    double car_vx=ego.sensor_fusion[i][3];
    double car_vy=ego.sensor_fusion[i][4];
    double car_v=sqrt(car_vx*car_vx+car_vy*car_vy);
    double car_s=ego.sensor_fusion[i][5];
    double car_d=ego.sensor_fusion[i][6];
    //std::cout<<"SENSOR DATA- x:"<<car_x<<", y:"<<car_y<<", vx:"<<car_vx<<", vy:"<<car_vy<<", s:"<<car_s<<", d:"<<car_d<<std::endl;
    
    //Calculate 10 point long trajectory for next 1s
    //Assuming constant velocity motion
    curve Pred_i;
    Pred_i.x.push_back(car_x);
    Pred_i.y.push_back(car_y);
    Pred_i.s.push_back(car_s);
    for (double t=.1;t<=1;t+=.1){
      double x_n=car_x+car_vx*t;
      double y_n=car_y+car_vy*t;
      double s_n=car_s+car_v*t;
      Pred_i.x.push_back(x_n);
      Pred_i.y.push_back(y_n);
      Pred_i.s.push_back(s_n);
      Pred_i.d.push_back(car_d);
    }
    predictions.push_back(Pred_i);
  }
  return predictions;
}
curve PathPlanner(worldData waypoints, carData ego){
  if (ego.previous_x.size()==0){
    //Initialize s,d
    vector<double> sd=getFrenet(ego.x,ego.y,ego.yaw,waypoints.x,waypoints.y);
    ego.end_s=sd[0];
    ego.end_d=sd[1];
  } else if (ego.previous_x.size()>45){
    curve old_Traj;
    old_Traj.x=ego.previous_x;
    old_Traj.y=ego.previous_y;
    return old_Traj;
  }
  vector<curve> predictions = PredictionModule(ego);
  vector<string> possible_successor_states = select_states(predictions, ego);
  return generate_trajectory(possible_successor_states[0], waypoints, ego);
}

#endif
/** simple_ft_calibration.cpp 

Quick and dirty calibration of the Sunrise F/T sensor
using the MaulLogic digital scales.

We subscribe to joint_states, sunrise/raw_data,
and maul_logic/wrench (weight at Fz).
We open and write a file "calibration.txt" with
sampled data, where each sample has the following
entries (=columns in the file):
timestamp 
sunrise/rawdata: fx fy fz tx ty tz 
maul: weight
tool-NOA: x y z r p Y
vector-projection of NOA along world-z: N*ez O*ez A*ez

We move the robot to the "almost-candle" position,
and from there move the wrist so that the tool points
exactly upwards (Fx,Fy=0,Fz=tool-weight,Tx=Ty=Tz=0),
then exactly dAownwards (Fx=Fy=0,Fz=-tool-weight, Tx=Ty=Tz=0).
In each position, we sample forces for some seconds,
then average the raw Fx,Fy,Fz,Tx,Ty,Tz values and
calculate the zero-offsets.

To calibrate forces, we move the robot to a position
above the maul-logic digital scales.
With the tool pointing down, we move slowly downwards
and record Sunrise Fz and the Maul Fz.
Linear regression then gives gain-Fz.
With the tool pointing sideways, we repeat this process,
which gives gain-Fy, gain-Fz, and gain-Tx, gain-Ty.

KEEP THE SAFETY SWITCH IN RANGE WHEN RUNNING THIS DEMO. 

2016.08.18 - new class

(C) 2016, fnh, hendrich@informatik.uni-hamburg.de
*/


#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Trigger.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

/**
 * Ensure that the workspace is clear before running this demo
 * together with pa10_reflexxes. Ensure that the robot safety
 * switch is in range!
 */
class SimpleFtCalibration {

  public:
    SimpleFtCalibration();            // constructor
    void generate_start_goal();           // hardcoded start position
    void run();                           // endless loop 
    void jointsUpdatedCallback( sensor_msgs::JointState jointState );
    void ftRawUpdatedCallback( geometry_msgs::WrenchStamped rawData );
    void maulLogicUpdatedCallback( geometry_msgs::WrenchStamped wrench );
    void createDataFile();
    void closeDataFile();
    void setEnabled( bool b );


  private:
    bool toggleCallback( std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res );

    ros::NodeHandle nh;
    ros::Subscriber jointStatesSubscriber;
    ros::Subscriber ftRawSubscriber;
    ros::Subscriber wrenchSubscriber;
    ros::Subscriber maulLogicSubscriber;
    ros::ServiceServer toggleServer;

    tf::TransformListener *tfl;
    tf::TransformBroadcaster *tbr;

    std::string rootFrame;  // usually "world"
    std::string toolFrame;
    std::string dataFilename;
    std::string rawWrenchTopic;
    std::string jointStatesTopic;
    FILE * dataFile;

    double commandRate;

    std::string tf_prefix;
    std::map<std::string,int> jointIndexMap;    // joint name -> array index
    std::vector<std::string> jointNames;       // pa10_s1_rotate , ...
    std::vector<double> jointAngles;           // current robot joint angles

    geometry_msgs::WrenchStamped ftRawData;
    geometry_msgs::WrenchStamped maulData;

    unsigned long n_joint_state_callbacks;
    unsigned long n_wrench_callbacks;
    unsigned long n_maul_callbacks;
    unsigned int n_files;

    bool traceEnabled;
};



SimpleFtCalibration::SimpleFtCalibration() : traceEnabled(false) {
  double seed = ros::Time::now().toSec();

  ros::NodeHandle nnh( "~" );
  nnh.param( "rate", commandRate, 100.0 );
  nnh.param( "filename", dataFilename, std::string( "calibration" )); 
  nnh.param( "tf_prefix", tf_prefix, std::string( "ur5_" ) ); // "" or "left_arm/" or ...
  nnh.param( "root_frame", rootFrame, std::string( "world" ));
  nnh.param( "tool_frame", toolFrame, std::string( "s_model_tool0" ));
  nnh.param( "joint_names", jointNames, {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});
  nnh.param( "raw_wrench_topic", rawWrenchTopic, std::string("robotiq_force_torque_wrench"));
  nnh.param( "joint_states_topic", jointStatesTopic, std::string("joint_states"));

  ROS_INFO( "output filename is %s", dataFilename.c_str() );
  ROS_INFO( "tf_prefix is %s", tf_prefix.c_str() );
  ROS_INFO( "root frame is %s", rootFrame.c_str() );
  ROS_INFO( "tool frame is %s", toolFrame.c_str() );

  n_joint_state_callbacks = 0;
  n_wrench_callbacks = 0;
  n_maul_callbacks = 0;
  n_files = 0;

  for( auto& name : jointNames)
  {
    name = tf_prefix + name;
  }

  for( unsigned int i=0; i< jointNames.size(); i++ ) {
    jointIndexMap[jointNames[i]] = i;
  }

  jointAngles.resize( 6 );

  jointStatesSubscriber = nh.subscribe<sensor_msgs::JointState>(
                              "joint_states", 1,
                              &SimpleFtCalibration::jointsUpdatedCallback, 
                              this );

  ftRawSubscriber = nh.subscribe<geometry_msgs::WrenchStamped>(
                              rawWrenchTopic, 1,
                              &SimpleFtCalibration::ftRawUpdatedCallback,
                              this );

  maulLogicSubscriber = nh.subscribe<geometry_msgs::WrenchStamped>(
                              "weight", 1, 
                              &SimpleFtCalibration::maulLogicUpdatedCallback,
                              this );

  toggleServer = nh.advertiseService("toggle_ft_calibration_procedure", &SimpleFtCalibration::toggleCallback, this);

  try {
    tfl = new tf::TransformListener( nh, ros::Duration(10) );
    ROS_INFO( "got a transform listener." );
  }
  catch( tf::TransformException& exception ) {
    ROS_ERROR( "TransformListener failed: %s", exception.what() );
    exit( 1 );
  }
} // end constructor


void SimpleFtCalibration::setEnabled( bool b ) {
  if( b )
  {
    closeDataFile();
    createDataFile();
  }
  traceEnabled = b;
  ROS_ERROR( "SimpleFtCalibration: setEnabled called with bool %d", b );
}


void SimpleFtCalibration::ftRawUpdatedCallback( const geometry_msgs::WrenchStamped rawData ) {
  n_wrench_callbacks++;
 
  ftRawData = rawData; 

  // ROS_INFO( "SimpleFtCalibration.sunriseRawUpdatedCallback..." );

  if (traceEnabled) { // calculate and write one line of data output
    tf::StampedTransform trafo;
    try {
      tfl->waitForTransform( rootFrame, toolFrame, ros::Time(0), ros::Duration(1) );
      tfl->lookupTransform( rootFrame, toolFrame, ros::Time(0), trafo ); 
    }
    catch (tf::TransformException& exception ) {
      ROS_ERROR( "TransformListener failed: %s", exception.what() );
      return;
    }

    // timestamp 
    fprintf( dataFile, "%16.4lf", ros::Time::now().toSec() );
    
    // Sunrise f/t raw data: already decoupled but not calibrated: fx fy fz tx ty tz
    fprintf( dataFile, "  %6.2lf %6.2lf %6.2lf  %6.2lf %6.2lf %6.2lf",
             ftRawData.wrench.force.x, // fx
             ftRawData.wrench.force.y, // fy
             ftRawData.wrench.force.z, // fz 
             ftRawData.wrench.torque.x, // tx
             ftRawData.wrench.torque.y, // ty
             ftRawData.wrench.torque.z ); // tz

    // MaulLogic weight: fz  %8.3lf   
    fprintf( dataFile, "  %8.4lf", maulData.wrench.force.z );

    // manipulator orientation NOA transform, (x y z) (r p Y)
    double roll, pitch, yaw;
    trafo.getBasis().getEulerYPR( yaw, pitch, roll );
    fprintf( dataFile, " %6.3lf %6.3lf %6.3lf %7.4lf %7.4lf %7.4lf",
             trafo.getOrigin().x(), trafo.getOrigin().y(), trafo.getOrigin().z(),
             roll, pitch, yaw );

    // vector projection of N O A axes along gravity vector, ez=(0, 0, -1)
    tf::Vector3 ez( 0, 0, -1 );
    double nz = trafo.getBasis().getColumn( 0 ).dot( ez );
    double oz = trafo.getBasis().getColumn( 1 ).dot( ez );
    double az = trafo.getBasis().getColumn( 2 ).dot( ez );
    fprintf( dataFile, " %9.3lf %9.3lf %9.3lf", nz, oz, az );
    fprintf( dataFile, "\n" );
  }
}


void SimpleFtCalibration::maulLogicUpdatedCallback( const geometry_msgs::WrenchStamped wrenchMsg ) {
  n_maul_callbacks++;

  maulData = wrenchMsg;
}


void SimpleFtCalibration::jointsUpdatedCallback( const sensor_msgs::JointState jointState ) {
  n_joint_state_callbacks++;

  unsigned int matched_joint_names = 0;
  for( unsigned int i=0; i < jointState.name.size(); i++ ) {
    // printf( "jUC: %d %s\n", i, jointState.name[i].c_str() );

    unsigned int found = jointIndexMap.count( jointState.name[i] );
    if (found > 0) {
      matched_joint_names++;
      unsigned int jointIndex = jointIndexMap[ jointState.name[i] ];
      jointAngles[jointIndex] = jointState.position[i];
    }
  }
  if (matched_joint_names != 6) {
    ROS_INFO( "jointsUpdatedCallback: invalid/incomplete jointState, matched %d joints",
              matched_joint_names );
  }
}


double get_clamped_random( double delta ) {
  double r0 = (2.0 * delta * rand()) / RAND_MAX; // [0, RAND_MAX) -> [0,2*delta)
  return r0 - delta; // [-delta,delta)
}


void SimpleFtCalibration::createDataFile() {
  ++n_files;
  std::string newName = dataFilename + "_" + std::to_string(n_files);
  if ((dataFile = fopen( newName.c_str(), "w" )) == NULL) {
    ROS_ERROR( "Could not open the data trace file %s.\n", dataFilename.c_str() );
    exit( 1 );
  }
  fprintf( dataFile, "# timestamp   raw-FX   raw-Fy   raw-FZ   raw-TX   raw-Ty   raw-TZ    maul   noa.x noa.y noa.z noa.r noa.p noa.Y  N*ez O*ez A*ez\n" );
}


void SimpleFtCalibration::closeDataFile() {
  if (dataFile != NULL) {
    fflush( dataFile );
    fclose( dataFile );
    ROS_INFO( "SimpleFtCalibration: output file '%s' closed.", dataFilename.c_str() );
  }
}


void SimpleFtCalibration::run() {
  ros::Rate loop_rate( commandRate );
  ROS_INFO( "loop rate is  %lf", commandRate );

  // waiting until joint_states is published...
  unsigned int secs = 0;
  boost::shared_ptr<sensor_msgs::JointState const> msgPtr;
  while( ros::ok() ) {
    msgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>( jointStatesTopic, ros::Duration(1) );
    if (msgPtr != NULL) { ROS_INFO( "... /joint_states ok." ); break; }
    secs++;
    ROS_INFO( "... waiting for /joint_states (%u secs)...", secs );
  }

  // waiting until wrench is published...
  secs = 0;
  boost::shared_ptr<geometry_msgs::WrenchStamped const> wrPtr;
  while( ros::ok() ) {
    wrPtr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>( rawWrenchTopic, ros::Duration(1) );
    if (wrPtr != NULL) { ROS_INFO( "... /raw_data ok." ); break; }
    secs++;
    ROS_INFO( "... waiting for /raw_data(%u secs)...", secs );
  }

  // no need to wait for /maul_logic/wrench

  // main loop
  ros::Rate rate( 100 );

  while( ros::ok() ) {
    ros::spinOnce();
    rate.sleep();
  }
}

bool SimpleFtCalibration::toggleCallback( std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res )
{
  setEnabled(!traceEnabled);
  res.success = true;
  return true;
}

/*
 * cntl-c signal handler: close the dataFile
 * before we exit.
 */
SimpleFtCalibration * sfc_ptr;


void SIGINT_handler( int signal )
{
  ROS_WARN( "SimpleFtCalibration: received cntl-c (SIGINT), shutting down..." );

  sfc_ptr->closeDataFile();
  exit( 0 );
}


int main( int argc, char** argv ) {
  ros::init( argc, argv, "calibration_logging_node", 1 ); // 1=no NoSigintHandler

  SimpleFtCalibration simpleFtCalibration;
  sfc_ptr = &simpleFtCalibration;
  simpleFtCalibration.run();
  exit( EXIT_SUCCESS );
}


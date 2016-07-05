#include<cstdio>
#include<ctime>
#include<cmath> 

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>

#define PI 3.1415926535

FILE* f;

/*Configuration arrays consist of:
	[0]-base point coordinates;
	[1]-link lengths
	[2]-joint angles
*/

std::vector<std::vector<float> > start_configuration(3, std::vector<float> (3));
std::vector<float> destination(3);
std::vector<float> joint_positions(3), joint_efforts(3);

void transformVector(std::vector<float>& vect, float angle);

//Unit tests
int unitTestsBody();
int getManipulatorConfiguration_test(std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration);
int updateManipulator_test (std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration);
int manipulatorLengths_test (std::vector<std::vector<float> > out_points, std::vector<std::vector<float> > out_configuration);
//int getPoint_test();


class Manipulator {
	private:
		std::vector<std::vector<float> > configuration;
		std::vector<std::vector<float> > points;		
	public:		
		Manipulator( std::vector<std::vector<float> > manp_configuration ) : configuration(3, std::vector<float> (3)), points(3, std::vector<float> (3)) {
			setManipulator( manp_configuration );		
		}
		//Working as planned
		void setManipulator( std::vector<std::vector<float> > manp_configuration );
		//Working as planned
		void getManipulatorPoints( std::vector<std::vector<float> >& out_points );
		//Working as planned
		void getManipulatorConfiguration( std::vector<std::vector<float> >& out_configuration );
		//Working as planned		
		void updateManipulator();
		//Working as planned
		void getRotAngle(std::vector<float> dest, float& rot_angle );
		//Working as planned
		void transformManipulator(float angle);
		//
		int getPoint ( float link_length, std::vector<float> stationary, std::vector<float> reference, std::vector<float>& moving );
		//
		int FABRIK( int iter, std::vector<float> dest );
		bool checkDestination ( std::vector<float> joint_positions );
		//
		void controlSynth (std::vector<float> pv_joint_positions, std::vector<float> joint_positions, std::vector<float>& control );
};

//Working as planned
int initialize_globals(){  
	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;

	start_configuration[1][0] = 0;
	start_configuration[1][1] = 4;
	start_configuration[1][2] = 4;

	start_configuration[2][0] = 0;
	start_configuration[2][1] = 0;
	start_configuration[2][2] = 1;

	destination[0] = 2;
	destination[1] = 1;
	destination[2] = 6;
}

//Working as planned
void joints_callback( const sensor_msgs::JointState::ConstPtr& inp_msg ) {
	std::vector<std::string>::const_iterator it1;

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"world_to_base");
	if (it1==inp_msg->name.end()) {
		fprintf(stderr, "world_to_base not found \n");
	}
	else {
		joint_positions[0] = inp_msg->position[it1 - inp_msg->name.begin()];
                
		joint_efforts[0] = inp_msg->effort[it1 - inp_msg->name.begin()];
	}

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"link1_to_link2");
	if (it1==inp_msg->name.end()) {
		fprintf(stderr, "link1_to_link2 not found \n");
	}
	else {
		joint_positions[1] = inp_msg->position[it1 - inp_msg->name.begin()];
                joint_efforts[1] = inp_msg->effort[it1 - inp_msg->name.begin()];
	}

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"link2_to_link3");
	if (it1==inp_msg->name.end()) {
		fprintf(stderr, "link2_to_link3 not found \n");
	}
	else {
		joint_positions[2] = inp_msg->position[it1 - inp_msg->name.begin()];
                joint_efforts[2] = inp_msg->effort[it1 - inp_msg->name.begin()];
	}
}

//----------------------------------------------------------
//-----------------MAIN FUNCTION----------------------------
//----------------------------------------------------------

int main( int argc, char **argv ) {
	std::vector<float> pv_joint_positions(3);
	std::vector<float> control(3);

	unitTestsBody();
	//getPoint_test();

	f = fopen("/home/pal/catkin_ws/logfile.csv", "wt");
	if (f == 0) {
		fprintf(stderr, "File not found \n");
	}
	
	initialize_globals();	
	
	ros::Publisher wtb_pub, l1tl2_pub, l2tl3_pub;

	ros::init(argc, argv, "manp_controller");

	ros::NodeHandle n;
	wtb_pub = n.advertise<std_msgs::Float64>("/manp/world_to_base_state_controller/command", 100);
	l1tl2_pub = n.advertise<std_msgs::Float64>("/manp/link1_to_link2_state_controller/command", 100);
	l2tl3_pub = n.advertise<std_msgs::Float64>("/manp/link2_to_link3_state_controller/command", 100);

	ros::Subscriber sub_coordinates = n.subscribe("/manp/joint_states", 100, joints_callback);
	ros::Rate r(100);
	ros::spinOnce();
	
	Manipulator model(start_configuration);
	pv_joint_positions = joint_positions;

	model.FABRIK(10, destination);
	r.sleep();
	while(ros::ok() & model.checkDestination(joint_positions) == false ) {
		
		ros::spinOnce();

		model.controlSynth(pv_joint_positions, joint_positions, control);		

		std_msgs::Float64 out_msg;
		out_msg.data = control[0];
		wtb_pub.publish(out_msg);

		out_msg.data = control[1];
		l1tl2_pub.publish(out_msg);

		out_msg.data = control[2];
		l2tl3_pub.publish(out_msg);

		pv_joint_positions = joint_positions;
    		
    		r.sleep();
  	}
	fclose(f);
  	return 0;
}

//----------------------------------------------------------
//-----------------Defining functions-----------------------
//----------------------------------------------------------

void Manipulator::setManipulator( std::vector<std::vector<float> > manp_configuration ) {

	configuration = manp_configuration;

	points[0] = configuration[0]; 
	
	points[1][0] = points[0][0] + configuration[1][1]*cos(configuration[2][0])*sin(configuration[2][1]);
	points[1][1] = points[0][1] + configuration[1][1]*sin(configuration[2][0])*sin(configuration[2][1]);
	points[1][2] = points[0][2] + configuration[1][1]*cos(configuration[2][1]);

	points[2][0] = points[1][0] + configuration[1][2]*cos(configuration[2][0])*sin(configuration[2][2]+configuration[2][1]);
	points[2][1] = points[1][1] + configuration[1][2]*sin(configuration[2][0])*sin(configuration[2][2]+configuration[2][1]);
	points[2][2] = points[1][2] + configuration[1][2]*cos(configuration[2][2]+configuration[2][1]);
}

void Manipulator::getManipulatorPoints( std::vector<std::vector<float> >& out_points ) {
	out_points = points;
}

void Manipulator::getManipulatorConfiguration( std::vector<std::vector<float> >& out_configuration ) {
	out_configuration = configuration;
}

void Manipulator::updateManipulator() {
	int k;

	configuration[0] = points[0];

	//Setting world_to_base angle
	if (points[2][1] == 0 & points[2][0] == 0) {
		configuration[2][0] = 0;
	}
	else {
		configuration[2][0] = atan2((points[2][1] - points[0][1]),(points[2][0] - points[0][0]));
	}
	
	//Setting link1_to_link2 angle

	if ( configuration[2][0] > 0 ) {

		if ( (points[1][0] > 0 & points[1][1] < 0) || (points[1][0] < 0 & points[1][1] < 0) || (points[1][0] == 0 & points[1][1] < 0) )
			k = -1;
		else k = 1;
	}
	else
	{
		if ( (points[1][0] > 0 & points[1][1] > 0) || (points[1][0] < 0 & points[1][1] > 0) || (points[1][0] == 0 & points[1][1] > 0) )
			k = -1;
		else k = 1;
	}	
	configuration[2][1] = atan2( k*pow(pow(points[1][0]-points[0][0],2) + pow(points[1][1]-points[0][1],2), 0.5), (points[1][2] - points[0][2]));

	//Setting link2_to_link3 angle

	if ( configuration[2][0] > 0 ) {

		if ( (points[2][0] > 0 & points[2][1] < 0) || (points[2][0] < 0 & points[2][1] < 0) || (points[2][0] == 0 & points[2][1] < 0) )
			k = -1;
		else k = 1;
	}
	else
	{
		if ( (points[2][0] > 0 & points[2][1] > 0) || (points[2][0] < 0 & points[2][1] > 0) || (points[2][0] == 0 & points[2][1] > 0) )
			k = -1;
		else k = 1;
	}
	configuration[2][2] = atan2( k*pow(pow(points[2][0]-points[1][0],2) + pow(points[2][1]-points[1][1],2), 0.5) ,(points[2][2] - points[1][2])) - configuration[2][1];
}

void Manipulator::getRotAngle(std::vector<float> dest, float& rot_angle) {
	int k;

	if (points[2][0] == 0 & points[2][1] == 0) {
		if ( (dest[1] - dest[0]) >= 0 )
			k = 1;
		else k = -1;
		rot_angle = k*acos( dest[0]/ (pow( pow( dest[0], 2) + pow(dest[1],2), 0.5 ) ) );
	}
	
	else
	{
		if ( (points[2][0]*dest[1] - points[2][1]*dest[0]) >= 0 )
			k = 1;
		else k = -1;

		rot_angle = k*acos( (points[2][0]*dest[0] + points[2][1]*dest[1])/ ( (configuration[1][1]*sin(configuration[2][1]) + configuration[1][2]*sin(configuration[2][2]+configuration[2][1]))*pow( pow(dest[0],2) + pow(dest[1],2), 0.5 )) );
	}

}

void Manipulator::transformManipulator(float angle) {
	
	double tmp = points[0][0]*cos(angle) - points[0][1]*sin(angle);
	points[0][1] = points[0][0]*sin(angle) + points[0][1]*cos(angle);
        points[0][0] = tmp;
	points[0][2] = points[0][2];

	tmp = points[1][0]*cos(angle) - points[1][1]*sin(angle);
	points[1][1] = points[1][0]*sin(angle) + points[1][1]*cos(angle);
        points[1][0] = tmp;
	points[1][2] = points[1][2];

	tmp = points[2][0]*cos(angle) - points[2][1]*sin(angle);
	points[2][1] = points[2][0]*sin(angle) + points[2][1]*cos(angle);
        points[2][0] = tmp;
	points[2][2] = points[2][2];
}


void transformVector(std::vector<float>& vect, float angle) {

	double tmp = vect[0]*cos(angle) + vect[1]*sin(angle);
	vect[1] = -vect[0]*sin(angle) + vect[1]*cos(angle);
        vect[0] = tmp;
	vect[2] = vect[2];
}


int Manipulator::getPoint ( float link_length, std::vector<float> stationary, std::vector<float> reference, std::vector<float>& moving ) {
	std::vector<float> vect(2);
	int i = -1; //Invoking segmentation fault

	if (fabs(points[0][0] - 0) <1e-6 & fabs(points[1][0] - 0) <1e-6 & fabs(points[2][0] - 0) <1e-6)
		i = 1;
	if (fabs(points[0][1] - 0) <1e-6 & fabs(points[1][1] - 0) <1e-6 & fabs(points[2][1] - 0) <1e-6)
		i = 0;
	if (i == -1)
		return -1;


	vect[0] = reference[i] - stationary[i];
	vect[1] = reference[2] - stationary[2];

	float k = pow(pow(vect[0],2) + pow(vect[1],2), 0.5);
	if ( k >= 1e-6 ) {
  		vect[0] /= k;
		vect[1] /= k;
	
		moving[i] = stationary[i] + link_length*vect[0];
		moving[2] = stationary[2] + link_length*vect[1];
        }
	else
	{
        	moving[i] = stationary[i];
        	moving[2] = stationary[2];
        }
	return 0;
}

int Manipulator::FABRIK( int iter, std::vector<float> dest ) {
	std::vector<std::vector<float> > pv_points(3, std::vector<float> (3));
	std::vector<std::vector<float> > out_points(3, std::vector<float> (3));
	std::vector<std::vector<float> > out_configuration(3, std::vector<float> (3));
	float rot_angle;
	int flag;
	
	//Check for the starting position
	getRotAngle( dest, rot_angle );
	fprintf(stderr, "rot_angle %lf\n", rot_angle);

	getManipulatorPoints(out_points);

	//Rotating vectors to destination
	transformManipulator( rot_angle );

	//Transforming to local coordinate system
	transformManipulator( -rot_angle );
	transformVector(dest, rot_angle);
	
	getManipulatorPoints(out_points);
	     
	//----------------ATTENTION-------------------------				
	//SOLUTION NOW IS IN THE LOCAL SYSTEM OF COORDINATES
				
	for ( int i = 0; i < iter/2; i++ ) {
	//Staring reverse pass				
		pv_points = points;

		//Moved point TWO to destination
		points[2] = dest;
			
		//Moved point ONE for the reverse pass
		flag = getPoint( configuration[1][2], points[2], pv_points[1], points[1] );
		if (flag == -1) {
			fprintf(stderr, "getPoint error\n");
			getManipulatorPoints(out_points);
			fprintf(stderr, "points[0] = : %e %e %e \n", points[0][0], points[0][1], points[0][2] );
			fprintf(stderr, "points[1] = : %e %e %e \n", points[1][0], points[1][1], points[1][2] );
			fprintf(stderr, "points[2] = : %e %e %e \n", points[2][0], points[2][1], points[2][2] );
			return -1;
		}
	
		//Moved point ZERO for the reverse pass
		flag = getPoint( configuration[1][1], points[1], pv_points[0], points[0]);
		if (flag == -1) {
			fprintf(stderr, "getPoint error\n");
			getManipulatorPoints(out_points);
			fprintf(stderr, "points[0] = : %e %e %e \n", points[0][0], points[0][1], points[0][2] );
			fprintf(stderr, "points[1] = : %e %e %e \n", points[1][0], points[1][1], points[1][2] );
			fprintf(stderr, "points[2] = : %e %e %e \n", points[2][0], points[2][1], points[2][2] );
			return -1;
		}
	
	//Starting direct pass
		pv_points[1] = points[1];			
		pv_points[2] = points[2];
				
		//Moved point points[0] to previous position
		points[0] = pv_points[0];
				
		//Moved point points[1] for the direct pass
		flag = getPoint( configuration[1][1], points[0], pv_points[1], points[1] );
		if (flag == -1) {
			fprintf(stderr, "getPoint error\n");
			getManipulatorPoints(out_points);
			fprintf(stderr, "points[0] = : %e %e %e \n", points[0][0], points[0][1], points[0][2] );
			fprintf(stderr, "points[1] = : %e %e %e \n", points[1][0], points[1][1], points[1][2] );
			fprintf(stderr, "points[2] = : %e %e %e \n", points[2][0], points[2][1], points[2][2] );
			return -1;
		}
				
		//Moved point points[2] for the direct pass
		flag = getPoint( configuration[1][2], points[1], pv_points[2], points[2] );
		if (flag == -1) {
			fprintf(stderr, "getPoint error\n");
			getManipulatorPoints(out_points);
			fprintf(stderr, "points[0] = : %e %e %e \n", points[0][0], points[0][1], points[0][2] );
			fprintf(stderr, "points[1] = : %e %e %e \n", points[1][0], points[1][1], points[1][2] );
			fprintf(stderr, "points[2] = : %e %e %e \n", points[2][0], points[2][1], points[2][2] );
			return -1;
		}
	}

	//Transforming to basic coordinate system
	transformManipulator( rot_angle );

	//----------------ATTENTION-------------------------				
	//SOLUTION NOW IS IN THE GLOBAL SYSTEM OF COORDINATES

	getManipulatorPoints( out_points );
	getManipulatorConfiguration( out_configuration );
	manipulatorLengths_test ( out_points, out_configuration );
	
	updateManipulator();
}

bool Manipulator::checkDestination ( std::vector<float> joint_positions  ) {
	return (joint_positions == configuration[2]);
}

void Manipulator::controlSynth (std::vector<float> pv_joint_positions, std::vector<float> joint_positions, std::vector<float>& control ) {
	std::vector<std::vector<float> > pid(3, std::vector<float>(3));
	std::vector<std::vector<float> > out_configuration(3, std::vector<float>(3));

	pid[0][0] = 0.1;
	pid[0][1] = 0.2;
	pid[0][2] = 0.3;
	
	pid[1][0] = 0;
	pid[1][1] = 0;
	pid[1][2] = 0;

	pid[2][0] = 0;
	pid[2][1] = 0;
	pid[2][2] = 0;
 
	control[0] = pid[0][0]*(configuration[2][0] - joint_positions[0]) + pid[0][1]*(2*configuration[2][0] - pv_joint_positions[0] - joint_positions[0]) + pid[0][2]*(joint_positions[0] - pv_joint_positions[0]);
	control[1] = pid[1][0]*(configuration[2][1] - joint_positions[1]) + pid[1][1]*(2*configuration[2][1] - pv_joint_positions[1] - joint_positions[1]) + pid[1][2]*(joint_positions[1] - pv_joint_positions[1]);
	control[2] = pid[2][0]*(configuration[2][2] - joint_positions[2] - joint_positions[1]);

	getManipulatorConfiguration( out_configuration );

	fprintf(f,"%lf;%lf;%lf;%lf;%lf;%lf\n", out_configuration[2][0], joint_positions[0], out_configuration[2][1], joint_positions[1], out_configuration[2][2], joint_positions[2]);
}

//----------------------------------------------------------
//---------------------Unit tests---------------------------
//----------------------------------------------------------


int unitTestsBody() {
	std::vector<std::vector<float> > start_configuration(3, std::vector<float> (3));
	std::vector<std::vector<float> > out_points(3, std::vector<float> (3));
	std::vector<std::vector<float> > out_configuration(3, std::vector<float> (3));

	std::vector<float> destination(3);
	float rot_angle;
	
	//------------------
	//Constructing test1
	//------------------

	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;	
	
	start_configuration[1][0] = 0;
	start_configuration[1][1] = 1;
	start_configuration[1][2] = 1;	
	
	start_configuration[2][0] = PI/2;
	start_configuration[2][1] = PI/4;
	start_configuration[2][2] = PI/3;
	
	Manipulator test1( start_configuration );

	test1.getManipulatorPoints( out_points );

	//Testing setManipulator&getManipulatorPoints
	if (fabs(out_points[0][0] - 0)  > 1e-6 || fabs(out_points[0][1] - 0) > 1e-6 || fabs(out_points[0][2] - 0) > 1e-6 ) {
		fprintf(stderr, "Wrong points[0]: %e %e %e \n", out_points[0][0], out_points[0][1], out_points[0][2]);
		return -1;
	}
	
	if (fabs(out_points[1][0] - 0) > 1e-6 || fabs(out_points[1][1] - 0.7071067812) > 1e-6 || fabs(out_points[1][2] - 0.7071067812) > 1e-6 ) {
		fprintf(stderr, "Wrong points[1]: %e %e %e \n", out_points[1][0], out_points[1][1], out_points[1][2]);
		return -1;
	}
	
	if (fabs(out_points[2][0] - 0)  > 1e-6 || fabs(out_points[2][1] - 1.6730326075) > 1e-6 || fabs(out_points[2][2] - 0.4482877361) > 1e-6 ) {
		fprintf(stderr, "Wrong points[2]: %e %e %e \n", out_points[2][0], out_points[2][1], out_points[2][2]);
		return -1;
	}

	//Testing getManipulatorConfiguration
	test1.getManipulatorConfiguration( out_configuration );	
	if( getManipulatorConfiguration_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing updateManipulator
	
	test1.updateManipulator();
	test1.getManipulatorConfiguration( out_configuration );
	if( updateManipulator_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing getRotAngle
	
	destination[0] = 3;
	destination[1] = 0;
	destination[2] = 13;

	test1.getRotAngle( destination, rot_angle);
	
	if (fabs(rot_angle + PI/2)  > 1e-6 ) {
		fprintf(stderr, "Wrong rot_angle: %e \n", rot_angle );
		return -1;
	}

	//Testing transform
	test1.transformManipulator(rot_angle);
	test1.getManipulatorPoints(out_points);
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;
	transformVector(destination, rot_angle);
	test1.transformManipulator(rot_angle);
	test1.getManipulatorPoints(out_points);	
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;	

	if ( (out_points[0][0] != 0 & out_points[1][0] != 0 & out_points[2][0] != 0) & (out_points[0][1] != 0 & out_points[1][1] != 0 & out_points[2][1] != 0) ) {
		fprintf(stderr, "Wrong rotation. Points x's %e %e %e \n", out_points[0][0], out_points[1][0], out_points[2][0]);
		fprintf(stderr, "Points y's %e %e %e \n", out_points[0][1], out_points[1][1], out_points[2][1]);
		return -1;
	}
		
	fprintf(stderr, "Test1 passed the test \n");
	
	//------------------
	//Constructing test2
	//------------------

	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;	
	
	start_configuration[1][0] = 2;
	start_configuration[1][1] = 3;
	start_configuration[1][2] = 4;	
	
	start_configuration[2][0] = PI/2;
	start_configuration[2][1] = PI/4;
	start_configuration[2][2] = PI/3;
	
	Manipulator test2( start_configuration );

	test2.getManipulatorPoints( out_points );

	//Testing setManipulator&getManipulatorPoints
	if (fabs(out_points[0][0] - 0)  > 1e-6 || fabs(out_points[0][1] - 0) > 1e-6 || fabs(out_points[0][2] - 0) > 1e-6 ) {
		fprintf(stderr, "Wrong points[0]: %e %e %e \n", out_points[0][0], out_points[0][1], out_points[0][2]);
		return -1;
	}
	
	if (fabs(out_points[1][0] - 0) > 1e-6 || fabs(out_points[1][1] - 2.1213203436) > 1e-6 || fabs(out_points[1][2] - 2.1213203436) > 1e-6 ) {
		fprintf(stderr, "Wrong points[1]: %e %e %e \n", out_points[1][0], out_points[1][1], out_points[1][2]);
		return -1;
	}
	
	if (fabs(out_points[2][0] - 0)  > 1e-6 || fabs(out_points[2][1] - 5.9850236487) > 1e-6 || fabs(out_points[2][2] - 1.0860441631) > 1e-6 ) {
		fprintf(stderr, "Wrong points[2]: %e %e %e \n", out_points[2][0], out_points[2][1], out_points[2][2]);
		return -1;
	}


	//Testing getManipulatorConfiguration
	test2.getManipulatorConfiguration( out_configuration );	
	if( getManipulatorConfiguration_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing updateManipulator
	test2.updateManipulator();
	test2.getManipulatorConfiguration( out_configuration );
	if( updateManipulator_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing getRotAngle
	
	destination[0] = 4;
	destination[1] = 4;
	destination[2] = 0;

	test2.getRotAngle( destination, rot_angle);
	
	if (fabs(rot_angle + PI/4)  > 1e-6 ) {
		fprintf(stderr, "Wrong rot_angle: %e \n", rot_angle );
		return -1;
	}

	//Testing transform
	test2.transformManipulator(rot_angle);
	test2.getManipulatorPoints(out_points);
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;
	transformVector(destination, rot_angle);
	test2.transformManipulator(rot_angle);
	test2.getManipulatorPoints(out_points);	
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;	

	if ( (out_points[0][0] != 0 & out_points[1][0] != 0 & out_points[2][0] != 0) & (out_points[0][1] != 0 & out_points[1][1] != 0 & out_points[2][1] != 0) ) {
		fprintf(stderr, "Wrong rotation. Points x's %e %e %e \n", out_points[0][0], out_points[1][0], out_points[2][0]);
		fprintf(stderr, "Points y's %e %e %e \n", out_points[0][1], out_points[1][1], out_points[2][1]);
		return -1;
	}
	
	fprintf(stderr, "Test2 passed the test \n");
	
	//------------------
	//Constructing test3
	//------------------	

	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;	
	
	start_configuration[1][0] = 0;
	start_configuration[1][1] = 1;
	start_configuration[1][2] = 1;	
	
	start_configuration[2][0] = 0;
	start_configuration[2][1] = 0;
	start_configuration[2][2] = 5*PI/6;
	
	Manipulator test3( start_configuration );

	test3.getManipulatorPoints( out_points );

	//Testing setManipulator&getManipulatorPoints
	if (fabs(out_points[0][0] - 0)  > 1e-6 || fabs(out_points[0][1] - 0) > 1e-6 || fabs(out_points[0][2] - 0) > 1e-6 ) {
		fprintf(stderr, "Wrong points[0]: %e %e %e \n", out_points[0][0], out_points[0][1], out_points[0][2]);
		return -1;
	}
	
	if (fabs(out_points[1][0] - 0) > 1e-6 || fabs(out_points[1][1] - 0) > 1e-6 || fabs(out_points[1][2] - 1) > 1e-6 ) {
		fprintf(stderr, "Wrong points[1]: %e %e %e \n", out_points[1][0], out_points[1][1], out_points[1][2]);
		return -1;
	}
	
	if (fabs(out_points[2][0] - 0.5)  > 1e-6 || fabs(out_points[2][1] - 0) > 1e-6 || fabs(out_points[2][2] - 0.1339745962) > 1e-6 ) {
		fprintf(stderr, "Wrong points[2]: %e %e %e \n", out_points[2][0], out_points[2][1], out_points[2][2]);
		return -1;
	}

	//Testing getManipulatorConfiguration
	test3.getManipulatorConfiguration( out_configuration );	
	if( getManipulatorConfiguration_test(out_configuration, start_configuration) == -1)
		return -1;
	
	//Testing updateManipulator
	test3.updateManipulator();
	test3.getManipulatorConfiguration( out_configuration );
	if( updateManipulator_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing getRotAngle
	
	destination[0] = 1;
	destination[1] = 1;
	destination[2] = 6;

	test3.getRotAngle( destination, rot_angle);
	
	if (fabs(rot_angle - PI/4)  > 1e-6 ) {
		fprintf(stderr, "Wrong rot_angle: %e \n", rot_angle );
		return -1;
	}

	//Testing transform
	test3.transformManipulator(rot_angle);
	test3.getManipulatorPoints(out_points);
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;
	transformVector(destination, rot_angle);
	test3.transformManipulator(rot_angle);
	test3.getManipulatorPoints(out_points);	
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;	

	if ( (out_points[0][0] != 0 & out_points[1][0] != 0 & out_points[2][0] != 0) & (out_points[0][1] != 0 & out_points[1][1] != 0 & out_points[2][1] != 0) ) {
		fprintf(stderr, "Wrong rotation. Points x's %e %e %e \n", out_points[0][0], out_points[1][0], out_points[2][0]);
		fprintf(stderr, "Points y's %e %e %e \n", out_points[0][1], out_points[1][1], out_points[2][1]);
		return -1;
	}

	fprintf(stderr, "Test3 passed the test \n");

	return 0;
}

int getManipulatorConfiguration_test(std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration) {

	if (fabs(out_configuration[0][0] - start_configuration[0][0]) > 1e-6 || fabs(out_configuration[0][1] - start_configuration[0][1]) > 1e-6 || fabs(out_configuration[0][2] - start_configuration[0][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[0]: %e %e %e \n", out_configuration[0][0], out_configuration[0][1], out_configuration[0][2]);
		return -1;
	}
	
	if (fabs(out_configuration[1][0] - start_configuration[1][0]) > 1e-6 || fabs(out_configuration[1][1] - start_configuration[1][1]) > 1e-6 || fabs(out_configuration[1][2] - start_configuration[1][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[1]: %e %e %e \n", out_configuration[1][0], out_configuration[1][1], out_configuration[1][2]);
		return -1;
	}	

	if (fabs(out_configuration[2][0] - start_configuration[2][0]) > 1e-6 || fabs(out_configuration[2][1] - start_configuration[2][1])  > 1e-6 || fabs(out_configuration[2][2] - start_configuration[2][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2]: %e %e %e \n", out_configuration[2][0], out_configuration[2][1], out_configuration[2][2]);	
		return -1;
	}

}

int updateManipulator_test (std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration) { 

	if (fabs(out_configuration[2][0] - start_configuration[2][0]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2][0]: %e \n", out_configuration[2][0]);
		return -1;
	}
	
	if (fabs(out_configuration[2][1] - start_configuration[2][1]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2][1]: %e \n", out_configuration[2][1]);
		return -1;
	}	

	if (fabs(out_configuration[2][2] - start_configuration[2][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2][2]: %e \n", out_configuration[2][2]);
		return -1;
	}
}

int manipulatorLengths_test (std::vector<std::vector<float> > out_points, std::vector<std::vector<float> > out_configuration) {
	float l1, l2;
	
	l1 = pow( pow( out_points[1][0] - out_points[0][0] ,2) + pow( out_points[1][1] - out_points[0][1] ,2) + pow( out_points[1][2] - out_points[0][2] ,2) , 0.5 );
	l2 = pow( pow( out_points[2][0] - out_points[1][0] ,2) + pow( out_points[2][1] - out_points[1][1] ,2) + pow( out_points[2][2] - out_points[1][2] ,2) , 0.5 );

	if (fabs(l1 - out_configuration[1][1])>1e-3 || fabs(l2- out_configuration[1][2])>1e-3 )
		fprintf(stderr, "links length error %e %e \n",l1, l2);

}

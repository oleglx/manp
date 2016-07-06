#include <vector>
#include <algorithm>
#include <cmath>
#include <cv.h>
#include <highgui.h>

#define w 800
using namespace cv;
using std::vector;

void getVector (Point2f stationary, Point2f moving, Point2f& vect);
void normalVector (Point2f& vect);
void getPoint ( float link_length, Point2f stationary, Point2f moving, Point2f& out );
void printImg(char* graph_window, Mat graph_image, Point2f zero, Point2f one, Point2f two, Point2f pv_zero, Point2f pv_one, Point2f pv_two, Point2f base, Point2f dest );

char graph_window[] = "Graph FABRIK";
Mat graph_image = Mat(Size(w,w),CV_8UC3,Scalar(255,255,255));

int getPoints_test();
int getCoordinates_test();

class Manipulator {
	private:
		std::vector<float> base_point, angles, link_lengths;
	public:
		Manipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			setManipulator( manp_base_point, manp_angles, manp_link_lengths );		
		}
		
		void setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			base_point = manp_base_point;
			angles = manp_angles;
			link_lengths = manp_link_lengths;
		}

		void getManipulator( std::vector<float>& manp_base_point, std::vector<float>& manp_angles, std::vector<float>& manp_link_lengths ) {
			manp_base_point = base_point;
			manp_angles = angles;
			manp_link_lengths = link_lengths;
		}

		void getPoints( Point2f& point_zero, Point2f& point_one, Point2f& point_two ) {	
			point_zero.x = base_point[0];
			point_zero.y = base_point[1];

			point_one.x = base_point[0] + link_lengths[0]*cos(angles[0]);
			point_one.y = base_point[1] + link_lengths[0]*sin(angles[0]);
			
			point_two.x = point_one.x + link_lengths[1]*cos(angles[1]);
			point_two.y = point_one.y + link_lengths[1]*sin(angles[1]);
		}

		void getCoordinates( Point2f point_zero, Point2f point_one, Point2f point_two ) {
			base_point[0] = point_zero.x;
			base_point[1] = point_zero.y;

			angles[0] = atan( (point_one.y - point_zero.y)/(point_one.x - point_zero.x) );
			angles[1] = atan( (point_two.y - point_one.y)/(point_two.x - point_one.x) );

			link_lengths[0] = pow(pow((point_one.x - point_zero.x),2) + pow((point_one.y - point_zero.y),2), 0.5);
			link_lengths[1] = pow(pow((point_two.x - point_one.x),2) + pow((point_two.y - point_one.y),2), 0.5);
		}

		// !! TO DO: Replace points with vector of points and place them under another cycle

		// !!! TO DO: Change iteration based calculation to delta based (basically for to while)

		// !!! TO DO: Place all the helping functions in class
		
		void FABRIK( int iter, Point2f zero, Point2f one, Point2f two, Point2f dest ) {
			Point2f zero_pv, one_pv, two_pv;
								
			for ( int i = 0; i < iter/2; i++ ) {
			//Staring reverse pass				
				zero_pv = zero;
				one_pv = one;			
				two_pv = two;

				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );

				//Moved point TWO to destination
				two = dest;

				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );
			
				//Moved point ONE for the reverse pass
				getPoint( link_lengths[1], two, one_pv, one );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );
				
				//Moved point ZERO for the reverse pass
				getPoint( link_lengths[0], one, zero_pv, zero);
				
				//Printing this step
				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );
						
			//Starting direct pass
				one_pv = one;			
				two_pv = two;
				
				//Moved point zero to previous position
				zero = zero_pv;

				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );
				
				//Moved point one for the direct pass
				getPoint( link_lengths[0], zero, one_pv, one );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );
				
				//Moved point two for the direct pass
				getPoint( link_lengths[1], one, two_pv, two );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two, zero_pv, one_pv, two_pv, zero_pv, dest );
			}

		}


};



int main( void ) {

	//--------Testing-----------

        if(getPoints_test()!=0)
        	return -1;

	if(getCoordinates_test()!=0)
		return -1;

	std::vector<float> manp_base_point(2);
	manp_base_point[0] = w/2;
	manp_base_point[1] = w/2;

	std::vector<float> manp_angles(2);
	manp_angles[0] = M_PI/2;
	manp_angles[1] = M_PI/2;

	std::vector<float> manp_link_lengths(2);
	manp_link_lengths[0] = w/5;
	manp_link_lengths[1] = w/5;
	
	Manipulator manp_1( manp_base_point, manp_angles, manp_link_lengths);

	Point2f zero;
	Point2f one;
	Point2f two;
	
	manp_1.getPoints(zero, one, two);

	Point2f dest;
	dest.x = 708;
	dest.y = 514;

	manp_1.FABRIK(  8, zero, one, two, dest );

	imshow(graph_window,graph_image);
	moveWindow(graph_window, 0, 200 );
	waitKey(0);
	return(0);
	
}

//--------------------------------------------------------------------------------------------------

void getVector (Point2f stationary, Point2f moving, Point2f& vect) {	
	vect.x = moving.x - stationary.x;
	vect.y = moving.y - stationary.y;
}

void normalVector (Point2f& vect) {
	float k = pow(pow(vect.x,2) + pow(vect.y,2), 0.5);
	
	vect.x /= k;
	vect.y /= k;
}

void getPoint ( float link_length, Point2f stationary, Point2f moving, Point2f& out ) {
	Point2f vect;

	getVector( stationary, moving, vect );

	normalVector( vect );
	
	out.x = stationary.x + link_length*vect.x;
	out.y = stationary.y + link_length*vect.y;

}

void printImg(char* graph_window, Mat graph_image, Point2f zero, Point2f one, Point2f two, Point2f pv_zero, Point2f pv_one, Point2f pv_two, Point2f base, Point2f dest ) {
	
	graph_image.setTo(Scalar(255,255,255));

	circle( graph_image, base, w/230, Scalar( 0, 255, 0 ), 2, 8 );
	circle( graph_image, dest, w/230, Scalar( 0, 255, 0 ), 2, 8 );

	line( graph_image, pv_zero, pv_one, Scalar( 80, 80, 80 ), 1, 8 );
	line( graph_image, pv_one, pv_two, Scalar( 80, 80, 80 ), 1, 8 );

	circle( graph_image, pv_zero, w/210, Scalar( 100, 100, 100 ), -1, 8 );
	circle( graph_image, pv_one, w/210, Scalar( 100, 100, 100 ), -1, 8 );
	circle( graph_image, pv_two, w/210, Scalar( 100, 100, 100 ), -1, 8 );

	line( graph_image, zero, one, Scalar( 0, 0, 0 ), 2, 8 );
	line( graph_image, one, two, Scalar( 0, 0, 0 ), 2, 8 );

	circle( graph_image, zero, w/200, Scalar( 255, 0, 0 ), -1, 8 );
	circle( graph_image, one, w/200, Scalar( 0, 0, 255 ), -1, 8 );
	circle( graph_image, two, w/200, Scalar( 0, 0, 255 ), -1, 8 );

	imshow(graph_window,graph_image);
	waitKey(0);
}

//------------------------------------Unit Tests--------------------------------------

int getPoints_test() { 
	vector<float> base_point(2), angles(2), link_lengths(2);

	Point2f zero, one, two;

        base_point[0] = 3;
        base_point[1] = 3;
        angles[0] = M_PI/4;
        angles[1] = M_PI/6;
        link_lengths[0] = 1;
        link_lengths[1] = 3;

	Manipulator test( base_point, angles, link_lengths );
        test.getPoints(zero, one, two);

        float eps = 1e-6;

        if ( fabs ( zero.x-3 ) > eps ) {
        	fprintf( stderr, "Wrong zero.x %lf\n", zero.x );
          	return -1;
        }

        if ( fabs ( zero.y-3 ) > eps ) {
        	fprintf( stderr, "Wrong zero.y %lf\n", zero.y );
        	return -1;
        }
	
	if ( fabs ( one.x-3.7071067811865475 ) > eps ) {
        	fprintf( stderr, "Wrong one.x %lf\n", one.x );
        	return -1;
        }
	
	if ( fabs ( one.y-3.7071067811865475 ) > eps ) {
        	fprintf( stderr, "Wrong one.y %lf\n", one.y );
        	return -1;
        }

	if ( fabs ( two.x-6.305182992539864 ) > eps ) {
        	fprintf( stderr, "Wrong two.x %lf\n", two.x );
        	return -1;
        }

	if ( fabs ( two.y-5.207106781186548 ) > eps ) {
        	fprintf( stderr, "Wrong two.y %lf\n", two.y );
        	return -1;
        }

        return 0;
}

int getCoordinates_test() { 
	Point2f point_zero, point_one, point_two;

	std::vector<float> base_point(2), angles(2), link_lengths(2);

	for (int i = 0; i < 2; i++) {
		base_point[i] = 0;
		angles[i] = 0;
		link_lengths[i] = 0;
	}

        point_zero.x = 45.3145123;
	point_zero.y = 523.1465463;

	point_one.x = 798.1368468;
	point_one.y = 12.462649;

	point_two.x = 43.496793;
	point_two.y = 1.9646437;

	Manipulator test(base_point, angles, link_lengths);
        test.getCoordinates(point_zero, point_one, point_two);
	test.getManipulator(base_point, angles, link_lengths);	

        float eps = 1e-5;

        if ( fabs ( base_point[0] - 45.3145123 ) > eps ) {
        	fprintf( stderr, "Wrong base_point[0] %le\n", base_point[0] );
          	return -1;
        }

        if ( fabs ( base_point[1]-523.1465463 ) > eps ) {
        	fprintf( stderr, "Wrong base_point[1] %le\n", base_point[1] );
        	return -1;
        }
	
	if ( fabs ( angles[0] + 0.5960537) > eps ) {
        	fprintf( stderr, "Wrong angles[0] %le\n", angles[0] );
        	return -1;
        }
	
	if ( fabs ( angles[1]-0.0139103 ) > eps ) {
        	fprintf( stderr, "Wrong angles[1] %le\n", angles[1] );
        	return -1;
        }

	if ( fabs ( link_lengths[0]-909.6919865 ) > 0.01) {
        	fprintf( stderr, "Wrong link[0] %le\n", link_lengths[0] );
        	return -1;
        }
	
	if ( fabs ( link_lengths[1]-754.7130706 ) > 0.01 ) {
        	fprintf( stderr, "Wrong link[1] %le\n", link_lengths[1] );
        	return -1;
        }


        return 0;
}

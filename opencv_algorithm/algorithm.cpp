#include <vector>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <cv.h>
#include <highgui.h>

#define w 800
using namespace cv;
using std::vector;

/*
void MyEllipse( Mat img, double angle );
void MyFilledCircle( Mat img, Point center );
void MyPolygon( Mat img );
void MyLine( Mat img, Point start, Point end );
*/ 
int main( void ){

	char graph_window[] = "Graph FABRIK";
	Mat graph_image = Mat(Size(w,w),CV_8UC3,Scalar(255,255,255));
	line(graph_image,Point(w/2,3*w/4),Point(w/2,w/2),Scalar( 0, 0, 0 ),2,8);
	circle(graph_image,Point(w/2,w/2),w/100,Scalar( 0, 0, 255 ),-1,8);
	circle(graph_image,Point(w/2,3*w/4),w/100,Scalar( 0, 0, 255 ),-1,8);	
	
	/*
	/// Windows names
	char atom_window[] = "Drawing 1: Atom";
	char rook_window[] = "Drawing 2: Rook";
	/// Create black empty images
	Mat atom_image = Mat::zeros( w, w, CV_8UC3 );
	Mat rook_image = Mat::zeros( w, w, CV_8UC3 );
	/// 1. Draw a simple atom:
	/// -----------------------
	/// 1.a. Creating ellipses
	MyEllipse( atom_image, 90 );
	MyEllipse( atom_image, 0 );
	MyEllipse( atom_image, 45 );
	MyEllipse( atom_image, -45 );
	/// 1.b. Creating circles
	MyFilledCircle( atom_image, Point( w/2, w/2) );
	/// 2. Draw a rook
	/// ------------------
	/// 2.a. Create a convex polygon
	MyPolygon( rook_image );
	/// 2.b. Creating rectangles
	rectangle( rook_image,
	Point( 0, 7*w/8 ),
	Point( w, w),
	Scalar( 0, 255, 255 ),-1,8 );
	/// 2.c. Create a few lines
	MyLine( rook_image, Point( 0, 15*w/16 ), Point( w, 15*w/16 ) );
	MyLine( rook_image, Point( w/4, 7*w/8 ), Point( w/4, w ) );
	MyLine( rook_image, Point( w/2, 7*w/8 ), Point( w/2, w ) );
	MyLine( rook_image, Point( 3*w/4, 7*w/8 ), Point( 3*w/4, w ) );
	/// 3. Display your stuff!
	imshow( atom_window, atom_image );
	moveWindow( atom_window, 0, 200 );
	imshow( rook_window, rook_image );
	moveWindow( rook_window, w, 200 );
	waitKey( 0 );
	return(0);
	*/
	
	imshow(graph_window,graph_image);
	moveWindow(graph_window, 0, 200 );
	waitKey(0);
	return(0);
	
}
/*
void MyEllipse( Mat img, double angle )
{
	int thickness = 2;
	int lineType = 8;
	ellipse( img,
	Point( w/2, w/2 ),
	Size( w/4, w/16 ),
	angle,
	0,
	360,
	Scalar( 255, 0, 0 ),
	thickness,
	lineType );
}
void MyFilledCircle( Mat img, Point center )
{
	int thickness = -1;
	int lineType = 8;
	circle( img,
	center,
	w/32,
	Scalar( 0, 0, 255 ),
	thickness,
	lineType );
}

void MyPolygon( Mat img )
{
	int lineType = 8;
	Point rook_points[1][20];
	rook_points[0][0] = Point( w/4, 7*w/8 );
	rook_points[0][1] = Point( 3*w/4, 7*w/8 );
	rook_points[0][2] = Point( 3*w/4, 13*w/16 );
	rook_points[0][3] = Point( 11*w/16, 13*w/16 );
	rook_points[0][4] = Point( 19*w/32, 3*w/8 );
	rook_points[0][5] = Point( 3*w/4, 3*w/8 );
	rook_points[0][6] = Point( 3*w/4, w/8 );
	rook_points[0][7] = Point( 26*w/40, w/8 );
	rook_points[0][8] = Point( 26*w/40, w/4 );
	rook_points[0][9] = Point( 22*w/40, w/4 );
	rook_points[0][10] = Point( 22*w/40, w/8 );
	rook_points[0][11] = Point( 18*w/40, w/8 );
	rook_points[0][12] = Point( 18*w/40, w/4 );
	rook_points[0][13] = Point( 14*w/40, w/4 );
	rook_points[0][14] = Point( 14*w/40, w/8 );
	rook_points[0][15] = Point( w/4, w/8 );
	rook_points[0][16] = Point( w/4, 3*w/8 );
	rook_points[0][17] = Point( 13*w/32, 3*w/8 );
	rook_points[0][18] = Point( 5*w/16, 13*w/16 );
	rook_points[0][19] = Point( w/4, 13*w/16 );
	const Point* ppt[1] = { rook_points[0] };
	int npt[] = { 20 };
	fillPoly( img,
	ppt,
	npt,
	1,
	Scalar( 255, 255, 255 ),
	lineType );
}

void MyLine( Mat img, Point start, Point end )
{
	int thickness = 2;
	int lineType = 8;
	line( img,
	start,
	end,
	Scalar( 0, 0, 0 ),
	thickness,
	lineType );
}
*/

class Manipulator {
	private:
		std::vector<float> base_point, angles, link_lengths;
		std::vector<float> point_one, point_two;
	public:
		Manipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			setManipulator( manp_base_point, manp_angles, manp_link_lengths );		
		}
		
		void setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			base_point.resize(2);
			angles.resize(2);
			link_lengths.resize(2);

			base_point = manp_base_point;
			angles = manp_angles;
			link_lengths = manp_link_lengths;
		}
		void convert() {
			point_one[1] = base_point[1] + link_lengths[1]*cos(angles[1]);
			point_one[2] = base_point[2] + link_lengths[1]*sin(angles[1]);
			
			point_two[1] = point_one[1] + link_lengths[2]*cos(angles[2]);
			point_two[2] = point_one[2] + link_lengths[2]*sin(angles[2]);
		}		
};	

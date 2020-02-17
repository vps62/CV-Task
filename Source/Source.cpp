#pragma once
#define VTK_LEGACY_SILENT

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/affine.hpp"
#include <opencv2/calib3d.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace std;

typedef float Coord;
typedef tuple<Coord, Coord, Coord> xyzPoint;

struct Coordinate {
	float x;
	float y;
	float z;
};

void savePoints(string file, vector<vector<xyzPoint>> images) {
	Coord x, y, z;
	ofstream filePoints(file.c_str());
	filePoints << images.size() << endl;
	for (auto vec : images) {
		filePoints << vec.size() << endl;
		for (auto p : vec) {
			x = get<0>(p);
			y = get<1>(p);
			z = get<2>(p);
			filePoints << x << " " << y << " " << z << endl;
		}
	}
	filePoints.close();
}

vector<vector<xyzPoint>> loadPoints(string file) {
	Coord coordenadas[3];
	vector<xyzPoint> temp;
	vector<vector<xyzPoint>> res;
	int numImages = 0;
	int numPoints = 0;
	ifstream filePoints(file.c_str());
	filePoints >> numImages;
	for (int i = 0; i < numImages; i++) {
		temp.clear();
		filePoints >> numPoints;
		for (int j = 0; j < numPoints; j++) {
			for (int k = 0; k < 3; k++) {
				filePoints >> coordenadas[k];
			}
			temp.push_back(make_tuple(coordenadas[0], coordenadas[1], coordenadas[2]));
		}
		res.push_back(temp);
	}
	return res;
}

xyzPoint rotate(xyzPoint &point, float angle, xyzPoint refPoint) {
	Coord x = get<0>(point);
	Coord y = get<1>(point);
	Coord z = get<2>(point);
	Coord refX = get<0>(refPoint);
	Coord refY = get<1>(refPoint);
	Coord refZ = get<2>(refPoint);

	Coord newX = x - refX;
	Coord newY = y - refY;
	Coord newZ = z - refZ;
	Coord tempX = newX;
	newX = tempX * cos(angle * CV_PI / 180.0) + newZ * sin(angle * CV_PI / 180.0);
	newY = newY;
	newZ = tempX * (-1 * sin(angle * CV_PI / 180.0)) + newZ * cos(angle * CV_PI / 180.0);
	newX += refX;
	newY += refY;
	newZ += refZ;

	return make_tuple(newX, newY, newZ);
}

vector<xyzPoint> rotatePoints(vector<xyzPoint> points, float angle, xyzPoint refPoint) {
	vector<xyzPoint> res;
	for (auto t : points) {
		res.push_back(rotate(t, angle, refPoint));
	}
	return res;
}

vector<xyzPoint> getPoints(Mat & img) {
	vector<xyzPoint> res;
	return res;
}

void writePoints(string file, vector<Coordinate> &points) {
	ofstream filePoints(file.c_str());
	filePoints << "VERSION .7" << endl;
	filePoints << "FIELDS x y z" << endl;
	filePoints << "SIZE 4 4 4" << endl;
	filePoints << "TYPE F F F" << endl;
	filePoints << "COUNT 1 1 1" << endl;
	filePoints << "WIDTH " << points.size() << endl;
	filePoints << "HEIGHT 1" << endl;
	filePoints << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	filePoints << "POINTS " << points.size() << endl;
	filePoints << "DATA ascii" << endl;
	Coord x = 0;
	Coord y = 0;
	Coord z = 0;

	for (int i = 0; i < points.size(); i++) {
		x = points[i].x;
		y = points[i].y;
		z = points[i].z;
		filePoints << x << " " << y << " " << z << " " << endl;
	}
	filePoints.close();
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

}

struct Cube {
	int height;
	int width;
	int length;
	Mat cubeMat;
	int xValue;
	int yValue;
	int zValue;
	Mat rotMat = Mat::zeros(3, 3, CV_32F);
	Mat transMat = Mat::zeros(1, 3, CV_32F);
	vector<Coordinate>cubeData;
	vector<Coordinate>corners;
	vector<Coordinate>frontFace;
	vector<Coordinate>backFace;
	vector<Coordinate>leftFace;
	vector<Coordinate>rightFace;
	vector<Coordinate>topFace;
	vector<Coordinate>bottomFace;
};

struct Camera {
	Mat cameraMat = Mat::zeros(3, 3, CV_32F); // Camera Matrix
	Mat rotMat = Mat::zeros(3, 3, CV_32F);    // Rotational Matrix
	Mat transMat = Mat::zeros(1, 3, CV_32F);  // Translation Matrix
	int resolutionV;						  // Vertical Resolution of  Camera
	int resolutionH;						  // Horizontal Resolution of  Camera
	int focalLength;
	int cameraCenterX;						  // Camera center poinit
	int cameraCenterY;
	float hFOV;								  // 
	float vFOV;
	int baseLength;							  // Baslength of cemra and emitter
};

struct Emitter {
	Mat rotMat = Mat::zeros(3, 3, CV_32F);    // Rotational Matrix
	Mat transMat = Mat::zeros(1, 3, CV_32F);  // Translation Matrix
	int resolutionV;
	int resolutionH;
	int hFOV;
	int vFOV;
	Mat emiPos;
};

void createCube(Cube cube, Coordinate transCubeVec, Coordinate setCubeVec, float cAngle)
{

	//Creat corners and faces of cube with initial values
	Coordinate corner;

	// cube corner 0
	corner.x = 0;
	corner.y = 0;
	corner.z = 0;
	cube.corners.push_back(corner);
	cube.frontFace.push_back(corner);
	cube.bottomFace.push_back(corner);
	cube.leftFace.push_back(corner);

	// cube corner 1
	corner.x = 0;
	corner.y = 0;
	corner.z = cube.length;
	cube.corners.push_back(corner);
	cube.frontFace.push_back(corner);
	cube.topFace.push_back(corner);
	cube.leftFace.push_back(corner);

	// cube corner 2
	corner.x = cube.length;
	corner.y = 0;
	corner.z = cube.length;
	cube.corners.push_back(corner);
	cube.frontFace.push_back(corner);
	cube.topFace.push_back(corner);
	cube.rightFace.push_back(corner);

	// cube corner 3
	corner.x = cube.length;
	corner.y = 0;
	corner.z = 0;
	cube.corners.push_back(corner);
	cube.frontFace.push_back(corner);
	cube.bottomFace.push_back(corner);
	cube.rightFace.push_back(corner);

	// cube corner 4
	corner.x = 0;
	corner.y = cube.length;
	corner.z = 0;
	cube.corners.push_back(corner);
	cube.backFace.push_back(corner);
	cube.bottomFace.push_back(corner);
	cube.leftFace.push_back(corner);

	// cube corner 5
	corner.x = 0;
	corner.y = cube.length;
	corner.z = cube.length;
	cube.corners.push_back(corner);
	cube.backFace.push_back(corner);
	cube.topFace.push_back(corner);
	cube.leftFace.push_back(corner);

	// cube corner 6
	corner.x = cube.length;
	corner.y = cube.length;
	corner.z = cube.length;
	cube.corners.push_back(corner);
	cube.backFace.push_back(corner);
	cube.topFace.push_back(corner);
	cube.rightFace.push_back(corner);

	// cube corner 7
	corner.x = cube.length;
	corner.y = cube.length;
	corner.z = 0;
	cube.corners.push_back(corner);
	cube.backFace.push_back(corner);
	cube.bottomFace.push_back(corner);
	cube.rightFace.push_back(corner);

	// Fill vector with all cube values
	Coordinate tempPoint;
	for (int i = 0; i < cube.height; i++)
	{
		for (int j = 0; j < cube.length; j++)
		{
			for (int k = 0; k < cube.width; k++)
			{
				tempPoint.x = i;
				tempPoint.y = j;
				tempPoint.z = k;
				cube.cubeData.push_back(tempPoint);
			}
		}
	}

	writePoints("cubePoints.pcd", cube.cubeData);

	// Translational Matrix
	cube.transMat.at<int>(0, 1) = transCubeVec.x;
	cube.transMat.at<int>(0, 2) = transCubeVec.y;
	cube.transMat.at<int>(0, 3) = transCubeVec.z;

	//Rotational Matrix

	//Rodrigues form [Theta, x,y,z]
	// theta = sqrt(a^2 + b^2 + c^2)
	// v = [a/theta, b/theta, c/theta]

	Mat normCubeVector = Mat::zeros(0, 3, CV_32FC1);
	normCubeVector.at<float>(0, 0) = (setCubeVec.x / cAngle);
	normCubeVector.at<float>(0, 1) = (setCubeVec.y / cAngle);
	normCubeVector.at<float>(0, 2) = (setCubeVec.z / cAngle);
	Rodrigues(normCubeVector, cube.rotMat);

	// only Visible value in 3d space

	/*for (int i = 0; i < cubeValues.size(); i++)
	{
		std::cout << cubeValues[i].xValue << ":" << cubeValues[i].yValue << ":" << cubeValues[i].zValue << endl;
	}*/
}

void createCamera(Camera camera, Coordinate setTrans, float rAngle, Coordinate setVec)
{
	// Camera Resolution

	camera.resolutionH = 2 * camera.cameraCenterX;
	camera.resolutionV = 2 * camera.cameraCenterY;

	// Camera Matrix
	camera.cameraMat.at<int>(0, 0) = camera.focalLength;
	camera.cameraMat.at<int>(0, 2) = camera.cameraCenterX;
	camera.cameraMat.at<int>(1, 1) = camera.focalLength;
	camera.cameraMat.at<int>(1, 2) = camera.cameraCenterY;
	camera.cameraMat.at<int>(2, 2) = 1;

	// Translational Matrix
	camera.transMat.at<int>(0, 1) = setTrans.x;
	camera.transMat.at<int>(0, 2) = setTrans.y;
	camera.transMat.at<int>(0, 3) = setTrans.z;

	//Rotational Matrix

	//Rodrigues form [Theta, x,y,z]
	// theta = sqrt(a^2 + b^2 + c^2)
	// v = [a/theta, b/theta, c/theta]

	Mat normVector = Mat::zeros(0, 3, CV_32FC1);
	normVector.at<float>(0, 0) = (setVec.x / rAngle);
	normVector.at<float>(0, 1) = (setVec.y / rAngle);
	normVector.at<float>(0, 2) = (setVec.z / rAngle);
	Rodrigues(normVector, camera.rotMat);

	//Calcualtion of FOV of camera
	// fovx = 2 arctan(height / (2 * fx)) fovy = 2 arctan(width / (2 * fy))

	camera.hFOV = 2 * atan(camera.resolutionH / (2 * camera.focalLength));
	camera.vFOV = 2 * atan(camera.resolutionV / (2 * camera.focalLength));

}

void createEmitter(Camera camera, Emitter emitter, Coordinate transEmiVec, Coordinate setEmiVec, float emiAngle)
{
	// Emitter Resolution (kept same as camera)

	emitter.resolutionH = 2 * camera.cameraCenterX;
	emitter.resolutionV = 2 * camera.cameraCenterY;

	// Translational Matrix of Emitter
	emitter.transMat.at<int>(0, 1) = transEmiVec.x;
	emitter.transMat.at<int>(0, 2) = transEmiVec.y;
	emitter.transMat.at<int>(0, 3) = transEmiVec.z;

	//Rotational Matrix of Emitter
	//Rodrigues form [Theta, x,y,z]
	// theta = sqrt(a^2 + b^2 + c^2)
	// v = [a/theta, b/theta, c/theta]

	Mat normEmiVector = Mat::zeros(0, 3, CV_32FC1);
	normEmiVector.at<float>(0, 0) = (setEmiVec.x / emiAngle);
	normEmiVector.at<float>(0, 1) = (setEmiVec.y / emiAngle);
	normEmiVector.at<float>(0, 2) = (setEmiVec.z / emiAngle);
	Rodrigues(normEmiVector, emitter.rotMat);

	//Calcualtion of FOV of Emitter
	// fovx = 2 arctan(height / (2 * fx)) fovy = 2 arctan(width / (2 * fy))
	emitter.hFOV = 2 * atan(camera.resolutionH / (2 * camera.focalLength));
	emitter.vFOV = 2 * atan(camera.resolutionV / (2 * camera.focalLength));
}

void scanObject(Cube cube, Camera camera)
{
	int temp = 0;
	string name;
	// scan the object and save iamges in folder
	string path = "Laser_Scan_Images/";

	for (int i = 0; i < cube.length; i++)
	{
		Mat image = Mat::zeros(camera.resolutionH, camera.resolutionV, CV_8UC3);
		Rect r = Rect(50, 50, cube.length, cube.length);
		rectangle(image, r, Scalar(255, 255, 255), 1, 8, 0);
		// Laser beam
		line(image, Point(50 + i, 50), Point(50 + i, 50 + cube.length), cv::Scalar(0, 0, 255), 1, 8);
		//imshow("image", image);

		string n = path + to_string(temp) + ".jpg";;
		imwrite(n, image);

		fstream file;
		file.open(path + "data.txt", fstream::in);
		file >> temp;
		file.close();
		temp = temp + 1;
	}
}

void createEmitterBeamPCD(string file, vector<Coordinate> &points)
{
	ofstream filePoints(file.c_str());
	filePoints << "VERSION .7" << endl;
	filePoints << "FIELDS x y z rgb" << endl;
	filePoints << "SIZE 4 4 4 4" << endl;
	filePoints << "TYPE F F F F" << endl;
	filePoints << "COUNT 1 1 1 1" << endl;
	filePoints << "WIDTH " << points.size() << endl;
	filePoints << "HEIGHT 1" << endl;
	filePoints << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	filePoints << "POINTS " << points.size() << endl;
	filePoints << "DATA ascii" << endl;
	Coord x = 0;
	Coord y = 0;
	Coord z = 0;
	Coord k = 16711680;
	//std::uint8_t r = 255, g = 0, b = 0;    // Example: Red color
	//std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
	//k = *reinterpret_cast<float*>(&rgb);

	for (int i = 0; i < points.size(); i++) {
		x = points[i].x;
		y = points[i].y;
		z = points[i].z;
		filePoints << x << " " << y << " " << z << " " << k << " " << endl;
	}
	filePoints.close();

}

int main(int argc, char ** argv)
{
	//===========================Cube=======================
		// Cube h, l ,w
		// Cube position (x,y,z)
		// Cube rotation
		// Create cube function

	Cube cube;
	cube.height = 20;
	cube.length = 20;
	cube.width = 20;
	cube.cubeMat = Mat::zeros(cube.height, cube.length, CV_32F);
	cube.transMat.at<float>(0, 0) = 0;
	cube.transMat.at<float>(0, 1) = 0;
	cube.transMat.at<float>(0, 2) = 2;
	Mat rot_vec = Mat::zeros(1, 3, CV_32F);

	Coordinate transCubeVec;
	transCubeVec.x = 10;
	transCubeVec.y = 0;
	transCubeVec.z = 0;

	float cAngle = { 3.14 / 4 };

	Coordinate setCubeVec;
	setCubeVec.x = 1;
	setCubeVec.y = 0;
	setCubeVec.z = 0;

	createCube(cube, transCubeVec, setCubeVec, cAngle);

	//===========================Camera=====================
	// Camera resolutiuon is 640 * 480
	// Center point (320, 240)
	// focal length 140
	// What is rodriguz angle? How does opencv handles it?
	// Rodrigues(rot_mat, rot_vec); opencv function check 
	// Create Camera Function:

	Camera camera;
	camera.focalLength = 100;
	camera.cameraCenterX = 200;
	camera.cameraCenterY = 200;
	camera.baseLength = 600; // baselength is in mm

	Coordinate transCamVec;
	transCamVec.x = 10;
	transCamVec.y = 0;
	transCamVec.z = 0;

	float rAngle = { 3.14 / 4 };

	Coordinate setVec;
	setVec.x = 1;
	setVec.y = 0;
	setVec.z = 0;

	//createCamera(camera, transCamVec, rAngle, setVec);

	//=====================Emitter========================================
		// Emitter is fixed on camera axis
		// Set position of the emitter wrt camera

	Emitter emitter;
	float emiAngle = 3.14 / 2;
	Coordinate setEmiVec;
	setEmiVec.x = 1;
	setEmiVec.y = 0;
	setEmiVec.z = 0;

	Coordinate transEmiVec;
	transCamVec.x = -10;
	transCamVec.y = 0;
	transCamVec.z = 0;

	//createEmitter(camera, emitter, transEmiVec, setEmiVec, emiAngle);
//=============================Laser Scanner=============================
//
//=======================================================================
	// Focus Emitter on Cube and scan cube by laser
	// Capture the images from the camera

	// Function to capture the images

	vector<int>laserPoints;
	Mat laserOutput = Mat::zeros(2 * camera.cameraCenterX, 2 * camera.cameraCenterY, CV_8UC3);
	//scanObject(cube, camera);

	// Process on raw data

	string imgName;
	string path = "Laser_Scan_Images/";

	Mat inputFrame;
	Mat hsvImage;
	Mat hsvImageFilter;
	float rayAngle = 355;
	vector<Coordinate>Points;
	for (int i = 0; i < 60; i++)
	{
		imgName = path + to_string(i) + ".jpg";
		inputFrame = imread(imgName);
		// Conversion BGR color to thre HSV format
		cvtColor(inputFrame, hsvImage, COLOR_BGR2HSV);

		//Filter the red color pixels from the images
		inRange(hsvImage, Scalar(0, 100, 100), Scalar(10, 255, 255), hsvImageFilter);

	/*	imshow("Input Image", inputFrame);
		imshow("Binary Image", hsvImageFilter);

		waitKey(500);*/

		Coordinate pt;
		for (int i = 0; i < hsvImageFilter.rows; i++)
		{
			for (int j = 0; j < hsvImageFilter.cols; j++)
			{
				if (hsvImageFilter.at<uchar>(i, j) == 255)
				{
					// Calculate real points	
					pt.x = (camera.baseLength * i) / (camera.focalLength * (1.0 / tan(rayAngle * CV_PI / 180.0)) - i);
					pt.y = (camera.baseLength * j) / (camera.focalLength * (1.0 / tan(rayAngle * CV_PI / 180.0)) - i);
					pt.z = (camera.baseLength * camera.focalLength) / (camera.focalLength * (1.0 / tan(rayAngle * CV_PI / 180.0)) - i);
					Points.push_back(pt);
				}
			}
		}
	}

	writePoints("outputPoints.pcd", Points);
	
	vector<Coordinate>beamData;
	Coordinate bData;
	for (int i = 0; i < cube.length; i++)
	{
		beamData.empty();
		beamData.clear();

		bData.x = i;
		bData.y = -7;
		bData.z = 0;
		for (int j = 0; j < cube.length; j++)
		{
			
			beamData.push_back(bData);
			bData.z = bData.z + 1;
		}

		/*bData.x = i;
		bData.y = -6;
		bData.z = 0;

		for (int j = 0; j < cube.length; j++)
		{
			beamData.push_back(bData);
			bData.z = bData.z + 1;
		}

		bData.x = i + 0.5;
		bData.y = -7;
		bData.z = 0;

		for (int j = 0; j < cube.length; j++)
		{
			beamData.push_back(bData);
			bData.z = bData.z + 1;
		}

		bData.x = i - 0.5;
		bData.y = -7;
		bData.z = 0;

		for (int j = 0; j < cube.length; j++)
		{
			beamData.push_back(bData);
			bData.z = bData.z + 1;
		}*/

		string name = "Laser_Beam_PCD/" + to_string(i) + ".pcd";
		createEmitterBeamPCD(name, beamData);
	}


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile("outputPoints.pcd", *cloud1);
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Show Result"));
	//viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud1, "Output");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample");
	////viewer->addCoordinateSystem(1.0);
	//viewer->spin();



	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("cubePoints.pcd", *cloud);
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Object Scanning"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cube");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	//viewer->spin();

	for (int i= 0; i < cube.length; i++)
	{	
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		string beamName = "Laser_Beam_PCD/" + to_string(i) + ".pcd";
		pcl::io::loadPCDFile(beamName, *point_cloud);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud, rgb, to_string(i));
		viewer->spinOnce(2000);
		
		//viewer->removePointCloud(to_string(i));

	}
 
	// Read Images 
	/*string beamImage;
	Mat beamMat;
	for (int i= 0; i < 60 ; i++)
	{
		beamImage  = "Laser_Scan_Images/ " + to_string(i) + ".jpg";
		beamMat = imread(beamImage);
		imshow("Camera captured Images", beamMat);
	}*/

	//cv::waitKey(1000);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer->addCoordinateSystem(1.0);
	
	

	string file = "xyz";
	vector<vector<xyzPoint>> images;
	//vector<xyzPoint> points;

		/*	if (i / 10 == 0)
			{
				name = "Image000" + to_string(i) + ".jpg";
			}
			else if (i / 100 == 0)
			{
				name = "Image00" + to_string(i) + ".jpg";
			}
			else if (i / 1000 == 0)
			{
				name = "Image0" + to_string(i) + ".jpg";
			}
			else
			{
				name = "Image" + to_string(i) + ".jpg";
			}*/

			//string n = path + to_string(i) + ".jpg";
			//cameraFrame = imread(n);

			//savePoints(file, images);

			/*cout << "Num_Images->" << images.size() << endl;
			float d_angle = 360.0 / images.size();
			cout << "d_angle->" << d_angle << endl;
			float actualAngle = 0;
			auto refPoint = make_tuple(get<0>(images.front().front()) + 0.5, get<1>(images.front().front()) + 0.5, get<2>(images.front().front()) + 0.5);
		*/
		/*for (int i = 0; i < Points.size(); i++)
		{
			cout << Points[i].x << Points[i].y << Points[i].z << endl;
		}*/

		//for (auto vec : images) {
		//	//auto temp = rotatePoints(vec, actualAngle, refPoint);
		//	actualAngle += d_angle;
		//	points.insert(points.end(), temp.begin(), temp.end());
		//}

	// spin once.....
	// 

	return 0;
}

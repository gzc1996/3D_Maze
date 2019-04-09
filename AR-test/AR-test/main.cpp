#include <iostream>
#include <fstream>
#include <opencv.hpp>
#include "MarkerDetector.hpp"
#include "3ds.h"
#include "Maze.h"
#include <glut.h>
#include "Texture.h"
#include "fmod.h"                              //音频库的头文件
#pragma comment(lib, "fmodvc.lib")	          //把音频库加入到链接器中

bool AR_mode = false;
bool Entry_flag = true;
bool Music_flag = true;
const int PAT_ROWS = 7;            //棋盘的行数
const int PAT_COLS = 10;           //棋盘的列数
const float CHESS_SIZE = 24.0;   //棋盘各自的尺寸，unit:mm
const float CUBE_SIZE = 0.2;
const int CUBE_CELL_NUM = 3;  //立方体大小为3*3*3
const int CUBE_ROW_NUM = 3; //每一行立方体的个数为3
const int CUBE_COL_NUM = 2;  //每一列立方体的个数为2
GLuint textureid[15]; // 定义纹理变量
FSOUND_STREAM *mp3back;

cv::VideoCapture cap;
cv::Mat mapx, mapy;
CameraCalibration calibration;
float cube_size = 0.5;
Maze maze;

GLfloat vertices[][3] = {
	{ -cube_size,-cube_size,-cube_size },
	{ cube_size,-cube_size,-cube_size },
	{ cube_size,cube_size,-cube_size },
	{ -cube_size,cube_size,-cube_size },
	{ -cube_size,-cube_size,cube_size },
	{ cube_size,-cube_size,cube_size },
	{ cube_size,cube_size,cube_size },
	{ -cube_size,cube_size,cube_size }
};

GLfloat colors[][3] = {
	{ 1.0,0.0,0.0 },
	{ 0.0,1.0,1.0 },
	{ 1.0,1.0,0.0 },
	{ 0.0,1.0,0.0 },
	{ 0.0,0.0,1.0 },
	{ 1.0,0.0,1.0 },
	{ 0.0,1.0,1.0 },
	{ 1.0,1.0,1.0 }
};

C3DSModel  draw3ds[6];

void polygon(int a, int b, int c, int d, float R, float G, float B, float size, int side, std::pair<float, float> p1, std::pair<float, float> p2, std::pair<float, float> p3, std::pair<float, float> p4);
void init();
void DrawCubeCell(std::vector<int> side, std::vector<std::pair<float, float> > points);
void Menu(int data);
void Special(int key, int x, int y);
void DrawCube_ID_1();
void DrawCube_ID_2();
void DrawCube_ID_3();
void DrawCube_ID_4();
void DrawCube_ID_5();
void DrawCube_ID_6();

Matrix44 buildProjectionMatrix(Matrix33 cameraMatrix, int screen_width, int screen_height)
{
    float d_near = 0.01;    
    float d_far = 100.0;   

    float f_x = cameraMatrix.data[0]; 
    float f_y = cameraMatrix.data[4]; 
    float c_x = cameraMatrix.data[2]; 
    float c_y = cameraMatrix.data[5]; 

	Matrix44 projectionMatrix = Matrix44();
    projectionMatrix.data[0] = -2.0 * f_x / screen_width;
    projectionMatrix.data[1] = 0.0;
    projectionMatrix.data[2] = 0.0;
    projectionMatrix.data[3] = 0.0;

    projectionMatrix.data[4] = 0.0;
    projectionMatrix.data[5] = 2.0 * f_y / screen_height;
    projectionMatrix.data[6] = 0.0;
    projectionMatrix.data[7] = 0.0;

    projectionMatrix.data[8] = 2.0 * c_x / screen_width - 1.0;
    projectionMatrix.data[9] = 2.0 * c_y / screen_height - 1.0;
    projectionMatrix.data[10] = -(d_far + d_near) / (d_far - d_near);
    projectionMatrix.data[11] = -1.0;

    projectionMatrix.data[12] = 0.0;
    projectionMatrix.data[13] = 0.0;
    projectionMatrix.data[14] = -2.0 * d_far * d_near / (d_far - d_near);
    projectionMatrix.data[15] = 0.0;

    return projectionMatrix;
}

//闲置函数
void idle(void)
{
    glutPostRedisplay();
}

void display(void)
{
	if (Entry_flag)
	{
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, 100, 0, 100);
		glColor3f(1, 1, 1);
		//绘制启动画面
		//绘制矩形，使得矩形大小和裁剪窗口大小一致
		//给该矩形添加贴图
		glEnable(GL_TEXTURE_2D); //启用纹理
		glBindTexture(GL_TEXTURE_2D, textureid[5]);
		//glTranslated(5, 5, 0);
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex2f(0, 0);
		glTexCoord2f(1, 0);
		glVertex2f(100, 0);
		glTexCoord2f(1, 1);
		glVertex2f(100, 100);
		glTexCoord2f(0, 1);
		glVertex2f(0, 100);
		glEnd();
		glDisable(GL_TEXTURE_2D); //启用纹理
		glutSwapBuffers();
		return;
	}
	if (AR_mode)
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		if (cap.isOpened()) {
			cv::Mat image_raw, temp;
			cap >> image_raw;

			// 通过线性插值修正部分误差       
			cv::Mat image;
			cv::remap(image_raw, image, mapx, mapy, cv::INTER_LINEAR);

			// 将opencv读入的BGR格式转换成RGB
			cv::Mat rgb;
			cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
			cv::flip(rgb, rgb, 0);
			glDepthMask(GL_FALSE);
			glDrawPixels(rgb.cols, rgb.rows, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);

			// 将opencv读入的BGR格式转换成BGRA
			cv::Mat bgra;
			cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);

			BGRAVideoFrame frame;
			frame.width = bgra.cols;
			frame.height = bgra.rows;
			frame.data = bgra.data;
			frame.stride = bgra.step;

			// Marker检测
			MarkerDetector detector(calibration);
			detector.processFrame(frame);
			std::vector<Transformation> transformations = detector.getTransformations();
			std::vector<Marker> markers = detector.markers;
			// 计算投影变换矩阵
			Matrix44 projectionMatrix = buildProjectionMatrix(calibration.getIntrinsic(), frame.width, frame.height);

			// opengl中加载投影变换矩阵
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(projectionMatrix.data);

			// opengl使用模型矩阵，并将当前的用户坐标系的原点移到屏幕中心
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			// 深度缓冲区读写有效
			glDepthMask(GL_TRUE);
			glPushMatrix();

			for (size_t i = 0; i < markers.size(); i++) {
				Matrix44 glMatrix = markers[i].transformation.getMat44();
				// 加载每一个Maker对应的视图矩阵
				glLoadMatrixf(reinterpret_cast<const GLfloat*>(glMatrix.data));
				if (markers[i].id == 0)
				{
					glColor3f(0.8f, 0.8f, 0.8f);
					glPushMatrix();
					glScaled(0.5, 0.5, 0.3);
					glTranslated(-8.0f, -8.0f, 0);
					glEnable(GL_TEXTURE_2D);
					maze.AR_display();
					glDisable(GL_TEXTURE_2D);
					glPopMatrix();
				}
				else if (markers[i].id == 1)
					DrawCube_ID_1();
				else if (markers[i].id == 2)
					DrawCube_ID_2();
				else if (markers[i].id == 3)
					DrawCube_ID_3();
				else if (markers[i].id == 4)
					DrawCube_ID_4();
				else if (markers[i].id == 5)
					DrawCube_ID_5();
				else if (markers[i].id == 6)
					DrawCube_ID_6();
			}

			glDisableClientState(GL_VERTEX_ARRAY);
			glPopMatrix();
		}
	}
	else
	{
		glClearColor(1.0, 1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);            // 清空颜色数据和深度数据
		glColor3f(0.8f, 0.8f, 0.8f);
		maze.Normal_display();
	}
    
    glutSwapBuffers();
}

void key(unsigned char key, int x, int y) {
    switch (key) {
	//ESC键退出
    case 0x1b:
        exit(1);
        break;
	case ' ':
		maze.keyboard();
	case 13:
		Entry_flag = false;
		break;
    default:
        break;
    }
}

void resize(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, (double)w / (double)h, 0.01, 100.0);
}

int main(int argc, char *argv[])
{
    if (!cap.open(1)) {
		std::cout << "opencv初始化摄像机失败..." << std::endl;
        return -1;
    }

    cv::Mat frame;
    cap >> frame;

    // 打开文件读入摄像机参数
    std::string filename("camera.xml");
    std::fstream file("camera.xml", std::ios::in);

    // 打开一个XML文件
    cv::FileStorage rfs(filename, cv::FileStorage::READ);
    if (!rfs.isOpened()) {
        std::cout << "没有在摄像头参数读" << std::endl;
        return -1;
    }

    // 相机参数读
    cv::Mat cameraMatrix, distCoeffs;
    rfs["intrinsic"] >> cameraMatrix;
    rfs["distortion"] >> distCoeffs;

    // 失真矫正
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
    //std::cout << cameraMatrix << std::endl;

    // 设置相机参数
    float fx = cameraMatrix.at<double>(0, 0);
    float fy = cameraMatrix.at<double>(1, 1);
    float cx = cameraMatrix.at<double>(0, 2);
    float cy = cameraMatrix.at<double>(1, 2);
    calibration = CameraCalibration(fx, fy, cx, cy);
    // opengl初始化
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(frame.cols, frame.rows);
    glutCreateWindow("3D Maze + 3D Puzzle AR");
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    glutIdleFunc(idle);

    glClearColor(0.0, 0.0, 1.0, 1.0);
    glEnable(GL_DEPTH_TEST);
	init();
	glutCreateMenu(Menu);
	glutAddMenuEntry("Map 1 of 2", 1);
	glutAddMenuEntry("Map 2 of 2", 2);
	glutAddMenuEntry("Fog", 3);
	glutAddMenuEntry("Normal Mode", 4);
	glutAddMenuEntry("AR Mode", 5);
	glutAddMenuEntry("Music", 6);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	glutSpecialFunc(Special);
    glutMainLoop();
	draw3ds[0].Release();
    return 0;
}

void polygon(int a, int b, int c, int d, float R, float G, float B, float size, int side, std::pair<float,float> p1, std::pair<float, float> p2, std::pair<float, float> p3, std::pair<float, float> p4)
{
	float cube_size = size;
	GLfloat vertices[][3] = {
		{ -cube_size,-cube_size,-cube_size },
		{ cube_size,-cube_size,-cube_size },
		{ cube_size,cube_size,-cube_size },
		{ -cube_size,cube_size,-cube_size },
		{ -cube_size,-cube_size,cube_size },
		{ cube_size,-cube_size,cube_size },
		{ cube_size,cube_size,cube_size },
		{ -cube_size,cube_size,cube_size }
	};
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor3f(0.0, 0.0, 0.0);
	glBegin(GL_POLYGON);
	glVertex3fv(vertices[a]);
	glVertex3fv(vertices[b]);
	glVertex3fv(vertices[c]);
	glVertex3fv(vertices[d]);
	glEnd();

	if (side != -1)
	{
		glEnable(GL_TEXTURE_2D); //启用纹理
		if(side == 5)
			glBindTexture(GL_TEXTURE_2D, textureid[0]);
		if (side == 4)
			glBindTexture(GL_TEXTURE_2D, textureid[2]);
		if (side == 3)
			glBindTexture(GL_TEXTURE_2D, textureid[4]);
		if (side == 2)
			glBindTexture(GL_TEXTURE_2D, textureid[1]);
		if (side == 1)
			glBindTexture(GL_TEXTURE_2D, textureid[3]);
		glColor3f(1, 1, 1);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);
		glTexCoord2f(p1.first, p1.second);
		glVertex3fv(vertices[a]);
		glTexCoord2f(p2.first, p2.second);
		glVertex3fv(vertices[b]);
		glTexCoord2f(p3.first, p3.second);
		glVertex3fv(vertices[c]);
		glTexCoord2f(p4.first, p4.second);
		glVertex3fv(vertices[d]);
		glEnd();
		glDisable(GL_TEXTURE_2D); //禁用纹理
	}
	else if (side == -1)
	{
		glColor3f(R, G, B);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);
		glVertex3fv(vertices[a]);
		glVertex3fv(vertices[b]);
		glVertex3fv(vertices[c]);
		glVertex3fv(vertices[d]);
		glEnd();
	}
}

void DrawCubeCell(std::vector<int> side, std::vector<std::pair<float,float> > points)
{
	std::vector<int> ramain_side;
	bool flag = true;
	for (int i = 0; i < side.size(); i++)
	{
		if (side[i] == 1)
			polygon(4, 7, 3, 0, 1.0f, 1.0f, 0.0f, CUBE_SIZE, 1,
				std::pair<float, float>(points[0 + i * 4].first, points[0 + i * 4].second),
				std::pair<float, float>(points[1 + i * 4].first, points[1 + i * 4].second),
				std::pair<float, float>(points[2 + i * 4].first, points[2 + i * 4].second),
				std::pair<float, float>(points[3 + i * 4].first, points[3 + i * 4].second));
		if (side[i] == 2)
			polygon(7, 6, 2, 3, 1.0f, 1.0f, 0.0f, CUBE_SIZE, 2,
				std::pair<float, float>(points[0 + i * 4].first, points[0 + i * 4].second),
				std::pair<float, float>(points[1 + i * 4].first, points[1 + i * 4].second),
				std::pair<float, float>(points[2 + i * 4].first, points[2 + i * 4].second),
				std::pair<float, float>(points[3 + i * 4].first, points[3 + i * 4].second));
		if (side[i] == 3)
			polygon(6, 5, 1 ,2, 1.0f, 1.0f, 0.0f, CUBE_SIZE, 3,
				std::pair<float, float>(points[0 + i * 4].first, points[0 + i * 4].second),//1, 2, 6, 5
				std::pair<float, float>(points[1 + i * 4].first, points[1 + i * 4].second),
				std::pair<float, float>(points[2 + i * 4].first, points[2 + i * 4].second),
				std::pair<float, float>(points[3 + i * 4].first, points[3 + i * 4].second));
		if (side[i] == 4)
			polygon(5, 4, 0, 1, 1.0f, 1.0f, 0.0f, CUBE_SIZE, 4,//1, 0, 4, 5
				std::pair<float, float>(points[0 + i * 4].first, points[0 + i * 4].second),
				std::pair<float, float>(points[1 + i * 4].first, points[1 + i * 4].second),
				std::pair<float, float>(points[2 + i * 4].first, points[2 + i * 4].second),
				std::pair<float, float>(points[3 + i * 4].first, points[3 + i * 4].second));
		if (side[i] == 5)
			polygon(4 ,5, 6, 7, 1.0f, 1.0f, 0.0f, CUBE_SIZE, 5,
				std::pair<float, float>(points[0 + i * 4].first, points[0 + i * 4].second),
				std::pair<float, float>(points[1 + i * 4].first, points[1 + i * 4].second),
				std::pair<float, float>(points[2 + i * 4].first, points[2 + i * 4].second),
				std::pair<float, float>(points[3 + i * 4].first, points[3 + i * 4].second));
		if (side[i] == 6)
			polygon(0, 3, 2, 1, 1.0f, 1.0f, 0.0f, CUBE_SIZE, 6,
				std::pair<float, float>(points[0 + i * 4].first, points[0 + i * 4].second),
				std::pair<float, float>(points[1 + i * 4].first, points[1 + i * 4].second),
				std::pair<float, float>(points[2 + i * 4].first, points[2 + i * 4].second),
				std::pair<float, float>(points[3 + i * 4].first, points[3 + i * 4].second));
	}

	for (int i = 1; i <= 6; i++)
	{
		for (int j = 0; j < side.size(); j++)
		{
			if (side[j] == i)
			{
				flag = false;
				break;
			}
		}
		if (flag)
			ramain_side.push_back(i);
		flag = true;
	}

	for (int i = 0; i < ramain_side.size(); i++)
	{
		if (ramain_side[i] == 1)
			polygon(4, 7, 3, 0, 1.0f, 1.0f, 0.0f, CUBE_SIZE, -1, std::pair<float, float>(0.0f, 0.0f),//3, 0, 4, 7
																						std::pair<float, float>(1.0f, 0.0f),
																						std::pair<float, float>(1.0f, 1.0f),
																						std::pair<float, float>(0.0f, 1.0f));
		if (ramain_side[i] == 2)
			polygon(7, 6, 2, 3, 1.0f, 1.0f, 0.0f, CUBE_SIZE, -1, std::pair<float, float>(0.0f, 0.0f),//2, 3, 7, 6
																						std::pair<float, float>(1.0f, 0.0f),
																						std::pair<float, float>(1.0f, 1.0f),
																						std::pair<float, float>(0.0f, 1.0f));
		if (ramain_side[i] == 3)
			polygon(6, 5, 1, 2, 1.0f, 1.0f, 0.0f, CUBE_SIZE, -1, std::pair<float, float>(0.0f, 0.0f),
																						std::pair<float, float>(1.0f, 0.0f),
																						std::pair<float, float>(1.0f, 1.0f),
																						std::pair<float, float>(0.0f, 1.0f));
		if (ramain_side[i] == 4)
			polygon(5, 4, 0, 1, 1.0f, 1.0f, 0.0f, CUBE_SIZE, -1, std::pair<float, float>(0.0f, 0.0f),
																						std::pair<float, float>(1.0f, 0.0f),
																						std::pair<float, float>(1.0f, 1.0f),
																						std::pair<float, float>(0.0f, 1.0f));
		if (ramain_side[i] == 5)
			polygon(4, 5, 6, 7, 1.0f, 1.0f, 0.0f, CUBE_SIZE, -1, std::pair<float, float>(0.0f, 0.0f),
																						std::pair<float, float>(1.0f, 0.0f),
																						std::pair<float, float>(1.0f, 1.0f),
																						std::pair<float, float>(0.0f, 1.0f));
		if (ramain_side[i] == 6)
			polygon(0, 3, 2, 1, 1.0f, 1.0f, 0.0f, CUBE_SIZE, -1, std::pair<float, float>(0.0f, 0.0f),
																						std::pair<float, float>(1.0f, 0.0f),
																						std::pair<float, float>(1.0f, 1.0f),
																						std::pair<float, float>(0.0f, 1.0f));
	}
}

void DrawCube_ID_1()
{
	std::vector<int> side;
	std::vector<std::pair<float, float> > points;
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	side.push_back(1);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	side.push_back(1);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
}

void DrawCube_ID_2()
{
	std::vector<int> side;
	std::vector<std::pair<float, float> > points;
	side.push_back(4);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.8f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.8f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
}

void DrawCube_ID_3()
{
	std::vector<int> side;
	std::vector<std::pair<float, float> > points;
	side.push_back(4);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	side.push_back(3);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);	
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.8f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(4);
	side.push_back(3);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.8f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.8f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(4);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.8f, 0.8f);DrawCubeCell(side, points);glPopMatrix();//突出
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
}

void DrawCube_ID_4()
{
	std::vector<int> side;
	std::vector<std::pair<float, float> > points;
	side.push_back(-1);
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.8f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.8f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.8f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.8f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, -0.8f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(1);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(1);
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
}

void DrawCube_ID_5()
{
	std::vector<int> side;
	std::vector<std::pair<float, float> > points;
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, -0.8f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.8f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, -0.8f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 4.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.4f, -0.8f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.8f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(4.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
}

void DrawCube_ID_6()
{
	std::vector<int> side;
	std::vector<std::pair<float, float> > points;
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 0.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.0f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(-1);
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(3);
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(2);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 1.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.4f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(3.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, -0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.8f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(-0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	glPushMatrix();glTranslatef(0.0f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(2.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.0f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(5.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.8f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(6.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(-0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(7.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.0f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
	std::swap(side, std::vector<int>());std::swap(points, std::vector<std::pair<float, float> >());
	side.push_back(5);
	side.push_back(2);
	side.push_back(3);
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 3.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(9.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(8.0 / (CUBE_ROW_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 6.0 / (CUBE_COL_NUM*CUBE_CELL_NUM)));
	points.push_back(std::pair<float, float>(1.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	points.push_back(std::pair<float, float>(0.0 / (CUBE_COL_NUM*CUBE_CELL_NUM), 2.0 / CUBE_CELL_NUM));
	glPushMatrix();glTranslatef(0.4f, 0.4f, 0.8f);DrawCubeCell(side, points);glPopMatrix();
}

void init()
{
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	//加载3DS模型
	draw3ds[0].Load("chanche.3ds");
	BuildTexture("images/d.jpg", textureid[0]);
	BuildTexture("images/e.jpg", textureid[1]);
	BuildTexture("images/f.jpg", textureid[2]);
	BuildTexture("images/g.jpg", textureid[3]);
	BuildTexture("images/h.jpg", textureid[4]);
	BuildTexture("images/entry.jpg", textureid[5]);
	maze.init();
	if (FSOUND_Init(44100, 32, 0))			// 把声音初始化为44khz
	{
		// 载入文件1.mp3
		mp3back = FSOUND_Stream_OpenFile("music/BGM.mp3", FSOUND_LOOP_NORMAL, 0);
		//初始化时即播放音乐
		FSOUND_Stream_Play(FSOUND_FREE, mp3back);
	}
}

void Menu(int data)
{
	maze.Menu(data);
	if (data == 6)
	{
		Music_flag = !Music_flag;
		if (Music_flag)
			FSOUND_Stream_Play(FSOUND_FREE, mp3back);
		else
			FSOUND_Stream_Stop(mp3back);
	}
		
	if (data == 5)
		AR_mode = true;
	if (data == 4)
		AR_mode = false;
}

void Special(int key, int x, int y)
{
	maze.special(key, x, y);
}
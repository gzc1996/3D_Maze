#pragma once
#include<math.h>
#include<windows.h>
#include <glut.h>

#define    PX_START    1.5//       起始横坐标
#define    PY_START    1.5//       起始纵坐标
#define    UNITSTEP    0.2//       步长
#define    ANGLE_START    -45//    起始及视角转换角度

class Maze {

public:
	Maze();
	~Maze();
	int wall(int x, int y);
	void calcDir();
	int drawDoor(int x, int y);
	int drawWall(int x, int y);
	int drawFloor(int x, int y);
	int drawPlayer(bool AR_mode);
	void drawMap(bool AR_mode);
	int drawSky(int x, int y);
	int drawFog(int x, int y);
	void init();
	void AR_display(void);
	void Normal_display(void);
	void Menu(int data);
	void aheadAndBack(float pos);
	void keyboard();
	void special(int key, int x, int y);
	void idle(void);
	void reshape(GLsizei w, GLsizei h);

private:
	GLuint texture_id[6]; // 定义纹理变量


	float      px = PX_START;
	float      py = PY_START;
	int        angle = ANGLE_START;            //行走方向
	int        dir_x;                //行走方向向量
	int        dir_y;
	int        MAP_SIZE = 16;
	char       **map;
	int        globalView = 0;    //是否查看俯视图
	bool flag = false;

	//H为墙位置 空格为路位置 E为出口位置
	char *map1[16] =
	{
		"HHHHHHHHHHHHHHHH",
		"H       H  F   H",
		"H H HHH H H    H",
		"H HH  H HH H H H",
		"H     H      H H",
		"HHHHHHHHHH HHH H",
		"H           H  H",
		"H HHHHH HHH HHHH",
		"H H   H   H    H",
		"H   HHHHHHH    H",
		"H H   H   H  H H",
		"H HHHHH HHHH H H",
		"H     H      H H",
		"HH HH HHHH HHH H",
		"H   H H    H   H",
		"HHHHHHHHHHHHHEHH"
	};
	/*char *map1[16] =
	{
		"HHHHHHHHHHHHHHHH",
		"H       H      H",
		"H H HHH H H    H",
		"H HH  H HH H H H",
		"H     H      H H",
		"HHHHHHHHHH HHH H",
		"H           H  H",
		"H HHHHH HHH HHHH",
		"H H   H   H    H",
		"H   HHHHHHH    H",
		"H H   H   H  H H",
		"H HHHHH HHHH H H",
		"H     H      H H",
		"HH HH HHHH HHH H",
		"H   H H    H   H",
		"HHHHHHHHHHHHHEHH"
	};*/
	char *map2[25] =
	{
		"HHHHHHHHHHHHHHHHHHHHHHHHH",
		"H    H H H      HH H    H",
		"H HH   H   H  H HHH  HH H",
		"H H    HH H HHH       HHH",
		"H H   HHH HH H HH H HHHHH",
		"H HH HHHH HH H H  HH  HHH",
		"H         H H  HH   HH HH",
		"H HH HHH   H HH  HH HH HH",
		"H       H  HHH H HH HH  H",
		"HH  H H  H H HH  H  HH HH",
		"HHH   H   HH HH H  HHH  H",
		"HHH HHH H  H H  HH  HHH H",
		"HHH HHH H    HH HH  HH  H",
		"HHH H   HHHH  H  H  H H H",
		"HH    HHH HH   H H HHH  H",
		"HHHH HHHHH HHH  HH  HH  H",
		"HHH  H HHHHH   HHHH HH HH",
		"HHH  HH   HH   HHH  HH  H",
		"HHH   HHH HHH HH HHH HH H",
		"HHH HH  HHHH  HHH HH  HHH",
		"HHH H    HH  HHHH HHH HHH",
		"H   H HHH    HH  H H  HHH",
		"H     HH HH     HH    HHH",
		"H HH HHHH HHHHH     H  HH",
		"HHHHHHHHHHHHHHHHHHHHHHEHH"
	};
};


#include "Maze.h"
#include "Texture.h"

Maze::Maze()
{
}


Maze::~Maze()
{
}

int Maze::wall(int x, int y)
{
	if (map[(int)x][(int)y] == 'H')//检测墙位置
		return    1;
	return    0;
}

void Maze::calcDir()
{
	if (angle<0 && angle>-180)    dir_x = 1;
	else    if (angle<180 && angle>0)    dir_x = -1;
	else    dir_x = 0;

	if (abs(angle) < 90)        dir_y = 1;
	else    if (abs(angle) > 90)        dir_y = -1;
	else    dir_y = 0;
}

int Maze::drawDoor(int x, int y)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id[2]);    //使用贴图纹理

	glPushMatrix();        //压入变换矩阵

	glTranslatef((float)x + 0.5, (float)y + 0.5, 0.0f);
	glBegin(GL_QUADS);
	//前面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	//后面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	//上面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	//下面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	//右面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	//左面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix(); //弹出变换矩阵
	return 1;
}

int Maze::drawWall(int x, int y)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id[0]);    //使用贴图纹理

	glPushMatrix();        //压入变换矩阵

	glTranslatef((float)x + 0.5, (float)y + 0.5, 0.0f);
	glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
	//前面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	//后面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glEnd();
	//上面
	glBindTexture(GL_TEXTURE_2D, texture_id[3]);    //使用贴图纹理
	glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	glBindTexture(GL_TEXTURE_2D, texture_id[0]);    //使用贴图纹理
	glEnd();
	//下面
	glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	//右面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	//左面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, 0.5f, -0.5f);

	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix(); //弹出变换矩阵
	return 1;
}

int Maze::drawFloor(int x, int y)
{
	glPushMatrix();        //压入变换矩阵
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id[1]);    //使用贴图纹理
	glColor3f(0.7, 0.7, 0.7);
	glTranslatef((float)x + 0.5, (float)y + 0.5, 0.0f);
	glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-0.5f, 0.5f, -0.5f);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	return    1;
}

int Maze::drawPlayer(bool AR_mode)
{
	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
	glColor3f(0.8, 0.0, 0.0);
	//glScaled(0.5, 0.5, 0.5);
	glTranslatef(px, py, 0.0f);
	glRotatef(-90, 1.0, 0.0, 0.0);//把z轴绕x旋转至第一人称视角方向
	glRotatef(-angle, 0.0, 1.0, 0.0);
	glScaled(0.5, 0.5, 0.5);
	if(globalView || AR_mode)
		glutSolidCone(0.3, 0.8, 20.0, 1);
	glEnable(GL_TEXTURE_2D);
	glPopMatrix();
	return    1;
	//glPushMatrix();
	//glDisable(GL_TEXTURE_2D);
	//glColor3f(0.8, 0.0, 0.0);
	//glTranslatef(px, py, 0.0f);
	//glRotatef(angle, 0.0, 0.0, 1.0);
	//glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
	//glVertex3f(-0.2f, -0.4f, 1.0f);
	//glVertex3f(0.2f, -0.4f, 1.0f);
	//glVertex3f(0.0f, 0.0f, 1.0f);
	//glVertex3f(0.0f, 0.4f, 1.0f);
	//glEnd();
	//glEnable(GL_TEXTURE_2D);
	//glPopMatrix();
	//return    1;
}

int Maze::drawFog(int x, int y)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id[5]);    //使用贴图纹理

	glPushMatrix();        //压入变换矩阵

	glTranslatef((float)x + 0.5, (float)y + 0.5, 0.0f);
	glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
						//前面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	//后面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	//上面
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, 0.5f, -0.5f);
	//下面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	//右面
	glTexCoord2f(0.0f, 1.0f); glVertex3f(0.5f, -0.5f, 0.5f);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(0.5f, 0.5f, -0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(0.5f, 0.5f, 0.5f);
	//左面
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.5f, -0.5f, -0.5f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-0.5f, -0.5f, 0.5f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-0.5f, 0.5f, 0.5f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.5f, 0.5f, -0.5f);

	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix(); //弹出变换矩阵
	return 1;
}

void Maze::drawMap(bool AR_mode)
{
	int x, y;
	for (x = 0; x < MAP_SIZE; x++)
		for (y = 0; y < MAP_SIZE; y++)
			if (map[x][y] == 'H')
				drawWall(x, y);
			else if (map[x][y] == 'E')
				drawDoor(x, y);
			else if (AR_mode == false)
			{
				if (map[x][y] == 'F')
					drawFog(x, y);
				if (globalView == 0)
				{
					drawFloor(x, y);
					drawSky(x, y);
				}
				else
					drawFloor(x, y);
			}
					
				drawPlayer(AR_mode);
}

int Maze::drawSky(int x, int y)
{
	glPushMatrix();        //压入变换矩阵
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id[4]);    //使用贴图纹理
	glColor3f(0.7, 0.7, 0.7);
	//glTranslatef((float)x + 0.5, (float)y + 0.5, 0.0f);
	glBegin(GL_QUADS);  //启用四边形带绘制模式绘制
	glTexCoord2f(0.0f, 0.0f);glVertex3f(-30.0f, 30.0f, 1.0f);
	glTexCoord2f(1.0f, 0.0f);glVertex3f(30.0f, 30.0f, 1.0f);
	glTexCoord2f(1.0f, 1.0f);glVertex3f(30.0f, -30.0f, 1.0f);
	glTexCoord2f(0.0f, 1.0f);glVertex3f(-30.0f, -30.0f, 1.0f);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	return    1;
}

void Maze::init()
{
	BuildTexture("images/wall.jpg", texture_id[0]);   //载入并标记墙纹理	
	BuildTexture("images/floor.jpg", texture_id[1]);  //载入并标记地面纹理
	BuildTexture("images/exit.jpg", texture_id[2]);  //载入并标记出口
	BuildTexture("images/walls.jpg", texture_id[3]);
	BuildTexture("images/sky.jpg", texture_id[4]);
	BuildTexture("images/fog.jpg", texture_id[5]);
	//glEnable(GL_CULL_FACE);                        //启用裁剪
	//glCullFace(GL_BACK);                           //背面裁剪(背面不可见)
	//glShadeModel(GL_FLAT);
	//glEnable(GL_AUTO_NORMAL);
	//glEnable(GL_TEXTURE_2D);
	calcDir();
	map = map1;
	MAP_SIZE = 16;
}

void Maze::Normal_display(void)
{
	if (px>(MAP_SIZE - 1) && px<MAP_SIZE && py>(MAP_SIZE - 3) && py<(MAP_SIZE - 2))//      出口检测
	{
		MessageBox(NULL, TEXT("胜利！"), TEXT("提示"), MB_OK);
		px = PX_START;
		py = PY_START;
		angle = ANGLE_START;
		calcDir();
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	drawMap(false);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);
	gluPerspective(90, 1.0, 0.01, 16.0);
	if (globalView == 0)
		gluLookAt(px, py, 0.0f, px + dir_x, py + dir_y, 0.0f, 0.0f, 0.0f, 1.0f);
	else
		gluLookAt(px, py, 7.0f, px, py, 0.0f, 0.0f, 1.0f, 0.0);
	if (px > 1 && px < 2 && py>11 && py < 12)
	{
		flag = 1;
	}
	if (flag)
	{
		GLfloat fogColor1[] = { 0.5,0.5,0.5,0.0 };//灰色雾
		glFogfv(GL_FOG_COLOR, fogColor1);   //fog’s color
		glFogf(GL_FOG_START, 3.0f);  // how far to start
		glFogf(GL_FOG_END, 10.0f); //how far to end
		glFogi(GL_FOG_MODE, GL_EXP);  //which mode
		glFogf(GL_FOG_DENSITY, 1.1f);
		glEnable(GL_FOG);
	}
	else 
		glDisable(GL_FOG);
}

void Maze::AR_display(void)
{
	//glClearColor(1.0, 1.0, 1.0, 1.0);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);            // 清空颜色数据和深度数据

	if (px>(MAP_SIZE - 1) && px<MAP_SIZE && py>(MAP_SIZE - 3) && py<(MAP_SIZE - 2))//      出口检测
	{
		MessageBox(NULL, TEXT("胜利！"), TEXT("提示"), MB_OK);
		px = PX_START;
		py = PY_START;
		angle = ANGLE_START;
		calcDir();
	}

	/*glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();*/
	drawMap(true);

	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//glEnable(GL_DEPTH_TEST);
	//gluPerspective(90, 1.0, 0.01, 16.0);
	//if (globalView == 0)
	//	gluLookAt(px, py, 0.0f, px + dir_x, py + dir_y, 0.0f, 0.0f, 0.0f, 1.0f);
	//else
	//	gluLookAt(px, py, 7.0f, px, py, 0.0f, 0.0f, 1.0f, 0.0);

	//glutSwapBuffers();
}

void Maze::Menu(int data)
{
	switch (data)
	{
	case    1:    map = map1;    MAP_SIZE = 16;    break;
	case    2:    map = map2;    MAP_SIZE = 25;    break;
	case    3:    flag = (flag + 1) % 2; break;
	default:    break;
	}
	angle = ANGLE_START;
	px = PX_START;
	py = PY_START;
	calcDir();
}

void Maze::aheadAndBack(float pos)
{
	float    step_x = pos * UNITSTEP * dir_x;
	float    step_y = pos * UNITSTEP * dir_y;
	if (wall(px + step_x, py) == 0)
		px += step_x;
	if (wall(px, py + step_y) == 0)
		py += step_y;
}

void Maze::keyboard()
{
		globalView = (globalView + 1) % 2;
}

void Maze::special(int key, int x, int y)
{
	switch (key)
	{
	case    GLUT_KEY_UP:    aheadAndBack(1.0);        break;
	case    GLUT_KEY_DOWN:    aheadAndBack(-1.0);    break;
	case    GLUT_KEY_LEFT:    angle += 45;    if (angle > 180)        angle = -135;     calcDir();    break;
	case    GLUT_KEY_RIGHT:angle -= 45;    if (angle < -180)    angle = 135;        calcDir();      break;
	default:    break;
	}
}

void Maze::idle(void)
{
	//Sleep(10);
	glutPostRedisplay();
}

void Maze::reshape(GLsizei w, GLsizei h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

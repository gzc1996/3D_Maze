#ifndef GL_TEXTURE_LOADER
#define GL_TEXTURE_LOADER

typedef struct													// ����һ���ṹ��
{
	GLubyte	*imageData;											// ͼ������ (���32bit)
	GLuint	bpp;												// ÿһ���ص�ͼ����ɫ���
	GLuint	width;												// ͼ����
	GLuint	height;												// ͼ��߶�
	GLuint	texID;												// ����ID
} TextureTga;



// ����TGA,BMP,JPG,GIF���ļ�
bool BuildTexture(char *filename,GLuint &texid);

#endif
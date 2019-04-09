#ifndef _3DS_H
#define _3DS_H

#include <windows.h>
#include <gl/gl.h>
#include <math.h>
#include <vector>
using namespace std;

#define  BYTE unsigned char
#define  WORD unsigned short 
#define  UINT unsigned int

// 定义3ds的一些有使用到的ChunkID
// 根Chunk,在每个文件的开始位置
const WORD PRIMARY=0x4D4D;
	const WORD PRIM_EDIT = 0x3D3D;					// ChunkID：3ds模型
		const WORD EDIT_MAT = 0xAFFF;				// ChunkID：材质
			const WORD MAT_NAME = 0xA000;			// ChunkID：材质名称
			const WORD MAT_AMB  = 0xA010;			// ChunkID：材质环境光属性（没使用到）
			const WORD MAT_DIF  = 0xA020;			// ChunkID：材质漫反射属性
			const WORD MAT_SPE  = 0xA030;			// ChunkID：材质镜面反射属性（没使用到）
			const WORD MAT_MAP =  0xA200;			// ChunkID：材质的纹理
				const WORD MAP_NAME = 0xA300;		// ChunkID：纹理的名称

		const WORD EDIT_OBJECT = 0x4000;			// ChunkID：3ds对象的面、点等信息
			const WORD OBJECT_INFO = 0x4100;		// ChunkID：对象的主要信息
				const WORD OBJECT_VERTEX = 0x4110;	// ChunkID：物体的顶点信息
				const WORD OBJECT_FACET = 0x4120;	// ChunkID：物体的面信息
					const WORD FACET_MAT = 0x4130;	// ChunkID：物体具有的材质
					const WORD FACET_SMOOTH =0x4150;// ChunkID：面光滑信息（没使用到）
				const WORD OBJECT_UV = 0x4140;		// ChunkID：纹理坐标信息
				const WORD OBJECT_LOCAL = 0x4160;
	const WORD PRIM_KEY=0xB000;						// ChunkID：所有的关键帧信息（没使用到）

const WORD COLOR_BYTE=0x0011;						// ChunkID：颜色

// 保存字符串
typedef struct
{
	char string[128];
} STRING;

// 2维向量
struct Vector2
{
	float x, y;
};

// 3维向量
struct _Vector3
{
public:
	// 向量初始化
	_Vector3() {}
	_Vector3(float X, float Y, float Z)	{ x = X; y = Y; z = Z; }
	// 向量相加
	_Vector3 operator+(_Vector3 vVector)	{ return _Vector3(vVector.x + x, vVector.y + y, vVector.z + z); }
	// 向量相加
	_Vector3 operator-(_Vector3 vVector)	{ return _Vector3(x - vVector.x, y - vVector.y, z - vVector.z); }
	// 向量点乘
	_Vector3 operator*(float num)		{ return _Vector3(x * num, y * num, z * num); }
	_Vector3 operator/(float num)		{ return _Vector3(x / num, y / num, z / num); }

	float x, y, z;
};

// 保存Chunk信息
typedef struct 
{
	WORD ID;										// Chunk的ID
	UINT length;									// Chunk的长度
} tChunk;

// 保存面信息：顶点与纹理坐标的索引值。
typedef struct
{
	int vertIndex[3];								// 3个顶点的索引值
	int	matID;										// 该面对应的材质ID
} tFace;

// 保存材质信息。
typedef struct
{
	STRING  matName;								// 材质的名称
	STRING  mapName;								// 纹理的名称（bmp，jpg等的文件名）
	BYTE	color[3];								// 材质颜色
	UINT	texureId;								// 纹理的ID（指向载入的纹理）
	bool	isTexMat;								// 该材质是不是包含有纹理
} tMaterial;

// 保存单个3ds对象
typedef struct
{
	int  numOfVerts;								// 该对象顶点的个数
	int  numOfFaces;								// 该对象面的个数
	int  numTexVertex;								// 该对象纹理坐标的个数
	STRING	 objName;								// 保存对象的名称
	_Vector3  *pVerts;								// 保存顶点坐标
	_Vector3  *pNormals;								// 保存点的法线量
	Vector2  *pTexVerts;							// 保存纹理坐标
	tFace	 *pFaces;								// 保存面信息（顶点索引及面对应的材质）
} t3DObject;

// 保存整个3ds模型
typedef struct
{
	int numOfObjects;								// 3ds对象的个数
	int numOfMaterials;								// 3ds材质的个数
	vector<tMaterial> pMaterials;					// 保存3ds材质
	vector<t3DObject> pObject;						// 保存3ds对象
} t3DModel;

// C3DSModel类
class C3DSModel
{
public:
	C3DSModel();
	~C3DSModel();
	BOOL Load(char *);								// 载入3ds文件
	void Render(void);								// 绘制3ds模型
	void Release(void);								// 释放3ds模型资源
private:
	void ReadChunk(tChunk *);						// 读取3ds的一个Chunk信息（Chunk的ID及长度）

	UINT ReadPrimary(UINT n);						// 读取3ds文件主要Chunk
		UINT ReadEdit(UINT n);						// 读取3ds物体主编辑Chunk
			UINT ReadObject(UINT n);						// 读取3ds对象
				UINT ReadObjectInfo(t3DObject *,UINT n);	// 读取3ds对象信息
					UINT ReadFacetInfo(t3DObject *,UINT n);	// 读取面信息
		UINT ReadMaterial(UINT n);					// 读取材质
			UINT ReadMatDif(tMaterial *, UINT n);	// 读取材质的漫反射属性
			UINT ReadMatMap(tMaterial *, UINT n);	// 读取材质的纹理
		UINT ReadKeyframe(UINT n);					// 读取帧信息（未使用）

	BYTE ReadByte(void);							// 从文件中读取1个字节
	WORD ReadWord(void);							// 从文件中读取2个字节
	UINT ReadUint(void);							// 从文件中读取4个字节
	float ReadFloat(void);							// 从文件中读取浮点数
	UINT ReadString(STRING *);						// 从文件中读取字符串（返回字符串长度）

	_Vector3 Cross(_Vector3, _Vector3);				// 计算两向量的叉积
	_Vector3 Normalize(_Vector3);						// 向量单位化
	void ComputeNormals(void);						// 计算顶点法线量

private:
	FILE *m_FilePtr;								// 3ds文件指针
	t3DModel m_3DModel;								// 保存3ds模型
};

#endif
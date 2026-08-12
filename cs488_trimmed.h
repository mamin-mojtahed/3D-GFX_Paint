// =======================================
// CS488/688 base code
// (written by Toshiya Hachisuka)
// =======================================
// NOTE:
// code trimmed without implementations
// find the initial base code of cs488.h at https://git.uwaterloo.ca/thachisu/cs488
// please add a fully implemented cs488.h to build the program

#pragma once
#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX


// OpenGL
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>


// image loader and writer
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"


// linear algebra 
#include "linalg.h"
using namespace linalg::aliases;


// animated GIF writer
#include "gif.h"


// misc
#include <iostream>
#include <vector>
#include <cfloat>
#include <algorithm>
using std::min, std::max;


// main window
static GLFWwindow* globalGLFWindow;


// window size and resolution
// (do not make it too large - will be slow!)
constexpr int globalWidth = 512;
constexpr int globalHeight = 384;


// degree and radian
constexpr float PI = 3.14159265358979f;
constexpr float DegToRad = PI / 180.0f;
constexpr float RadToDeg = 180.0f / PI;


// for ray tracing
constexpr float Epsilon = 5e-5f;


// amount the camera moves with a mouse and a keyboard
constexpr float ANGFACT = 0.2f;
float SCLFACT = 0.1f;

// fixed camera parameters
constexpr float globalAspectRatio = float(globalWidth / float(globalHeight));
constexpr float globalFOV = 45.0f; // vertical field of view
constexpr float globalDepthMin = Epsilon; // for rasterization
constexpr float globalDepthMax = 100.0f; // for rasterization
constexpr float globalFilmSize = 0.032f; //for ray tracing
const float globalDistanceToFilm = globalFilmSize / (2.0f * tan(globalFOV * DegToRad * 0.5f)); // for ray tracing


// particle system related
bool globalEnableParticles = false;
constexpr float deltaT = 0.002f;
static float3 globalGravity = float3(0.0f, -9.8f, 0.0f);
constexpr float globalGravitationalExpr = 2e-3f;
constexpr int globalNumParticles = 100;


// dynamic camera parameters
float3 globalEye = float3(0.0f, 0.0f, 1.5f);
float3 globalLookat = float3(0.0f, 0.0f, 0.0f);
float3 globalUp = normalize(float3(0.0f, 1.0f, 0.0f));
float3 globalViewDir; // should always be normalize(globalLookat - globalEye)
float3 globalRight; // should always be normalize(cross(globalViewDir, globalUp));
bool globalShowRaytraceProgress = false; // for ray tracing


// mouse event
static bool mouseLeftPressed;
static double m_mouseX = 0.0;
static double m_mouseY = 0.0;


// rendering algorithm
enum enumRenderType {
	RENDER_RASTERIZE,
	RENDER_RAYTRACE,
	RENDER_IMAGE,
};
enumRenderType globalRenderType = RENDER_IMAGE;
int globalFrameCount = 0;
static bool globalRecording = false;
static GifWriter globalGIFfile;
constexpr int globalGIFdelay = 1;
static bool rendersVertices = false;
static bool rendersLightingAndShadow = true;
static bool globalEnableEnvMap = false;
static bool loadedEnvMap = false;
constexpr int globalMaxRecursionLevel = 10; // for recursive raytracing limit
constexpr float globalCollisionBox = 0.5f; // for particle collision against a box
constexpr float3 globalSphereCenter = float3(0.0f); // for particle constraints on a spherical surface
constexpr float globalSphereRadius = 0.5f; // for particle constraints on a spherical surface
static int globalConstraintType = 2; // 0: none, 1: box, 2: sphere, 3: gravitational field

// ---PROJECT---
// Project Engine Hooks & Properties (populated by project.h)
static int globalOptState = 0; // 0 = Full, 1 = No Collision, 2 = No Raytracing, 3 = No Optimization
static int globalNPRLevel = 0; // 0 = OFF, 1 = Standard (4 tones), 2 = Medium (8 tones), 3 = Fine (16 tones)

static bool (*projectKeyCallback)(int) = nullptr;
static bool (*projectMouseButtonCallback)(int, int, int) = nullptr;
static bool (*projectCursorPosCallback)(double, double) = nullptr;
static bool (*projectScrollCallback)(double, double) = nullptr;
static void (*projectSimulationStep)() = nullptr;
// ---PROJECT---

// OpenGL related data (do not modify it if it is working)
static GLuint GLFrameBufferTexture;
static GLuint FSDraw;
static const std::string FSDrawSource = R"(
	#version 120

	uniform sampler2D input_tex;
	uniform vec4 BufInfo;

	void main()
	{
		gl_FragColor = texture2D(input_tex, gl_FragCoord.st * BufInfo.zw);
	}
)";
static const char* PFSDrawSource = FSDrawSource.c_str();



// fast random number generator based pcg32_fast
#include <stdint.h>
namespace PCG32 {
	static uint64_t mcg_state = 0xcafef00dd15ea5e5u;	// must be odd
	static uint64_t const multiplier = 6364136223846793005u;
	uint32_t pcg32_fast(void) {
		uint64_t x = mcg_state;
		const unsigned count = (unsigned)(x >> 61);
		mcg_state = x * multiplier;
		x ^= x >> 22;
		return (uint32_t)(x >> (22 + count));
	}
	float rand() {
		return float(double(pcg32_fast()) / 4294967296.0);
	}
}



// image with a depth buffer
// (depth buffer is not always needed, but hey, we have a few GB of memory, so it won't be an issue...)
class Image {
public:
	std::vector<float3> pixels;
	std::vector<float> depths;
	int width = 0, height = 0;

	static float toneMapping(const float r);

	static float gammaCorrection(const float r, const float gamma = 1.0f);

	void resize(const int newWdith, const int newHeight);

	void clear();

	Image(int _width = 0, int _height = 0);

	bool valid(const int i, const int j) const;

	float& depth(const int i, const int j);

	float3& pixel(const int i, const int j);

	void load(const char* fileName);
	void save(const char* fileName);
};

// main image buffer to be displayed
Image FrameBuffer(globalWidth, globalHeight);

// environment map for A2
Image EnvMap;
static int envMapCX;
static int envMapCY;
void preloadEnvMap(const char* path);

// you may want to use the following later for progressive ray tracing
Image AccumulationBuffer(globalWidth, globalHeight);
unsigned int sampleCount = 0;



// keyboard events (you do not need to modify it unless you want to)
void keyFunc(GLFWwindow* window, int key, int scancode, int action, int mods);



// mouse button events (you do not need to modify it unless you want to)
void mouseButtonFunc(GLFWwindow* window, int button, int action, int mods);



// mouse button events (you do not need to modify it unless you want to)
void cursorPosFunc(GLFWwindow* window, double mouse_x, double mouse_y);

void scrollFunc(GLFWwindow* window, double xoffset, double yoffset);




class PointLightSource {
public:
	float3 position, wattage;
};



class Ray {
public:
	float3 o, d;
	Ray() : o(), d(float3(0.0f, 0.0f, 1.0f)) {}
	Ray(const float3& o, const float3& d) : o(o), d(d) {}
};



// uber material
// "type" will tell the actual type
// ====== implement it in A2, if you want ======
enum enumMaterialType {
	MAT_LAMBERTIAN,
	MAT_METAL,
	MAT_GLASS
};
class Material {
public:
	std::string name;

	enumMaterialType type = MAT_LAMBERTIAN;
	float eta = 1.0f;
	float glossiness = 1.0f;

	float3 Ka = float3(0.0f);
	float3 Kd = float3(0.9f);
	float3 Ks = float3(0.0f);
	float3 Kt = float3(1.0f); // assumption based on Piazza post
	float Ns = 0.0;

	// support 8-bit texture
	bool isTextured = false;
	unsigned char* texture = nullptr;
	int textureWidth = 0;
	int textureHeight = 0;

	Material() {};
	virtual ~Material() {};

	void setReflectance(const float3& c);

	float3 fetchTexture(const float2& tex) const;

	float3 BRDF(const float3& wi, const float3& wo, const float3& n) const;

	float PDF(const float3& wGiven, const float3& wSample) const;

	float3 sampler(const float3& wGiven, float& pdfValue) const;
};





class HitInfo {
public:
	float t; // distance
	float3 P; // location
	float3 N; // shading normal vector
	float2 T; // texture coordinate
	const Material* material; // const pointer to the material of the intersected object
};



// axis-aligned bounding box
class AABB {
private:
	float3 minp, maxp, size;

public:
	// changed to const for later use in BVH
	float3 get_minp() const { return minp; };
	float3 get_maxp() const { return maxp; };
	float3 get_size() const { return size; };


	AABB();

	void reset();

	int getLargestAxis();

	void fit(const float3& v);

	float area() const;


	bool intersect(HitInfo& minHit, const Ray& ray) const;
};




// triangle
struct Triangle {
	float3 positions[3];
	float3 normals[3];
	float2 texcoords[3];
	int idMaterial = 0;
	AABB bbox;
	float3 center;
};

// line equation test
float lineEquationTest(const float2& line_pa, const float2& line_pb, const float2& p);

// inside traingle test
bool insideTriangle(const float3 &test, const float2& p);

// triangle mesh
static float3 shade(const HitInfo& hit, const float3& viewDir, const int level = 0);
class TriangleMesh {
public:
	std::vector<Triangle> triangles;
	std::vector<Material> materials;
	AABB bbox;

	// for managing visibility and BVH
	bool visible = true;
	int scene_bvh_index = -1;

	void transform(const float4x4& m);

	// extra transformation helper functions

	void translate(const float3& t);

	void scale(const float3& s);

	void rotate(const float3& r_deg);

	// rotate around an arbitrary axis
	void rotateCustom(const float3& og_axis, const float& angle_deg);

	void rasterizeTriangle(const Triangle& tri, const float4x4& plm) const;


	bool raytraceTriangle(HitInfo& result, const Ray& ray, const Triangle& tri, float tMin, float tMax) const;


	// some precalculation for bounding boxes (you do not need to change it)
	void preCalc();


	// load .obj file (you do not need to modify it unless you want to change something)
	bool load(const char* filename, const float4x4& ctm = linalg::identity);

	~TriangleMesh();


	bool bruteforceIntersect(HitInfo& result, const Ray& ray, float tMin = 0.0f, float tMax = FLT_MAX);

	void createSingleTriangle();


	// === you do not need to modify the followings in this class ===
	void loadTexture(const char* fname, const int i);

	// put loadTexture() out of private to let custom textures be loaded in project
private:

	std::string GetBaseDir(const std::string& filepath);
	std::string base_dir;

	void LoadMTL(const std::string fileName);

	void ParseOBJ(const char* fileName, int& nVertices, float** vertices, float** normals, float** texcoords, int& nIndices, int** indices, int** materialids);
};


// BVH node (for A2 extra)
class BVHNode {
public:
	bool isLeaf = false;
	int idLeft = -1, idRight = -1;
	int triListNum = 0;
	int* triList = nullptr;
	AABB bbox;

	BVHNode() = default;
	~BVHNode();

	// Disable copy semantics to prevent shallow copies
	BVHNode(const BVHNode&) = delete;
	BVHNode& operator=(const BVHNode&) = delete;

	// Enable move semantics for vector reallocations
	BVHNode(BVHNode&& other) noexcept;

	BVHNode& operator=(BVHNode&& other) noexcept;
};

// ====== implement it in A2 extra ======
// fill in the missing parts
class BVH {
public:
	const TriangleMesh* triangleMesh = nullptr;
	BVHNode* node = nullptr;

	const float costBBox = 1.0f;
	const float costTri = 1.0f;

	int leafNum = 0;
	int nodeNum = 0;

	BVH() {}
	~BVH();

	void clear();

	// Disable copy constructors to prevent shallow-copy dangling pointers
	BVH(const BVH&) = delete;
	BVH& operator=(const BVH&) = delete;

	// Enable move semantics
	BVH(BVH&& other) noexcept;

	BVH& operator=(BVH&& other) noexcept;

	// you may keep this part as-is
	// *: moved code into class
	void build(const TriangleMesh* mesh);

	bool intersect(HitInfo& result, const Ray& ray, float tMin = 0.0f, float tMax = FLT_MAX) const;
	bool traverse(HitInfo& result, const Ray& ray, int node_id, float tMin, float tMax) const;

private:
	void sortAxis(int* obj_index, const char axis, const int li, const int ri) const;
	int splitBVH(int* obj_index, const int obj_num, const AABB& bbox);
};


// sort bounding boxes (in case you want to build SAH-BVH)
// void BVH::sortAxis(int* obj_index, const char axis, const int li, const int ri) const;


#define SAHBVH // use this in once you have SAH-BVH
// int BVH::splitBVH(int* obj_index, const int obj_num, const AABB& bbox);
	// ====== exntend it in A2 extra ======
#ifndef SAHBVH
	// simple bvh
#else
	// implelement SAH-BVH here
#endif


// **part of original functions**
// Optimized BVH ray traversal with dynamic minHit distance pruning
// bool BVH::traverse(HitInfo& minHit, const Ray& ray, int node_id, float tMin, float tMax) const;









// ====== implement it in A3 ======
// fill in the missing parts
class Particle {
public:
	float3 position = float3(0.0f);
	float3 velocity = float3(0.0f);
	float3 prevPosition = position;

	// better distinct force system
	float3 accForce = float3(0.0f);
	bool gravityAffects = true;

	// project fields; for painting
	bool wasColliding = false;
	float2 prevHitUV = float2(0.0f, 0.0f);

	void reset();

	void step();
};


class ParticleSystem {
public:
	std::vector<Particle> particles;
	TriangleMesh particlesMesh;
	TriangleMesh sphere;
	const char* sphereMeshFilePath = 0;
	float sphereSize = 0.0f;
	ParticleSystem() {};

	void updateMesh();

	void initialize();

	// extend this in A3
	// add some particle-particle interaction here
	// spherical particles can be implemented here
	void step();
};
static ParticleSystem globalParticleSystem;








// scene definition
class Scene {
public:
	std::vector<TriangleMesh*> objects;
	std::vector<PointLightSource*> pointLightSources;
	std::vector<BVH> bvhs;

	void addObject(TriangleMesh* pObj);
	void addLight(PointLightSource* pObj);

	void preCalc();

	// ray-scene intersection
	// accelerated by BVH
	bool intersect(HitInfo& minHit, const Ray& ray, float tMin = 0.0f, float tMax = FLT_MAX) const;

	// camera -> screen matrix (given to you for A1)
	float4x4 perspectiveMatrix(float fovy, float aspect, float zNear, float zFar) const;

	// model -> camera matrix (given to you for A1)
	float4x4 lookatMatrix(const float3& _eye, const float3& _center, const float3& _up) const;

	// Tests if an AABB intersects the camera frustum using the projection-lookat matrix
	bool intersectFrustum(const AABB& bbox, const float4x4& plm) const;

	// BVH recursion to ignore hidden geometry before rasterization
	void rasterizeBVHNode(const TriangleMesh* mesh, const BVHNode* nodeArr, int node_id, const float4x4& plm) const;

	// rasterizer
	// accelerated by BVH
	void Rasterize() const;

	// eye ray generation (given to you for A2)
	Ray eyeRay(int x, int y) const;

	// ray tracing (you probably don't need to change it in A2)
	void Raytrace() const;

};
static Scene globalScene;

float3 reflect(const float3& I, const float3& N);

float3 refract(const float3& I, const float3& N, float& eta, bool& internalReflect);

// ====== implement it in A2 ======
// fill in the missing parts
static float3 shade(const HitInfo& hit, const float3& viewDir, const int level);







// OpenGL initialization (you will not use any OpenGL/Vulkan/DirectX... APIs to render 3D objects!)
// you probably do not need to modify this in A0 to A3.
class OpenGLInit {
public:
	OpenGLInit();

	virtual ~OpenGLInit();
};



// main window
// you probably do not need to modify this in A0 to A3.
class CS488Window {
public:
	// put this first to make sure that the glInit's constructor is called before the one for CS488Window
	OpenGLInit glInit;

	CS488Window() {}
	virtual ~CS488Window() {}

	void(*process)() = NULL;

	void start() const;
};

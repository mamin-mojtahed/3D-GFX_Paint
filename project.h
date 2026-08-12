// CS488 Final Project - Amin Mojtahed
#pragma once
#include <queue>
#include "cs488.h"

const char* squareObjPath = "../media/project-media/square.obj";
const char* toolSolidBGPath = "../media/project-media/JSPaint-tools-solidBG.png";
const char* toolGridBGPath = "../media/project-media/JSPaint-tools-gridBG.png";

const char* colorPalettePath = "../media/project-media/colour-pallete.jpg";
static TriangleMesh colorPalette;

const char* toolObjPath[] = {
    "../media/project-media/eraser-edited.obj",
    "../media/project-media/bucket-edited.obj",
    "../media/project-media/ink-edited.obj",
    "../media/project-media/magnifier-edited.obj",
    "../media/project-media/pencil-edited.obj",
    "../media/project-media/brush-edited-bald.obj",
    "../media/project-media/airspray-edited.obj",
    "../media/project-media/text-edited.obj"
};

static TriangleMesh toolbar[8];
static TriangleMesh canvas;
static TriangleMesh curToolObj;
static int curToolIndex = 4;
const int totalIconsInAtlas = 16;
const int startLogoIndex = 2;

// project-specific dynamic camera and states
static float toolZDepth = 0.95f; // distance of the tool from the camera origin
static bool mouseRightPressed = false;
static int globalCollisionMode = 0; // 0 = All Tool Vertices & Faces, 1 = Virtual Particles, 2 = No Tool and Collision of Mouse Coordinate on Click

// rigid body variables
static Particle toolAnchor; // Particle A of tool; controlled by mouse
static Particle toolTip;    // Particle B of tool; flexible to move
static float3 baseTipPos = float3(0.0f, 0.0f, 1.05f);
static float3 baseAnchorPos = float3(0.0f, 0.0f, 1.55f);
static float toolLength = length(baseAnchorPos - baseTipPos);
float3 baseToolDir = normalize(baseAnchorPos - baseTipPos);
float3 curToolDir = baseToolDir;
const float globalDampingFactor = 0.72f;

// deformable spring/bend variables
static float globalSpringStiffness = 1000.0f;

// debug
static bool debugEnabled = false;
TriangleMesh tipDebug;
TriangleMesh anchorDebug;

// declair for hair
// Hair strand configuration
const char* hairStrandObjPath = "../media/project-media/hair-strand.obj";
// Parametric Hair Grid Dimensions
const int GRID_COLS = 10;
const int GRID_ROWS = 2;
const int NUM_HAIRS = GRID_COLS * GRID_ROWS; // Automatically computes total hairs (20)
const int SEGMENTS_PER_HAIR = 3; // 3 rigid segments per strand (4 particles)
// Bounding box bounds on bald brush -Z surface
const float HAIR_MIN_X = -0.07f;
const float HAIR_MAX_X =  0.07f;
const float HAIR_MIN_Y = -0.01f;
const float HAIR_MAX_Y =  0.008f;
const float HAIR_SURFACE_Z = 0.129f;
void updateHairAnchors();
void resetAllHairParticles();
void toggleHairVisibility(bool visible);

// Airspray cone configuration
const int NUM_SPRAY_SPRINGS = 21; // Match 21 base vertices of the cone OBJ
static float3 sprayLocalTipOffsets[NUM_SPRAY_SPRINGS];
const float SPRAY_LENGTH = 0.12f;
const float SPRAY_MAX_RADIUS = 0.06f;
static float3 lastToolTipPos = float3(0.0f);

void updateSprayAnchors();
void resetAllSprayParticles();
void toggleSprayVisibility(bool visible);
void initSprayPhysics();

// declare for collision
bool resolveParticleMeshCollision(Particle& p, const TriangleMesh& mesh);
// Global debug mesh to visualize all vertex samples
static TriangleMesh debugFacesMesh;
static bool debugFacesAdded = false;

// declare for painting
// Active drawing color [R, G, B]
static unsigned char currentColor[3] = {255, 0, 0};

// --- BVH Optimization (Refitting) ---
// Recursively updates bounding boxes for rigid meshes without rebuilding the tree
inline void refitBVH(BVH& bvh) {
    if (!bvh.node || bvh.nodeNum == 0 || !bvh.triangleMesh) return;

    auto refitNode = [&](auto& self, int node_id) -> void {
        if (bvh.node[node_id].isLeaf) {
            bvh.node[node_id].bbox.reset();
            for (int i = 0; i < bvh.node[node_id].triListNum; i++) {
                const Triangle& tri = bvh.triangleMesh->triangles[bvh.node[node_id].triList[i]];
                bvh.node[node_id].bbox.fit(tri.positions[0]);
                bvh.node[node_id].bbox.fit(tri.positions[1]);
                bvh.node[node_id].bbox.fit(tri.positions[2]);
            }
        } else {
            int idL = bvh.node[node_id].idLeft;
            int idR = bvh.node[node_id].idRight;
            bvh.node[node_id].bbox.reset();
            if (idL >= 0) {
                self(self, idL);
                bvh.node[node_id].bbox.fit(bvh.node[idL].bbox.get_minp());
                bvh.node[node_id].bbox.fit(bvh.node[idL].bbox.get_maxp());
            }
            if (idR >= 0) {
                self(self, idR);
                bvh.node[node_id].bbox.fit(bvh.node[idR].bbox.get_minp());
                bvh.node[node_id].bbox.fit(bvh.node[idR].bbox.get_maxp());
            }
        }
    };
    refitNode(refitNode, 0);
}

// Struct for rigid body physics simulation
struct RigidBody {
    static std::vector<RigidBody*>& getAllBodies() {
        static std::vector<RigidBody*> allBodies;
        return allBodies;
    }

    Particle* anchor = nullptr;
    Particle* tip = nullptr;
    TriangleMesh* bodyMesh = nullptr; // The mesh representing the rigid body
    std::vector<Triangle> baseTriangles; // Store base mesh for absolute transformation

    float3 baseTipPos = float3(0.0f, 0.0f, 0.0f);
    float3 baseAnchorPos = float3(0.0f, 0.0f, 0.5f);
    float toolLength = length(baseAnchorPos - baseTipPos);
    float3 baseToolDir = normalize(baseAnchorPos - baseTipPos);
    float3 curToolDir = baseToolDir;

    bool isEnabled = true;
    bool inSceneDebug = false;

    // Per-instance debug meshes & base geometry caches
    TriangleMesh tipDebug;
    TriangleMesh anchorDebug;
    std::vector<Triangle> baseTipDebugTriangles;
    std::vector<Triangle> baseAnchorDebugTriangles;

    std::function<void(Particle&, TriangleMesh&)> anchorStep = nullptr;

    RigidBody() {
        registerSelf();
    }

    void registerSelf() {
        auto& bodies = getAllBodies();
        if (std::find(bodies.begin(), bodies.end(), this) == bodies.end()) {
            bodies.push_back(this);
        }
    }

    void setup(float3 tipPos, float3 anchorPos, TriangleMesh* mesh = nullptr) {
        baseTipPos = tipPos;
        baseAnchorPos = anchorPos;
        toolLength = length(baseAnchorPos - baseTipPos);
        baseToolDir = normalize(baseAnchorPos - baseTipPos);
        curToolDir = baseToolDir;
        bodyMesh = mesh;
    }

    void initDebug(float debugScaleMultiplier = 1.5f) {
        if (!anchor || !tip) return;

        bool addedTip = false;
        if (tipDebug.triangles.empty()) {
            tipDebug.triangles.resize(1); // Single triangle
            if (tipDebug.materials.empty()) tipDebug.materials.resize(1);
            tipDebug.materials[0].Kd = float3(0.0f, 1.0f, 0.0f);
            addedTip = true;
        }

        bool addedAnchor = false;
        if (anchorDebug.triangles.empty()) {
            anchorDebug.triangles.resize(1); // Single triangle
            if (anchorDebug.materials.empty()) anchorDebug.materials.resize(1);
            anchorDebug.materials[0].Kd = float3(1.0f, 0.0f, 0.0f);
            addedAnchor = true;
        }

        updateDebugMesh();

        // Add debug particles dynamically without erasing later
        if (addedTip) { globalScene.addObject(&tipDebug); tipDebug.visible = false; }
        if (addedAnchor) { globalScene.addObject(&anchorDebug); anchorDebug.visible = false; }
    }

    void init() {
        if (!anchor || !tip) return;

        tip->position = baseTipPos;
        tip->velocity = float3(0.0f);
        tip->prevPosition = tip->position;

        anchor->position = baseAnchorPos;
        anchor->velocity = float3(0.0f);
        anchor->prevPosition = anchor->position;

        curToolDir = baseToolDir;
        toolLength = length(anchor->position - tip->position);

        if (bodyMesh) {
            baseTriangles = bodyMesh->triangles;
        }

        updateMeshTransform();
        initDebug();
    }

    void updateDebugMesh() {
        if (!anchor || !tip) return;
        
        const float particleSize = 0.025f; // Small triangle size

        // Calculate simple camera-facing billboard triangles
        if (!tipDebug.triangles.empty()) {
            tipDebug.triangles[0].positions[0] = tip->position;
            tipDebug.triangles[0].positions[1] = tip->position + particleSize * globalUp;
            tipDebug.triangles[0].positions[2] = tip->position + particleSize * globalRight;
            tipDebug.triangles[0].normals[0] = -globalViewDir;
            tipDebug.triangles[0].normals[1] = -globalViewDir;
            tipDebug.triangles[0].normals[2] = -globalViewDir;
            
            tipDebug.preCalc();
            int bIdx = tipDebug.scene_bvh_index;
            if (bIdx >= 0 && bIdx < globalScene.bvhs.size()) {
                if (globalScene.bvhs[bIdx].nodeNum == 0) globalScene.bvhs[bIdx].build(&tipDebug);
                else refitBVH(globalScene.bvhs[bIdx]);
            }
        }
        
        if (!anchorDebug.triangles.empty()) {
            anchorDebug.triangles[0].positions[0] = anchor->position;
            anchorDebug.triangles[0].positions[1] = anchor->position + particleSize * globalUp;
            anchorDebug.triangles[0].positions[2] = anchor->position + particleSize * globalRight;
            anchorDebug.triangles[0].normals[0] = -globalViewDir;
            anchorDebug.triangles[0].normals[1] = -globalViewDir;
            anchorDebug.triangles[0].normals[2] = -globalViewDir;
            
            anchorDebug.preCalc();
            int bIdx = anchorDebug.scene_bvh_index;
            if (bIdx >= 0 && bIdx < globalScene.bvhs.size()) {
                if (globalScene.bvhs[bIdx].nodeNum == 0) globalScene.bvhs[bIdx].build(&anchorDebug);
                else refitBVH(globalScene.bvhs[bIdx]);
            }
        }
    }

    void simulatePhysics() {
        if (!anchor || !tip) return;

        if (anchorStep && !mouseRightPressed) anchorStep(*anchor, anchorDebug);
        bool anchorCollided = resolveParticleMeshCollision(*anchor, canvas);

        tip->step();
        bool tipCollided = resolveParticleMeshCollision(*tip, canvas);

        if (curToolIndex == 2) {
            for (int i = 0; i < 8; i++) tipCollided = resolveParticleMeshCollision(*tip, toolbar[i]) || tipCollided;
        }

        float3 targetDir = normalize(anchor->position - tip->position);
        tip->position = anchor->position - targetDir * toolLength;
        tipCollided = resolveParticleMeshCollision(*tip, canvas) || tipCollided;

        tip->velocity = (tip->position - tip->prevPosition) / deltaT;
        tip->velocity *= globalDampingFactor;
        tip->prevPosition = tip->position - tip->velocity * deltaT;

        curToolDir = targetDir;
        if (!anchorDebug.materials.empty()) anchorDebug.materials[0].Kd = anchorCollided ? float3(0.0f, 0.0f, 1.0f) : float3(1.0f, 0.0f, 0.0f);
        if (!tipDebug.materials.empty()) tipDebug.materials[0].Kd = tipCollided ? float3(0.0f, 0.0f, 1.0f) : float3(0.0f, 1.0f, 0.0f);

        updateDebugMesh();
    }

    void updateMeshTransform() {
        if (!bodyMesh || baseTriangles.empty() || !anchor || !tip) return;
        bodyMesh->triangles = baseTriangles;

        const float3 targetDir = normalize(anchor->position - tip->position);
        const float3 rotationAxis = cross(baseToolDir, targetDir);
        const float dotVal = std::max(-1.0f, std::min(1.0f, dot(baseToolDir, targetDir)));
        const float angleDeg = acosf(dotVal) * RadToDeg;

        if (angleDeg > Epsilon && length(rotationAxis) > Epsilon) bodyMesh->rotateCustom(rotationAxis, angleDeg);
        bodyMesh->translate(tip->position);
        bodyMesh->preCalc();

        int bIdx = bodyMesh->scene_bvh_index;
        if (bIdx >= 0 && bIdx < globalScene.bvhs.size()) {
            if (globalScene.bvhs[bIdx].nodeNum == 0) globalScene.bvhs[bIdx].build(bodyMesh);
            else refitBVH(globalScene.bvhs[bIdx]);
        }
    }

    void toggleDebug(bool enable) {
        if (enable && isEnabled) {
            if (!inSceneDebug) {
                tipDebug.visible = true;
                anchorDebug.visible = true;
                inSceneDebug = true;
            }
        } else {
            if (inSceneDebug) {
                tipDebug.visible = false;
                anchorDebug.visible = false;
                inSceneDebug = false;
            }
        }
    }

    void setEnable(bool visible) {
        isEnabled = visible;
        toggleDebug(debugEnabled);
    }
};

// Struct for bendable (deformable) bodies for brush hair
struct BendBody : public RigidBody {
    float minDist = 0.0f;

    BendBody() : RigidBody() {}

    void setupBend(Particle* anchor, Particle* tip, float minDist) {
        this->anchor = anchor;
        this->tip = tip;
        this->minDist = minDist;
        this->toolLength = minDist; // Sync length parameter for relative scaling
        initDebug();
    }

    void simulate() {
        if (!anchor || !tip) return;

        float3 dir = tip->position - anchor->position;
        float dist = length(dir);
        if (dist < minDist && dist > Epsilon) {
            float3 targetDir = normalize(dir);
            
            // Scale stiffness by 1/deltaT so k = 500.0f delivers adequate velocity impulse
            float3 springForce = targetDir * (minDist - dist) * (globalSpringStiffness / deltaT);

            // Equal and opposite forces to straighten symmetrically
            tip->accForce = springForce;
        }
    }
};

static RigidBody toolRigidBody;

void toolAnchorStep(Particle& anchor, TriangleMesh& anchorDebug) {
    double mx = (m_mouseX == 0.0 && m_mouseY == 0.0) ? (globalWidth * 0.5) : m_mouseX;
    double my = (m_mouseX == 0.0 && m_mouseY == 0.0) ? (globalHeight * 0.5) : m_mouseY;
    const Ray ray = globalScene.eyeRay(mx, globalHeight - my);
    anchor.prevPosition = anchor.position;
    anchor.position = ray.o + ray.d * toolZDepth;
    resolveParticleMeshCollision(anchor, canvas);
}

void initToolPhysics() {
    toolRigidBody.setup(baseTipPos, baseAnchorPos, &curToolObj);
    toolRigidBody.tip = &toolTip;
    toolRigidBody.anchor = &toolAnchor;
    toolRigidBody.anchorStep = toolAnchorStep;
    toolRigidBody.init();
}

void updateInkToolMeshColor() {
    if (curToolIndex == 2 && !curToolObj.materials.empty()) {
        float3 sampledColor = float3(
            currentColor[0] / 255.0f,
            currentColor[1] / 255.0f,
            currentColor[2] / 255.0f
        );
        for (auto& mat : curToolObj.materials) {
            mat.Kd = sampledColor;
            mat.isTextured = false; // Disable default texture so diffuse color renders directly
        }
    }
}

void selectCurrentToolByIndex(int index) {
    if (index < 0 || index >= 8) return;

    if (index == curToolIndex) {
        printf("Tool %d already selected.\n", index);
        return;
    }

    printf("Tool %d selected.\n", index);
    toolbar[curToolIndex].loadTexture(toolSolidBGPath, 0);
    curToolIndex = index;
    toolbar[index].loadTexture(toolGridBGPath, 0);

    debugFacesMesh.triangles.clear();
    debugFacesMesh.visible = false;

    toggleHairVisibility(curToolIndex == 5);
    toggleSprayVisibility(curToolIndex == 6);

    // Tool object update
    curToolObj.materials.clear();
    curToolObj.triangles.clear();
    curToolObj.load(toolObjPath[curToolIndex]);

    // update ink mesh color exclusively on selection
    if (curToolIndex == 2) {
        updateInkToolMeshColor();
    }

    toolRigidBody.baseTriangles = curToolObj.triangles;
    toolRigidBody.updateMeshTransform();
    
    // Dynamically reset hair positions to current tool tip when brush is selected
    if (curToolIndex == 5) resetAllHairParticles();
    if (curToolIndex == 6) resetAllSprayParticles();

    // Synchronize BVHs across the scene for the new tool mesh
    globalScene.preCalc();
}

bool updateCurrentToolSelection(int mouse_x, int mouse_y) {
    const Ray ray = Ray(globalEye, toolRigidBody.tip->position - globalEye);
    for (int i = 0; i < 8; i++) {
        HitInfo tempHit;
        if (toolbar[i].bbox.intersect(tempHit, ray)) {
            selectCurrentToolByIndex(i);
            return true;
        }
    }
    return false;
}

// Hair strand configuration
static TriangleMesh hairMeshes[NUM_HAIRS][SEGMENTS_PER_HAIR];
static RigidBody hairRigidBodies[NUM_HAIRS][SEGMENTS_PER_HAIR];
static BendBody hairBendBodies[NUM_HAIRS][SEGMENTS_PER_HAIR];
static Particle hairParticles[NUM_HAIRS][SEGMENTS_PER_HAIR + 2];
// Local attachment anchors on bald brush surface in -Z direction (Z ~ 0.129f)
static float3 hairLocalAnchors[NUM_HAIRS];

// Airspray virtual cone configuration
static Particle sprayParticles[NUM_SPRAY_SPRINGS][3]; // [0] Virtual Anchor, [1] Surface Anchor, [2] Dynamic Tip
static BendBody sprayBendBodies[NUM_SPRAY_SPRINGS];

void debugParticlesToggle() {
    debugEnabled = !debugEnabled;

    for (RigidBody* body : RigidBody::getAllBodies()) {
        body->toggleDebug(debugEnabled);
    }

    globalScene.preCalc();
}

void resetAllHairParticles() {
    if (!toolRigidBody.anchor || !toolRigidBody.tip) return;

    const float3 targetDir = normalize(toolRigidBody.anchor->position - toolRigidBody.tip->position);
    const float3 rotationAxis = cross(toolRigidBody.baseToolDir, targetDir);
    const float dotVal = std::max(-1.0f, std::min(1.0f, dot(toolRigidBody.baseToolDir, targetDir)));
    const float angleRad = acosf(dotVal);
    const float segLength = 0.0355f;

    for (int h = 0; h < NUM_HAIRS; h++) {
        for (int p = 0; p < SEGMENTS_PER_HAIR + 2; p++) {
            // Compute local offset relative to brush tip surface
            float3 localPos = hairLocalAnchors[h] + float3(0.0f, 0.0f, (1 - p) * segLength);

            if (angleRad > Epsilon && length(rotationAxis) > Epsilon) {
                float3 u = normalize(rotationAxis);
                float c = cosf(angleRad);
                float s = sinf(angleRad);
                
                localPos = u * dot(u, localPos) + c * cross(cross(u, localPos), u) + s * cross(u, localPos);
            }

            // Dynamically set world position relative to current brush tip position safely above canvas
            hairParticles[h][p].position = toolRigidBody.tip->position + localPos;
            hairParticles[h][p].prevPosition = hairParticles[h][p].position;
            hairParticles[h][p].velocity = float3(0.0f);
            hairParticles[h][p].accForce = float3(0.0f);
            hairParticles[h][p].wasColliding = false;
        }

        for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
            hairRigidBodies[h][s].updateMeshTransform();
        }
    }
}

void initHairPhysics() {
    int idx = 0;
    for (int r = 0; r < GRID_ROWS; r++) {
        float tY = (GRID_ROWS > 1) ? (float)r / (GRID_ROWS - 1) : 0.5f;
        float y = HAIR_MIN_Y + tY * (HAIR_MAX_Y - HAIR_MIN_Y);
        for (int c = 0; c < GRID_COLS; c++) {
            float tX = (GRID_COLS > 1) ? (float)c / (GRID_COLS - 1) : 0.5f;
            float x = HAIR_MIN_X + tX * (HAIR_MAX_X - HAIR_MIN_X);
            hairLocalAnchors[idx++] = float3(x, y, HAIR_SURFACE_Z);
        }
    }

    const float segLength = 0.0355f;
    for (int h = 0; h < NUM_HAIRS; h++) {
        for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
            hairMeshes[h][s].load(hairStrandObjPath);
            if (hairMeshes[h][s].materials.empty()) hairMeshes[h][s].materials.resize(1);

            float3 baseAnchor = float3(0.0f, 0.0f, -s * segLength);
            float3 baseTip = float3(0.0f, 0.0f, -(s + 1) * segLength);

            hairRigidBodies[h][s].setup(baseTip, baseAnchor, &hairMeshes[h][s]);
            hairRigidBodies[h][s].anchor = &hairParticles[h][s + 1];
            hairRigidBodies[h][s].tip    = &hairParticles[h][s + 2];
            
            // Add to scene immediately to lock index, but keep invisible
            globalScene.addObject(&hairMeshes[h][s]);
            hairMeshes[h][s].visible = false;
        }
        for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
            hairBendBodies[h][s].setupBend(&hairParticles[h][s], &hairParticles[h][s + 2], 2.0f * segLength);
            hairRigidBodies[h][s].init();
        }
    }

    resetAllHairParticles();
    toggleHairVisibility(curToolIndex == 5);
}

// Auto-generates planar UV coordinates for meshes lacking OBJ texture coordinates
void generatePlanarUVs(TriangleMesh& mesh) {
    mesh.preCalc(); // Compute AABB bounding box
    float3 minp = mesh.bbox.get_minp();
    float3 maxp = mesh.bbox.get_maxp();
    float3 size = mesh.bbox.get_size();

    // Select the two largest bounding box dimensions to project UVs onto
    int axisX = 0, axisY = 1;
    if (size.z > size.x && size.z > size.y) {
        axisX = 0; axisY = 2; // Project onto X-Z plane
    } else if (size.y > size.x && size.y > size.z) {
        axisX = 0; axisY = 1; // Project onto X-Y plane
    } else {
        axisX = 1; axisY = 2; // Project onto Y-Z plane
    }

    float scaleX = (size[axisX] > Epsilon) ? size[axisX] : 1.0f;
    float scaleY = (size[axisY] > Epsilon) ? size[axisY] : 1.0f;

    // Normalize 3D vertex positions into 2D [0, 1] texture coordinates
    for (auto& tri : mesh.triangles) {
        for (int k = 0; k < 3; k++) {
            float u = (tri.positions[k][axisX] - minp[axisX]) / scaleX;
            float v = (tri.positions[k][axisY] - minp[axisY]) / scaleY;
            tri.texcoords[k] = float2(u, v);
        }
    }
}

void paintCanvas(Particle& p, const HitInfo& hit, int toolIndex, const TriangleMesh& mesh) {
    // --- TOOL 4 (MAGNIFIER) ---
    if (toolIndex == 3) {
        globalLookat = hit.P;
        float3 viewDir = normalize(globalViewDir);
        globalEye = hit.P - viewDir * 1.0f;
        globalViewDir = normalize(globalLookat - globalEye);
        globalRight = normalize(cross(globalViewDir, globalUp));
        toolZDepth = 0.5f;

        const Ray ray = globalScene.eyeRay(m_mouseX, globalHeight - m_mouseY);
        toolAnchor.position = ray.o + ray.d * toolZDepth;
        toolAnchor.prevPosition = toolAnchor.position;

        p.prevHitUV = hit.T;
        p.wasColliding = true;
        return;
    }

    // --- TOOL 3 (INK / EYEDROPPER) ---
    if (toolIndex == 2) {
        if (hit.material) {
            if (hit.material->isTextured && hit.material->texture && 
                hit.material->textureWidth > 0 && hit.material->textureHeight > 0) {
                int texW = hit.material->textureWidth;
                int texH = hit.material->textureHeight;
                int px = int(hit.T.x * texW) % texW;
                int py = int(hit.T.y * texH) % texH;
                if (px < 0) px += texW;
                if (py < 0) py += texH;

                int idx = (py * texW + px) * 3;
                currentColor[0] = hit.material->texture[idx + 0];
                currentColor[1] = hit.material->texture[idx + 1];
                currentColor[2] = hit.material->texture[idx + 2];
            } else {
                currentColor[0] = (unsigned char)(std::min(1.0f, std::max(0.0f, hit.material->Kd.x)) * 255.0f);
                currentColor[1] = (unsigned char)(std::min(1.0f, std::max(0.0f, hit.material->Kd.y)) * 255.0f);
                currentColor[2] = (unsigned char)(std::min(1.0f, std::max(0.0f, hit.material->Kd.z)) * 255.0f);
            }
            updateInkToolMeshColor();
        }
        p.prevHitUV = hit.T;
        p.wasColliding = true;
        return;
    }

    // Painting tools only operate on the canvas
    if (&mesh != &canvas || canvas.materials.empty() || !canvas.materials[0].isTextured) return;

    Material& mat = canvas.materials[0];
    int texW = mat.textureWidth;
    int texH = mat.textureHeight;

    if (!mat.texture || texW <= 0 || texH <= 0) return;

    int currX = int(hit.T.x * texW) % texW;
    int currY = int(hit.T.y * texH) % texH;
    if (currX < 0) currX += texW;
    if (currY < 0) currY += texH;

    unsigned char r = currentColor[0];
    unsigned char g = currentColor[1];
    unsigned char b = currentColor[2];

    // --- TOOL 8 (TEXT TOOL) ---
    if (toolIndex == 7) {
        // manual 5x7 bitmap font for "Lorem ipsum"
        static const unsigned char font5x7[11][7] = {
            { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3E }, // 'L'
            { 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C }, // 'o'
            { 0x00, 0x00, 0x16, 0x1A, 0x10, 0x10, 0x10 }, // 'r'
            { 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x20, 0x1C }, // 'e'
            { 0x00, 0x00, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A }, // 'm'
            { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // ' '
            { 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x1C }, // 'i'
            { 0x00, 0x00, 0x1C, 0x22, 0x22, 0x1C, 0x20 }, // 'p'
            { 0x00, 0x00, 0x1E, 0x20, 0x1C, 0x02, 0x3C }, // 's'
            { 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x1C }, // 'u'
            { 0x00, 0x00, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A }  // 'm'
        };

        const int scale = 2; // Text size scale factor
        const int charW = 6 * scale;
        const int charH = 7 * scale;
        const int totalW = 11 * charW;
        const int totalH = charH;

        int startX = currX - totalW / 2;
        int startY = currY - totalH / 2;

        for (int c = 0; c < 11; c++) {
            int charOffsetX = startX + c * charW;
            for (int row = 0; row < 7; row++) {
                unsigned char rowByte = font5x7[c][row];
                for (int col = 0; col < 6; col++) {
                    if (col < 6 && (rowByte & (1 << (5 - col)))) {
                        for (int sy = 0; sy < scale; sy++) {
                            for (int sx = 0; sx < scale; sx++) {
                                int px = charOffsetX + col * scale + sx;
                                int py = startY + row * scale + sy;

                                if (px >= 0 && px < texW && py >= 0 && py < texH) {
                                    int idx = (py * texW + px) * 3;
                                    mat.texture[idx + 0] = r;
                                    mat.texture[idx + 1] = g;
                                    mat.texture[idx + 2] = b;
                                }
                            }
                        }
                    }
                }
            }
        }

        p.prevHitUV = hit.T;
        p.wasColliding = true;
        return;
    }

    // --- TOOL 2 (BUCKET FILL) ---
    if (toolIndex == 1) {
        int startIdx = (currY * texW + currX) * 3;
        unsigned char targetR = mat.texture[startIdx + 0];
        unsigned char targetG = mat.texture[startIdx + 1];
        unsigned char targetB = mat.texture[startIdx + 2];

        if (targetR == r && targetG == g && targetB == b) {
            p.prevHitUV = hit.T;
            p.wasColliding = true;
            return;
        }

        std::queue<std::pair<int, int>> q;
        q.push({currX, currY});

        mat.texture[startIdx + 0] = r;
        mat.texture[startIdx + 1] = g;
        mat.texture[startIdx + 2] = b;

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        // Perform flood fill using BFS
        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();

            for (int i = 0; i < 4; i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx >= 0 && nx < texW && ny >= 0 && ny < texH) {
                    int nIdx = (ny * texW + nx) * 3;
                    if (mat.texture[nIdx + 0] == targetR &&
                        mat.texture[nIdx + 1] == targetG &&
                        mat.texture[nIdx + 2] == targetB) {
                        
                        mat.texture[nIdx + 0] = r;
                        mat.texture[nIdx + 1] = g;
                        mat.texture[nIdx + 2] = b;
                        q.push({nx, ny});
                    }
                }
            }
        }

        p.prevHitUV = hit.T;
        p.wasColliding = true;
        return;
    }

    // --- OTHER TOOLS (ERASER & BRUSHES) ---
    int brushRadius = 4;

    // treat erasor as a white brush for simplicity
    if (toolIndex == 0) { 
        r = 255; g = 255; b = 255;
        brushRadius = 10;
    }

    int startX = p.wasColliding ? (int(p.prevHitUV.x * texW) % texW) : currX;
    int startY = p.wasColliding ? (int(p.prevHitUV.y * texH) % texH) : currY;
    if (startX < 0) startX += texW;
    if (startY < 0) startY += texH;

    auto drawStamp = [&](int px, int py) {
        for (int dy = -brushRadius; dy <= brushRadius; dy++) {
            for (int dx = -brushRadius; dx <= brushRadius; dx++) {
                if (dx * dx + dy * dy <= brushRadius * brushRadius) {
                    int cx = px + dx;
                    int cy = py + dy;
                    if (cx >= 0 && cx < texW && cy >= 0 && cy < texH) {
                        int idx = (cy * texW + cx) * 3;
                        mat.texture[idx + 0] = r;
                        mat.texture[idx + 1] = g;
                        mat.texture[idx + 2] = b;
                    }
                }
            }
        }
    };

    int diff_x = currX - startX;
    int diff_y = currY - startY;
    int steps = std::max(std::abs(diff_x), std::abs(diff_y));

    for (int i = 0; i <= steps; i++) {
        float t = (steps == 0) ? 0.0f : (float)i / steps;
        int stepX = int(startX + t * diff_x);
        int stepY = int(startY + t * diff_y);
        drawStamp(stepX, stepY);
    }

    p.prevHitUV = hit.T;
    p.wasColliding = true;
}

bool resolveParticleMeshCollision(Particle& p, const TriangleMesh& mesh) {
    float3 disp = p.position - p.prevPosition;
    float dist = length(disp);
    if (dist < Epsilon) return false;

    float3 dir = disp / dist;
    Ray ray(p.prevPosition, dir);

    HitInfo minHit;
    minHit.t = FLT_MAX;
    bool collided = false;
    bool useFallback = true;
    
    int meshIndex = mesh.scene_bvh_index;

    // apply optimization for collision detection
    if (globalOptState == 0) {
        const float margin = 0.05f;
        float3 pMin = float3(
            std::min(p.position.x, p.prevPosition.x) - 0.002f,
            std::min(p.position.y, p.prevPosition.y) - 0.002f,
            std::min(p.position.z, p.prevPosition.z) - 0.002f
        );
        float3 pMax = float3(
            std::max(p.position.x, p.prevPosition.x) + 0.002f,
            std::max(p.position.y, p.prevPosition.y) + 0.002f,
            std::max(p.position.z, p.prevPosition.z) + 0.002f
        );
        float3 mMin = mesh.bbox.get_minp() - float3(margin);
        float3 mMax = mesh.bbox.get_maxp() + float3(margin);
        
        if (pMax.x < mMin.x || pMin.x > mMax.x || pMax.y < mMin.y || pMin.y > mMax.y || pMax.z < mMin.z || pMin.z > mMax.z) {
            p.wasColliding = false;
            return false; 
        }

        if (meshIndex >= 0 && meshIndex < (int)globalScene.bvhs.size() && globalScene.bvhs[meshIndex].node) {
            collided = globalScene.bvhs[meshIndex].intersect(minHit, ray, 0.0f, dist);
            useFallback = false;
        }
    }

    if (useFallback) {
        HitInfo hit;
        for (const auto& tri : mesh.triangles) {
            if (mesh.raytraceTriangle(hit, ray, tri, 0.0f, dist)) {
                if (hit.t < minHit.t) {
                    minHit = hit;
                    collided = true;
                }
            }
        }
    }

    if (collided) {
        if (dot(minHit.N, dir) > 0.0f) {
            minHit.N = -minHit.N;
        }

        const float skinOffset = 0.001f;
        p.position = minHit.P + minHit.N * skinOffset;

        float3 vel = (p.position - p.prevPosition) / deltaT;
        float vNormal = dot(vel, minHit.N);
        
        if (vNormal < 0.0f) {
            float3 vTan = vel - vNormal * minHit.N;
            float vTanLen = length(vTan);
            
            const float mu_s = 0.6f;
            const float mu_k = 0.4f;
            
            if (vTanLen > Epsilon) {
                float maxFriction = mu_s * std::abs(vNormal);
                if (vTanLen <= maxFriction) {
                    vTan = float3(0.0f);
                } else {
                    float frictionDrop = mu_k * std::abs(vNormal);
                    vTan = vTan * std::max(0.0f, vTanLen - frictionDrop) / vTanLen;
                }
            }
            
            vel = vTan;
            p.prevPosition = p.position - vel * deltaT;
        }
        
        paintCanvas(p, minHit, curToolIndex, mesh);
    } else {
        p.wasColliding = false;
    }

    return collided;
}

void updateHairAnchors() {
    if (curToolIndex != 5 || !toolRigidBody.anchor || !toolRigidBody.tip) return;

    const float3 targetDir = normalize(toolRigidBody.anchor->position - toolRigidBody.tip->position);
    const float3 rotationAxis = cross(toolRigidBody.baseToolDir, targetDir);
    const float dotVal = std::max(-1.0f, std::min(1.0f, dot(toolRigidBody.baseToolDir, targetDir)));
    const float angleRad = acosf(dotVal);
    const float segLength = 0.0355f;

    for (int h = 0; h < NUM_HAIRS; h++) {
        float3 localSurface = hairLocalAnchors[h];
        float3 localVirtual = hairLocalAnchors[h] + float3(0.0f, 0.0f, segLength);

        if (angleRad > Epsilon && length(rotationAxis) > Epsilon) {
            float3 u = normalize(rotationAxis);
            float c = cosf(angleRad);
            float s = sinf(angleRad);
            
            localSurface = u * dot(u, localSurface) + c * cross(cross(u, localSurface), u) + s * cross(u, localSurface);
            localVirtual = u * dot(u, localVirtual) + c * cross(cross(u, localVirtual), u) + s * cross(u, localVirtual);
        }

        hairParticles[h][0].prevPosition = hairParticles[h][0].position;
        hairParticles[h][0].position = toolRigidBody.tip->position + localVirtual;

        hairParticles[h][1].prevPosition = hairParticles[h][1].position;
        hairParticles[h][1].position = toolRigidBody.tip->position + localSurface;
    }
}

// functionality to freeze tool anchor and move tool tip around it when holding right-click
void rotateToolWithMouse(float dx, float dy) {
    if (!toolRigidBody.anchor || !toolRigidBody.tip) return;

    Particle* anchor = toolRigidBody.anchor;
    Particle* tip = toolRigidBody.tip;

    float3 dir = tip->position - anchor->position;
    float len = length(dir);
    if (len < Epsilon) return;
    dir /= len;

    const float sensitivity = 0.005f;
    float angleX = -dx * sensitivity;
    float angleY =  dy * sensitivity;

    auto rotateVector = [](const float3& v, const float3& axis, float angle) -> float3 {
        float c = cosf(angle);
        float s = sinf(angle);
        float3 u = normalize(axis);
        return u * dot(u, v) + c * cross(cross(u, v), u) + s * cross(u, v);
    };

    if (std::abs(angleX) > Epsilon) {
        dir = rotateVector(dir, globalUp, angleX);
    }
    if (std::abs(angleY) > Epsilon) {
        dir = rotateVector(dir, globalRight, angleY);
    }

    dir = normalize(dir);
    tip->prevPosition = tip->position;
    tip->position = anchor->position + dir * toolRigidBody.toolLength;
    resolveParticleMeshCollision(*tip, canvas);

    float3 actualDir = normalize(tip->position - anchor->position);
    tip->position = anchor->position + actualDir * toolRigidBody.toolLength;
    resolveParticleMeshCollision(*tip, canvas);
}

// main simulation loop for physics and interactions within the project
void simulateToolPhysics() {
    if (!debugFacesAdded) {
        globalScene.addObject(&debugFacesMesh);
        debugFacesAdded = true;
    }

if (globalCollisionMode == 2) {
        toolRigidBody.setEnable(false);
        curToolObj.visible = false;
        toggleHairVisibility(false);
        toggleSprayVisibility(false); 
        debugFacesMesh.visible = false;

        static Particle dummyMouseParticles[NUM_HAIRS];
        static Particle dummySprayParticles[NUM_SPRAY_SPRINGS];
        static Particle dummyMouseParticle;

        if (mouseLeftPressed) {
            double mx = (m_mouseX == 0.0 && m_mouseY == 0.0) ? (globalWidth * 0.5) : m_mouseX;
            double my = (m_mouseX == 0.0 && m_mouseY == 0.0) ? (globalHeight * 0.5) : m_mouseY;
            Ray ray = globalScene.eyeRay(mx, globalHeight - my);

            HitInfo minHit;
            minHit.t = FLT_MAX;
            HitInfo tempHit;
            const TriangleMesh* hitMesh = nullptr;
            int hitToolbarIndex = -1;

            for (const auto& tri : canvas.triangles) {
                if (canvas.raytraceTriangle(tempHit, ray, tri, 0.0f, FLT_MAX)) {
                    if (tempHit.t < minHit.t) {
                        minHit = tempHit;
                        hitMesh = &canvas;
                    }
                }
            }

            for (int i = 0; i < 8; i++) {
                for (const auto& tri : toolbar[i].triangles) {
                    if (toolbar[i].raytraceTriangle(tempHit, ray, tri, 0.0f, FLT_MAX)) {
                        if (tempHit.t < minHit.t) {
                            minHit = tempHit;
                            hitMesh = &toolbar[i];
                            hitToolbarIndex = i;
                        }
                    }
                }
            }

            for (const auto& tri : colorPalette.triangles) {
                if (colorPalette.raytraceTriangle(tempHit, ray, tri, 0.0f, FLT_MAX)) {
                    if (tempHit.t < minHit.t) {
                        minHit = tempHit;
                        hitMesh = &colorPalette;
                        hitToolbarIndex = -1;
                    }
                }
            }

            if (hitMesh && minHit.t != FLT_MAX) {
                if (hitToolbarIndex != -1) {
                    selectCurrentToolByIndex(hitToolbarIndex);
                } else if (curToolIndex == 5) { // Tool 6: Brush
                    for(int h = 0; h < NUM_HAIRS; h++) {
                        float3 offset = globalRight * hairLocalAnchors[h].x + globalUp * hairLocalAnchors[h].y;
                        Ray hRay(ray.o + offset, ray.d);
                        
                        HitInfo hHit; hHit.t = FLT_MAX;
                        HitInfo tHit;
                        for (const auto& tri : hitMesh->triangles) {
                            if (hitMesh->raytraceTriangle(tHit, hRay, tri, 0.0f, FLT_MAX)) {
                                if (tHit.t < hHit.t) hHit = tHit;
                            }
                        }
                        if (hHit.t != FLT_MAX) {
                            paintCanvas(dummyMouseParticles[h], hHit, curToolIndex, *hitMesh);
                        }
                    }
                } else if (curToolIndex == 6) {
                    for(int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
                        float3 offset = globalRight * sprayLocalTipOffsets[s].x + globalUp * sprayLocalTipOffsets[s].y;
                        Ray sRay(ray.o + offset, ray.d);
                        
                        HitInfo sHit; sHit.t = FLT_MAX;
                        HitInfo tHit;
                        for (const auto& tri : hitMesh->triangles) {
                            if (hitMesh->raytraceTriangle(tHit, sRay, tri, 0.0f, FLT_MAX)) {
                                if (tHit.t < sHit.t) sHit = tHit;
                            }
                        }
                        if (sHit.t != FLT_MAX) {
                            paintCanvas(dummySprayParticles[s], sHit, curToolIndex, *hitMesh);
                        }
                    }
                } else { 
                    paintCanvas(dummyMouseParticle, minHit, curToolIndex, *hitMesh);
                }
            }
        } else {
            for(int h = 0; h < NUM_HAIRS; h++) {
                dummyMouseParticles[h].wasColliding = false;
            }
            for(int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
                dummySprayParticles[s].wasColliding = false;
            }
            dummyMouseParticle.wasColliding = false;
        }
        return; 
    } else {
        toolRigidBody.setEnable(true);
        curToolObj.visible = true;
        if (curToolIndex == 5) toggleHairVisibility(true);
        if (curToolIndex == 6) toggleSprayVisibility(true); 
    }

    toolRigidBody.simulatePhysics();
    toolRigidBody.updateMeshTransform();

    if (globalCollisionMode == 0) {
        float3 velocity = (toolRigidBody.tip->position - toolRigidBody.tip->prevPosition) / deltaT;
        const int samples = 4; 
        
        float3 maxDisplacement = float3(0.0f);
        float maxDispSq = 0.0f;

        for (auto& tri : curToolObj.triangles) {
            for (int i = 0; i <= samples; i++) {
                for (int j = 0; j <= samples - i; j++) {
                    float u = (float)i / samples;
                    float v = (float)j / samples;
                    float w = 1.0f - u - v;

                    Particle fParticle;
                    fParticle.position = tri.positions[0] * u + tri.positions[1] * v + tri.positions[2] * w;
                    fParticle.prevPosition = fParticle.position - velocity * deltaT;
                    
                    float3 originalPos = fParticle.position;
                    
                    resolveParticleMeshCollision(fParticle, canvas);
                    for (int k = 0; k < 8; k++) {
                        resolveParticleMeshCollision(fParticle, toolbar[k]);
                    }
                    resolveParticleMeshCollision(fParticle, colorPalette);
                    
                    float3 disp = fParticle.position - originalPos;
                    float dispSq = dot(disp, disp);
                    if (dispSq > maxDispSq) {
                        maxDispSq = dispSq;
                        maxDisplacement = disp;
                    }
                }
            }
        }

        if (maxDispSq > Epsilon) {
            toolRigidBody.tip->position += maxDisplacement;
            
            float3 actualDir = normalize(toolRigidBody.tip->position - toolRigidBody.anchor->position);
            toolRigidBody.tip->position = toolRigidBody.anchor->position + actualDir * toolRigidBody.toolLength;
            
            toolRigidBody.updateMeshTransform();
        }

        if (debugEnabled) {
            int dbgIdx = 0;
            for (auto& tri : curToolObj.triangles) {
                for (int i = 0; i <= samples; i++) {
                    for (int j = 0; j <= samples - i; j++) {
                        float u = (float)i / samples;
                        float v = (float)j / samples;
                        float w = 1.0f - u - v;

                        float3 pPos = tri.positions[0] * u + tri.positions[1] * v + tri.positions[2] * w;

                        if (dbgIdx >= debugFacesMesh.triangles.size()) {
                            debugFacesMesh.triangles.resize(dbgIdx + 1);
                            if (debugFacesMesh.materials.empty()) debugFacesMesh.materials.resize(1);
                            debugFacesMesh.materials[0].Kd = float3(1.0f, 1.0f, 0.0f);
                        }
                        
                        const float pSize = 0.005f;
                        debugFacesMesh.triangles[dbgIdx].positions[0] = pPos;
                        debugFacesMesh.triangles[dbgIdx].positions[1] = pPos + pSize * globalUp;
                        debugFacesMesh.triangles[dbgIdx].positions[2] = pPos + pSize * globalRight;
                        debugFacesMesh.triangles[dbgIdx].normals[0] = -globalViewDir;
                        debugFacesMesh.triangles[dbgIdx].normals[1] = -globalViewDir;
                        debugFacesMesh.triangles[dbgIdx].normals[2] = -globalViewDir;
                        dbgIdx++;
                    }
                }
            }

            debugFacesMesh.triangles.resize(dbgIdx);
            debugFacesMesh.visible = true;
            debugFacesMesh.preCalc();
            int bIdx = debugFacesMesh.scene_bvh_index;
            if (bIdx >= 0 && bIdx < globalScene.bvhs.size()) {
                if (globalScene.bvhs[bIdx].nodeNum == 0) globalScene.bvhs[bIdx].build(&debugFacesMesh);
                else refitBVH(globalScene.bvhs[bIdx]);
            }
        } else {
            debugFacesMesh.visible = false;
        }
    } else {
        debugFacesMesh.visible = false;
    }

    // --- TOOL 6 (HAIR SIMULATION) ---
    if (curToolIndex == 5) {
        updateHairAnchors();

        for (int h = 0; h < NUM_HAIRS; h++) {
            for (int p = 0; p < SEGMENTS_PER_HAIR + 2; p++) {
                hairParticles[h][p].accForce = float3(0.0f);
            }
        }
        for (int h = 0; h < NUM_HAIRS; h++) {
            for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
                hairBendBodies[h][s].simulate();
            }
        }
        for (int h = 0; h < NUM_HAIRS; h++) {
            for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
                hairRigidBodies[h][s].simulatePhysics();
            }
        }
        // apply optimization for collision detection
        if (globalOptState == 0) {
            float3 cMin = canvas.bbox.get_minp();
            float3 cMax = canvas.bbox.get_maxp();
            float3 hMin = float3(FLT_MAX);
            float3 hMax = float3(-FLT_MAX);

            for (int h = 0; h < NUM_HAIRS; h++) {
                for (int p = 0; p < SEGMENTS_PER_HAIR + 2; p++) {
                    hMin.x = std::min(hMin.x, std::min(hairParticles[h][p].position.x, hairParticles[h][p].prevPosition.x));
                    hMin.y = std::min(hMin.y, std::min(hairParticles[h][p].position.y, hairParticles[h][p].prevPosition.y));
                    hMin.z = std::min(hMin.z, std::min(hairParticles[h][p].position.z, hairParticles[h][p].prevPosition.z));
                    hMax.x = std::max(hMax.x, std::max(hairParticles[h][p].position.x, hairParticles[h][p].prevPosition.x));
                    hMax.y = std::max(hMax.y, std::max(hairParticles[h][p].position.y, hairParticles[h][p].prevPosition.y));
                    hMax.z = std::max(hMax.z, std::max(hairParticles[h][p].position.z, hairParticles[h][p].prevPosition.z));
                }
            }
            
            hMin -= float3(0.002f);
            hMax += float3(0.002f);

            bool clusterCollides = !(hMax.x < cMin.x || hMin.x > cMax.x || hMax.y < cMin.y || hMin.y > cMax.y || hMax.z < cMin.z || hMin.z > cMax.z);

            if (clusterCollides) {
                for (int h = 0; h < NUM_HAIRS; h++) {
                    for (int p = 0; p < SEGMENTS_PER_HAIR + 2; p++) {
                        resolveParticleMeshCollision(hairParticles[h][p], canvas);
                    }
                }
            }
        // unoptimized collision detection
        } else {
            for (int h = 0; h < NUM_HAIRS; h++) {
                for (int p = 0; p < SEGMENTS_PER_HAIR + 2; p++) {
                    resolveParticleMeshCollision(hairParticles[h][p], canvas);
                }
            }
        }
    }

    // --- TOOL 7 (SPRAY POINT SIMULATION) ---
    if (curToolIndex == 6) {
        updateSprayAnchors();
        for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
            for (int p = 0; p < 3; p++) {
                sprayParticles[s][p].accForce = float3(0.0f);
            }
        }
        for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
            sprayBendBodies[s].simulate();
        }
        for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
            sprayParticles[s][2].step();

            float L_s = length(sprayLocalTipOffsets[s]);
            float3 anchorPos = sprayParticles[s][1].position;
            float3 dir = sprayParticles[s][2].position - anchorPos;
            float dist = length(dir);
            if (dist > Epsilon) {
                sprayParticles[s][2].position = anchorPos + (dir / dist) * L_s;
            }

            sprayParticles[s][2].velocity = (sprayParticles[s][2].position - sprayParticles[s][2].prevPosition) / deltaT;
            sprayParticles[s][2].velocity *= globalDampingFactor;
            sprayParticles[s][2].prevPosition = sprayParticles[s][2].position - sprayParticles[s][2].velocity * deltaT;
        }
        for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
            resolveParticleMeshCollision(sprayParticles[s][2], canvas);
        }
    }

    toolRigidBody.updateMeshTransform();

    if (curToolIndex == 5) {
        for (int h = 0; h < NUM_HAIRS; h++) {
            for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
                hairRigidBodies[h][s].updateMeshTransform();
                hairBendBodies[h][s].updateDebugMesh();
            }
        }
    }

    if (curToolIndex == 6) {
        for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
            sprayBendBodies[s].updateDebugMesh();
        }
    }
}

void toggleHairVisibility(bool visible) {
    static bool currentlyVisible = false;
    if (currentlyVisible == visible) return;

    for (int h = 0; h < NUM_HAIRS; h++) {
        for (int s = 0; s < SEGMENTS_PER_HAIR; s++) {
            hairRigidBodies[h][s].setEnable(visible);
            hairBendBodies[h][s].setEnable(visible);
            hairMeshes[h][s].visible = visible;
        }
    }
    currentlyVisible = visible;

    if (visible) resetAllHairParticles();
}

void resetAllSprayParticles() {
    if (!toolRigidBody.anchor || !toolRigidBody.tip) return;

    const float3 targetDir = normalize(toolRigidBody.anchor->position - toolRigidBody.tip->position);
    const float3 rotationAxis = cross(toolRigidBody.baseToolDir, targetDir);
    const float dotVal = std::max(-1.0f, std::min(1.0f, dot(toolRigidBody.baseToolDir, targetDir)));
    const float angleRad = acosf(dotVal);

    for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
        float3 localTip = sprayLocalTipOffsets[s];
        float L_s = length(localTip);
        float3 localDir = (L_s > Epsilon) ? (localTip / L_s) : float3(0.0f, 0.0f, -1.0f);
        float3 localVirtual = -localDir * L_s;

        if (angleRad > Epsilon && length(rotationAxis) > Epsilon) {
            float3 u = normalize(rotationAxis);
            float c = cosf(angleRad);
            float s_val = sinf(angleRad);

            localTip = u * dot(u, localTip) + c * cross(cross(u, localTip), u) + s_val * cross(u, localTip);
            localVirtual = u * dot(u, localVirtual) + c * cross(cross(u, localVirtual), u) + s_val * cross(u, localVirtual);
        }

        sprayParticles[s][0].position = toolRigidBody.tip->position + localVirtual;
        sprayParticles[s][0].prevPosition = sprayParticles[s][0].position;
        sprayParticles[s][0].velocity = float3(0.0f);
        sprayParticles[s][0].accForce = float3(0.0f);

        sprayParticles[s][1].position = toolRigidBody.tip->position;
        sprayParticles[s][1].prevPosition = sprayParticles[s][1].position;
        sprayParticles[s][1].velocity = float3(0.0f);
        sprayParticles[s][1].accForce = float3(0.0f);

        sprayParticles[s][2].position = toolRigidBody.tip->position + localTip;
        sprayParticles[s][2].prevPosition = sprayParticles[s][2].position;
        sprayParticles[s][2].velocity = float3(0.0f);
        sprayParticles[s][2].accForce = float3(0.0f);
        sprayParticles[s][2].wasColliding = false;
    }
}

void updateSprayAnchors() {
    if (curToolIndex != 6 || !toolRigidBody.anchor || !toolRigidBody.tip) return;

    const float3 targetDir = normalize(toolRigidBody.anchor->position - toolRigidBody.tip->position);
    const float3 rotationAxis = cross(toolRigidBody.baseToolDir, targetDir);
    const float dotVal = std::max(-1.0f, std::min(1.0f, dot(toolRigidBody.baseToolDir, targetDir)));
    const float angleRad = acosf(dotVal);

    for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
        float3 localTip = sprayLocalTipOffsets[s];
        float L_s = length(localTip);
        float3 localDir = (L_s > Epsilon) ? (localTip / L_s) : float3(0.0f, 0.0f, -1.0f);
        float3 localVirtual = -localDir * L_s;

        if (angleRad > Epsilon && length(rotationAxis) > Epsilon) {
            float3 u = normalize(rotationAxis);
            float c = cosf(angleRad);
            float s_val = sinf(angleRad);

            localVirtual = u * dot(u, localVirtual) + c * cross(cross(u, localVirtual), u) + s_val * cross(u, localVirtual);
        }

        sprayParticles[s][0].prevPosition = sprayParticles[s][0].position;
        sprayParticles[s][0].position = toolRigidBody.tip->position + localVirtual;
        sprayParticles[s][1].prevPosition = sprayParticles[s][1].position;
        sprayParticles[s][1].position = toolRigidBody.tip->position;
    }
}

void toggleSprayVisibility(bool visible) {
    static bool currentlyVisible = false;
    if (currentlyVisible == visible) return;

    for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
        sprayBendBodies[s].setEnable(visible);
    }
    currentlyVisible = visible;
    if (visible) resetAllSprayParticles();
}

void initSprayPhysics() {
    // manual vertex coordinates derived from cone-for-spray.obj after modeling
    static const float3 coneBaseOffsets[NUM_SPRAY_SPRINGS] = {
        float3( 0.000000f, -0.070710f, -0.212132f),
        float3( 0.029476f, -0.073852f, -0.208990f),
        float3( 0.056332f, -0.082997f, -0.199845f),
        float3( 0.078183f, -0.097334f, -0.185509f),
        float3( 0.093087f, -0.115588f, -0.167255f),
        float3( 0.099720f, -0.136137f, -0.146705f),
        float3( 0.097493f, -0.157156f, -0.125687f),
        float3( 0.086603f, -0.176776f, -0.106066f),
        float3( 0.068017f, -0.193256f, -0.089587f),
        float3( 0.043388f, -0.205129f, -0.077713f),
        float3( 0.014904f, -0.211342f, -0.071500f),
        float3(-0.014904f, -0.211342f, -0.071500f),
        float3(-0.043388f, -0.205129f, -0.077713f),
        float3(-0.068017f, -0.193256f, -0.089587f),
        float3(-0.086603f, -0.176776f, -0.106066f),
        float3(-0.097493f, -0.157156f, -0.125687f),
        float3(-0.099720f, -0.136137f, -0.146705f),
        float3(-0.093087f, -0.115588f, -0.167255f),
        float3(-0.078183f, -0.097334f, -0.185509f),
        float3(-0.056332f, -0.082997f, -0.199845f),
        float3(-0.029476f, -0.073852f, -0.208990f)
    };

    for (int s = 0; s < NUM_SPRAY_SPRINGS; s++) {
        sprayLocalTipOffsets[s] = coneBaseOffsets[s];
        float L_s = length(sprayLocalTipOffsets[s]);
        sprayBendBodies[s].setupBend(&sprayParticles[s][0], &sprayParticles[s][2], 2.0f * L_s);
    }
    resetAllSprayParticles();
    toggleSprayVisibility(curToolIndex == 6);
}

// --- PROJECT EVENT HOOKS ---

inline bool projectKeyboardCallback(int key) {
    float diff = 50.0f * SCLFACT * 10.0f;
    switch (key) {
        case GLFW_KEY_EQUAL:
        case GLFW_KEY_MINUS:
            globalGravity.z += (key == GLFW_KEY_EQUAL) ? -diff : diff;
            printf("Gravity adjusted to: %f\n", globalGravity.z);
            return true;
        case GLFW_KEY_RIGHT_BRACKET:
        case GLFW_KEY_LEFT_BRACKET:
            globalSpringStiffness += (key == GLFW_KEY_RIGHT_BRACKET) ? diff : -diff;
            printf("Spring stiffness adjusted to: %f\n", globalSpringStiffness);
            return true;
        case GLFW_KEY_B:
            globalOptState = (globalOptState + 1) % 4;
            if (globalOptState == 0) printf("Optimization Mode [0/3]: FULL OPTIMIZATION (Default)\n");
            else if (globalOptState == 1) printf("Optimization Mode [1/3]: NO Collision & Particle Optimization\n");
            else if (globalOptState == 2) printf("Optimization Mode [2/3]: NO Ray Tracing Optimization\n");
            else if (globalOptState == 3) printf("Optimization Mode [3/3]: NO OPTIMIZATION (Complete Brute-Force)\n");
            return true;
        case GLFW_KEY_SLASH:
            debugParticlesToggle();
            return true;
        case GLFW_KEY_1: case GLFW_KEY_2: case GLFW_KEY_3: case GLFW_KEY_4:
        case GLFW_KEY_5: case GLFW_KEY_6: case GLFW_KEY_7: case GLFW_KEY_8:
            selectCurrentToolByIndex(key - GLFW_KEY_1);
            return true;
        case GLFW_KEY_N:
            globalNPRLevel = (globalNPRLevel + 1) % 4;
            if (globalNPRLevel == 0) printf("NPR / Cel Shading: OFF (Smooth)\n");
            else printf("NPR / Cel Shading: Level %d (%d Discrete Tones)\n", globalNPRLevel, (globalNPRLevel == 1) ? 4 : (globalNPRLevel == 2) ? 8 : 16);
            return true;
        case GLFW_KEY_C:
            globalCollisionMode = (globalCollisionMode + 1) % 3;
            if (globalCollisionMode == 0) printf("(Collision Mode: All Tool Vertices & Faces)\n");
            else if (globalCollisionMode == 1) printf("(Collision Mode: Virtual Particles)\n");
            else if (globalCollisionMode == 2) printf("(Collision Mode: No Tool and Collision of Mouse Coordinate on Click)\n");
            return true;
    }
    return false;
}

inline bool projectMouseButtonCallbackFunc(int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        if (updateCurrentToolSelection(m_mouseX, m_mouseY)) {
            mouseLeftPressed = true; // Sync for cs488
            if (globalRenderType == RENDER_RAYTRACE) {
                AccumulationBuffer.clear();
                sampleCount = 0;
            }
            return true;
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            mouseRightPressed = true;
            glfwSetInputMode(globalGLFWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            return true;
        } else if (action == GLFW_RELEASE) {
            mouseRightPressed = false;
            glfwSetInputMode(globalGLFWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            return true;
        }
    }
    return false;
}

inline bool projectCursorPosCallbackFunc(double mouse_x, double mouse_y) {
    if (mouseRightPressed) {
        const float dx = float(mouse_x - m_mouseX);
        const float dy = -float(mouse_y - m_mouseY);
        rotateToolWithMouse(dx, dy);
        return true; // prevent cs488 from registering camera drag
    }
    return false;
}

inline bool projectScrollCallbackFunc(double xoffset, double yoffset) {
    toolZDepth += 0.1f * (float)yoffset;
    return true;
}

// Light definition to mimic the original main
static PointLightSource projectLight;

// --- PROJECT INITIALIZATION ---

inline void initProject(int argc, const char* argv[]) {
    // Inject logic into base engine via callbacks
    projectKeyCallback = projectKeyboardCallback;
    projectMouseButtonCallback = projectMouseButtonCallbackFunc;
    projectCursorPosCallback = projectCursorPosCallbackFunc;
    projectScrollCallback = projectScrollCallbackFunc;
    projectSimulationStep = simulateToolPhysics;

    // Overwrite the base physics g for tool handling; leaning towards canvas at -Z
    globalGravity = float3(0.0f, 0.0f, -1000.0f);

    // Overwrite starting view
    globalEye = float3(0.0f, 0.0f, 2.0f);

    TriangleMesh baseSquare;
    bool objLoadSucceed = baseSquare.load(squareObjPath);
    if (!objLoadSucceed) {
        printf("Invalid .obj file.\n");
    }

    // TOOLBAR TILES SETUP
    for (int i = 0; i < 8; i++) {
        toolbar[i] = baseSquare;

        int currentLogoIndex = startLogoIndex + i; 
        float uMin = (float)currentLogoIndex / (float)totalIconsInAtlas;
        float uMax = (float)(currentLogoIndex + 1) / (float)totalIconsInAtlas;

        for (auto& tri : toolbar[i].triangles) {
            for (int k = 0; k < 3; k++) {
                float originalU = tri.texcoords[k].x; 
                tri.texcoords[k].x = uMin + originalU * (uMax - uMin);
            }
        }

        toolbar[i].scale(float3(0.25f));
        toolbar[i].translate(float3(-0.75f + (i % 2) * 0.15f, 0.40f - (i / 2) * 0.15f, 0.0f));
        toolbar[i].loadTexture(toolSolidBGPath, 0);
        globalScene.addObject(&toolbar[i]);
    }
    toolbar[curToolIndex].loadTexture(toolGridBGPath, 0);

    // COLOUR PALETTE OBJ SETUP
    colorPalette = baseSquare;
    colorPalette.scale(float3(0.55f, 0.55f, 1.0f)); 
    colorPalette.translate(float3(-0.675f, -0.32f, 0.0f));
    colorPalette.loadTexture(colorPalettePath, 0);
    globalScene.addObject(&colorPalette);

    // DYNAMIC CANVAS LOADING
    bool customCanvasLoaded = false;
    if (argc > 1) {
        customCanvasLoaded = canvas.load(argv[1]);
        if (!customCanvasLoaded) {
            printf("Failed to load custom canvas OBJ: %s. Falling back to default.\n", argv[1]);
        }
        canvas.scale(float3(2.0f));
    }

    if (!customCanvasLoaded) {
        canvas = baseSquare;
        canvas.materials[0].isTextured = false;
        canvas.scale(float3(2.0f));
    }

    // CANVAS UV AND TEXTURE INITIALIZATION
    if (canvas.materials.empty()) {
        canvas.materials.resize(1);
    }

    bool hasValidUVs = false;
    for (const auto& tri : canvas.triangles) {
        for (int k = 0; k < 3; k++) {
            if (tri.texcoords[k].x != 0.0f || tri.texcoords[k].y != 0.0f) {
                hasValidUVs = true;
                break;
            }
        }
        if (hasValidUVs) break;
    }

    if (!hasValidUVs) {
        printf("No UV coordinates detected in canvas OBJ. Generating planar UVs...\n");
        generatePlanarUVs(canvas);
    }

    bool hasExistingTexture = canvas.materials[0].isTextured && 
                             (canvas.materials[0].texture != nullptr) &&
                             canvas.materials[0].textureWidth > 0 &&
                             canvas.materials[0].textureHeight > 0;

    if (!hasExistingTexture) {
        const int texRes = 512;
        canvas.materials[0].isTextured = true;
        canvas.materials[0].textureWidth = texRes;
        canvas.materials[0].textureHeight = texRes;
        
        canvas.materials[0].texture = new unsigned char[texRes * texRes * 3];
        for (int i = 0; i < texRes * texRes * 3; i++) {
            canvas.materials[0].texture[i] = 255;
        }
    } else {
        printf("Preserving imported canvas texture (%dx%d).\n", 
               canvas.materials[0].textureWidth, canvas.materials[0].textureHeight);
    }

    globalScene.addObject(&canvas);

    // --- TOOL OBJECT SETUP ---
    curToolObj.load(toolObjPath[curToolIndex]);
    globalScene.addObject(&curToolObj);
    toolRigidBody.anchorStep = toolAnchorStep;

    initToolPhysics();
    initHairPhysics();
    initSprayPhysics();

    // Lighting setup for the project
    // copied from main.cpp; overwrite if needed
    projectLight.position = float3(3.0f, 3.0f, 3.0f);
    projectLight.wattage = float3(1000.0f, 1000.0f, 1000.0f);
    globalScene.addLight(&projectLight);

    globalRenderType = RENDER_RASTERIZE;
}

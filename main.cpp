#include <GL/glut.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <functional>
#include <cmath>
#include <limits>
#include <iomanip>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Simple structure to hold mesh data
// Vertex data structure including skinning information for up to 4 bones
struct Vertex {
    float position[3];
    float normal[3];
    float texCoord[2];
    unsigned int boneIDs[4] = {0,0,0,0};
    float boneWeights[4] = {0.f,0.f,0.f,0.f};
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

std::vector<Mesh> meshes;

// Bone mapping and animation state
struct BoneInfo {
    aiMatrix4x4 offset;
    aiMatrix4x4 finalTransform;
};
std::map<std::string, unsigned int> boneMapping;
std::vector<BoneInfo> boneInfo;
aiMatrix4x4 globalInverse;

// Animation data
const aiScene* scene = nullptr;
Assimp::Importer importer;
float animationTime = 0.0f;

// Camera controls
float camX = 0.0f, camY = 0.0f, camZ = 5.0f;
float yaw = 0.0f, pitch = 0.0f;

// Model bounding box data
float modelCenter[3] = {0.0f, 0.0f, 0.0f};
float modelRadius = 1.0f;

void loadModel(const std::string& path) {
    std::cout << "Loading model: " << path << std::endl;
    scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals);
    if(!scene) {
        std::cerr << "Failed to load: " << path << std::endl;
        exit(1);
    }
    std::cout << "\tMeshes: " << scene->mNumMeshes << "  Animations: " << scene->mNumAnimations << std::endl;
    meshes.clear();
    boneMapping.clear();
    boneInfo.clear();
    globalInverse = scene->mRootNode->mTransformation;
    globalInverse.Inverse();
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();
    for(unsigned int i = 0; i < scene->mNumMeshes; ++i) {
        aiMesh* aMesh = scene->mMeshes[i];
        Mesh mesh;
        mesh.vertices.resize(aMesh->mNumVertices);
        for(unsigned int v = 0; v < aMesh->mNumVertices; ++v) {
            aiVector3D pos = aMesh->mVertices[v];
            aiVector3D norm = aMesh->mNormals[v];
            aiVector3D uv = aMesh->HasTextureCoords(0) ? aMesh->mTextureCoords[0][v] : aiVector3D(0,0,0);
            mesh.vertices[v] = {{pos.x, pos.y, pos.z}, {norm.x, norm.y, norm.z}, {uv.x, uv.y}};
            if(pos.x < minX) minX = pos.x;
            if(pos.y < minY) minY = pos.y;
            if(pos.z < minZ) minZ = pos.z;
            if(pos.x > maxX) maxX = pos.x;
            if(pos.y > maxY) maxY = pos.y;
            if(pos.z > maxZ) maxZ = pos.z;
        }
        // Load bone weights
        for(unsigned int b=0; b<aMesh->mNumBones; ++b) {
            aiBone* bone = aMesh->mBones[b];
            unsigned int boneIndex = 0;
            auto it = boneMapping.find(bone->mName.C_Str());
            if(it == boneMapping.end()) {
                boneIndex = boneInfo.size();
                boneMapping[bone->mName.C_Str()] = boneIndex;
                BoneInfo info;
                info.offset = bone->mOffsetMatrix;
                boneInfo.push_back(info);
            } else {
                boneIndex = it->second;
            }
            for(unsigned int w=0; w<bone->mNumWeights; ++w) {
                const aiVertexWeight& vw = bone->mWeights[w];
                Vertex& vert = mesh.vertices[vw.mVertexId];
                for(int k=0;k<4;++k) {
                    if(vert.boneWeights[k]==0.f) {
                        vert.boneIDs[k] = boneIndex;
                        vert.boneWeights[k] = vw.mWeight;
                        break;
                    }
                }
            }
        }

        mesh.indices.reserve(aMesh->mNumFaces*3);
        for(unsigned int f = 0; f < aMesh->mNumFaces; ++f) {
            aiFace face = aMesh->mFaces[f];
            for(unsigned int j=0;j<face.mNumIndices;++j)
                mesh.indices.push_back(face.mIndices[j]);
        }
        std::cout << "\tMesh " << i << " vertices: " << mesh.vertices.size()
                  << " indices: " << mesh.indices.size() << std::endl;
        meshes.push_back(std::move(mesh));
    }
    modelCenter[0] = (minX + maxX) * 0.5f;
    modelCenter[1] = (minY + maxY) * 0.5f;
    modelCenter[2] = (minZ + maxZ) * 0.5f;
    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;
    modelRadius = std::max({dx, dy, dz}) * 0.5f;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\tBounding box min(" << minX << ", " << minY << ", " << minZ
              << ") max(" << maxX << ", " << maxY << ", " << maxZ << ")" << std::endl;
    std::cout << "\tModel center(" << modelCenter[0] << ", " << modelCenter[1]
              << ", " << modelCenter[2] << ") radius " << modelRadius << std::endl;
    camX = 0.0f;
    camY = 0.0f;
    camZ = modelRadius * 3.0f;
    std::cout << "\tCamera start position(" << camX << ", " << camY << ", "
              << camZ << ")" << std::endl;
}

void drawMeshes() {
    static bool first = true;
    if(first) {
        std::cout << "Drawing " << meshes.size() << " meshes" << std::endl;
        for(size_t i = 0; i < meshes.size(); ++i) {
            std::cout << "\tMesh " << i << " vertices: " << meshes[i].vertices.size()
                      << " indices: " << meshes[i].indices.size() << std::endl;
        }
        first = false;
    }
    for(const Mesh& mesh : meshes) {
        glBegin(GL_TRIANGLES);
        for(unsigned int idx : mesh.indices) {
            const Vertex& v = mesh.vertices[idx];
            aiVector3D pos(0,0,0);
            for(int i=0;i<4;++i){
                if(v.boneWeights[i] > 0.f){
                    aiMatrix4x4 transform = boneInfo[v.boneIDs[i]].finalTransform;
                    aiVector3D p(v.position[0], v.position[1], v.position[2]);
                    p *= transform;
                    pos += p * v.boneWeights[i];
                }
            }
            if(pos == aiVector3D(0,0,0))
                pos = aiVector3D(v.position[0], v.position[1], v.position[2]);
            glNormal3fv(v.normal);
            glVertex3f(pos.x, pos.y, pos.z);
        }
        glEnd();
    }
}

// Helper functions to interpolate animation keys
static aiQuaternion interpolateRotation(aiNodeAnim* channel, double time) {
    if(channel->mNumRotationKeys == 1)
        return channel->mRotationKeys[0].mValue;

    unsigned int index = 0;
    for(; index < channel->mNumRotationKeys - 1; ++index) {
        if(time < channel->mRotationKeys[index + 1].mTime)
            break;
    }
    unsigned int next = index + 1;
    double delta = channel->mRotationKeys[next].mTime - channel->mRotationKeys[index].mTime;
    double factor = (time - channel->mRotationKeys[index].mTime) / delta;
    aiQuaternion out;
    aiQuaternion::Interpolate(out,
                              channel->mRotationKeys[index].mValue,
                              channel->mRotationKeys[next].mValue,
                              static_cast<float>(factor));
    out.Normalize();
    return out;
}

static aiVector3D interpolatePosition(aiNodeAnim* channel, double time) {
    if(channel->mNumPositionKeys == 1)
        return channel->mPositionKeys[0].mValue;

    unsigned int index = 0;
    for(; index < channel->mNumPositionKeys - 1; ++index) {
        if(time < channel->mPositionKeys[index + 1].mTime)
            break;
    }
    unsigned int next = index + 1;
    double delta = channel->mPositionKeys[next].mTime - channel->mPositionKeys[index].mTime;
    double factor = (time - channel->mPositionKeys[index].mTime) / delta;
    aiVector3D start = channel->mPositionKeys[index].mValue;
    aiVector3D end = channel->mPositionKeys[next].mValue;
    return start + (end - start) * static_cast<float>(factor);
}

void updateAnimation(float delta) {
    if(!scene || !scene->HasAnimations()) return;
    animationTime += delta;
    const aiAnimation* anim = scene->mAnimations[0];
    double ticksPerSecond = anim->mTicksPerSecond != 0 ? anim->mTicksPerSecond : 25.0;
    double timeInTicks = animationTime * ticksPerSecond;
    double animationTimeTicks = fmod(timeInTicks, anim->mDuration);

    // Recursive update of all bones
    std::function<void(const aiNode*, const aiMatrix4x4&)> readNode;
    readNode = [&](const aiNode* node, const aiMatrix4x4& parent){
        std::string name(node->mName.C_Str());
        aiMatrix4x4 nodeTransform = node->mTransformation;
        // find corresponding animation channel
        aiNodeAnim* channel = nullptr;
        for(unsigned int i=0;i<anim->mNumChannels;++i){
            if(name == anim->mChannels[i]->mNodeName.C_Str()){ channel = anim->mChannels[i]; break; }
        }
        if(channel){
            aiQuaternion rot = interpolateRotation(channel, animationTimeTicks);
            aiVector3D pos = interpolatePosition(channel, animationTimeTicks);
            aiMatrix4x4 mat = aiMatrix4x4(rot.GetMatrix());
            mat.a4 = pos.x; mat.b4 = pos.y; mat.c4 = pos.z;
            nodeTransform = mat;
        }
        aiMatrix4x4 global = parent * nodeTransform;
        auto it = boneMapping.find(name);
        if(it != boneMapping.end()){
            unsigned int index = it->second;
            boneInfo[index].finalTransform = globalInverse * global * boneInfo[index].offset;
        }
        for(unsigned int i=0;i<node->mNumChildren;++i)
            readNode(node->mChildren[i], global);
    };

    aiMatrix4x4 identity; // identity matrix
    readNode(scene->mRootNode, identity);
}

void display() {
    static int lastTime = glutGet(GLUT_ELAPSED_TIME);
    int currentTime = glutGet(GLUT_ELAPSED_TIME);
    float delta = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;
    static int frame = 0;
    std::cout << "Frame " << frame++ << " delta " << delta
              << " cam(" << camX << "," << camY << "," << camZ
              << ") yaw " << yaw << " pitch " << pitch << std::endl;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(pitch, 1,0,0);
    glRotatef(yaw, 0,1,0);
    glTranslatef(-camX, -camY, -camZ);
    glTranslatef(-modelCenter[0], -modelCenter[1], -modelCenter[2]);

    updateAnimation(delta);
    drawMeshes();

    glutSwapBuffers();
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Increase the far clipping plane to avoid culling animated models that
    // move further from the origin.
    gluPerspective(45.0f, (float)w/h, 0.1f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int) {
    const float speed = 0.1f;
    switch(key) {
        case 'w': camZ -= speed; break;
        case 's': camZ += speed; break;
        case 'a': camX -= speed; break;
        case 'd': camX += speed; break;
    }
    std::cout << "Camera position(" << camX << "," << camY << "," << camZ
              << ")" << std::endl;
}

void arrows(int key, int, int) {
    const float rot = 2.0f;
    switch(key) {
        case GLUT_KEY_UP: pitch += rot; break;
        case GLUT_KEY_DOWN: pitch -= rot; break;
        case GLUT_KEY_LEFT: yaw += rot; break;
        case GLUT_KEY_RIGHT: yaw -= rot; break;
    }
    std::cout << "View yaw " << yaw << " pitch " << pitch << std::endl;
}

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <file.fbx>" << std::endl;
        return 1;
    }
    std::cout << "Starting viewer" << std::endl;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("FBX Viewer");
    glEnable(GL_DEPTH_TEST);

    loadModel(argv[1]);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(arrows);

    glutMainLoop();
    return 0;
}


#include <GL/glut.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>

// Simple structure to hold mesh data
struct Vertex {
    float position[3];
    float normal[3];
    float texCoord[2];
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

std::vector<Mesh> meshes;

// Animation data
const aiScene* scene = nullptr;
Assimp::Importer importer;
float animationTime = 0.0f;

// Camera controls
float camX = 0.0f, camY = 0.0f, camZ = 5.0f;
float yaw = 0.0f, pitch = 0.0f;

void loadModel(const std::string& path) {
    scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals);
    if(!scene) {
        std::cerr << "Failed to load: " << path << std::endl;
        exit(1);
    }
    meshes.clear();
    for(unsigned int i = 0; i < scene->mNumMeshes; ++i) {
        aiMesh* aMesh = scene->mMeshes[i];
        Mesh mesh;
        mesh.vertices.resize(aMesh->mNumVertices);
        for(unsigned int v = 0; v < aMesh->mNumVertices; ++v) {
            aiVector3D pos = aMesh->mVertices[v];
            aiVector3D norm = aMesh->mNormals[v];
            aiVector3D uv = aMesh->HasTextureCoords(0) ? aMesh->mTextureCoords[0][v] : aiVector3D(0,0,0);
            mesh.vertices[v] = {{pos.x, pos.y, pos.z}, {norm.x, norm.y, norm.z}, {uv.x, uv.y}};
        }
        mesh.indices.reserve(aMesh->mNumFaces*3);
        for(unsigned int f = 0; f < aMesh->mNumFaces; ++f) {
            aiFace face = aMesh->mFaces[f];
            for(unsigned int j=0;j<face.mNumIndices;++j)
                mesh.indices.push_back(face.mIndices[j]);
        }
        meshes.push_back(std::move(mesh));
    }
}

void drawMeshes() {
    for(const Mesh& mesh : meshes) {
        glBegin(GL_TRIANGLES);
        for(unsigned int idx : mesh.indices) {
            const Vertex& v = mesh.vertices[idx];
            glNormal3fv(v.normal);
            glVertex3fv(v.position);
        }
        glEnd();
    }
}

void updateAnimation(float delta) {
    if(!scene || !scene->HasAnimations()) return;
    animationTime += delta;
    const aiAnimation* anim = scene->mAnimations[0];
    double ticksPerSecond = anim->mTicksPerSecond != 0 ? anim->mTicksPerSecond : 25.0;
    double timeInTicks = animationTime * ticksPerSecond;
    double animationTimeTicks = fmod(timeInTicks, anim->mDuration);

    // Apply first channel's rotation to whole model for demonstration
    if(anim->mNumChannels > 0) {
        aiNodeAnim* channel = anim->mChannels[0];
        if(channel->mNumRotationKeys > 0) {
            aiQuatKey& key = channel->mRotationKeys[0];
            aiQuaternion q = key.mValue;
            glRotatef(q.w*180.0f, q.x, q.y, q.z);
        }
    }
}

void display() {
    static int lastTime = glutGet(GLUT_ELAPSED_TIME);
    int currentTime = glutGet(GLUT_ELAPSED_TIME);
    float delta = (currentTime - lastTime) / 1000.0f;
    lastTime = currentTime;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(pitch, 1,0,0);
    glRotatef(yaw, 0,1,0);
    glTranslatef(-camX, -camY, -camZ);

    updateAnimation(delta);
    drawMeshes();

    glutSwapBuffers();
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w/h, 0.1f, 100.0f);
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
}

void arrows(int key, int, int) {
    const float rot = 2.0f;
    switch(key) {
        case GLUT_KEY_UP: pitch += rot; break;
        case GLUT_KEY_DOWN: pitch -= rot; break;
        case GLUT_KEY_LEFT: yaw += rot; break;
        case GLUT_KEY_RIGHT: yaw -= rot; break;
    }
}

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <file.fbx>" << std::endl;
        return 1;
    }
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


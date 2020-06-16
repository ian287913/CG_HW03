#pragma once
#include <Common.h>
#include <ResourcePath.h>

#include "OpenMesh.h"
#include "MeshObject.h"
#include "DrawModelShader.h"
#include "PickingShader.h"
#include "PickingTexture.h"
#include "DrawPickingFaceShader.h"
#include "DrawTextureShader.h"
#include "DrawPointShader.h"

#include "DrawTargetShader.h"
#include "MyGLMHelper.h"

using namespace glm;
using namespace std;


class MapPanel
{
public:
	MapPanel();
	~MapPanel();

	void Init();
	void DrawBackground();
	void DrawVertices();
	void DrawEdges();

	void CalculateBounderies();

	vector<vec3> vertices;
	vector<vec3> edges;


private:
	GLuint vbo;
	DrawPointShader myShader;



};

MapPanel::MapPanel()
{}
MapPanel::~MapPanel()
{}

void MapPanel::CalculateBounderies()
{

}



void MapPanel::Init()
{
	myShader.Init();
	glGenBuffers(1, &vbo);
	myShader.Enable();
	myShader.SetMVMat(scale(1.2, 1.2, 1) * translate(-0.5, -0.5, 0));
	myShader.SetPMat(translate(0, 0, 0));
	myShader.Disable();
}
void MapPanel::DrawBackground()
{
	//	clear screen
	glClearColor(0.8, 0.8, 0.8, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//	draw points
	vector<vec3> originPoints;
	originPoints.clear();
	originPoints.push_back(vec3(0, 0, 0));
	originPoints.push_back(vec3(0, 1, 0));
	originPoints.push_back(vec3(1, 0, 0));
	originPoints.push_back(vec3(1, 1, 0));

	/*for (int i = 0; i < 10; i++)
		for (int j = 0; j < 10; j++)
			targetPoints.push_back(vec3(0.1f * i, 0.1f * j, 0));*/

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * originPoints.size(), &originPoints[0].x, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glm::vec4 pointColor(0.0, 0.0, 1.0, 1.0);
	myShader.Enable();
	
	myShader.SetPointColor(pointColor);
	myShader.SetPointSize(8.0);

	glDrawArrays(GL_POINTS, 0, originPoints.size());

	myShader.Disable();

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
void MapPanel::DrawVertices()
{
	if (vertices.size() < 1) return;

	//	draw points
	/*vertices.clear();
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			vertices.push_back(vec3(0.25 * i, 0.25 * j, 0));
		}
	}*/

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * vertices.size(), &vertices[0].x, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glm::vec4 pointColor(1.0, 0.0, 0.0, 1.0);
	myShader.Enable();
	myShader.SetPointColor(pointColor);
	myShader.SetPointSize(5.0);

	glDrawArrays(GL_POINTS, 0, vertices.size());
	myShader.Disable();
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
void MapPanel::DrawEdges()
{
	if (edges.size() < 2) return;

	// 1rst attribute buffer : vertices
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * edges.size(), &edges[0].x, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	myShader.Enable();
	myShader.SetPointColor(vec4(0.0, 1.0, 0.0, 1.0));
	glLineWidth(3);

	glDrawArrays(GL_LINES, 0, edges.size());
	myShader.Disable();
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}



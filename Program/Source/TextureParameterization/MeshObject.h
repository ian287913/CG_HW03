#pragma once

#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <Common.h>

namespace OMT//OpenMesh Triangle mesh
{
	using namespace std;
	/*----------------------------------------------------------------------*/

	/*�w�q�ϥΪ���ǫשM���ݩ�*/
	struct MyTraits : OpenMesh::DefaultTraits
	{
		// let Point and Normal be a vector made from doubles
		typedef OpenMesh::Vec3d Point;
		typedef OpenMesh::Vec3d Normal;

		// add normal property to vertices and faces
		VertexAttributes(OpenMesh::Attributes::Normal);
		FaceAttributes(OpenMesh::Attributes::Normal);

		// Already defined in OpenMesh::DefaultTraits
		// HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );

		// Uncomment next line to disable attribute PrevHalfedge
		// HalfedgeAttributes( OpenMesh::Attributes::None );
		//
		// or
		//
		// HalfedgeAttributes( 0 );
	};
	/*----------------------------------------------------------------------*/

	/*�w�q�`��type*/
	typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>	    MyMesh;
	typedef OpenMesh::Vec3d									Vector3d;	//Vec3D type
	typedef MyMesh::Scalar									Scalar;	//Scalar type
	typedef MyMesh::Point									Point;	//Point type
	typedef MyMesh::Normal									Normal;	//Normal type
	typedef MyMesh::VertexHandle							VHandle;	//VertexHandle type
	typedef MyMesh::HalfedgeHandle							HEHandle;	//HalfedgeHandle type
	typedef MyMesh::EdgeHandle							    EHandle;	//edgeHandle type
	typedef MyMesh::FaceHandle								FHandle;	//FaceHandle type
	//-------------Vertex iterators & circulators-------------
	typedef MyMesh::VertexIter								VIter;	//VertexIter type
	typedef MyMesh::VertexVertexIter						VVIter;	//VertexVertexIter type
	typedef MyMesh::VertexEdgeIter							VEIter;	//VertexEdgeIter type
	typedef MyMesh::VertexFaceIter							VFIter;	//VertexFaceIter type
	typedef MyMesh::EdgeIter								EIter;	//EdgeIterT	type
	typedef MyMesh::FaceIter								FIter;	//FaceIter type
	typedef MyMesh::FaceVertexIter							FVIter;	//FaceVertexIter type
	typedef MyMesh::FaceEdgeIter							FEIter;	//FaceEdgeIter type
	typedef MyMesh::FaceHalfedgeIter						FHEIter;	//FaceEdgeIter type
	typedef MyMesh::FaceFaceIter							FFIter;	//FaceFaceIter type
	typedef MyMesh::VertexOHalfedgeIter						VOHEIter;	//VertexOutHalfEdge type
	typedef MyMesh::ConstVertexVertexIter					CVVIter;	//ConstVertexVertexIter type
	/*----------------------------------------------------------------------*/

	/*�w�q�B�~��Ƶ��c*/
	using namespace OpenMesh;
	/*----------------------------------------------------------------------*/

}

typedef OpenMesh::TriMesh_ArrayKernelT<>  TriMesh;

class MyMesh : public TriMesh
{
public:
	MyMesh();
	~MyMesh();

	int FindVertex(MyMesh::Point pointToFind);
	void ClearMesh();
};

class GLMesh
{
public:
	GLMesh();
	~GLMesh();

	bool Init(std::string fileName);
	void Render();

	MyMesh mesh;
	MyMesh *mesh_p;
	GLuint vao;
	GLuint ebo;
	GLuint vboVertices, vboNormal;

private:

	bool LoadModel(std::string fileName);
	void LoadToShader();
};

class MeshObject
{
public:
	MeshObject();
	~MeshObject();

	bool Init(std::string fileName);
	void Render();
	void RenderSelectedFace();
	bool AddSelectedFace(unsigned int faceID);
	void DeleteSelectedFace(unsigned int faceID);
	bool FindClosestPoint(unsigned int faceID, glm::vec3 worldPos, glm::vec3& closestPos);

	//	ian: try to get handles of selectedFaces
	std::vector<int> GetSelectedFaces();

private:
	GLMesh model;
	std::vector<unsigned int> selectedFace;
	std::vector<unsigned int*> fvIDsPtr;
	std::vector<int> elemCount;

};


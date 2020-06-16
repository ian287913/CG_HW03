#include "MeshObject.h"
#include <Eigen/Sparse>
#include <map>
#include <algorithm>
#include <stdio.h>

#include <STB/stb_image.h>

#define Quad
//#define Harmonic

struct OpenMesh::VertexHandle const OpenMesh::PolyConnectivity::InvalidVertexHandle;

#pragma region MyMesh

MyMesh::MyMesh()
{
	request_vertex_normals();
	request_vertex_status();
	request_face_status();
	request_edge_status();
}

MyMesh::~MyMesh()
{

}

int MyMesh::FindVertex(MyMesh::Point pointToFind)
{
	int idx = -1;
	for (MyMesh::VertexIter v_it = vertices_begin(); v_it != vertices_end(); ++v_it)
	{
		MyMesh::Point p = point(*v_it);
		if (pointToFind == p)
		{
			idx = v_it->idx();
			break;
		}
	}

	return idx;
}

void MyMesh::ClearMesh()
{
	if (!faces_empty())
	{
		for (MyMesh::FaceIter f_it = faces_begin(); f_it != faces_end(); ++f_it)
		{
			delete_face(*f_it, true);
		}

		garbage_collection();
	}
}

#pragma endregion

#pragma region GLMesh

GLMesh::GLMesh()
{

}

GLMesh::~GLMesh()
{

}

bool GLMesh::Init(std::string fileName)
{
	if (LoadModel(fileName))
	{
		LoadToShader();
		return true;
	}
	return false;
}

void GLMesh::Render()
{
	glBindVertexArray(vao);
	glDrawElements(GL_TRIANGLES, mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}


bool GLMesh::LoadModel(std::string fileName)
{
	OpenMesh::IO::Options ropt;
	if (OpenMesh::IO::read_mesh(mesh, fileName, ropt))
	{
		if (!ropt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_vertex_normals())
		{
			mesh.request_face_normals();
			mesh.update_normals();
			mesh.release_face_normals();
		}

		return true;
	}

	return false;
}

void GLMesh::LoadToShader()
{
	std::vector<MyMesh::Point> vertices;
	vertices.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vertices.push_back(mesh.point(*v_it));

		MyMesh::Point p = mesh.point(*v_it);
	}

	std::vector<MyMesh::Normal> normals;
	normals.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		normals.push_back(mesh.normal(*v_it));
	}

	std::vector<unsigned int> indices;
	indices.reserve(mesh.n_faces() * 3);
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
	{
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
		{
			indices.push_back(fv_it->idx());
		}
	}

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vboVertices);
	glBindBuffer(GL_ARRAY_BUFFER, vboVertices);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Point) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &vboNormal);
	glBindBuffer(GL_ARRAY_BUFFER, vboNormal);
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::Normal) * normals.size(), &normals[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glGenBuffers(1, &ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), &indices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void GLMesh::init()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// VAO
	glDeleteVertexArrays(1, &vao);
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// Load shader
	// setShader("vertex.vs.glsl", "fragment.fs.glsl");

	glUseProgram(shaderProgram);
	MatricesIdx = glGetUniformBlockIndex(shaderProgram, "MatVP");
	ModelID = glGetUniformLocation(shaderProgram, "Model");
	M_KaID = glGetUniformLocation(shaderProgram, "Material.Ka");
	M_KdID = glGetUniformLocation(shaderProgram, "Material.Kd");
	M_KsID = glGetUniformLocation(shaderProgram, "Material.Ks");
	// VBO
	load2Buffer();
	// Set background color
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glUseProgram(0);
}

#pragma endregion

MeshObject::MeshObject()
{
	
}

MeshObject::~MeshObject()
{
}

bool MeshObject::Init(std::string fileName)
{
	selectedFace.clear();

	return model.Init(fileName);
}

void MeshObject::Render()
{
	glBindVertexArray(model.vao);
	glDrawElements(GL_TRIANGLES, model.mesh.n_faces() * 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void MeshObject::RenderSelectedFace()
{
	if (selectedFace.size() > 0)
	{
		std::vector<unsigned int*> offsets(selectedFace.size());
		for (int i = 0; i < offsets.size(); ++i)
		{
			offsets[i] = (GLuint*)(selectedFace[i] * 3 * sizeof(GLuint));
		}

		std::vector<int> count(selectedFace.size(), 3);

		glBindVertexArray(model.vao);
		glMultiDrawElements(GL_TRIANGLES, &count[0], GL_UNSIGNED_INT, (const GLvoid **)&offsets[0], selectedFace.size());
		glBindVertexArray(0);
	}
}

bool MeshObject::AddSelectedFace(unsigned int faceID)
{
	if (std::find(selectedFace.begin(), selectedFace.end(), faceID) == selectedFace.end() &&
		faceID >= 0 && faceID < model.mesh.n_faces())
	{
		selectedFace.push_back(faceID);
		return true;
	}
	return false;
}

void MeshObject::DeleteSelectedFace(unsigned int faceID)
{
	selectedFace.erase(std::remove(selectedFace.begin(), selectedFace.end(), faceID), selectedFace.end());
}

std::vector<glm::vec3> MeshObject::GetSelectedFaces()
{
	std::vector<glm::vec3> foundEdges;
	foundEdges.clear();

	std::cout << "selectedFace: \n";

	for (int i = 0; i < selectedFace.size(); i++)
	{

		/*for (OpenMesh::PolyConnectivity::FVIter fv_it = model.mesh.fv_iter(model.mesh.face_handle(selectedFace[i]));
			fv_it != OMT::fv_end(chosenFace_iter); ++fv_it)
		{
			std::cout << "V\n";

			///OMT::Point v = point(fv_it.handle());
			///glVertex3dv(v.data());
		}*/

		
		std::cout << selectedFace[i] << "\n";
	}

	return foundEdges;
}


bool MeshObject::FindClosestPoint(unsigned int faceID, glm::vec3 worldPos, glm::vec3& closestPos)
{
	OpenMesh::FaceHandle fh = model.mesh.face_handle(faceID);
	if (!fh.is_valid())
	{
		return false;
	}
	
	double minDistance = 0.0;
	MyMesh::Point p(worldPos.x, worldPos.y, worldPos.z);
	MyMesh::FVIter fv_it = model.mesh.fv_iter(fh);
	MyMesh::VertexHandle closestVH = *fv_it;
	MyMesh::Point v1 = model.mesh.point(*fv_it);
	++fv_it;

	minDistance = (p - v1).norm();
	for (; fv_it.is_valid(); ++fv_it)
	{
		MyMesh::Point v = model.mesh.point(*fv_it);
		double distance = (p - v).norm();
		if (minDistance > distance)
		{
			minDistance = distance;
			closestVH = *fv_it;
		}
	}
	MyMesh::Point closestPoint = model.mesh.point(closestVH);
	closestPos.x = closestPoint[0];
	closestPos.y = closestPoint[1];
	closestPos.z = closestPoint[2];
	return true;
}

// Will
void MeshObject::reInitPatch()
{
	patch.mesh.clear();

	std::vector<MyMesh::VertexHandle> face_vhs;
	std::map<int, MyMesh::VertexHandle> vhs;


	for (std::map<int, OMT::FIter>::iterator it_m = sellectedFace_iterMap.begin(); it_m != sellectedFace_iterMap.end(); ++it_m) {
		face_vhs.clear();

		for (OMT::FVIter fv_it = model.mesh.fv_begin((*it_m).second); fv_it != model.mesh.fv_end((*it_m).second); ++fv_it) {
			MyMesh::VertexHandle vh;
			std::map<int, MyMesh::VertexHandle>::iterator vhs_it = vhs.find((*fv_it).idx());

			// not found
			if (vhs_it == vhs.end()) {
				MyMesh::Point p = MyMesh::Point(model.mesh.point(*fv_it)[0], model.mesh.point(*fv_it)[1], model.mesh.point(*fv_it)[2]);
				vh = patch.mesh.add_vertex(p);
				vhs.insert(std::pair<int, MyMesh::VertexHandle>(fv_it->idx(), vh));
			}
			// found
			else
				vh = vhs_it->second;

			face_vhs.push_back(vh);
		}

		patch.mesh.add_face(face_vhs);
	}

	patch.init();

	patch.selectedFaceSet.clear();
	patch.selectedFace.clear();

	for (OMT::FIter f_it = patch.mesh.faces_begin(); f_it != patch.mesh.faces_end(); ++f_it)
		patch.selectedFaceSet.insert(f_it->idx());
	for (std::set<unsigned int>::iterator it_s = patch.selectedFaceSet.begin(); it_s != patch.selectedFaceSet.end(); ++it_s)
		patch.selectedFace.push_back(*it_s);

	float uvRotateAngle = 0.0f;
	// Parameterization
	patch.Parameterization(uvRotateAngle);
}
void GLMesh::Parameterization(float uvRotateAngle) {
	texCoords.clear();
	edgeConnectivity.clear();

	if (selectedFace.size() <= 0)
		return;

	std::sort(selectedFace.begin(), selectedFace.end());

	OpenMesh::HPropHandleT<double> heWeight;
	OpenMesh::VPropHandleT<int> row;

	mesh.add_property(heWeight, "heWeight");
	mesh.add_property(row, "row");
	mesh.request_vertex_texcoords2D();

	//calculate weight
	MyMesh::HalfedgeHandle heh;
	for (MyMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		if (!mesh.is_boundary((*e_it))) {
			GLdouble angle1, angle2, w;

			MyMesh::HalfedgeHandle _heh = mesh.halfedge_handle((*e_it), 0);
			MyMesh::Point pFrom = mesh.point(mesh.from_vertex_handle(_heh));
			MyMesh::Point pTo = mesh.point(mesh.to_vertex_handle(_heh));
			//Tri_Mesh::Point p1 = point(mesh.opposite_vh(_heh));
			//Tri_Mesh::Point p2 = point(mesh.opposite_he_opposite_vh(_heh));
			MyMesh::Point p1 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(_heh)));
			MyMesh::Point p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(_heh))));

			double edgeLen = (pFrom - pTo).length();

			// weight from to
			OpenMesh::Vec3d v1 = (OpenMesh::Vec3d)(pTo - pFrom);
			v1.normalize();

			OpenMesh::Vec3d v2 = (OpenMesh::Vec3d)(p1 - pFrom);
			v2.normalize();

			angle1 = std::acos(OpenMesh::dot(v1, v2));

			v2 = (OpenMesh::Vec3d)(p2 - pFrom);
			v2.normalize();

			angle2 = std::acos(OpenMesh::dot(v1, v2));

			w = (std::tan(angle1 / 2.0f) + std::tan(angle2 / 2.0f)) / edgeLen;

			mesh.property(heWeight, mesh.opposite_halfedge_handle(_heh)) = w;
		}

		else {
			if (!heh.is_valid())
				heh = mesh.halfedge_handle((*e_it), 1);
		}
	}

	// calculate matrix size
	int count = 0;
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (mesh.is_boundary(*v_it)) {
			mesh.property(row, *v_it) = -1;
		}
		else {
			mesh.property(row, *v_it) = count++;
		}
	}

	// calculate perimeter
	double perimeter = 0;
	std::vector<double> segLength;
	std::vector<MyMesh::VertexHandle> vhs;
	MyMesh::HalfedgeHandle hehNext = heh;
	do {
		MyMesh::Point from = mesh.point(mesh.from_vertex_handle(hehNext));
		MyMesh::Point to = mesh.point(mesh.to_vertex_handle(hehNext));
		perimeter += (from - to).length();

		segLength.push_back(perimeter);
		vhs.push_back(mesh.from_vertex_handle(hehNext));

		hehNext = mesh.next_halfedge_handle(hehNext);
	} while (heh != hehNext);

	MyMesh::TexCoord2D st(0.0, 0.0);

	mesh.set_texcoord2D(vhs[0], MyMesh::TexCoord2D(1.0, 0.5));

	for (int i = 1; i < vhs.size(); ++i) {
		double angle = 2 * M_PI * segLength[i - 1] / perimeter;

		st[0] = (std::cos(angle) + 1) / 2;
		st[1] = (std::sin(angle) + 1) / 2;


		mesh.set_texcoord2D(vhs[i], st);
	}


	typedef Eigen::SparseMatrix<double> SpMat;

	SpMat A(count, count);
	Eigen::VectorXd BX(count);
	Eigen::VectorXd BY(count);
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > linearSolver;

	BX.setZero();
	BY.setZero();

	// fill matrix
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (!mesh.is_boundary(*v_it)) {
			int i = mesh.property(row, *v_it);
			double totalWeight = 0;

			for (MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it) {
				MyMesh::HalfedgeHandle _heh = mesh.find_halfedge(*v_it, *vv_it);
				double w = mesh.property(heWeight, _heh);

				if (mesh.is_boundary(*vv_it)) {
					MyMesh::TexCoord2D texCoord = mesh.texcoord2D(*vv_it);
					BX[i] += w * texCoord[0];
					BY[i] += w * texCoord[1];
				}

				else {
					int j = mesh.property(row, *vv_it);
					A.insert(i, j) = -w;
				}

				totalWeight += w;
			}

			A.insert(i, i) = totalWeight;
		}
	}

	A.makeCompressed();

	// solve linear system
	SpMat At = A.transpose();
	linearSolver.compute(At * A);

	Eigen::VectorXd TX = linearSolver.solve(At * BX);
	Eigen::VectorXd TY = linearSolver.solve(At * BY);

	// set texcoord
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (!mesh.is_boundary(*v_it)) {
			int i = mesh.property(row, *v_it);
			mesh.set_texcoord2D(*v_it, MyMesh::TexCoord2D(TX[i], TY[i]));
		}
	}

	// request vertex texcoord, if not exist
	if (!mesh.has_vertex_texcoords2D()) {
		mesh.request_vertex_texcoords2D();
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
			mesh.set_texcoord2D(*v_it, MyMesh::TexCoord2D(-1, -1));
		}
	}

	// map texcoord back to origin mesh
	int index = 0;
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
		MyMesh::FaceHandle fh = *f_it;
		MyMesh::FaceHandle selectedFace_h = mesh.face_handle(selectedFace[index++]);

		MyMesh::FaceVertexIter fv_it = mesh.fv_iter(fh);
		MyMesh::FaceVertexIter selectedfv_it = mesh.fv_iter(selectedFace_h);

		for (; fv_it.is_valid() && selectedfv_it.is_valid(); ++fv_it, ++selectedfv_it) {
			MyMesh::TexCoord2D texCoord = mesh.texcoord2D(*fv_it);
			mesh.set_texcoord2D(*selectedfv_it, texCoord);
		}
	}

	LoadTexCoordToShader();

	fvIDsPtr.swap(std::vector<unsigned int*>(selectedFace.size()));
	for (int i = 0; i < fvIDsPtr.size(); ++i) {
		fvIDsPtr[i] = (GLuint*)(selectedFace[i] * 3 * sizeof(GLuint));
	}
	elemCount.swap(std::vector<int>(selectedFace.size(), 3));

	//puts("selectedFace: ");
	//for (std::vector<unsigned int>::iterator it = selectedFace.begin(); it != selectedFace.end(); it++) {
	//	printf("%u, ", *it);
	//}
	//puts("");

	return;
}
void GLMesh::LoadTexCoordToShader() {
	if (mesh.has_vertex_texcoords2D()) {
		texCoords.clear();
		edgeConnectivity.clear();
		for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
			MyMesh::TexCoord2D texCoord = mesh.texcoord2D(*v_it);
			// Get Point
			texCoords.push_back(texCoord);

			// Get Edge
			for (MyMesh::VVIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it) {
				// from
				edgeConnectivity.push_back(glm::vec2(texCoord[0], texCoord[1]));

				// to
				MyMesh::TexCoord2D thenTexCoord = mesh.texcoord2D(*vv_it);
				edgeConnectivity.push_back(glm::vec2(thenTexCoord[0], thenTexCoord[1]));
			}
		}

		glUseProgram(texProgram);
		glBindVertexArray(vao);

		glGenBuffers(1, &texCoordVBO);
		glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(MyMesh::TexCoord2D) * texCoords.size(), &texCoords[0], GL_STATIC_DRAW);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);
	}

	return;
}
// Load Obj to buffer
void GLMesh::load2Buffer()
{
	// Get vertices form mesh
	vertices.clear();
	verticesIdx.clear();
	for (OMT::VIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		float *vertexMesh = mesh.point(*v_it).data();
		glm::vec3 vertexGLM(vertexMesh[0], vertexMesh[1], vertexMesh[2]);
		vertices.push_back(vertexGLM);

		verticesIdx.push_back(glm::vec3((float)v_it->idx()));
	}
	verticesIdxSize = verticesIdx.size();
	std::cout << "Vertex Number: " << vertices.size() << std::endl;

	// Set mesh's face vertex indices
	std::vector<GLfloat> faceIndicesVec;
	faceIndicesVec.clear();
	faceIndex.clear();
	faceVertexIndex.clear();
	for (OMT::FIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
	{
		faceIndicesVec.push_back((float)(f_it->idx()));
#ifdef TEST
		std::cout << "Face" << *f_it << ": ";
#endif // TEST
		for (OMT::FVIter fv_it = mesh.fv_begin(*f_it); fv_it != mesh.fv_end(*f_it); ++fv_it)
		{
#ifdef TEST
			std::cout << *fv_it << " ";
#endif // TEST
			faceVertexIndex.push_back(fv_it->idx());
			faceIndex.push_back(glm::vec3((float)(f_it->idx())));
		}
#ifdef TEST
		std::cout << std::endl;
#endif // TEST
	}
	faceIndices = new GLfloat[faceIndicesVec.size()];
	for (int u = 0; u < faceIndicesVec.size(); u++)
		faceIndices[u] = faceIndicesVec[u];
	faceVertexSize = faceVertexIndex.size();
	std::cout << "Face Number: " << faceIndicesVec.size() << "\n\n";

	// Set mesh's edge vertex indices
	edgeVertexIndex.clear();
	for (OMT::EIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
	{
#ifdef TEST
		std::cout << "Edge" << *e_it << ": ";
#endif // TEST	
		GLuint from;
		from = mesh.from_vertex_handle(mesh.halfedge_handle((*e_it), 0)).idx();
		GLuint to;
		to = mesh.to_vertex_handle(mesh.halfedge_handle((*e_it), 0)).idx();
#ifdef TEST
		std::cout << from << " -> " << to << std::endl;
#endif // TEST
		edgeVertexIndex.push_back(to);
		edgeVertexIndex.push_back(from);
	}
	edgeVertexSize = edgeVertexIndex.size();

	if (vertices.size() <= 0 || verticesIdx.size() <= 0 || faceIndex.size() <= 0 || faceVertexIndex.size() <= 0 || edgeVertexIndex.size() <= 0)
		return;

	// Point VBO
	glGenBuffers(1, &pointVBO);
	glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);
	verticesSize = vertices.size();

	// Point ID VBO
	glGenBuffers(1, &pointIdxVBO);
	glBindBuffer(GL_ARRAY_BUFFER, pointIdxVBO);
	glBufferData(GL_ARRAY_BUFFER, verticesIdx.size() * sizeof(glm::vec3), &verticesIdx[0], GL_STATIC_DRAW);

	// Face ID VBO
	glGenBuffers(1, &faceIdxVBO);
	glBindBuffer(GL_ARRAY_BUFFER, faceIdxVBO);
	glBufferData(GL_ARRAY_BUFFER, faceIndex.size() * sizeof(glm::vec3), &faceIndex[0], GL_STATIC_DRAW);

	// UV VBO
	glGenBuffers(1, &texCoordVBO);
	glBindBuffer(GL_ARRAY_BUFFER, texCoordVBO);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), NULL, GL_STATIC_DRAW);
	uvsSize = uvs.size();

	// Normal VBO
	glGenBuffers(1, &normalVBO);
	glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), NULL, GL_STATIC_DRAW);
	normalsSize = normals.size();

	// Point EBO
	glGenBuffers(1, &pointEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pointEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, faceVertexIndex.size() * sizeof(GLint), &faceVertexIndex[0], GL_STATIC_DRAW);

	// Edge EBO
	glGenBuffers(1, &edgeEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, edgeVertexIndex.size() * sizeof(GLint), &edgeVertexIndex[0], GL_STATIC_DRAW);

	std::vector<MyMesh::Normal> normals;
	normals.reserve(mesh.n_vertices());
	for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		normals.push_back(mesh.normal(*v_it));
	}
}
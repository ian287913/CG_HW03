#include "DrawTargetShader.h"
#include "ResourcePath.h"

DrawTargetShader::DrawTargetShader()
{
}

DrawTargetShader::~DrawTargetShader()
{
}

bool DrawTargetShader::Init()
{
	if (!ShaderObject::Init())
	{
		return false;
	}

	if (!AddShader(GL_VERTEX_SHADER, ResourcePath::shaderPath + "drawPoint.vs.glsl"))
	{
		return false;
	}

	if (!AddShader(GL_FRAGMENT_SHADER, ResourcePath::shaderPath + "drawPoint.fs.glsl"))
	{
		return false;
	}

	if (!Finalize())
	{
		return false;
	}

	um4mvLocation = GetUniformLocation("um4mv");
	if (um4mvLocation == -1)
	{
		puts("Get uniform loaction error: um4mv");
		return false;
	}

	um4pLocation = GetUniformLocation("um4p");
	if (um4pLocation == -1)
	{
		puts("Get uniform loaction error: um4p");
		return false;
	}

	pointColorLocation = GetUniformLocation("pointColor");
	if (pointColorLocation == -1)
	{
		puts("Get uniform loaction error: pointColor");
		return false;
	}


	return true;
}

void DrawTargetShader::Enable()
{
	ShaderObject::Enable();
	glPushAttrib(GL_POINT_BIT);
}

void DrawTargetShader::Disable()
{
	glPopAttrib();
	ShaderObject::Disable();
}

void DrawTargetShader::SetMVMat(const glm::mat4& mat)
{
	glUniformMatrix4fv(um4mvLocation, 1, GL_FALSE, glm::value_ptr(mat));
}

void DrawTargetShader::SetPMat(const glm::mat4& mat)
{
	glUniformMatrix4fv(um4pLocation, 1, GL_FALSE, glm::value_ptr(mat));
}

void DrawTargetShader::SetPointColor(const glm::vec4& pointColor)
{
	glUniform4fv(pointColorLocation, 1, glm::value_ptr(pointColor));
}

void DrawTargetShader::SetPointSize(float pointSize)
{
	glPointSize(pointSize);
}

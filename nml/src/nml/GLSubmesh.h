/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_NML_GLSUBMESH_H_
#define _CARTO_NML_GLSUBMESH_H_

#include "GLBase.h"

#include <memory>
#include <string>
#include <vector>

namespace carto::nml {
    class Submesh;
    class SubmeshOpList;
    class GLMesh;
    class GLResourceManager;

    class GLSubmesh final : public std::enable_shared_from_this<GLSubmesh> {
    public:
        explicit GLSubmesh(const Submesh& submesh);
        explicit GLSubmesh(const GLMesh& glMesh, const SubmeshOpList& submeshOpList);

        void create(GLResourceManager& resourceManager);

        void draw(GLResourceManager& resourceManager, const RenderState& renderState);

        void calculateRayIntersections(const cglib::ray3<double>& ray, std::vector<RayIntersection>& intersections) const;

        const std::string& getMaterialId() const;

        int getDrawCallCount() const;
        int getTotalGeometrySize() const;

    private:
        void uploadSubmesh(GLResourceManager& resourceManager);
        
        static GLint convertType(int type);
        static void convertToFloatBuffer(const std::string& str, std::vector<float>& buf);
        static void convertToByteBuffer(const std::string& str, std::vector<unsigned char>& buf);

        GLint _glType;
        std::vector<int> _vertexCounts;
        std::string _materialId;

        std::vector<float> _positionBuffer;
        std::vector<float> _normalBuffer;
        std::vector<float> _uvBuffer;
        std::vector<unsigned char> _colorBuffer;
        std::vector<unsigned int> _vertexIdBuffer;

        GLuint _glPositionVBOId;
        GLuint _glNormalVBOId;
        GLuint _glUVVBOId;
        GLuint _glColorVBOId;
    };
}

#endif

/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_VT_GLEXTENSIONS_H_
#define _CARTO_VT_GLEXTENSIONS_H_

#include <string>

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

namespace carto { namespace vt {
    class GLExtensions final {
    public:
        GLExtensions();

        bool GL_OES_vertex_array_object_supported() const { return _GL_OES_vertex_array_object_supported; }
        void glBindVertexArrayOES(GLuint array);
        void glDeleteVertexArraysOES(GLsizei n, const GLuint* arrays);
        void glGenVertexArraysOES(GLsizei n, GLuint* arrays);
        GLboolean glIsVertexArrayOES(GLuint array);

        bool GL_EXT_discard_framebuffer_supported() const { return _GL_EXT_discard_framebuffer_supported; }
        void glDiscardFramebufferEXT(GLenum target, GLsizei numAttachments, const GLenum* attachments);

        bool GL_EXT_texture_filter_anisotropic_supported() const { return _GL_EXT_texture_filter_anisotropic_supported; }

        bool GL_OES_packed_depth_stencil_supported() const { return _GL_OES_packed_depth_stencil_supported; }

        bool GL_OES_standard_derivatives_supported() const { return _GL_OES_standard_derivatives_supported; }

    private:
        bool _GL_OES_vertex_array_object_supported = false;
        bool _GL_EXT_discard_framebuffer_supported = false;
        bool _GL_EXT_texture_filter_anisotropic_supported = false;
        bool _GL_OES_packed_depth_stencil_supported = false;
        bool _GL_OES_standard_derivatives_supported = false;

#ifdef GL_OES_vertex_array_object
        PFNGLBINDVERTEXARRAYOESPROC _glBindVertexArrayOES = nullptr;
        PFNGLDELETEVERTEXARRAYSOESPROC _glDeleteVertexArraysOES = nullptr;
        PFNGLGENVERTEXARRAYSOESPROC _glGenVertexArraysOES = nullptr;
        PFNGLISVERTEXARRAYOESPROC _glIsVertexArrayOES = nullptr;
#endif

#ifdef GL_EXT_discard_framebuffer
        PFNGLDISCARDFRAMEBUFFEREXTPROC _glDiscardFramebufferEXT = nullptr;
#endif
    };
} }

#endif

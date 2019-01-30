/*
 * Copyright (c) 2016 CartoDB. All rights reserved.
 * Copying and using this code is allowed only according
 * to license terms, as given in https://cartodb.com/terms/
 */

#ifndef _CARTO_VT_VIEWSTATE_H_
#define _CARTO_VT_VIEWSTATE_H_

#include <cmath>
#include <array>

#include <cglib/vec.h>
#include <cglib/mat.h>
#include <cglib/bbox.h>
#include <cglib/frustum3.h>

namespace carto { namespace vt {
    struct ViewState final {
        float zoom;
        float aspect;
        float resolution;
        float zoomScale;
        cglib::vec3<double> origin;
        cglib::frustum3<double> frustum;
        std::array<cglib::vec3<float>, 3> orientation;

        explicit ViewState(const cglib::mat4x4<double>& projectionMatrix, const cglib::mat4x4<double>& cameraMatrix, float zoom, float aspect, float resolution, float scaleFactor) : zoom(zoom), aspect(aspect), resolution(resolution), zoomScale(std::pow(2.0f, -zoom) * scaleFactor) {
            cglib::mat4x4<double> invCameraMatrix = cglib::inverse(cameraMatrix);
            origin = cglib::proj_p(cglib::col_vector(invCameraMatrix, 3));
            frustum = cglib::gl_projection_frustum(projectionMatrix * cameraMatrix);
            for (int i = 0; i < 3; i++) {
                orientation[i] = cglib::vec3<float>::convert(cglib::proj_o(cglib::col_vector(invCameraMatrix, i)));
            }
        }
    };
} }

#endif

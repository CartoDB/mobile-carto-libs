#include "BitmapCanvas.h"
#include "Color.h"

#include <cglib/vec.h>

namespace {
    float calculateLineDistance(const cglib::vec2<float>& p, const cglib::vec2<float>& p1, const cglib::vec2<float>& p2) {
        cglib::vec2<float> center = (p1 + p2) * 0.5f;
        float len = cglib::length(p2 - p1);
        cglib::vec2<float> dir = (p2 - p1) * (1.0f / len);
        return cglib::dot_product(p - center, cglib::vec2<float>(dir(1), -dir(0)));
    }
}

namespace carto { namespace vt {
    BitmapCanvas::BitmapCanvas(int width, int height, bool sdfMode) : _width(width), _height(height), _sdfMode(sdfMode), _data(width * height) {
    }

    void BitmapCanvas::setOpacity(float opacity) {
        _opacity = std::max(0.0f, std::min(1.0f, opacity));
    }

    void BitmapCanvas::setColor(const Color& color) {
        _color = color.rgba8();
    }

    void BitmapCanvas::drawPixel(int x, int y, float a) {
        std::uint8_t* frameColor = reinterpret_cast<std::uint8_t*>(&_data.at(y * _width + x));
        if (a >= 1.0f && _opacity >= 1.0f) {
            for (int c = 0; c < 4; c++) {
                frameColor[c] = _color[c];
            }
        }
        else if (a > 0.0f) {
            int factor = static_cast<int>(std::max(0.0f, std::min(1.0f, a)) * _opacity * 255.0f);
            int blend1 = 1 + factor;
            int blend2 = 255 - factor;
            for (int c = 0; c < 4; c++) {
                int comp1 = _color[c] * blend1;
                int comp2 = frameColor[c] * blend2;
                frameColor[c] = static_cast<std::uint8_t>((comp1 + comp2) >> 8);
            }
        }
    }

    void BitmapCanvas::drawSDFPixel(int x, int y, float dist) {
        std::uint8_t* frameColor = reinterpret_cast<std::uint8_t*>(&_data.at(y * _width + x));
        std::uint8_t val = static_cast<std::uint8_t>(std::max(0.0f, std::min(255.0f, dist * (128.0f / BITMAP_SDF_SCALE) + 127.5f)));
        for (int c = 0; c < 4; c++) {
            frameColor[c] = std::max(frameColor[c], val);
        }
    }

    void BitmapCanvas::drawRectangle(float x0, float y0, float x1, float y1) {
        if (x0 >= x1 || y0 >= y1) {
            return;
        }
        for (int y = 0; y < _height; y++) {
            float yd0 = 0.5f + y + 0.5f - y0;
            float yd1 = 0.5f + y1 - y - 0.5f;
            if (!_sdfMode && (yd0 <= 0 || yd1 <= 0)) {
                continue;
            }
            for (int x = 0; x < _width; x++) {
                float xd0 = 0.5f + x + 0.5f - x0;
                float xd1 = 0.5f + x1 - x - 0.5f;
                if (!_sdfMode && (xd0 <= 0 || xd1 <= 0)) {
                    continue;
                }

                if (_sdfMode) {
                    drawSDFPixel(x, y, std::min(std::min(xd0, xd1), std::min(yd0, yd1)));
                }
                else {
                    drawPixel(x, y, std::min(1.0f, xd0) * std::min(1.0f, xd1) * std::min(1.0f, yd0) * std::min(1.0f, yd1));
                }
            }
        }
    }
    
    void BitmapCanvas::drawTriangle(float x0, float y0, float x1, float y1, float x2, float y2) {
        cglib::vec2<float> p0(x0, y0);
        cglib::vec2<float> p1(x1, y1);
        cglib::vec2<float> p2(x2, y2);
        for (int y = 0; y < _height; y++) {
            for (int x = 0; x < _width; x++) {
                cglib::vec2<float> p(x + 0.5f, y + 0.5f);
                float d1 = 0.5f + calculateLineDistance(p, p0, p1);
                float d2 = 0.5f + calculateLineDistance(p, p1, p2);
                float d3 = 0.5f + calculateLineDistance(p, p2, p0);
                if (!_sdfMode && (d1 <= 0 || d2 <= 0 || d3 <= 0)) {
                    continue;
                }
                
                if (_sdfMode) {
                    drawSDFPixel(x, y, std::min(d1, std::min(d2, d3)));
                }
                else {
                    drawPixel(x, y, std::min(1.0f, d1) * std::min(1.0f, d2) * std::min(1.0f, d3));
                }
            }
        }
    }
    
    void BitmapCanvas::drawEllipse(float x0, float y0, float rx, float ry) {
        if (rx <= 0 || ry <= 0) {
            return;
        }
        float rscale = std::min(rx, ry);
        float xscale = 1.0f / rx;
        float yscale = 1.0f / ry;
        for (int y = 0; y < _height; y++) {
            float yd = (y + 0.5f - y0) * yscale;
            for (int x = 0; x < _width; x++) {
                float xd = (x + 0.5f - x0) * xscale;
                float rd = 0.5f + (1 - xd * xd - yd * yd) * rscale;
                if (!_sdfMode && rd <= 0) {
                    continue;
                }

                if (_sdfMode) {
                    drawSDFPixel(x, y, rd);
                }
                else {
                    drawPixel(x, y, std::min(1.0f, rd));
                }
            }
        }
    }

    std::shared_ptr<BitmapImage> BitmapCanvas::buildBitmapImage() const {
        return std::make_shared<BitmapImage>(_sdfMode, 1.0f, std::make_shared<Bitmap>(_width, _height, _data));
    }
}}

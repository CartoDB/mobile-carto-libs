#include "FontManager.h"
#include "Font.h"
#include "GlyphMap.h"

#include <atomic>
#include <mutex>
#include <memory>
#include <array>
#include <map>
#include <unordered_map>

#include <freetype/freetype.h>
#include <freetype/ftsnames.h>
#include <freetype/ttnameid.h>
#include <freetype/ftrender.h>
#include <freetype/ftstroke.h>
#include <freetype/ftmodapi.h>

#include <hb.h>
#include <hb-ft.h>

namespace carto { namespace vt {
    class FontManagerLibrary {
    public:
        FontManagerLibrary() : _library(nullptr) {
            std::lock_guard<std::recursive_mutex> lock(_mutex);

            FT_Init_FreeType(&_library);

            FT_Int spread = std::min(RENDER_SPREAD, static_cast<int>(BITMAP_SDF_SCALE));
            FT_Property_Set(_library, "sdf", "spread", &spread);
        }

        ~FontManagerLibrary() {
            std::lock_guard<std::recursive_mutex> lock(_mutex);

            FT_Done_FreeType(_library);
            _library = nullptr;
        }

        FT_Library getLibrary() const {
            return _library;
        }

        std::recursive_mutex& getMutex() const {
            return _mutex;
        }

    private:
        inline static constexpr int RENDER_SPREAD = 3; // NOTE: keep it equal or smaller than BITMAP_SDF_SCALE

        FT_Library _library;
        static std::recursive_mutex _mutex; // use global lock as harfbuzz is not thread-safe on every platform
    };

    std::recursive_mutex FontManagerLibrary::_mutex;

    class FontManagerFont : public Font {
    public:
        explicit FontManagerFont(const std::shared_ptr<FontManagerLibrary>& library, const std::shared_ptr<GlyphMap>& glyphMap, const std::vector<unsigned char>* data, const std::shared_ptr<const Font>& baseFont) : _library(library), _baseFont(baseFont), _glyphMap(glyphMap), _face(nullptr), _font(nullptr) {
            std::lock_guard<std::recursive_mutex> lock(_library->getMutex());

            // Load FreeType font
            if (data) {
                int error = FT_New_Memory_Face(_library->getLibrary(), data->data(), static_cast<FT_Long>(data->size()), 0, &_face);
                if (error == 0) {
                    error = FT_Set_Char_Size(_face, 0, static_cast<int>(RENDER_SIZE * 64.0f), 0, 0);
                }
            }

            // Create HarfBuzz font
            if (_face) {
                _font = hb_ft_font_create(_face, nullptr);
                if (_font) {
                    hb_ft_font_set_funcs(_font);
                }
            }
            
            // Initialize HarfBuzz buffer for glyph shaping
            _buffer = hb_buffer_create();
            if (_buffer) {
                hb_buffer_set_unicode_funcs(_buffer, hb_unicode_funcs_get_default());
            }
        }

        virtual ~FontManagerFont() {
            std::lock_guard<std::recursive_mutex> lock(_library->getMutex());
            
            if (_buffer) {
                hb_buffer_destroy(_buffer);
                _buffer = nullptr;
            }

            if (_font) {
                hb_font_destroy(_font);
                _font = nullptr;
            }

            if (_face) {
                FT_Done_Face(_face);
                _face = nullptr;
            }
        }

        virtual Metrics getMetrics(float size) const override {
            float ascent = _face->size->metrics.ascender / 64.0f * size / RENDER_SIZE;
            float descent = _face->size->metrics.descender / 64.0f * size / RENDER_SIZE;
            float height = _face->size->metrics.height / 64.0f * size / RENDER_SIZE;
            return Metrics(ascent, descent, height);
        }

        virtual std::vector<Glyph> shapeGlyphs(const std::uint32_t* utf32Text, std::size_t len, float size, bool rtl) const override {
            std::lock_guard<std::recursive_mutex> lock(_library->getMutex());

            // Find first font that covers all the characters. If not possible, use the last
            unsigned int fontId = 0;
            const FontManagerFont* font = nullptr;
            for (const FontManagerFont* currentFont = this; currentFont; fontId++) {
                if (currentFont->_font) {
                    font = currentFont;
                    hb_buffer_clear_contents(_buffer);
                    hb_buffer_add_utf32(_buffer, utf32Text, static_cast<unsigned int>(len), 0, static_cast<unsigned int>(len));
                    hb_buffer_set_direction(_buffer, rtl ? HB_DIRECTION_RTL : HB_DIRECTION_LTR);
                    hb_buffer_guess_segment_properties(_buffer);
                    hb_shape(font->_font, _buffer, nullptr, 0);

                    unsigned int infoCount = 0;
                    const hb_glyph_info_t* info = hb_buffer_get_glyph_infos(_buffer, &infoCount);
                    bool allValid = std::all_of(info, info + infoCount, [](const hb_glyph_info_t& glyphInfo) { return glyphInfo.codepoint != 0; });
                    if (allValid) {
                        break;
                    }
                }

                currentFont = dynamic_cast<const FontManagerFont*>(currentFont->_baseFont.get());
            }
            if (!font) {
                return std::vector<Glyph>();
            }

            // Get glyph list and glyph positions
            unsigned int infoCount = 0;
            const hb_glyph_info_t* info = hb_buffer_get_glyph_infos(_buffer, &infoCount);
            unsigned int posCount = 0;
            const hb_glyph_position_t* pos = hb_buffer_get_glyph_positions(_buffer, &posCount);

            // Copy glyphs, render/cache bitmaps
            std::vector<Glyph> glyphs;
            glyphs.reserve(infoCount);
            for (unsigned int i = 0; i < infoCount; i++) {
                if (info[i].codepoint != 0) { // ignore 'missing glyph' glyphs
                    CodePoint remappedCodePoint = info[i].codepoint | (fontId << 24);
                    auto it = _codePointGlyphMap.find(remappedCodePoint);
                    if (it == _codePointGlyphMap.end()) {
                        GlyphMap::GlyphId glyphId = addFreeTypeGlyph(font->_face, info[i].codepoint);
                        if (!glyphId) {
                            continue;
                        }
                        it = _codePointGlyphMap.insert({ remappedCodePoint, glyphId }).first;
                    }
                    if (const GlyphMap::Glyph* baseGlyph = _glyphMap->getGlyph(it->second)) {
                        float glyphScale = size / RENDER_SIZE;
                        cglib::vec2<float> glyphSize(static_cast<float>(baseGlyph->width), static_cast<float>(baseGlyph->height));
                        Glyph glyph(info[i].codepoint, *baseGlyph, glyphSize * glyphScale, baseGlyph->origin * glyphScale, cglib::vec2<float>(0, 0));
                        glyphs.push_back(glyph);
                        if (i < posCount) {
                            glyphs.back().offset += cglib::vec2<float>(pos[i].x_offset / 64.0f, pos[i].y_offset / 64.0f) * glyphScale;
                            glyphs.back().advance = cglib::vec2<float>(pos[i].x_advance / 64.0f, pos[i].y_advance / 64.0f) * glyphScale;
                        }
                    }
                }
            }
            return glyphs;
        }

        virtual std::shared_ptr<GlyphMap> getGlyphMap() const override {
            return _glyphMap;
        }

    private:
        inline static constexpr int RENDER_PADDING = 3;
        inline static constexpr int RENDER_SIZE = GLYPH_RENDER_SIZE - RENDER_PADDING;

        GlyphMap::GlyphId addFreeTypeGlyph(FT_Face face, CodePoint codePoint) const {
            FT_Error error = FT_Load_Glyph(face, codePoint, FT_LOAD_NO_BITMAP | FT_LOAD_NO_HINTING);
            if (error != 0) {
                return 0;
            }
            error = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_SDF);
            if (error != 0) {
                return 0;
            }

            int width  = face->glyph->bitmap.width;
            int height = face->glyph->bitmap.rows;
            float xOffset = std::ceil(-face->glyph->metrics.horiBearingX / 64.0f);
            float yOffset = std::ceil((face->glyph->metrics.height - face->glyph->metrics.horiBearingY) / 64.0f);
            float distScale = 128.0f / BITMAP_SDF_SCALE / 1024.0f;
            const FT_Short* distBuffer = reinterpret_cast<const FT_Short*>(face->glyph->bitmap.buffer);

            std::vector<std::uint32_t> glyphBitmapData(width * height);
            for (std::size_t i = 0; i < glyphBitmapData.size(); i++) {
                float dist = distBuffer[i] * distScale;
                std::uint32_t val = static_cast<std::uint8_t>(std::max(0.0f, std::min(255.0f, dist + 127.5f)));
                glyphBitmapData[i] = (val << 24) | (val << 16) | (val << 8) | val;
            }
            std::shared_ptr<Bitmap> glyphBitmap = std::make_shared<Bitmap>(width, height, std::move(glyphBitmapData));
            return _glyphMap->loadBitmapGlyph(glyphBitmap, true, cglib::vec2<float>(-xOffset, -RENDER_PADDING - yOffset));
        }

        const std::shared_ptr<FontManagerLibrary> _library;
        const std::shared_ptr<const Font> _baseFont;
        std::shared_ptr<GlyphMap> _glyphMap;
        mutable std::unordered_map<CodePoint, GlyphMap::GlyphId> _codePointGlyphMap;
        FT_Face _face;
        hb_font_t* _font;
        hb_buffer_t* _buffer;
    };

    class FontManager::Impl {
    public:
        explicit Impl(int maxGlyphMapWidth, int maxGlyphMapHeight) : _maxGlyphMapWidth(maxGlyphMapWidth), _maxGlyphMapHeight(maxGlyphMapHeight), _library(std::make_shared<FontManagerLibrary>()) { }

        std::string loadFontData(const std::vector<unsigned char>& data) {
            std::lock_guard<std::mutex> lock(_mutex);

            if (data.empty()) {
                return std::string();
            }

            FontManagerLibrary library;
            FT_Face face;
            int error = FT_New_Memory_Face(library.getLibrary(), data.data(), static_cast<FT_Long>(data.size()), 0, &face);
            if (error != 0) {
                return std::string();
            }
            std::string fullName, family, subFamily;
            for (unsigned int i = 0; i < FT_Get_Sfnt_Name_Count(face); i++) {
                FT_SfntName sfntName;
                error = FT_Get_Sfnt_Name(face, i, &sfntName);
                if (error != 0) {
                    continue;
                }
                std::string name = readSfntName(sfntName);
                if (name.empty()) {
                    continue;
                }
                switch (sfntName.name_id) {
                case TT_NAME_ID_FULL_NAME:
                    fullName = name;
                    break;
                case TT_NAME_ID_FONT_FAMILY:
                case TT_NAME_ID_PREFERRED_FAMILY:
                    family = name;
                    break;
                case TT_NAME_ID_FONT_SUBFAMILY:
                case TT_NAME_ID_PREFERRED_SUBFAMILY:
                    subFamily = name;
                    break;
                default:
                    break;
                }
            }

            std::string registeredName = fullName;
            if (!fullName.empty()) {
                _fontDataMap[fullName] = data;
            }
            if (!family.empty()) {
                if (!subFamily.empty()) {
                    _fontDataMap[family + " " + subFamily] = data;
                    if (registeredName.empty()) {
                        registeredName = family + " " + subFamily;
                    }
                }
                else {
                    _fontDataMap[family] = data;
                    if (registeredName.empty()) {
                        registeredName = family;
                    }
                }
            }
            FT_Done_Face(face);
            return registeredName;
        }

        std::shared_ptr<const Font> getFont(const std::string& name, const std::shared_ptr<const Font>& baseFont) const {
            std::lock_guard<std::mutex> lock(_mutex);

            // Try to use already cached font
            auto fontIt = _fontMap.find(std::make_pair(name, baseFont));
            if (fontIt != _fontMap.end()) {
                return fontIt->second;
            }

            // Check if we have font corresponding to the name
            auto fontDataIt = _fontDataMap.find(name);
            if (fontDataIt == _fontDataMap.end()) {
                return std::shared_ptr<Font>();
            }

            // Get existing glyph map or create new one
            auto glyphMapIt = _glyphMapMap.find(name);
            if (glyphMapIt == _glyphMapMap.end()) {
                glyphMapIt = _glyphMapMap.emplace(name, std::make_shared<GlyphMap>(_maxGlyphMapWidth, _maxGlyphMapHeight)).first;
            }

            // Create new font
            auto font = std::make_shared<FontManagerFont>(_library, glyphMapIt->second, &fontDataIt->second, baseFont);

            // Preload often-used characters
            std::vector<std::uint32_t> glyphPreloadTable;
            std::for_each(_glyphPreloadTable.begin(), _glyphPreloadTable.end(), [&glyphPreloadTable](char c) { glyphPreloadTable.push_back(c); });
            for (std::size_t i = 0; i < glyphPreloadTable.size(); i++) {
                font->shapeGlyphs(&glyphPreloadTable[i], 1, 1.0f, false);
            }

            // Cache the font
            _fontMap[std::make_pair(name, baseFont)] = font;
            return font;
        }

    private:
        static std::string readSfntName(const FT_SfntName& sfntName) {
            static const std::pair<int, int> be16Encodings[] = {
                { TT_PLATFORM_APPLE_UNICODE, TT_APPLE_ID_DEFAULT },
                { TT_PLATFORM_APPLE_UNICODE, TT_APPLE_ID_UNICODE_1_1 },
                { TT_PLATFORM_APPLE_UNICODE, TT_APPLE_ID_ISO_10646 },
                { TT_PLATFORM_APPLE_UNICODE, TT_APPLE_ID_UNICODE_2_0 },
                { TT_PLATFORM_ISO, TT_ISO_ID_10646 },
                { TT_PLATFORM_MICROSOFT, TT_MS_ID_UNICODE_CS },
                { TT_PLATFORM_MICROSOFT, TT_MS_ID_SYMBOL_CS },
                { -1, -1 }
            };

            for (int i = 0; be16Encodings[i].first != -1; i++) {
                if (be16Encodings[i].first == sfntName.platform_id && be16Encodings[i].second == sfntName.encoding_id) {
                    std::string name;
                    for (unsigned int j = 0; j < sfntName.string_len; j += 2) {
                        const char* c = reinterpret_cast<const char*>(sfntName.string) + j;
                        if (c[0] != 0) {
                            return std::string(); // simply ignore complex names
                        }
                        name.append(1, c[1]);
                    }
                    return name;
                }
            }
            return std::string(reinterpret_cast<const char*>(sfntName.string), sfntName.string_len);
        }

        const std::string _glyphPreloadTable = " 0123456789abcdefghijklmnopqrstuvxyzwABCDEFGHIJKLMNOPQRSTUVXYZ-,.";
        const int _maxGlyphMapWidth;
        const int _maxGlyphMapHeight;
        std::map<std::string, std::vector<unsigned char>> _fontDataMap;
        std::shared_ptr<FontManagerLibrary> _library;
        mutable std::map<std::pair<std::string, std::shared_ptr<const Font>>, std::shared_ptr<FontManagerFont>> _fontMap;
        mutable std::map<std::string, std::shared_ptr<GlyphMap>> _glyphMapMap;
        mutable std::mutex _mutex;
    };

    FontManager::FontManager(int maxGlyphMapWidth, int maxGlyphMapHeight) : _impl(std::make_unique<Impl>(maxGlyphMapWidth, maxGlyphMapHeight)) {
    }

    FontManager::~FontManager() {
    }

    std::string FontManager::loadFontData(const std::vector<unsigned char>& data) {
        return _impl->loadFontData(data);
    }

    std::shared_ptr<const Font> FontManager::getFont(const std::string& name, const std::shared_ptr<const Font>& baseFont) const {
        return _impl->getFont(name, baseFont);
    }
} }

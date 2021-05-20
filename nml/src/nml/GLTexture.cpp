#include "GLTexture.h"
#include "GLResourceManager.h"
#include "Package.h"

#include "rg_etc1.h"

#include "PVRTTexture.h"
#include "PVRTDecompress.h"

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

#ifndef GL_TEXTURE_MAX_LEVEL
#define GL_TEXTURE_MAX_LEVEL 0x813D
#endif

#ifndef GL_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_TEXTURE_MAX_ANISOTROPY_EXT 0x84FE
#endif

#ifndef GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF
#endif

#ifndef GL_ETC1_RGB8_OES
#define GL_ETC1_RGB8_OES 0x8D64
#endif

#ifndef GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG
#define GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG 0x8C00
#define GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG 0x8C01
#define GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG 0x8C02
#define GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG 0x8C03
#endif

#include <cassert>
#include <mutex>
#include <memory>
#include <string>
#include <sstream>
#include <unordered_set>

namespace carto { namespace nml {
    GLTexture::GLTexture(std::shared_ptr<Texture> texture) :
        _texture(texture),
        _glTextureId(0)
    {
    }
    
    void GLTexture::create(GLResourceManager& resourceManager) {
        if (_glTextureId == 0) {
            uploadTexture(resourceManager);
        }
    }
    
    void GLTexture::bind(GLResourceManager& resourceManager, int texUnit) {
        if (_glTextureId == 0) {
            uploadTexture(resourceManager);
        }

        glActiveTexture(GL_TEXTURE0 + texUnit);
        glBindTexture(GL_TEXTURE_2D, _glTextureId);
    }
    
    int GLTexture::getTextureSize() const {
        if (!_texture) {
            return 0;
        }
    
        std::size_t size = 0;
        for (int i = 0; i < _texture->mipmaps_size(); i++) {
            size += _texture->mipmaps(i).size();
        }
        return static_cast<int>(size);
    }

    void GLTexture::transcodeIfNeeded(Texture& texture) {
        switch (texture.format()) {
        case Texture::ETC1:
            if (!hasGLExtension("GL_OES_compressed_ETC1_RGB8_texture")) {
                uncompressTexture(texture);
            }
            break;
        case Texture::PVRTC:
            if (!hasGLExtension("GL_IMG_texture_compression_pvrtc")) {
                uncompressTexture(texture);
            }
            break;
        default:
            break;
        }
    }

    void GLTexture::registerGLExtensions() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_extensions.empty()) {
            const char* extensionsString = reinterpret_cast<const char*>(glGetString(GL_EXTENSIONS));
            if (!extensionsString) {
                return;
            }
            std::stringstream ss(extensionsString);
            std::string s;
            while (getline(ss, s, ' ')) {
                _extensions.insert(s);
            }
        }
    }

    GLuint GLTexture::getSamplerWrapMode(int wrapMode) {
        switch (wrapMode) {
        case Sampler::CLAMP:
            return GL_CLAMP_TO_EDGE;
        default:
            return GL_REPEAT; // ignore MIRROR, etc
        }
    }

    bool GLTexture::hasGLExtension(const char* ext) {
        std::lock_guard<std::mutex> lock(_mutex);
        return _extensions.find(ext) != _extensions.end();
    }

    void GLTexture::updateSampler(bool hasSampler, const Sampler& sampler, bool complete) {
        if (hasSampler) {
            switch (sampler.filter()) {
            case Sampler::NEAREST:
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                break;
            case Sampler::BILINEAR:
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                break;
            case Sampler::TRILINEAR:
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, complete ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                break;
            }
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, getSamplerWrapMode(sampler.wrap_s()));
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, getSamplerWrapMode(sampler.wrap_t()));
        } else {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, complete ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        }
        
        if (hasGLExtension("GL_EXT_texture_filter_anisotropic")) {
            GLint aniso = 0;
            glGetIntegerv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &aniso);
            if (aniso > 0) {
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, aniso);
            }
        }
    }
    
    void GLTexture::updateMipLevel(int level, const Texture& texture) {
        GLint glFormat = -1, glFormatInternal = -1;
        std::string glTextureData = texture.mipmaps(level);
        switch (texture.format()) {
        case Texture::LUMINANCE8:
            glFormat = glFormatInternal = GL_LUMINANCE;
            break;
        case Texture::RGB8:
            glFormat = glFormatInternal = GL_RGB;
            break;
        case Texture::RGBA8:
            glFormat = glFormatInternal = GL_RGBA;
            break;
        case Texture::ETC1:
            if (hasGLExtension("GL_OES_compressed_ETC1_RGB8_texture")) {
                GLuint size = 8 * ((texture.width() + 3) >> 2) * ((texture.height() + 3) >> 2);
                std::size_t offset = 16;
                glCompressedTexImage2D(GL_TEXTURE_2D, level, GL_ETC1_RGB8_OES, texture.width(), texture.height(), 0, size, &glTextureData[offset]);
                return;
            }
            {
                Texture textureCopy(texture);
                uncompressTexture(textureCopy);
                updateMipLevel(level, textureCopy);
            }
            return;
        case Texture::PVRTC:
            if (hasGLExtension("GL_IMG_texture_compression_pvrtc")) {
                const PVRTextureHeaderV3* header = reinterpret_cast<const PVRTextureHeaderV3*>(glTextureData.data());
                GLuint format = 0;
                switch (header->u64PixelFormat)
                {
                case ePVRTPF_PVRTCI_2bpp_RGB:
                    format = GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG;
                    break;
                case ePVRTPF_PVRTCI_4bpp_RGB:
                    format = GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG;
                    break;
                case ePVRTPF_PVRTCI_2bpp_RGBA:
                    format = GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG;
                    break;
                case ePVRTPF_PVRTCI_4bpp_RGBA:
                    format = GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;
                    break;
                }
    
                // Workaround for Apple-specific limitation - textures have to be squares (sic!)
                if (texture.width() == texture.height()) {
                    GLuint size = static_cast<GLuint>(glTextureData.size()) - PVRTEX3_HEADERSIZE;
                    glCompressedTexImage2D(GL_TEXTURE_2D, level, format, texture.width(), texture.height(), 0, size, &glTextureData[PVRTEX3_HEADERSIZE]);
                    return;
                }
            }
            {
                Texture textureCopy(texture);
                uncompressTexture(textureCopy);
                updateMipLevel(level, textureCopy);
            }
            return;
        default:
            assert(false);
        }
    
        if (!glTextureData.empty()) {
            glTexImage2D(GL_TEXTURE_2D, level, glFormatInternal, texture.width(), texture.height(), 0, glFormat, GL_UNSIGNED_BYTE, glTextureData.data());
        }
    }
    
    void GLTexture::updateMipMaps(const Texture& texture) {
        for (int i = 0; i < texture.mipmaps_size(); i++) {
            updateMipLevel(i, texture);
        }
    }
    
    void GLTexture::uploadTexture(GLResourceManager& resourceManager) {
        if (!_texture) {
            return;
        }

        const Texture& texture = *_texture;
        if (texture.width() < 1 || texture.height() < 1) {
            return;
        }

        _glTextureId = resourceManager.allocateTexture(shared_from_this());
    
        glBindTexture(GL_TEXTURE_2D, _glTextureId);
        updateMipMaps(texture);
        updateSampler(texture.has_sampler(), texture.sampler(), texture.mipmaps_size() > 1);
    }
    
    void GLTexture::uncompressTexture(Texture& texture) {
        switch (texture.format()) {
        case Texture::ETC1:
            for (int i = 0; i < texture.mipmaps_size(); i++) {
                std::string textureData = texture.mipmaps(i);
                int etc1Width = (texture.width() + 3) & ~3, etc1Height = (texture.height() + 3) & ~3;
                std::vector<unsigned int> etc1Image(texture.width() * texture.height());
                std::size_t offset = 16;
                for (int y = 0; y + 4 <= etc1Height; y += 4) {
                    for (int x = 0; x + 4 <= etc1Width; x += 4) {
                        unsigned int block[4 * 4];
                        rg_etc1::unpack_etc1_block(&textureData[offset], block);
                        offset += 4 * 4 / 2;
                        for (int yb = 0; yb < 4; yb++) {
                            if (y + yb >= texture.height()) {
                                continue;
                            }
                            for (int xb = 0; xb < 4; xb++) {
                                if (x + xb >= texture.width()) {
                                    continue;
                                }
                                etc1Image[(y + yb) * texture.width() + x + xb] = block[yb * 4 + xb];
                            }
                        }
                    }
                }
                textureData.clear();
                if (!etc1Image.empty()) {
                    textureData.assign(reinterpret_cast<const char*>(etc1Image.data()), reinterpret_cast<const char*>(etc1Image.data() + etc1Image.size()));
                }
                texture.set_mipmaps(i, textureData);
            }
            texture.set_format(Texture::RGBA8);
            break;
        
        case Texture::PVRTC:
            for (int i = 0; i < texture.mipmaps_size(); i++) {
                std::string textureData = texture.mipmaps(i);
                const PVRTextureHeaderV3* header = reinterpret_cast<const PVRTextureHeaderV3*>(textureData.data());
                bool bpp2 = header->u64PixelFormat == ePVRTPF_PVRTCI_2bpp_RGB || header->u64PixelFormat == ePVRTPF_PVRTCI_2bpp_RGBA;
                std::vector<unsigned long> pvrtcImage(texture.width() * texture.height());
                PVRTDecompressPVRTC(&textureData[PVRTEX3_HEADERSIZE], bpp2, texture.width(), texture.height(), reinterpret_cast<unsigned char*>(&pvrtcImage[0]));
    
                textureData.clear();
                if (!pvrtcImage.empty()) {
                    textureData.assign(reinterpret_cast<const char*>(pvrtcImage.data()), reinterpret_cast<const char*>(pvrtcImage.data() + pvrtcImage.size()));
                }
                texture.set_mipmaps(i, textureData);
            }
            texture.set_format(Texture::RGBA8);
            break;
        
        default:
            break;
        }
    }

    std::mutex GLTexture::_mutex;
    std::unordered_set<std::string> GLTexture::_extensions;
} }

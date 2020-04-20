#include "BitmapManager.h"

#include <algorithm>

namespace carto { namespace vt {
    BitmapManager::BitmapManager(std::shared_ptr<BitmapLoader> loader) : _bitmapLoader(std::move(loader)) {
    }

    std::shared_ptr<const BitmapImage> BitmapManager::getBitmapImage(const std::string& fileName) const {
        std::lock_guard<std::mutex> lock(_mutex);

        auto it = _bitmapImageMap.find(fileName);
        if (it != _bitmapImageMap.end()) {
            return it->second;
        }

        return std::shared_ptr<BitmapImage>();
    }

    std::shared_ptr<const BitmapImage> BitmapManager::loadBitmapImage(const std::string& fileName, bool sdfMode, float resolution) {
        std::lock_guard<std::mutex> lock(_mutex);

        auto it = _bitmapImageMap.find(fileName);
        if (it != _bitmapImageMap.end()) {
            return it->second;
        }

        std::shared_ptr<BitmapImage> bitmapImage;
        if (std::shared_ptr<const Bitmap> bitmap = _bitmapLoader->load(fileName, resolution)) {
            bitmapImage = std::make_shared<BitmapImage>(sdfMode, 1.0f / resolution, bitmap);
        }
        _bitmapImageMap[fileName] = bitmapImage;
        return bitmapImage;
    }

    void BitmapManager::storeBitmapImage(const std::string& fileName, const std::shared_ptr<const BitmapImage>& bitmapImage) {
        std::lock_guard<std::mutex> lock(_mutex);

        _bitmapImageMap[fileName] = bitmapImage;
    }

    std::shared_ptr<const BitmapPattern> BitmapManager::getBitmapPattern(const std::string& fileName) const {
        std::lock_guard<std::mutex> lock(_mutex);

        auto it = _bitmapPatternMap.find(fileName);
        if (it != _bitmapPatternMap.end()) {
            return it->second;
        }

        return std::shared_ptr<BitmapPattern>();
    }

    std::shared_ptr<const BitmapPattern> BitmapManager::loadBitmapPattern(const std::string& fileName, float widthScale, float heightScale) {
        std::lock_guard<std::mutex> lock(_mutex);

        // Check if we have this pattern cached already
        std::string patternId = fileName;
        if (widthScale != 1.0f || heightScale != 1.0f) {
            patternId += "#" + std::to_string(widthScale) + "," + std::to_string(heightScale);
        }
        auto it = _bitmapPatternMap.find(patternId);
        if (it != _bitmapPatternMap.end()) {
            return it->second;
        }

        // Load the bitmap
        float resolution = 1.0f;
        std::shared_ptr<const Bitmap> bitmap = _bitmapLoader->load(fileName, resolution);

        // Create pattern for the bitmap
        std::shared_ptr<BitmapPattern> bitmapPattern;
        if (bitmap) {
            bitmapPattern = std::make_shared<BitmapPattern>(widthScale / resolution, heightScale / resolution, bitmap);
        }
        _bitmapPatternMap[patternId] = bitmapPattern;
        return bitmapPattern;
    }

    void BitmapManager::storeBitmapPattern(const std::string& fileName, const std::shared_ptr<const BitmapPattern>& bitmapPattern) {
        std::lock_guard<std::mutex> lock(_mutex);

        _bitmapPatternMap[fileName] = bitmapPattern;
    }

    std::shared_ptr<const Bitmap> BitmapManager::scale(const std::shared_ptr<const Bitmap>& bitmap, int width, int height) {
        if (!bitmap) {
            return bitmap;
        }
        if (bitmap->width < 1 || bitmap->height < 1) {
            return bitmap;
        }
        if (width == bitmap->width && height == bitmap->height) {
            return bitmap;
        }
        
        // Use bilinear interpolation of pixel values for scaling
        std::vector<std::uint32_t> data(width * height, 0);
        for (int y = 0; y < height; y++) {
            int sy = 256 * y * bitmap->height / height;
            int y0 = (sy >> 8) + 0;
            int y1 = std::min((sy >> 8) + 1, bitmap->height - 1);
            int dy = sy & 255;
            for (int x = 0; x < width; x++) {
                int sx = 256 * x * bitmap->width / width;
                int x0 = (sx >> 8) + 0;
                int x1 = std::min((sx >> 8) + 1, bitmap->width - 1);
                int dx = sx & 255;

                int weights[4] = { (256 - dx) * (256 - dy), dx * (256 - dy), (256 - dx) * dy, dx * dy };
                int indices[4] = { y0 * bitmap->width + x0, y0 * bitmap->width + x1, y1 * bitmap->width + x0, y1 * bitmap->width + x1 };
                std::uint8_t* destColor = reinterpret_cast<std::uint8_t*>(&data[y * width + x]);
                for (int i = 0; i < 4; i++) {
                    const std::uint8_t* sourceColor = reinterpret_cast<const std::uint8_t*>(&bitmap->data[indices[i]]);
                    for (int c = 0; c < 4; c++) {
                        destColor[c] += static_cast<std::uint8_t>((sourceColor[c] * weights[i]) >> 16);
                    }
                }
            }
        }
        return std::make_shared<Bitmap>(width, height, std::move(data));
    }

    std::shared_ptr<const Bitmap> BitmapManager::scaleToPOT(const std::shared_ptr<const Bitmap>& bitmap) {
        if (!bitmap) {
            return bitmap;
        }

        int width = 1;
        while (width < bitmap->width) { width *= 2; }
        int height = 1;
        while (height < bitmap->height) { height *= 2; }
        return BitmapManager::scale(bitmap, width, height);
    }
} }

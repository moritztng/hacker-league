#define STB_TRUETYPE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_truetype.h"
#include "stb_image_write.h"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <memory>

struct CharInfo {
    unsigned short x0, y0, x1, y1;
    float xoff, yoff, xadvance;
};

int main() {
    constexpr const char* ttfFile = "DejaVuSans.ttf";
    constexpr const char* pngFile = "font.png";
    constexpr const char* headerFile = "font.h";
    constexpr int atlasWidth = 256;
    constexpr int atlasHeight = 256;
    constexpr float fontSize = 64;
    constexpr int firstChar = 32;
    constexpr int numChars = 27;

    std::ifstream fontFile(ttfFile, std::ios::binary | std::ios::ate);
    if (!fontFile) {
        std::cerr << "error: can't open font file" << std::endl;
        return EXIT_FAILURE;
    }

    auto fileSize = fontFile.tellg();
    fontFile.seekg(0, std::ios::beg);
    std::vector<unsigned char> ttfBuffer(static_cast<size_t>(fileSize));
    fontFile.read(reinterpret_cast<char*>(ttfBuffer.data()), fileSize);
    fontFile.close();

    stbtt_fontinfo font;
    if (!stbtt_InitFont(&font, ttfBuffer.data(), 0)) {
        std::cerr << "error: can't initialize font" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<unsigned char> atlasBitmap(atlasWidth * atlasHeight, 0);
    std::vector<CharInfo> charInfo(numChars);

    stbtt_pack_context packContext;
    stbtt_PackBegin(&packContext, atlasBitmap.data(), atlasWidth, atlasHeight, 0, 1, nullptr);
    stbtt_PackSetOversampling(&packContext, 1, 1);

    std::vector<stbtt_packedchar> packedChars(numChars);
    stbtt_PackFontRange(&packContext, ttfBuffer.data(), 0, fontSize, firstChar, numChars, packedChars.data());
    stbtt_PackEnd(&packContext);

    auto scale = stbtt_ScaleForPixelHeight(&font, fontSize);
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(&font, &ascent, &descent, &lineGap);

    for (int i = 0; i < numChars; ++i) {
        const auto& pc = packedChars[i];
        auto& ci = charInfo[i];

        ci.x0 = static_cast<unsigned short>(pc.x0);
        ci.y0 = static_cast<unsigned short>(pc.y0);
        ci.x1 = static_cast<unsigned short>(pc.x1);
        ci.y1 = static_cast<unsigned short>(pc.y1);
        ci.xoff = pc.xoff;
        ci.yoff = pc.yoff;
        ci.xadvance = pc.xadvance;
    }

    stbi_write_png(pngFile, atlasWidth, atlasHeight, 1, atlasBitmap.data(), atlasWidth);

    std::ofstream header(headerFile);
    if (!header) {
        std::cerr << "error: can't create header file." << std::endl;
        return EXIT_FAILURE;
    }

    header << "#pragma once\n\n";
    header << "struct CharInfo {\n";
    header << "    unsigned short x0, y0, x1, y1;\n";
    header << "    float xoff, yoff, xadvance;\n";
    header << "};\n\n";
    header << "const int ATLAS_WIDTH = " << atlasWidth << ";\n";
    header << "const int ATLAS_HEIGHT = " << atlasHeight << ";\n";
    header << "const float FONT_SIZE = " << fontSize << ";\n";
    header << "const int FIRST_CHAR = " << firstChar << ";\n";
    header << "const int NUM_CHARS = " << numChars << ";\n";
    header << "const float FONT_ASCENT = " << ascent * scale << ";\n";
    header << "const float FONT_DESCENT = " << descent * scale << ";\n";
    header << "const float FONT_LINE_GAP = " << lineGap * scale << ";\n\n";
    header << "const CharInfo CHAR_INFO[NUM_CHARS] = {\n";

    for (const auto& ci : charInfo) {
        header << "    {" << ci.x0 << ", " << ci.y0 << ", " << ci.x1 << ", " << ci.y1 << ", "
               << ci.xoff << ", " << ci.yoff << ", " << ci.xadvance << "},\n";
    }

    header << "};\n";
    header.close();

    return EXIT_SUCCESS;
}

#include "../ResourceLoader.hpp"
#include "../renderer/Texture.hpp"
#include "../Debug.hpp"
#include "../Util.hpp"

// This is a very limited TGA reader. 24/32-bit Truecolor only.

namespace ups
{
	// Version which is told a type - can be used to force non-default loading.
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_TGA>(const std::string &filePath) const
	{
		UPS_LOGF("Looking at RL_LT_TGA for %s", filePath.c_str());
		
		std::FILE *f = std::fopen(filePath.c_str(), "rb");
		long fSize = getFileSize(f);
		
		// The absolute minimum size is 18 bytes.
		if (fSize <= 18) return new Texture(Texture::TT_EMPTY, 0, 0);
		
		// Read the header
		uint8  tgaIDLength;		readRaw(f, tgaIDLength);
		uint8  tgaColorMapType;	readRaw(f, tgaColorMapType);
		uint8  tgaImageType;	readRaw(f, tgaImageType);
		uint16 tgaCMapStart;	readRaw(f, tgaCMapStart);
		uint16 tgaCMapLength;	readRaw(f, tgaCMapLength);
		uint8  tgaCMapDepth;	readRaw(f, tgaCMapDepth);
		uint16 tgaXOffset;		readRaw(f, tgaXOffset);
		uint16 tgaYOffset;		readRaw(f, tgaYOffset);
		uint16 tgaWidth;		readRaw(f, tgaWidth);
		uint16 tgaHeight;		readRaw(f, tgaHeight);
		uint8  tgaPixelDepth;	readRaw(f, tgaPixelDepth);
		uint8  tgaImageDesc;	readRaw(f, tgaImageDesc);
		
		// Skip the image ID field.
		std::fseek(f, tgaIDLength, SEEK_CUR);
		
		// Interpret the tgaImageType field
		bool isCompressed =  (tgaImageType & 0x08) >> 3;
		bool isFlippedX = (tgaImageType & 0x40) >> 6;
		bool isFlippedY = !((tgaImageType & 0x80) >> 7);
		long imageDataType = tgaImageType & 0x07;
		long imageAttribBits = tgaImageDesc & 0x07;
		
		// Infer some more convenience data
		uint8 tgaCMapDepthBytes = (tgaCMapDepth + imageAttribBits + 7) >> 3;
		uint8 tgaPixelDepthBytes = (tgaPixelDepth + 7) >> 3;
		
		std::printf("Loading TGA: %d %d %s %ld\n", tgaWidth, tgaHeight, isCompressed ? "compressed" : "uncompressed", imageDataType);
		std::printf("%s %s\n", isFlippedX ? "flipped X" : "un-flipped X", isFlippedY ? "flipped Y" : "un-flipped Y");
		
		// Get out of here if we can't handle the file
		if (isCompressed || imageDataType != 2 || (tgaPixelDepth != 24 && tgaPixelDepth != 32)) return new Texture(Texture::TT_EMPTY, 0, 0);
		
		std::printf("TGA supported: depth = %d\n", tgaPixelDepth);

		// Load the palette (if it has one)
		uint64 *tgaPalette = 0;
		if (tgaColorMapType != 0)
		{
			tgaPalette = new uint64[tgaCMapLength];
			for (uint16 i = 0; i < tgaCMapLength; ++i)
			{
				tgaPalette[i] = 0;
				std::fread(tgaPalette + i, tgaCMapDepthBytes, 1, f);
			}
		}
		
		// Fit to the nearest power of two (lots of advantages)
		uint32 outputWidth = potAbove(tgaWidth);
		uint32 outputHeight = potAbove(tgaHeight);

		// Go to the bitmap data, and start reading into an image object
		Texture *outputTexture = new Texture(Texture::TT_RGBA8, outputWidth, outputHeight, tgaWidth, tgaHeight);
		uint8 *outputImage = new uint8[outputWidth * outputHeight * 4];
		uint8 *pixelBuffer = new uint8[tgaPixelDepthBytes];
		
		for (uint16 yRaw = 0; yRaw < outputHeight; ++yRaw)
		{
			uint16 y = isFlippedY ? tgaHeight - 1 - yRaw : yRaw;
			for (uint16 xRaw = 0; xRaw < outputWidth; ++xRaw)
			{
				uint16 x = isFlippedX ? tgaWidth - 1 - xRaw : xRaw;
				uint8 *outputImageOffset = outputImage + 4 * (y * outputHeight + x);
				
				if (yRaw >= tgaHeight || xRaw >= tgaWidth)
				{
					uint8 *outputImageOffset = outputImage + 4 * (yRaw * outputHeight + xRaw);
					*(  outputImageOffset) = 0;
					*(++outputImageOffset) = 0;
					*(++outputImageOffset) = 0;
					*(++outputImageOffset) = 0;
					continue;
				}
				
				switch (imageDataType)
				{
				case 2: // True Colour
					std::fread(pixelBuffer, 1, tgaPixelDepthBytes, f);
					
					switch(tgaPixelDepth)
					{
					case 24:
						*(  outputImageOffset) = pixelBuffer[2];
						*(++outputImageOffset) = pixelBuffer[1];
						*(++outputImageOffset) = pixelBuffer[0];
						*(++outputImageOffset) = 255;
						break;
					case 32:
						*(  outputImageOffset) = pixelBuffer[2];
						*(++outputImageOffset) = pixelBuffer[1];
						*(++outputImageOffset) = pixelBuffer[0];
						*(++outputImageOffset) = pixelBuffer[3];
						break;
					}
					
					break;
				}
			}
		}
		
		delete[] pixelBuffer;
		
		outputTexture->setData(outputImage);
		fclose(f);

		return outputTexture;
	}
}
#include "../ResourceLoader.hpp"
#include "../renderer/Texture.hpp"
#include "../Debug.hpp"
#include "../Util.hpp"

namespace
{
	struct BMPPaletteEntry3
	{
		ups::uint8 b, g, r;
	};
	
	struct BMPPaletteEntry4
	{
		ups::uint8 b, g, r, padding;
	};
	
	// The size of the second header determines the type of bitmap. The OS/2 bitmap types aren't supported here.
	enum BMPBitmapType
	{
		BITMAPCOREHEADER,
		OS22XBITMAPHEADER,
		BITMAPINFOHEADER,
		BITMAPV2INFOHEADER,
		BITMAPV3INFOHEADER,
		BITMAPV4HEADER,
		BITMAPV5HEADER,
		BITMAPUNKNOWN
	};
}

namespace ups
{
	// Version which is told a type - can be used to force non-default loading.
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_BMP>(const std::string &filePath) const
	{
		UPS_LOGF("Looking at RL_LT_BMP for %s", filePath.c_str());
		
		std::FILE *f = std::fopen(filePath.c_str(), "rb");
		long fSize = getFileSize(f);

		// The absolute minimum size is 26 bytes - a 2.x bitmap without any image data.
		if (fSize <= 26) return new Texture(Texture::TT_EMPTY, 0, 0);

		// Read the first header part (all BMP files after 1987 have this section).
		// There is actually an earlier version too, but there's no point supporting it
		uint16 bmpFileType;		readRaw(f, bmpFileType);
		uint32 bmpFileSize;		readRaw(f, bmpFileSize);
		uint16 bmpReserved1;	readRaw(f, bmpReserved1);
		uint16 bmpReserved2;	readRaw(f, bmpReserved2);
		uint32 bmpBitmapOffset;	readRaw(f, bmpBitmapOffset);
		if (bmpFileType != 0x4D42) return new Texture(Texture::TT_EMPTY, 0, 0); // Magic number is "BM" in LE, which is "MB" or 0x4D42 in BE
		if (bmpFileSize != fSize) return new Texture(Texture::TT_EMPTY, 0, 0);
		
		// Infer the type of bitmap from the header size. This turns out to be the standard way to do this.
		BMPBitmapType bmpInferredBitmapType = BITMAPUNKNOWN;
		uint32 bmpHeaderSize;	readRaw(f, bmpHeaderSize);

		switch (bmpHeaderSize)
		{
			case 12:  bmpInferredBitmapType = BITMAPCOREHEADER; break; // Windows 2.x
			case 40:  bmpInferredBitmapType = BITMAPINFOHEADER; break; // Windows 3.x
			case 52:  bmpInferredBitmapType = BITMAPV2INFOHEADER; break; // Adobe claims this and V3 Info Header were part of the BMP spec at some point.
			case 56:  bmpInferredBitmapType = BITMAPV3INFOHEADER; break;
			case 108: bmpInferredBitmapType = BITMAPV4HEADER; break; // Windows 4.x
			case 124: bmpInferredBitmapType = BITMAPV5HEADER; break; // Windows 5.x

			case 64:  bmpInferredBitmapType = OS22XBITMAPHEADER; return new Texture(Texture::TT_EMPTY, 0, 0); // We don't support OS/2 bitmaps
			default: return new Texture(Texture::TT_EMPTY, 0, 0); // Some weird size - we can't know for sure how the header is structured, so we don't try.
		}
		
		int32  bmpWidth = 0;
		int32  bmpByteWidth = 0;
		int32  bmpHeight = 0;
		uint16 bmpPlanes = 0;
		uint16 bmpBitsPerPixel = 0;
		uint32 bmpCompression = 0;
		uint16 bmpPaletteLength = 0;
		uint16 bmpPaletteEntrySize = 0;
		bool bmpFlippedY = true;
		uint8 *bmpPalette = 0;
		
		// BITMAPCOREHEADER had 16-bit height and width, but all other versions have these as 32-bit
		// Also, a padding bit was added to the palette entries for later versions
		if (bmpInferredBitmapType == BITMAPCOREHEADER)
		{
			int16 bmpWidthShort;	readRaw(f, bmpWidthShort);
			int16 bmpHeightShort;	readRaw(f, bmpHeightShort);
			bmpWidth	= bmpWidthShort;
			bmpHeight	= bmpHeightShort;

			bmpPaletteEntrySize = 3;
		}
		else
		{
			readRaw(f, bmpWidth);
			readRaw(f, bmpHeight);

			bmpPaletteEntrySize = 4;
		}

		// A negative height --> Y is top-down
		if (bmpHeight < 0)
		{
			bmpHeight = -bmpHeight;
			bmpFlippedY = false;
		}

		// Making use of do-while so we can break executation in stages.
		// Pretty much the most disgusting misuse of a do-while, but I reckon it actually works pretty well for this.
		do
		{
			bmpPlanes;						readRaw(f, bmpPlanes);
			bmpBitsPerPixel;				readRaw(f, bmpBitsPerPixel);

			if (bmpInferredBitmapType == BITMAPCOREHEADER) break;

			bmpCompression;					readRaw(f, bmpCompression);
			uint32 bmpSizeOfBitmap;			readRaw(f, bmpSizeOfBitmap);
			int32  bmpHorzResolution;		readRaw(f, bmpHorzResolution);
			int32  bmpVertResolution;		readRaw(f, bmpVertResolution);
			uint32 bmpColoursUsed;			readRaw(f, bmpColoursUsed);
			uint32 bmpColoursImpt;			readRaw(f, bmpColoursImpt);

			if (bmpInferredBitmapType == BITMAPINFOHEADER) break;

			uint32 bmpBitmaskR;				readRaw(f, bmpBitmaskR);
			uint32 bmpBitmaskG;				readRaw(f, bmpBitmaskG);
			uint32 bmpBitmaskB;				readRaw(f, bmpBitmaskB);

			if (bmpInferredBitmapType == BITMAPV2INFOHEADER) break;

			uint32 bmpBitmaskA;				readRaw(f, bmpBitmaskA);

			if (bmpInferredBitmapType == BITMAPV3INFOHEADER) break;

			uint32 bmpCSType;				readRaw(f, bmpCSType);
			int32 bmpRedX;					readRaw(f, bmpRedX);
			int32 bmpRedY;					readRaw(f, bmpRedY);
			int32 bmpRedZ;					readRaw(f, bmpRedZ);
			int32 bmpGreenX;				readRaw(f, bmpGreenX);
			int32 bmpGreenY;				readRaw(f, bmpGreenY);
			int32 bmpGreenZ;				readRaw(f, bmpGreenZ);
			int32 bmpBlueX;					readRaw(f, bmpBlueX);
			int32 bmpBlueY;					readRaw(f, bmpBlueY);
			int32 bmpBlueZ;					readRaw(f, bmpBlueZ);
			uint32 bmpGammaR;				readRaw(f, bmpGammaR);
			uint32 bmpGammaG;				readRaw(f, bmpGammaG);
			uint32 bmpGammaB;				readRaw(f, bmpGammaB);

			if (bmpInferredBitmapType == BITMAPV4HEADER) break;

			uint32 bmpIntent;				readRaw(f, bmpIntent);
			uint32 bmpProfileData;			readRaw(f, bmpProfileData);
			uint32 bmpProfileSize;			readRaw(f, bmpProfileSize);
			uint32 bmpReserved;				readRaw(f, bmpReserved);

		} while (false);

		// We don't support any special compression - palette or uncompressed only.
		// Side-note: pretty much nobody supports those :(. They're actually super easy to implement, but since ROS Stage doesn't,
		// there's no point at all.
		if (bmpCompression != 0) return new Texture(Texture::TT_EMPTY, 0, 0);

		// With 1, 4, or 8 bits per pixel, we expect a palette. No bitmaps with other BPP values have a palette
		if (bmpBitsPerPixel == 1 || bmpBitsPerPixel == 4 || bmpBitsPerPixel == 8)
		{
			bmpPaletteLength = 1 << bmpBitsPerPixel;
			bmpPalette = new uint8[bmpPaletteLength * bmpPaletteEntrySize];
			std::fread(bmpPalette, 1, bmpPaletteLength * bmpPaletteEntrySize, f);
			
			for (int i = 0; i < bmpPaletteLength; ++i)
			{
				std::printf("PAL %d %d %d\n", bmpPalette[i * bmpPaletteEntrySize], bmpPalette[i * bmpPaletteEntrySize + 1], bmpPalette[i * bmpPaletteEntrySize + 2]);
			}
		}

		// Calculate the size of each bitmap line, including padding (it's padded to a multiple of 4 bytes)
		bmpByteWidth = ((bmpWidth * bmpBitsPerPixel + 31) >> 3) & (~0x03);
		
		// Fit to the nearest power of two (lots of advantages)
		uint32 outputWidth = potAbove(bmpWidth);
		uint32 outputHeight = potAbove(bmpHeight);

		// Go to the bitmap data, and start reading into an image object
		Texture *outputTexture = new Texture(Texture::TT_RGBA8, outputWidth, outputHeight);
		std::fseek(f, bmpBitmapOffset, SEEK_SET);
		printf("bmpBitmapOffset %d\n", bmpBitmapOffset);
		uint8 bitStart = (8 - (bmpBitsPerPixel & 0x07)) & 0x07;
		uint32 bitMask = (uint64(1) << bmpBitsPerPixel) - 1;
		
		uint8 *outputImage = new uint8[outputWidth * outputHeight * 4];
		uint8 *lineData = new uint8[bmpByteWidth];
		for (int32 bmpY = 0; bmpY < bmpHeight; ++bmpY)
		{
			std::fread(lineData, 1, bmpByteWidth, f);
			int32 y = bmpFlippedY ? bmpHeight - 1 - bmpY : bmpY;

			unsigned int reverseBit = 0;
			unsigned int lineOffset = 0;
			for (int32 x = 0; x < bmpWidth; ++x)
			{
				uint8 valueR, valueG, valueB, valueA;
				uint8 bit = bitStart - reverseBit;

				if (bmpPaletteLength == 0)
				{
					// No palette
					if (bmpBitsPerPixel == 16)
					{
						uint16 rawColour = *(uint16 *)(lineData + lineOffset);
						valueR = (rawColour & 0x7c00) >> 7;
						valueG = (rawColour & 0x03e0) >> 2;
						valueB = (rawColour & 0x1f) << 3;
						valueA = 255;
					}
					else if (bmpBitsPerPixel == 24)
					{
						valueR = lineData[lineOffset + 2];
						valueG = lineData[lineOffset + 1];
						valueB = lineData[lineOffset + 0];
						valueA = 255;
					}
					else if (bmpBitsPerPixel == 32)
					{
						valueR = lineData[lineOffset + 2];
						valueG = lineData[lineOffset + 1];
						valueB = lineData[lineOffset + 0];
						valueA = lineData[lineOffset + 3];
					}
				}
				else
				{
					// Palette
					uint8 paletteEntryIndex = (lineData[lineOffset] >> bit) & bitMask;
					printf("PE:%d\n", paletteEntryIndex);

					if (bmpPaletteEntrySize == 3)
					{
						BMPPaletteEntry3 &paletteEntry = *(BMPPaletteEntry3 *)&(bmpPalette[paletteEntryIndex * bmpPaletteEntrySize]);
						valueR = paletteEntry.r;
						valueG = paletteEntry.g;
						valueB = paletteEntry.b;
						valueA = 255;
					}
					else
					{
						BMPPaletteEntry4 &paletteEntry = *(BMPPaletteEntry4 *)&(bmpPalette[paletteEntryIndex * bmpPaletteEntrySize]);
						valueR = paletteEntry.r;
						valueG = paletteEntry.g;
						valueB = paletteEntry.b;
						valueA = 255;
					}
				}

				uint32 outputImageIndex = (y * outputWidth + x) * 4;
				outputImage[  outputImageIndex] = valueR;
				outputImage[++outputImageIndex] = valueG;
				outputImage[++outputImageIndex] = valueB;
				outputImage[++outputImageIndex] = valueA;

				reverseBit += bmpBitsPerPixel;
				if (reverseBit >= 8)
				{
					lineOffset += reverseBit >> 3;
					reverseBit &= 0x07;
				}
			}
		}
		
		FILE *fTest = std::fopen("ftest.raw", "wb");
		std::fwrite(outputImage, 1, outputWidth * outputHeight * 4, fTest);
		std::fclose(fTest);
		
		printf("oh wow, we made it!\n\n\n\n\n\n\n\n\n\n\n\n\n");
		outputTexture->setData(outputImage);
		
		// Clean up
		delete[] lineData;
		delete[] bmpPalette;
		fclose(f);

		return new Resource();
	}
}
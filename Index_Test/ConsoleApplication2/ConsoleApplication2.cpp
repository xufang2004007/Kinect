// ConsoleApplication2.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <stdio.h>
#include <iostream>

using namespace std;
typedef unsigned short USHORT;
typedef struct _NUI_DEPTH_IMAGE_PIXEL
{
	USHORT playerIndex;
	USHORT depth;
} 	NUI_DEPTH_IMAGE_PIXEL;

int main()
{
	NUI_DEPTH_IMAGE_PIXEL * pBufferRun = nullptr;
	const NUI_DEPTH_IMAGE_PIXEL * pBufferStart = pBufferRun;
	const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferStart + (640 * 480);
	while (1)
	{
		cout << pBufferRun - pBufferStart << endl;
		++pBufferRun;
	}
    return 0;
}


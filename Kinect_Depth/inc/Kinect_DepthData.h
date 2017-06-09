//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

struct Data_Pixel
{
	USHORT Depth;			// 保留深度图像值
	BYTE intensity;			// 保存RGB值
	//BYTE Red_intensity;
	//BYTE Green_intensity;
	//BYTE Blue_intensity;
};
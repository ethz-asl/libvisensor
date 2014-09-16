/*
 * visensor_datatypes.cpp
 *
 *  Created on: Apr 3, 2012
 *      Author: burrimi
 */

#include <visensor/visensor_datatypes.hpp>


namespace visensor
{

ViFrame::ViFrame()
{
	allocated_image_bytes=0;
	height=0;
	width=0;
	camera_id=0;
	timestamp=0;
	timestamp_host=0;
	id=0;
	image_type=ViImageType::NOT_SET;
	useCorners=0;
//	allocated_image_bytes=image_size;
//	image = new unsigned char[allocated_image_bytes];
}

ViFrame::~ViFrame()
{
}

int ViFrame::getBufferSize()
{
	return allocated_image_bytes;
}

void ViFrame::setImageRawPtr(boost::shared_ptr<uint8_t> buffer,
                                 uint32_t buffer_size) {
	image=buffer;
	allocated_image_bytes=buffer_size;
}

uint8_t* ViFrame::getImageRawPtr()
{
	return image.get();
}

} //namespace visensor


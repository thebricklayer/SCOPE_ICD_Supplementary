/*
 * Copyright (C) 2014 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "image.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include "o3d3xx_camera/err.h"
#include "o3d3xx_framegrabber/byte_buffer.hpp"


o3d3xx::ImageBuffer::ImageBuffer()
  : o3d3xx::ByteBuffer(),
    extrinsics_({0., 0., 0., 0., 0., 0.}),
    exposure_times_({0,0,0}),
    time_stamp_(std::chrono::system_clock::now())
{ }

o3d3xx::ImageBuffer::ImageBuffer(const o3d3xx::ImageBuffer& src_buff)
  : o3d3xx::ImageBuffer()
{
  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
}

o3d3xx::ImageBuffer&
o3d3xx::ImageBuffer::operator= (const o3d3xx::ImageBuffer& src_buff)
{
  if (this == &src_buff)
    {
      return *this;
    }

  this->SetBytes(const_cast<std::vector<std::uint8_t>&>(src_buff.bytes_),
                 true);
  return *this;
}

o3d3xx::ImageBuffer::~ImageBuffer()
{
  // DLOG(INFO) << "Image buffer dtor";
}



std::vector<point4D> 
o3d3xx::ImageBuffer::XYZImage()
{
  this->Organize();
  return this->xyz_image_;
}

std::vector<float>
o3d3xx::ImageBuffer::Extrinsics()
{
  this->Organize();
  return this->extrinsics_;
}

std::vector<std::uint32_t>
o3d3xx::ImageBuffer::ExposureTimes()
{
  this->Organize();
  return this->exposure_times_;
}

float
o3d3xx::ImageBuffer::IlluTemp()
{
  this->Organize();
  return this->illu_temp_;
}

o3d3xx::TimePointT
o3d3xx::ImageBuffer::TimeStamp()
{
  this->Organize();
  return this->time_stamp_;
}

void o3d3xx::ImageBuffer::setPosition(float position[3], float dims[3]) {
  this->position[0] = position[0]; this->position[1] = position[1]; this->position[2] = position[2];
  this->dims[0] = dims[0]; this->dims[1] = dims[1]; this->dims[2] = dims[2];
}


void
o3d3xx::ImageBuffer::Organize()
{
  if (! this->Dirty())
    {
      return;
    }
  // get indices to the start of each chunk of interest in the image buffer
  // NOTE: These could get optimized by using apriori values if necessary
  std::size_t INVALID_IDX = std::numeric_limits<std::size_t>::max();
  std::size_t xidx, yidx, zidx, aidx, raw_aidx, cidx, didx, uidx = INVALID_IDX;
  std::size_t extidx = INVALID_IDX;

  xidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CARTESIAN_X);
  yidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CARTESIAN_Y);
  zidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CARTESIAN_Z);
  aidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::AMPLITUDE);
  raw_aidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::RAW_AMPLITUDE);
  cidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::CONFIDENCE);
  didx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::RADIAL_DISTANCE);
  uidx =
    o3d3xx::get_chunk_index(this->bytes_, o3d3xx::image_chunk::UNIT_VECTOR_ALL);
  extidx =
    o3d3xx::get_chunk_index(this->bytes_,
                            o3d3xx::image_chunk::EXTRINSIC_CALIBRATION);

  // We *must* have the confidence image. If we do not, we bail out now
  if (cidx == INVALID_IDX)
    {
      throw o3d3xx::error_t(O3D3XX_IMG_CHUNK_NOT_FOUND);
    }

  const std::uint32_t header_version =  o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+12);
  // for the *big* time stamp minimum header version 2 is needed
  if( header_version > 1 )
    {
      // Retrieve the timespamp information from the confidence data
      const std::uint32_t timestampSec =
        o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+40);
      const std::uint32_t timestampNsec =
        o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+44);
      // convert the time stamp into a TimePointT
      this->time_stamp_ = std::chrono::system_clock::time_point {
                            std::chrono::seconds{timestampSec}
                            + std::chrono::nanoseconds{timestampNsec}
                          };
    }
  else
    {
      // There is no *big* time stamp in chunk version 1
      this->time_stamp_ = std::chrono::system_clock::now();
    }

  bool AMP_OK = aidx != INVALID_IDX;
  bool RAW_AMP_OK = raw_aidx != INVALID_IDX;
  bool RDIST_OK = didx != INVALID_IDX;
  bool UVEC_OK = uidx != INVALID_IDX;
  bool CARTESIAN_OK =
    (xidx != INVALID_IDX) && (yidx != INVALID_IDX) && (zidx != INVALID_IDX);
  bool EXTRINSICS_OK = extidx != INVALID_IDX;


  // Get how many bytes to increment in the buffer for each pixel
  // NOTE: These can be discovered dynamically, however, for now we use our a
  // priori info of the pixel data types
  std::size_t cincr = 1; // uint8_t
  std::size_t xincr = xidx != INVALID_IDX ? 2 : 0; // int16_t
  std::size_t yincr = yidx != INVALID_IDX ? 2 : 0; // int16_t
  std::size_t zincr = zidx != INVALID_IDX ? 2 : 0; // int16_t
  std::size_t aincr = aidx != INVALID_IDX ? 2 : 0; // uint16_t
  std::size_t raw_aincr = raw_aidx != INVALID_IDX ? 2 : 0; // uint16_t
  std::size_t dincr = didx != INVALID_IDX ? 2 : 0; // uint16_t
  std::size_t uincr = uidx != INVALID_IDX ? 4 * 3 : 0; // float32 * 3
  std::size_t extincr = extidx != INVALID_IDX ? 4 : 0; // float32

  // NOTE: we use the `cidx' corresponding to the confidence image because
  // it is an invariant in terms of what we send to the camera as valid pcic
  // schemas we wish to process.
  std::uint32_t width =
    o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+16);
  std::uint32_t height =
    o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+20);

  std::uint32_t num_points = width * height;


  //
  // setup images
  //

  this->xyz_image_.reserve(height * width);
  this->xyz_image_.clear();

  // move index pointers to where pixel data starts -- we assume
  // (I think safely) that all header sizes will be uniform in the data stream,
  // so, we use our invariant of the confidence image header
  std::uint32_t pixel_data_offset =
    o3d3xx::mkval<std::uint32_t>(this->bytes_.data()+cidx+8);

  cidx += pixel_data_offset;
  didx += RDIST_OK ? pixel_data_offset : 0;
  uidx += UVEC_OK ? pixel_data_offset : 0;
  aidx += AMP_OK ? pixel_data_offset : 0;
  raw_aidx += RAW_AMP_OK ? pixel_data_offset : 0;
  if (CARTESIAN_OK)
    {
      xidx += pixel_data_offset;
      yidx += pixel_data_offset;
      zidx += pixel_data_offset;
    }
  extidx += EXTRINSICS_OK ? pixel_data_offset : 0;

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  std::uint16_t bad_pixel = std::numeric_limits<std::uint16_t>::quiet_NaN();
  std::int16_t bad_pixel_s = std::numeric_limits<std::int16_t>::quiet_NaN();

  int col = 0;
  int row = -1;
  int xyz_col = 0;
  int uvec_col = 0;

  std::uint8_t conf_row_ptr;

  std::int16_t x_, y_, z_;
  float e_x, e_y, e_z;
  point4D pt;
  std::string fname = "clouds/cloud_big_3_9-" + std::to_string(this->num) + ".txt";
  FILE *f = fopen(fname.c_str(), "w");
  this->num++;
  for (std::size_t i = 0; i < num_points;
       ++i, xidx += xincr, yidx += yincr, zidx += zincr,
         cidx += cincr, aidx += aincr, didx += dincr,
         raw_aidx += raw_aincr, uidx += uincr)
    {
      col = i % width;

      conf_row_ptr = this->bytes_.at(cidx);
      if ((conf_row_ptr & 0x1) == 1)
        {
          continue;
        }
      else
        {
          // convert the coord frame  to a right-handed frame for the point
          // cloud
          if (CARTESIAN_OK)
            {
              x_ = o3d3xx::mkval<std::int16_t>(this->bytes_.data()+zidx);
              y_ = -o3d3xx::mkval<std::int16_t>(this->bytes_.data()+xidx);
              z_ = -o3d3xx::mkval<std::int16_t>(this->bytes_.data()+yidx);
              // this->position[0] += .05;
              // convert units to meters for the point cloud
              pt.point[0] = x_ / 1000.0f;
              pt.point[1] = y_ / 1000.0f;
              pt.point[2] = z_ / 1000.0f;
              pt.point[3] = 1.0f;
              if(pt.point[0] > this->position[0] + 0.05 || pt.point[0] < this->position[0] - this->dims[0] - 0.5)
                continue;
              if(pt.point[1] > this->position[1] + this->dims[1] || pt.point[1] < this->position[1] - this->dims[1])
                continue;
              if(pt.point[2] > this->position[2] + this->dims[2] || pt.point[2] < this->position[2] - this->dims[2])
                continue;
              fprintf(f, "%d  %d  %d\n", x_, y_, z_);
              
            }
          else
            {
                continue;
            }

          // keep depth image data as mm
        }
        xyz_image_.push_back(pt);
    }


  //
  // Parse out the extrinsics
  //
  if (EXTRINSICS_OK)
    {
      for (std::size_t i = 0; i < 6; ++i, extidx += extincr)
        {
          this->extrinsics_[i] =
            o3d3xx::mkval<float>(this->bytes_.data()+extidx);
        }
    }

  //
  // OK, now we want to see if the temp illu and exposure times are present,
  // if they are, we want to parse them out and store them in the image buffer.
  // Since the extrinsics are invariant and should *always* be present, we use
  // the current index of the extrinsics.
  if (EXTRINSICS_OK)
    {
      std::size_t extime_idx = extidx;
      int bytes_left = this->bytes_.size() - extime_idx;

      // Read extime (6 bytes string + 3x 4 bytes uint32_t)
      if(bytes_left >= 18
	 && std::equal(this->bytes_.begin() + extidx,
		       this->bytes_.begin() + extidx + 6,
		       std::begin("extime")))
	{
	  extime_idx += 6;
	  bytes_left -= 6;

	  // 3 exposure times
	  for (std::size_t i = 0; i < 3; ++i)
	    {
	      if ((bytes_left - 6) <= 0)
		{
		  break;
		}
	      
	      std::uint32_t extime =
		o3d3xx::mkval<std::uint32_t>(
	          this->bytes_.data()+extime_idx);
	      
	      this->exposure_times_.at(i) = extime;
	      
	      extime_idx += 4;
	      bytes_left -= 4;
	    }
	}
      else
	{
	  std::fill(this->exposure_times_.begin(),
		    this->exposure_times_.end(), 0);
	}

      // Read temp_illu (9 bytes string + 4 bytes float)
      if(bytes_left >= 13
	 && std::equal(this->bytes_.begin() + extidx,
		       this->bytes_.begin() + extidx + 8,
		       std::begin("temp_illu")))
	{
	  extime_idx += 9;
	  bytes_left -= 9;

	  this->illu_temp_ =
	    o3d3xx::mkval<float>(this->bytes_.data() + extime_idx);

	  extime_idx += 4;
	  bytes_left -= 4;
	}
      else
	{
	  this->illu_temp_ = 0;
	}
    }


  fclose(f);
  this->_SetDirty(false);
  // Perform sub sampling if image has more than 5000 points
  num_points = xyz_image_.size();
  if(num_points > 5000) {
    std::vector<point4D> temp_points;
    temp_points.reserve(num_points);
    if(num_points < 10000) {
      for(std::size_t i = 0; i < num_points; i+=2) {
        temp_points.push_back(xyz_image_[i]);
      }
    } else {
      for(std::size_t i = 0; i < num_points; i+=3) {
        temp_points.push_back(xyz_image_[i]);
      }
    }
    xyz_image_ = temp_points;
  } 
}

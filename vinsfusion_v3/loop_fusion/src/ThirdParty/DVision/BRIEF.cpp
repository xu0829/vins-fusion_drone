// VINS_OPENCV4_COMPAT: OpenCV 4 兼容头。由 tools/apply_opencv4_compat.sh 插入。
// 这些 C 风格常量在 OpenCV 4 里已不再由 opencv2/opencv.hpp 自动带出，需显式包含。
#include <opencv2/core/types_c.h>                 // 终止条件常量
#include <opencv2/imgproc/types_c.h>              // 阈值/连通域/颜色转换/形态学常量
#include <opencv2/imgproc/imgproc_c.h>            // 抗锯齿线型、字体常量
#include <opencv2/calib3d/calib3d_c.h>            // 棋盘格检测标志
#include <opencv2/imgcodecs/legacy/constants_c.h> // 图像读取模式常量
/**
 * File: BRIEF.cpp
 * Author: Dorian Galvez
 * Date: September 2010
 * Description: implementation of BRIEF (Binary Robust Independent 
 *   Elementary Features) descriptor by 
 *   Michael Calonder, Vincent Lepetitand Pascal Fua
 *   + close binary tests (by Dorian Galvez-Lopez)
 * License: see the LICENSE.txt file
 *
 */

#include "BRIEF.h"
#include "../DUtils/DUtils.h"
#include <boost/dynamic_bitset.hpp>
#include <vector>

using namespace std;
using namespace DVision;

// ----------------------------------------------------------------------------

BRIEF::BRIEF(int nbits, int patch_size, Type type):
  m_bit_length(nbits), m_patch_size(patch_size), m_type(type)
{
  assert(patch_size > 1);
  assert(nbits > 0);
  generateTestPoints();
}

// ----------------------------------------------------------------------------

BRIEF::~BRIEF()
{
}

// ---------------------------------------------------------------------------

void BRIEF::compute(const cv::Mat &image, 
    const std::vector<cv::KeyPoint> &points,
    vector<bitset> &descriptors,
    bool treat_image) const
{
  const float sigma = 2.f;
  const cv::Size ksize(9, 9);
  
  cv::Mat im;
  if(treat_image)
  {
    cv::Mat aux;
    if(image.depth() == 3)
    {
      cv::cvtColor(image, aux, CV_RGB2GRAY);
    }
    else
    {
      aux = image;
    }

    cv::GaussianBlur(aux, im, ksize, sigma, sigma);
    
  }
  else
  {
    im = image;
  }
  
  assert(im.type() == CV_8UC1);
  assert(im.isContinuous());
  
  // use im now
  const int W = im.cols;
  const int H = im.rows;
  
  descriptors.resize(points.size());
  std::vector<bitset>::iterator dit;

  std::vector<cv::KeyPoint>::const_iterator kit;
  
  int x1, y1, x2, y2;
  
  dit = descriptors.begin();
  for(kit = points.begin(); kit != points.end(); ++kit, ++dit)
  {
    dit->resize(m_bit_length);
    dit->reset();

    for(unsigned int i = 0; i < m_x1.size(); ++i)
    {
      x1 = (int)(kit->pt.x + m_x1[i]);
      y1 = (int)(kit->pt.y + m_y1[i]);
      x2 = (int)(kit->pt.x + m_x2[i]);
      y2 = (int)(kit->pt.y + m_y2[i]);
      
      if(x1 >= 0 && x1 < W && y1 >= 0 && y1 < H 
        && x2 >= 0 && x2 < W && y2 >= 0 && y2 < H)
      {
        if( im.ptr<unsigned char>(y1)[x1] < im.ptr<unsigned char>(y2)[x2] )
        {
          dit->set(i);
        }        
      } // if (x,y)_1 and (x,y)_2 are in the image
            
    } // for each (x,y)
  } // for each keypoint
}

// ---------------------------------------------------------------------------

void BRIEF::generateTestPoints()
{  
  m_x1.resize(m_bit_length);
  m_y1.resize(m_bit_length);
  m_x2.resize(m_bit_length);
  m_y2.resize(m_bit_length);

  const float g_mean = 0.f;
  const float g_sigma = 0.2f * (float)m_patch_size;
  const float c_sigma = 0.08f * (float)m_patch_size;
  
  float sigma2;
  if(m_type == RANDOM)
    sigma2 = g_sigma;
  else
    sigma2 = c_sigma;
  
  const int max_v = m_patch_size / 2;
  
  DUtils::Random::SeedRandOnce();
  
  for(int i = 0; i < m_bit_length; ++i)
  {
    int x1, y1, x2, y2;
    
    do
    {
      x1 = DUtils::Random::RandomGaussianValue(g_mean, g_sigma);
    } while( x1 > max_v || x1 < -max_v);
    
    do
    {
      y1 = DUtils::Random::RandomGaussianValue(g_mean, g_sigma);
    } while( y1 > max_v || y1 < -max_v);
    
    float meanx, meany;
    if(m_type == RANDOM)
      meanx = meany = g_mean;
    else
    {
      meanx = x1;
      meany = y1;
    }
    
    do
    {
      x2 = DUtils::Random::RandomGaussianValue(meanx, sigma2);
    } while( x2 > max_v || x2 < -max_v);
    
    do
    {
      y2 = DUtils::Random::RandomGaussianValue(meany, sigma2);
    } while( y2 > max_v || y2 < -max_v);
    
    m_x1[i] = x1;
    m_y1[i] = y1;
    m_x2[i] = x2;
    m_y2[i] = y2;
  }

}

// ----------------------------------------------------------------------------



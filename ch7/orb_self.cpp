#include <iostream>
#include <opencv2/core/core.hpp>

typedef vector<uint32_t> DescType;

void ComputeORB(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, 
    Vector<DescType> &descriptors)
{
    const int half_patch_size = 8;
    const int half_boundary = 16;
    int bad_points = 0;
   for (auto &kp : keypoints)
   {
        if (kp.pt.x < half_boundary || kp.pt.y < half_boundary) ||
            kp.pt.x >= img.cols - half_boundary || kp.pt.y >= img.rows -half_boundary)
        {
            bad_points++;
            descriptors.push_back({});
            continue;
        }

        float m01 = 0, m10 = 0;
        for (int dx = -half_patch_size; dx < half_patch_size; ++dx)
        {
            for (dy = -half_patch_size; dy < half_patch_size; ++dy)
            {
                uchar pixel = img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
                m01 += dx * pixel;
                m10 += dy * pixel;
            }            
        }
        
        float m_sqrt = sqrt(m01 * m01 + m10 * m10);
        float sin_theta = m01 / m_sqrt;
        float cos_theta = m10 / m_sqrt;    

        // TODO .....
   }
   
    
}
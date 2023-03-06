#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        cout << "usage: feature_extraction img1 img2 " << endl;
        return 1;
    }
    // -- read image
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);

    // -- initial
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptor_1,descriptor_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // -- First: detect Orented FAST
    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    // -- Second: compute BRIEF descriptor by Oriented FAST
    descriptor->compute(img_1,keypoints_1,descriptor_1);
    descriptor->compute(img_2,keypoints_2,descriptor_2);

    // -- Third: match the BRIEF descriptor of two images, using Hamming distance
    vector<DMatch> matches;
    matcher->match(descriptor_1,descriptor_2,matches);

    // -- 4th: choose matches
    double min_dist = 10000, max_dist = 0;

    // find the max/min distance
    for (int i = 0; i < descriptor_1.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    // only for funny
    min_dist = min_element(matches.begin(),matches.end(),
        [](const DMatch &m1,const DMatch &m2){return m1.distance < m2.distance;})->distance;
    max_dist = max_element(matches.begin(),matches.end(),
        [](const DMatch &m1,const DMatch &m2){return m1.distance < m2.distance;})->distance;

    printf("-- Max dist: %f \n",max_dist);
    printf("-- Min dist: %f \n",min_dist);   

    // the distance is more than 2*min_dist, or 30
    vector<DMatch> good_matches;
    for (int i = 0; i < descriptor_1.rows; i++)
    {
        if (matches[i].distance < max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }        
    }   

    // 5th: draw result 
    Mat img_match;
    Mat img_goomatch;
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,good_matches,img_goomatch);
    imshow(" all pair matches ", img_match);
    imshow(" good matches ", img_goomatch);
    waitKey(0);

    return 0;

    
}
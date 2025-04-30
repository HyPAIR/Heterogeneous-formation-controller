#include <splined_voronoi/voronoi_generation.h>

namespace voronoi_generation
{

bool is_surrounding_occupied(const cv::Mat& input_img, cv::Point2i pixel, int search_size)
{
    for (int xoff = -search_size; xoff <= search_size; ++xoff)
    {
        for (int yoff = -search_size; yoff <= search_size; ++yoff)
        {
            cv::Point2i surr_pixel = pixel + cv::Point2i(xoff, yoff);
            // Check if the pixel is within a map
            if (surr_pixel.x < 0 || surr_pixel.x >= input_img.cols || surr_pixel.y < 0 ||
                surr_pixel.y >= input_img.rows)
            {
                continue;
            }
            if (input_img.at<uchar>(surr_pixel) != 0)
            {
                return true;
            }
        }
    }
    return false;
}

void draw_edges(const voronoi_diagram<double>& vd, const cv::Mat& input_img, cv::Mat& output_img, const std::vector<cv::Point2i>& points)
{
    bool internal_edges_only = true;
    output_img = cv::Mat(input_img.size(), CV_8UC1, cv::Scalar(127));
    int num_edges = vd.edges().size();
    int count_curved = 0;
    for (auto edge : vd.edges())
    {
        if (!edge.is_primary())
        {
            continue;
        }
        if (internal_edges_only && !edge.is_finite())
        {
            continue;
        }
        if (edge.is_finite())
        {
            cv::Point2i vertex0(edge.vertex0()->x(), edge.vertex0()->y());
            cv::Point2i vertex1(edge.vertex1()->x(), edge.vertex1()->y());
            // dont show all edges which are curved or which endpoints are outside the image or occupied
            if (edge.is_curved())
            {
                count_curved++;
                continue;
            }
            // dont include all edges which start or end is outside of map
            if (vertex0.x > output_img.cols || vertex0.x < 0 || vertex0.y < 0 || vertex0.y > output_img.rows)
            {
                continue;
            }
            if (vertex1.x > output_img.cols || vertex1.x < 0 || vertex1.y < 0 || vertex1.y > output_img.rows)
            {
                continue;
            }
            // dont include edges which start or end is inside obstacle or directly surrounded by obstacle
            if (is_surrounding_occupied(input_img, vertex0, 2))
            {
                continue;
            }
            if (is_surrounding_occupied(input_img, vertex1, 2))
            {
                continue;
            }
            // draw all remaining edges
            cv::line(output_img, vertex0, vertex1, cv::Scalar(255), 1, 8);
        }
    }

    BOOST_FOREACH (const cv::Point2i point, points)
    {
        output_img.at<uchar>(point) = 0;
    }
}

bool create_boost_voronoi(const cv::Mat& img_binary, cv::Mat& voronoi_img)
{
    if (img_binary.empty())
    {
        ROS_ERROR("Got empty image, aborting!");
        return false;
    }
    // get contour of obstacles to reduce amount of obstacles in image
    cv::Mat canny_output;
    cv::Canny(img_binary, canny_output, 100, 200);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat img_binary_contours = cv::Mat(canny_output.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat img_binary_contours_out = cv::Mat(canny_output.size(), CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(255);
        cv::drawContours(img_binary_contours, contours, (int)i, color, 1, cv::LINE_4, hierarchy, 0);
        cv::drawContours(img_binary_contours_out, contours, (int)i, color, 3, cv::LINE_4, hierarchy, 0);
    }

    // get all obstacle points from contours
    std::vector<cv::Point2i> points;
    cv::Mat non_zero_coordinates;
    cv::findNonZero(img_binary_contours, non_zero_coordinates);
    for (int i = 0; i < non_zero_coordinates.total(); i++)
    {
        points.push_back(non_zero_coordinates.at<cv::Point2i>(i));
    }
    std::vector<Segment> segments;

    boost::polygon::voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);

    // filtering and creation of voronoi diagram as image
    draw_edges(vd, img_binary, voronoi_img, points);

    return true;
}
} // namespace voronoi_generation

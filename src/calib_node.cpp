#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <Eigen/Dense>

std::vector<Eigen::Vector3d> clicked_points;

void pointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    Eigen::Vector3d p(msg->point.x, msg->point.y, msg->point.z);
    clicked_points.push_back(p);
    ROS_INFO("Received point: [%.3f, %.3f, %.3f]", p.x(), p.y(), p.z());

    if (clicked_points.size() < 3)
    {
        ROS_WARN("Need at least 3 points to compute calibration.");
        return;
    }
    // RANSAC parameters
    const int max_iterations = 100;
    const double distance_threshold = 0.02; // 2cm
    int best_inlier_count = 0;
    Eigen::Vector4d best_plane;

    size_t N = clicked_points.size();
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        // Randomly select 3 unique points
        std::vector<int> idxs;
        while (idxs.size() < 3)
        {
            int idx = rand() % N;
            if (std::find(idxs.begin(), idxs.end(), idx) == idxs.end())
                idxs.push_back(idx);
        }
        Eigen::Vector3d p1 = clicked_points[idxs[0]];
        Eigen::Vector3d p2 = clicked_points[idxs[1]];
        Eigen::Vector3d p3 = clicked_points[idxs[2]];

        // Compute plane normal
        Eigen::Vector3d v1 = p2 - p1;
        Eigen::Vector3d v2 = p3 - p1;
        Eigen::Vector3d normal = v1.cross(v2);
        if (normal.norm() < 1e-6) continue; // Degenerate

        normal.normalize();
        double d = -normal.dot(p1);

        // Count inliers
        int inlier_count = 0;
        for (const auto& pt : clicked_points)
        {
            double dist = std::abs(normal.dot(pt) + d);
            if (dist < distance_threshold)
                ++inlier_count;
        }

        if (inlier_count > best_inlier_count)
        {
            best_inlier_count = inlier_count;
            best_plane << normal, d;
        }
    }

    // Least squares fit using inliers
    std::vector<Eigen::Vector3d> inliers;
    for (const auto& pt : clicked_points)
    {
        double dist = std::abs(best_plane.head<3>().dot(pt) + best_plane[3]);
        if (dist < distance_threshold)
            inliers.push_back(pt);
    }
    Eigen::MatrixXd A(inliers.size(), 3);
    for (size_t i = 0; i < inliers.size(); ++i)
        A.row(i) = inliers[i];
    Eigen::VectorXd b = -Eigen::VectorXd::Ones(inliers.size());
    Eigen::Vector4d plane;
    Eigen::MatrixXd AA(inliers.size(), 4);
    AA << A, Eigen::VectorXd::Ones(inliers.size());
    plane = AA.colPivHouseholderQr().solve(b);
    Eigen::Vector3d n = plane.head<3>();
    double d = plane[3];
    n.normalize();

    // Ensure z-axis angle <= 90deg and x-axis alignment
    Eigen::Vector3d z_base(0, 0, 1);
    if (n.dot(z_base) < 0)
        n = -n, d = -d;

    // Find rotation: n -> z_base, preserve x as much as possible
    Eigen::Vector3d x_cam(1, 0, 0);
    Eigen::Vector3d y_cam = n.cross(x_cam).normalized();
    Eigen::Vector3d x_new = y_cam.cross(n).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_new;
    R.col(1) = y_cam;
    R.col(2) = n;

    // Translation: move plane to z=0 in base_link
    double z_offset = d; // plane: n^T * X + d = 0
    Eigen::Vector3d t = -z_offset * n;

    // Convert rotation to quaternion
    Eigen::Quaterniond q(R);

    // 反转变换：base_link->camera_link
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = R_inv * (-t);
    // 修正 t_inv，使地面点变换到 base_link 后 z=0
    double z_sum = 0.0;
    for (const auto& pt : inliers) {
        Eigen::Vector3d pt_base = R_inv * pt + t_inv;
        z_sum += pt_base.z();
    }
    double z_mean = z_sum / inliers.size();
    t_inv.z() -= z_mean;
    Eigen::Quaterniond q_inv(R_inv);

    // Print result: base_link as parent, camera_link as child
    ROS_INFO_STREAM("rosrun tf2_ros static_transform_publisher "
        << t_inv.x() << " " << t_inv.y() << " " << t_inv.z() << " "
        << q_inv.x() << " " << q_inv.y() << " " << q_inv.z() << " " << q_inv.w()
        << " /base_link /camera_link");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_extrinsic_calib_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/clicked_point", 100, pointCallback);

    ros::spin();
    return 0;
}

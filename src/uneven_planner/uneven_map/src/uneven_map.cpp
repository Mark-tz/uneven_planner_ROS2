// method UnevenMap::init()
#include "uneven_map/uneven_map.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "lightning/srv/save_map.hpp"


namespace uneven_planner
{
    RXS2 UnevenMap::filter(Eigen::Vector3d pos, vector<Eigen::Vector3d> points)
    {
        RXS2 rs2;

        Eigen::Vector3d mean_points = Eigen::Vector3d::Zero();
        for (size_t i=0; i<points.size(); i++)
            mean_points+=points[i];

        mean_points /= (double)points.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (size_t i=0; i<points.size(); i++)
        {
            Eigen::Vector3d v = points[i] - mean_points;
            cov += v * v.transpose();
        }
        cov /= (double)points.size();
        Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();
        Eigen::Matrix3d V = es.pseudoEigenvectors();
        Eigen::MatrixXd::Index evalsMax;
        D.minCoeff(&evalsMax);
        Eigen::Matrix<double, 3, 1> n = V.col(evalsMax);
        n.normalize();
        if (n(2, 0) < 0.0)
            n = -n;
        
        rs2.sigma = D(evalsMax) / D.sum() * 3.0;
        if (isnan(rs2.sigma))
        {
            rs2.sigma = 1.0;
            n = Eigen::Vector3d(1.0, 0.0, 0.0);
        }
        rs2.z = mean_points.z();
        rs2.zb.x() = n(0, 0);
        rs2.zb.y() = n(1, 0);

        return rs2;
    }

    Eigen::Matrix3d UnevenMap::skewSym(Eigen::Vector3d vec)
    {
        Eigen::Matrix3d skem_sym;
        skem_sym << 0.0    , -vec(2), vec(1) , \
                    vec(2) , 0.0    , -vec(0), \
                    -vec(1), vec(0) , 0.0       ;
        return skem_sym;
    }

    // zb, yb = (zb x xyaw).normalized(), xb = yb x zb
    // using Sherman-Morrison formula
    double UnevenMap::calYawFromR(Eigen::Matrix3d R)
    {
        Eigen::Vector2d p(R(0, 2), R(1, 2));
        Eigen::Vector2d b(R(0, 0), R(1, 0));
        Eigen::Vector2d x = (Eigen::Matrix2d::Identity()+p*p.transpose()/(1.0-p.squaredNorm()))*b;
        return atan2(x(1), x(0));
    }

    void UnevenMap::normSO2(double& yaw)
    {
        while (yaw < -M_PI)
            yaw += 2*M_PI;
        while (yaw > M_PI)
            yaw -= 2*M_PI;
        return;
    }



    bool UnevenMap::init(std::shared_ptr<rclcpp::Node> node)
    {
        // 保存 node 指针用于日志
        node_ = node;
        
        // 参数声明和读取
        node->declare_parameter<int>("uneven_map/iter_num", 10);
        node->declare_parameter<double>("uneven_map/ellipsoid_x", 0.5);
        node->declare_parameter<double>("uneven_map/ellipsoid_y", 0.5);
        node->declare_parameter<double>("uneven_map/ellipsoid_z", 0.5);
        node->declare_parameter<double>("uneven_map/xy_resolution", 0.1);
        node->declare_parameter<double>("uneven_map/yaw_resolution", 0.1);
        node->declare_parameter<double>("uneven_map/min_cnormal", 0.5);
        node->declare_parameter<double>("uneven_map/max_rho", 1.0);
        node->declare_parameter<double>("uneven_map/gravity", 9.8);
        node->declare_parameter<double>("uneven_map/mass", 1.0);
        node->declare_parameter<double>("uneven_map/map_size_x", 20.0);
        node->declare_parameter<double>("uneven_map/map_size_y", 20.0);
        node->declare_parameter<double>("uneven_map/z_min", -0.01);
        node->declare_parameter<double>("uneven_map/z_max", 5.0);
        node->declare_parameter<int>("uneven_map/init_knn", 7);
        node->declare_parameter<double>("uneven_map/init_xy_max", 0.5);
        node->declare_parameter<double>("uneven_map/global_voxel_res", 0.1);

        node->get_parameter("uneven_map/iter_num", iter_num);
        node->get_parameter("uneven_map/ellipsoid_x", ellipsoid_x);
        node->get_parameter("uneven_map/ellipsoid_y", ellipsoid_y);
        node->get_parameter("uneven_map/ellipsoid_z", ellipsoid_z);
        node->get_parameter("uneven_map/xy_resolution", xy_resolution);
        node->get_parameter("uneven_map/yaw_resolution", yaw_resolution);
        node->get_parameter("uneven_map/min_cnormal", min_cnormal);
        node->get_parameter("uneven_map/max_rho", max_rho);
        node->get_parameter("uneven_map/gravity", gravity);
        node->get_parameter("uneven_map/mass", mass);
        node->get_parameter("uneven_map/z_min", z_min);
        node->get_parameter("uneven_map/z_max", z_max);
        node->get_parameter("uneven_map/init_knn", init_knn);
        node->get_parameter("uneven_map/init_xy_max", init_xy_max);
        node->get_parameter("uneven_map/global_voxel_res", global_voxel_res);
        
        double map_size_x, map_size_y;
        node->get_parameter("uneven_map/map_size_x", map_size_x);
        node->get_parameter("uneven_map/map_size_y", map_size_y);
        map_size = Eigen::Vector3d(map_size_x, map_size_y, 2.0 * M_PI + 5e-2);
    
        origin_pub   = node->create_publisher<sensor_msgs::msg::PointCloud2>("/origin_map", 10);
        filtered_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_map", 10);
        zb_pub       = node->create_publisher<visualization_msgs::msg::Marker>("/zb_map", 10);
        so2_test_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/so2_map", 10);
    
        using namespace std::chrono_literals;
        vis_timer_ = node->create_wall_timer(1s, std::bind(&UnevenMap::updateVisualizations, this));
        
        // origin and boundary
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        map_origin = min_boundary;

        // resolution
        xy_resolution_inv = 1.0 / xy_resolution;
        yaw_resolution_inv = 1.0 / yaw_resolution;

        // voxel num
        voxel_num(0) = ceil(map_size(0) / xy_resolution);
        voxel_num(1) = ceil(map_size(1) / xy_resolution);
        voxel_num(2) = ceil(map_size(2) / yaw_resolution);

        // idx
        min_idx = Eigen::Vector3i::Zero();
        max_idx = voxel_num - Eigen::Vector3i::Ones();

        // datas
        int buffer_size  = voxel_num(0) * voxel_num(1) * voxel_num(2);
        map_buffer = vector<RXS2>(buffer_size, RXS2());
        c_buffer   = vector<double>(buffer_size, 1.0);
        occ_buffer = vector<char>(buffer_size, 0);
        occ_r2_buffer = vector<char>(getXYNum(), 0);
        world_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        world_cloud_plane.reset(new pcl::PointCloud<pcl::PointXY>());

        // Get global map service client
        getmap_client_ = node_->create_client<lightning::srv::GetGlobalMap>("/lightning/get_global_map");
        while (!getmap_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<lightning::srv::GetGlobalMap::Request>();
        request->res = static_cast<float>(global_voxel_res);
        getmap_client_->async_send_request(request, 
            [this](rclcpp::Client<lightning::srv::GetGlobalMap>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(node_->get_logger(), "Initial map received successfully.");
                    generateMapFromCloud(std::make_shared<sensor_msgs::msg::PointCloud2>(result->map));
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to get initial map.");
                }
            });

        // Global map subscriber
        global_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_global", 10, std::bind(&UnevenMap::globalMapCallback, this, std::placeholders::_1));

        // Periodically request global map to refresh
        map_timer_ = node->create_wall_timer(2s, [this]() {
            auto req = std::make_shared<lightning::srv::GetGlobalMap::Request>();
            req->res = static_cast<float>(global_voxel_res);
            getmap_client_->async_send_request(req, [this](rclcpp::Client<lightning::srv::GetGlobalMap>::SharedFuture future) {
                auto res = future.get();
                if (res->success) {
                    generateMapFromCloud(std::make_shared<sensor_msgs::msg::PointCloud2>(res->map));
                }
            });
        });

        return true;
    }




    void UnevenMap::generateMapFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloudMapOrigin;
        pcl::fromROSMsg(*cloud_msg, cloudMapOrigin);

        pcl::CropBox<pcl::PointXYZ> clipper;
        clipper.setMin(Eigen::Vector4f(-10.0, -10.0, z_min, 1.0));
        clipper.setMax(Eigen::Vector4f(10.0, 10.0, z_max, 1.0));
        clipper.setInputCloud(cloudMapOrigin.makeShared());
        clipper.filter(*world_cloud);

        pcl::VoxelGrid<pcl::PointXYZ> dwzFilter;
        dwzFilter.setLeafSize(0.01, 0.01, 0.01);
        dwzFilter.setInputCloud(world_cloud);
        dwzFilter.filter(*world_cloud);

        world_cloud_plane->points.clear();
        for (size_t i = 0; i < world_cloud->points.size(); i++)
        {
            pcl::PointXY p;
            p.x = world_cloud->points[i].x;
            p.y = world_cloud->points[i].y;
            world_cloud_plane->points.emplace_back(p);
        }
        world_cloud->width = world_cloud->points.size();
        world_cloud->height = 1;
        world_cloud->is_dense = true;
        world_cloud->header.frame_id = "map";
        world_cloud_plane->width = world_cloud_plane->points.size();
        world_cloud_plane->height = 1;
        world_cloud_plane->is_dense = true;
        world_cloud_plane->header.frame_id = "map";
        kd_tree.setInputCloud(world_cloud);
        kd_tree_plane.setInputCloud(world_cloud_plane);
        pcl::toROSMsg(*world_cloud, origin_cloud_msg);

        // construct map: SO(2) --> RXS2
        const double box_r = max(max(ellipsoid_x, ellipsoid_y), ellipsoid_z);
        const Eigen::Vector3d ellipsoid_vecinv(1.0 / ellipsoid_x, 1.0 / ellipsoid_y, 1.0 / ellipsoid_z);

        for (int x = 0; x < voxel_num[0]; x++)
            for (int y = 0; y < voxel_num[1]; y++)
                for (int yaw = 0; yaw < voxel_num[2]; yaw++)
                    for (int iter = 0; iter < iter_num; iter++)
                    {
                        Eigen::Vector3d map_pos;
                        RXS2 map_rs2 = map_buffer[toAddress(x, y, yaw)];
                        double map_c = c_buffer[toAddress(x, y, yaw)];
                        indexToPos(Eigen::Vector3i(x, y, yaw), map_pos);

                        Eigen::Vector3d xyaw(cos(map_pos(2)), sin(map_pos(2)), 0.0);
                        Eigen::Vector3d zb(map_rs2.zb(0), map_rs2.zb(1), map_c);
                        Eigen::Vector3d yb = zb.cross(xyaw).normalized();
                        Eigen::Vector3d xb = yb.cross(zb);
                        Eigen::Matrix3d RT;
                        RT.row(0) = xb;
                        RT.row(1) = yb;
                        RT.row(2) = zb;
                        Eigen::Vector3d world_pos(map_pos(0), map_pos(1), map_rs2.z);
                        world_pos.head(2) += xb.head(2) * 0.12;

                        vector<int> Idxs;
                        vector<float> SquaredDists;
                        if (iter == 0)
                        {
                            pcl::PointXY pxy;
                            pxy.x = world_pos(0);
                            pxy.y = world_pos(1);
                            std::vector<int> idxs_knn;
                            std::vector<float> dists_knn;
                            if (kd_tree_plane.nearestKSearch(pxy, init_knn, idxs_knn, dists_knn) > 0) {
                                double min_z = std::numeric_limits<double>::infinity();
                                double min_xy = std::numeric_limits<double>::infinity();
                                for (size_t k = 0; k < idxs_knn.size(); ++k) {
                                    int ii = idxs_knn[k];
                                    const auto &pt2 = world_cloud_plane->points[ii];
                                    double dx = pt2.x - world_pos(0);
                                    double dy = pt2.y - world_pos(1);
                                    double dxy = std::sqrt(dx*dx + dy*dy);
                                    if (dxy < min_xy) min_xy = dxy;
                                    min_z = std::min(min_z, static_cast<double>(world_cloud->points[ii].z));
                                }
                                if (min_xy <= init_xy_max && std::isfinite(min_z)) {
                                    world_pos(2) = min_z;
                                } else {
                                    world_pos(2) = 0.0;
                                }
                            } else {
                                world_pos(2) = 0.0;
                            }
                        }

                        vector<Eigen::Vector3d> points;
                        pcl::PointXYZ pt;
                        pt.x = world_pos(0);
                        pt.y = world_pos(1);
                        pt.z = world_pos(2);
                        if (kd_tree.radiusSearch(pt, box_r, Idxs, SquaredDists) > 0)
                        {
                            for (size_t i = 0; i < Idxs.size(); i++)
                            {
                                Eigen::Vector3d temp_pos(world_cloud->points[Idxs[i]].x, \
                                                         world_cloud->points[Idxs[i]].y, \
                                                         world_cloud->points[Idxs[i]].z);
                                Eigen::Vector3d temp_subtract = temp_pos - world_pos;
                                Eigen::Vector3d temp_inrob = RT * temp_subtract;
                                if (ellipsoid_vecinv.cwiseProduct(temp_inrob).squaredNorm() < 1.0)
                                {
                                    points.emplace_back(temp_pos);
                                }
                            }
                        }
                        if (points.empty())
                        {
                            RXS2 rxs2_z;
                            rxs2_z.z = world_pos(2);
                            map_buffer[toAddress(x, y, yaw)] = rxs2_z;
                            c_buffer[toAddress(x, y, yaw)] = map_buffer[toAddress(x, y, yaw)].getC();
                        }
                        else
                        {
                            map_buffer[toAddress(x, y, yaw)] = UnevenMap::filter(map_pos, points);
                            c_buffer[toAddress(x, y, yaw)] = map_buffer[toAddress(x, y, yaw)].getC();
                        }
                    }

        // occ map
        for (int x = 0; x < voxel_num[0]; x++)
            for (int y = 0; y < voxel_num[1]; y++)
                for (int yaw = 0; yaw < voxel_num[2]; yaw++)
                {
                    if (c_buffer[toAddress(x, y, yaw)] < min_cnormal || map_buffer[toAddress(x, y, yaw)].sigma > max_rho)
                    {
                        occ_buffer[toAddress(x, y, yaw)] = 1;
                        occ_r2_buffer[x * voxel_num(1) + y] = 1;
                    }
                }
        updateVisualizations();
        map_ready = true;
    }

    void UnevenMap::updateVisualizations()
    {
        if (!map_ready) return;

        //  to pcl and marker msg
        zb_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        zb_msg.header.frame_id = "map";
        zb_msg.pose.orientation.w = 1.0;
        zb_msg.scale.x = 0.006;
        zb_msg.color.a = 0.6;
        geometry_msgs::msg::Point p1, p2;
        
        pcl::PointCloud<pcl::PointXYZI> grid_map_filtered;
        pcl::PointXYZI pt_filtered;
        double sigma_sum = 0.0;
        int published_cnt = 0;
        int yaw = floor(M_2_PI*yaw_resolution_inv);
        for (int x=0; x<voxel_num[0]; x++)
            for (int y=0; y<voxel_num[1]; y++)
            {
                if (occ_buffer[toAddress(x, y, yaw)]==1) {
                    // publish a special marker for tilt-occupied cells
                    double c = c_buffer[toAddress(x, y, yaw)];
                    bool tilt_occ = (c < min_cnormal);
                    if (tilt_occ) {
                        Eigen::Vector3d filtered_p;
                        indexToPos(Eigen::Vector3i(x, y, yaw), filtered_p);
                        geometry_msgs::msg::Point p_occ;
                        p_occ.x = filtered_p.x();
                        p_occ.y = filtered_p.y();
                        p_occ.z = filtered_p.z();
                        // encode special value via an auxiliary marker list
                        // here we piggyback on so2_point to render small red dots for tilt occupancy
                        geometry_msgs::msg::Point occ_dot = p_occ;
                        // store into marker array's points via zb_msg as small tilt dots
                        // here reuse zb_msg with tiny scale and bright color
                        visualization_msgs::msg::Marker tilt_dot;
                        tilt_dot.id = 2;
                        tilt_dot.type = visualization_msgs::msg::Marker::POINTS;
                        tilt_dot.header.frame_id = "map";
                        tilt_dot.pose.orientation.w = 1.0;
                        tilt_dot.scale.x = 0.01;
                        tilt_dot.scale.y = 0.01;
                        tilt_dot.color.a = 1.0;
                        tilt_dot.color.r = 1.0;
                        tilt_dot.color.g = 0.0;
                        tilt_dot.color.b = 0.0;
                        tilt_dot.points.emplace_back(occ_dot);
                        so2_test_msg.markers.emplace_back(tilt_dot);
                    }
                    continue;
                }
                Eigen::Vector3d filtered_p;
                RXS2 rs2 = map_buffer[toAddress(x, y, yaw)];
                double c = c_buffer[toAddress(x, y, yaw)];
                indexToPos(Eigen::Vector3i(x, y, yaw), filtered_p);
                p1.x = pt_filtered.x = filtered_p.x();
                p1.y = pt_filtered.y = filtered_p.y();
                p1.z = pt_filtered.z = rs2.z;
                pt_filtered.intensity = rs2.sigma;
                grid_map_filtered.emplace_back(pt_filtered);
                sigma_sum += rs2.sigma;
                published_cnt++;

                p2.x = p1.x + 1.5 * xy_resolution * rs2.zb.x();
                p2.y = p1.y + 1.5 * xy_resolution * rs2.zb.y();
                p2.z = p1.z + 1.5 * xy_resolution * c;
                if (x%2==0 && y%2==0)
                {
                    zb_msg.points.emplace_back(p1);
                    zb_msg.points.emplace_back(p2);
                }
            }
        grid_map_filtered.width = grid_map_filtered.points.size();
        grid_map_filtered.height = 1;
        grid_map_filtered.is_dense = true;
        grid_map_filtered.header.frame_id = "map";
        pcl::toROSMsg(grid_map_filtered, filtered_cloud_msg);

        int occ_xy = 0;
        for (int i = 0; i < getXYNum(); ++i) {
            occ_xy += int(occ_r2_buffer[i]);
        }
        double avg_sigma = published_cnt > 0 ? sigma_sum / published_cnt : 0.0;
        RCLCPP_INFO(node_->get_logger(), "uneven_map: world=%zu filtered=%zu occ_xy=%d/%d avg_sigma=%.3f z_clip=[%.3f,%.3f]",
                    world_cloud->points.size(), grid_map_filtered.points.size(), occ_xy, getXYNum(), avg_sigma, z_min, z_max);

        // so2_test_msg
        visualization_msgs::msg::Marker so2_line;
        visualization_msgs::msg::Marker so2_point;
        so2_line.id = 0;
        so2_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        so2_line.header.frame_id = "map";
        so2_line.pose.orientation.w = 1.0;
        so2_line.scale.x = 0.01;
        so2_line.color.a = 0.6;
        so2_point.id = 1;
        so2_point.type = visualization_msgs::msg::Marker::POINTS;
        so2_point.header.frame_id = "map";
        so2_point.pose.orientation.w = 1.0;
        so2_point.scale.x = 0.015;
        so2_point.scale.y = 0.015;
        so2_point.color.a = 1.0;
        so2_point.color.r = 0.8;
        geometry_msgs::msg::Point p0;
        double r_res = 0.8;
        int ri_res = floor(r_res * xy_resolution_inv);
        int yaw_draw = floor(M_2_PI*yaw_resolution_inv);
        for (int x=0; x<voxel_num[0]; x+=ri_res)
            for (int y=0; y<voxel_num[1]; y+=ri_res)
            {
                Eigen::Vector3d filtered_p;
                RXS2 rs2 = map_buffer[toAddress(x, y, yaw_draw)];
                indexToPos(Eigen::Vector3i(x, y, yaw_draw), filtered_p);
                p1.x = p0.x = filtered_p.x();
                p1.y = p0.y = filtered_p.y();
                p1.z = p0.z = rs2.z;
                so2_point.points.emplace_back(p0);

                p2.x = p1.x + 1.5 * xy_resolution * rs2.zb.x();
                p2.y = p1.y + 1.5 * xy_resolution * rs2.zb.y();
                p2.z = p1.z + 1.5 * xy_resolution * c_buffer[toAddress(x, y, yaw_draw)];
                so2_line.points.emplace_back(p1);
                so2_line.points.emplace_back(p2);
            }
        so2_test_msg.markers.emplace_back(so2_line);
        so2_test_msg.markers.emplace_back(so2_point);

        origin_pub->publish(origin_cloud_msg);
        zb_pub->publish(zb_msg);
        filtered_pub->publish(filtered_cloud_msg);
        so2_test_pub->publish(so2_test_msg);
    }

    void UnevenMap::globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        generateMapFromCloud(msg);
    }
}
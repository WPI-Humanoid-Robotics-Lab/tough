#include "perception_common/SampleCADModels.h"

namespace perception_common
{
    SamplingParams::SamplingParams(int number_of_points, float step_size, SampleType sample_type)
    {
        this->number_of_points = number_of_points;
        this->step_size = step_size;
        this->sample_type = sample_type;
    }

    SampleCADModels::SampleCADModels(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    {
        nh_ = node_handle;
        pnh_ = private_node_handle;
    }

    SampleCADModels::~SampleCADModels()
    {
        // Destructor code goes here
    }
    
    bool SampleCADModels::meshToPointCloud(std::string &filename_model,
                                           sensor_msgs::PointCloud2 &cloud_out,
                                           pcl::PointCloud<pcl::PointXYZ> &normals,
                                           SamplingParams &params)
    {
        shapes::Mesh                shape;
        sensor_msgs::PointCloud     cloud;

        if (this->stlToShape(filename_model, shape) == false)
            return false;
        this->shapeToPointCloud(shape, cloud, normals, params);
        sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud_out);

        if (cloud_out.data.size() < 10)
            return false;
        else
            return true;
    }

    bool SampleCADModels::meshToPointCloud(std::string &filename_model,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr &normals,
                                           SamplingParams &params)
    {
        sensor_msgs::PointCloud2    cloud_out2;
        bool                        ret_value;

        ret_value = meshToPointCloud(filename_model, cloud_out2, *normals, params);
        pcl::fromROSMsg(cloud_out2, *cloud_out);

        return ret_value;
    }

    bool SampleCADModels::meshToPointCloud(std::string &filename_model,
                                           pcl::PointCloud<pcl::PointXYZ> &cloud_out,
                                           pcl::PointCloud<pcl::PointXYZ> &normals,
                                           SamplingParams &params)
    {
        sensor_msgs::PointCloud2    cloud_out2;
        bool                        ret_value;

        ret_value = meshToPointCloud(filename_model, cloud_out2, normals, params);
        pcl::fromROSMsg(cloud_out2, cloud_out);

        return ret_value;
    }

    bool SampleCADModels::meshesToPointCloud(std::vector<std::string> &filename_model,
                                             sensor_msgs::PointCloud2 &cloud_out,
                                             pcl::PointCloud<pcl::PointXYZ> &normals,
                                             SamplingParams &params)
    {
        shapes::Mesh                    shape;
        sensor_msgs::PointCloud         cloud;
        pcl::PointCloud<pcl::PointXYZ>  pcl_cloud;

        for (int i = 0; i < filename_model.size(); i++)
        {
            if (this->stlToShape(filename_model.at(i), shape) == false)
                return false;
            this->shapeToPointCloud(shape, cloud, normals, params);
            sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud_out);

            pcl::fromROSMsg(cloud_out, pcl_cloud);
        }

        pcl::toROSMsg(pcl_cloud, cloud_out);

        if (cloud_out.data.size() < 10)
            return false;
        else
            return true;
    }

    bool SampleCADModels::meshesToPointCloud(std::vector<std::string> &filename_model,
                                             pcl::PointCloud<pcl::PointXYZ> &cloud_out,
                                             pcl::PointCloud<pcl::PointXYZ> &normals,
                                             SamplingParams &params)
    {
        sensor_msgs::PointCloud2    cloud_out2;
        bool                        ret_value;

        ret_value = meshesToPointCloud(filename_model, cloud_out2, normals, params);
        pcl::fromROSMsg(cloud_out2, cloud_out);

        return ret_value;

    }

    bool SampleCADModels::meshesToPointCloud(std::vector<std::string> &filename_model,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr &normals,
                                             SamplingParams &params)
    {
        sensor_msgs::PointCloud2    cloud_out2;
        bool                        ret_value;

        if (cloud_out == NULL)
            cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        if (normals == NULL)
            normals = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        ret_value = meshesToPointCloud(filename_model, cloud_out2, *normals, params);
        pcl::fromROSMsg(cloud_out2, *cloud_out);

        return ret_value;
    }

    void SampleCADModels::shapeToPointCloud(shapes::Mesh &shape, sensor_msgs::PointCloud &cloud,
                                             pcl::PointCloud<pcl::PointXYZ> &normals, SamplingParams &params)
    {
        cloud.header.frame_id = shape.STRING_NAME;
        srand(time(NULL));

        if (params.sample_type == SampleType::RANDOM)
        {
            float max_area = 0;
            float running_tot = 0;
            std::map<int, float> areas;

            for (int i = 0; i < shape.triangle_count; i++)
            {
                int density = 10000;
                float area;

                geometry_msgs::Point    v1;
                geometry_msgs::Point    v2;
                geometry_msgs::Point    v3;

                v1.x = shape.vertices[3*shape.triangles[3*i]];
                v1.y = shape.vertices[3*shape.triangles[3*i] + 1];
                v1.z = shape.vertices[3*shape.triangles[3*i] + 2];

                v2.x = shape.vertices[3*shape.triangles[3*i + 1]];
                v2.y = shape.vertices[3*shape.triangles[3*i + 1] + 1];
                v2.z = shape.vertices[3*shape.triangles[3*i + 1] + 2];

                v3.x = shape.vertices[3*shape.triangles[3*i + 2]];
                v3.y = shape.vertices[3*shape.triangles[3*i + 2] + 1];
                v3.z = shape.vertices[3*shape.triangles[3*i + 2] + 2];

                area = this->findArea(v1, v2, v3, density);

                running_tot = running_tot + area;
                areas[i] = running_tot;
                if (area > max_area)
                    max_area = area;
            }

            for (int i = 0; i < params.number_of_points; i++)
            {
                float prob = rand() % int(running_tot);
                int imax = areas.size();
                int imin = 0;
                int index = 0;
                int imid;

                while (imax >= imin)
                {
                    imid = imin + (int) ((imax - imin) / 2);

                    if (areas[imid] <= prob)
                        if (areas[imid + 1] > prob)
                        {
                            index = imid;
                            break;
                        }
                        else
                            imin = imid + 1;
                    else if (areas[imid] > prob)
                        if (areas[imid - 1] <= prob)
                        {
                            index = imid;
                            break;
                        }
                        else
                            imax = imid - 1;
                }

                geometry_msgs::Point    v1;
                geometry_msgs::Point    v2;
                geometry_msgs::Point    v3;

                v1.x = shape.vertices[3*shape.triangles[3*index]];
                v1.y = shape.vertices[3*shape.triangles[3*index] + 1];
                v1.z = shape.vertices[3*shape.triangles[3*index] + 2];

                v2.x = shape.vertices[3*shape.triangles[3*index + 1]];
                v2.y = shape.vertices[3*shape.triangles[3*index + 1] + 1];
                v2.z = shape.vertices[3*shape.triangles[3*index + 1] + 2];

                v3.x = shape.vertices[3*shape.triangles[3*index + 2]];
                v3.y = shape.vertices[3*shape.triangles[3*index + 2] + 1];
                v3.z = shape.vertices[3*shape.triangles[3*index + 2] + 2];

                // Find normals
                geometry_msgs::Vector3  normal;
                this->getNormal(v1, v2, v3, normal);
                pcl::PointXYZ   normal_pt(normal.x, normal.y, normal.z);
                normals.points.push_back(normal_pt);

                // Sample point
                geometry_msgs::Point32  point = sampleRandomPoint(v1, v2, v3);
                cloud.points.push_back(point);
            }
        }
        else if (params.sample_type == SampleType::GRID)
        {
            for (int i = 0; i < (shape.triangle_count); i++)
            {
                geometry_msgs::Point    v1;
                geometry_msgs::Point    v2;
                geometry_msgs::Point    v3;

                v1.x = shape.vertices[3*shape.triangles[3*i]];
                v1.y = shape.vertices[3*shape.triangles[3*i] + 1];
                v1.z = shape.vertices[3*shape.triangles[3*i] + 2];

                v2.x = shape.vertices[3*shape.triangles[3*i + 1]];
                v2.y = shape.vertices[3*shape.triangles[3*i + 1] + 1];
                v2.z = shape.vertices[3*shape.triangles[3*i + 1] + 2];

                v3.x = shape.vertices[3*shape.triangles[3*i + 2]];
                v3.y = shape.vertices[3*shape.triangles[3*i + 2] + 1];
                v3.z = shape.vertices[3*shape.triangles[3*i + 2] + 2];

                // Find normals
                geometry_msgs::Vector3  normal;
                this->getNormal(v1, v2, v3, normal);
                pcl::PointXYZ   normal_pt(normal.x, normal.y, normal.z);
                normals.points.push_back(normal_pt);

                std::list<geometry_msgs::Point32>   samples;
                this->sampleGridPoints(v1, v2, v3, samples, params.step_size);
                while (!samples.empty())
                {
                    geometry_msgs::Point32  point = samples.front();
                    samples.pop_front();
                    cloud.points.push_back(point);
                }
            }
        }
    }

    bool SampleCADModels::stlToShape(std::string &filename, shapes::Mesh &shape)
    {
        Eigen::Vector3d scale(1.0, 1.0, 1.0);

        try
        {
            shape = *(shapes::createMeshFromResource(filename, scale));
        }
        catch (...)
        {
            ROS_ERROR_STREAM("SampleCADModels::stlToShape - Could not load resource file.");
            return false;
        }

        if (shape.triangle_count < 4)
            return false;

        return true;
    }

    void SampleCADModels::getNormal(geometry_msgs::Point v1, geometry_msgs::Point v2,
                                    geometry_msgs::Point v3, geometry_msgs::Vector3 &normal)
    {
        Eigen::Vector3d vec1, vec2;
        Eigen::Vector3d ev1, ev2, ev3;
        Eigen::Vector3d enormal;

        ev1 = Eigen::Vector3d(v1.x, v1.y, v1.z);
        ev2 = Eigen::Vector3d(v2.x, v2.y, v2.z);
        ev3 = Eigen::Vector3d(v3.x, v3.y, v3.z);

        vec1 = ev2 - ev1;
        vec2 = ev3 - ev1;

        try
        {
            enormal = vec1.cross(vec2);
            enormal.normalize();

            normal.x = enormal(0);
            normal.y = enormal(1);
            normal.z = enormal(2);
        }
        catch (...)
        {
            ROS_WARN_STREAM("SampleCADModels::getNormal exception. Block 1.");
        }
    }

    double SampleCADModels::findArea(geometry_msgs::Point v1, geometry_msgs::Point v2,
                                     geometry_msgs::Point v3, int scale)
    {
        Eigen::Vector3d ev1, ev2, ev3;
        Eigen::Vector3d vec1, vec2;
        Eigen::Vector3d cross_prod;

        ev1 = Eigen::Vector3d(v1.x, v1.y, v1.z);
        ev2 = Eigen::Vector3d(v2.x, v2.y, v2.z);
        ev3 = Eigen::Vector3d(v3.x, v3.y, v3.z);

        vec1 = (ev2 - ev1) * scale;
        vec2 = (ev3 - ev1) * scale;

        cross_prod = vec1.cross(vec2);

        return ((double) std::floor(0.5 * cross_prod.norm()));
    }

    // http://mathworld.wolfram.com/TrianglePointPicking.html
    // One could make changes to the sampling algorithm by using the
    //      v_x = v_1 + a*(v_2 - v_1) + b*(v_3 - v_1)
    //      where, 'a' and 'b' are uniformly distributed in [0, 1]
    geometry_msgs::Point32 SampleCADModels::sampleRandomPoint(geometry_msgs::Point v1,
                                                              geometry_msgs::Point v2,
                                                              geometry_msgs::Point v3)
    {
        geometry_msgs::Point32 pt;
        float r1, r2;

        r1 = rand() / (float) RAND_MAX;
        r2 = rand() / (float) RAND_MAX;

        pt.x = (1 - sqrt(r1)) * v1.x + (sqrt(r1) * (1 - r2)) * v2.x + (sqrt(r1) * r2) * v3.x;
        pt.y = (1 - sqrt(r1)) * v1.y + (sqrt(r1) * (1 - r2)) * v2.y + (sqrt(r1) * r2) * v3.y;
        pt.z = (1 - sqrt(r1)) * v1.z + (sqrt(r1) * (1 - r2)) * v2.z + (sqrt(r1) * r2) * v3.z;

        return pt;
    }

    void SampleCADModels::sampleGridPoints(geometry_msgs::Point v1, geometry_msgs::Point v2,
                                           geometry_msgs::Point v3,
                                           std::list<geometry_msgs::Point32> &samples,
                                           float step_size)
    {
        Eigen::Vector3d ev1, ev2, ev3;
        Eigen::Vector3d vec1, vec2;

        ev1 = Eigen::Vector3d(v1.x, v1.y, v1.z);
        ev2 = Eigen::Vector3d(v2.x, v2.y, v2.z);
        ev3 = Eigen::Vector3d(v3.x, v3.y, v3.z);

        vec1 = ev3 - ev1;
        vec2 = ev2 - ev1;

        for (int i = 0; i < ((int) (vec1.norm() / step_size)); i++)
        {
            float x = step_size * i;
            for (int j = 0; j < ((int) ((vec2.norm() - (vec2.norm() / vec1.norm()) * x) / step_size)); j++)
            {
                float y = step_size * j;

                geometry_msgs::Point32 p;

                p.x = p.x + x * (vec1(0) / vec1.norm()) + y * (vec2(0) / vec2.norm());
                p.y = p.y + x * (vec1(1) / vec1.norm()) + y * (vec2(1) / vec2.norm());
                p.z = p.z + x * (vec1(2) / vec1.norm()) + y * (vec2(2) / vec2.norm());

                samples.push_back(p);
            }
        }
    }

    bool SampleCADModels::filterPointsWithInnerNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &in_normals,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
                                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &out_normals)
    {
        Eigen::Vector3d                             tot(0.0f, 0.0f, 0.0f);
        Eigen::Vector3d                             centroid;
        pcl::PointCloud<pcl::PointXYZ>::iterator    it, it2;

        // Check if in_cloud == NULL
        if (in_cloud == NULL)
            return false;
        if (in_normals == NULL)
            return false;

        if (out_cloud == NULL)
            out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        if (out_normals == NULL)
            out_normals = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // Compute centroid
        for (it = in_cloud->points.begin(); it < in_cloud->points.end(); it++)
            tot = tot + Eigen::Vector3d(it->x, it->y, it->z);

        centroid = tot / in_cloud->points.size();

        it = in_cloud->points.begin();

        for (it2 = in_normals->points.begin(); it2 < in_normals->points.end(); it2++, it++)
        {
            pcl::PointXYZ  pt(*it);
            pcl::PointXYZ  norm(*it2);

            Eigen::Vector3d  v((pt.x - centroid(0)),(pt.y - centroid(1)),(pt.z - centroid(2)));
            Eigen::Vector3d  e_norm(norm.x, norm.y, norm.z);

            double dot = v.dot(e_norm);

            if (dot > 0)
            {
                out_cloud->points.push_back(pt);
                out_normals->points.push_back(norm);
            }
        }

        out_cloud->header = in_cloud->header;
        out_cloud->width = out_cloud->points.size();
        out_cloud->height = 1;
        out_normals->header = in_normals->header;
        out_normals->width = out_normals->points.size();
        out_normals->height = 1;

        return true;
    }

    bool SampleCADModels::filterOccludedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &normals,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr &output_normals,
                                               Eigen::Vector3d      to_origin)
    {
        if (input == NULL)
            return false;
        if (normals == NULL)
            return false;

        if (std::isnan(to_origin(0)) || std::isnan(to_origin(1)) || std::isnan(to_origin(2)))
            return false;

        if (output == NULL)
            output = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        if (output_normals == NULL)
            output_normals = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        ROS_INFO_STREAM("Normals size - " << input->points.size());

        to_origin = -to_origin;
        if (normals->size() != 0)
        {
            pcl::PointCloud<pcl::PointXYZ>::iterator      it = input->points.begin();
            pcl::PointCloud<pcl::PointXYZ>::iterator      it_n;

            int i = 0;

            ROS_INFO_STREAM("i - " << input->points.size());

            for (it_n = normals->points.begin(); it_n < normals->points.end(); it_n++, it++)
            {
                i++;
                pcl::PointXYZ   norm(*it_n);
                pcl::PointXYZ   pt(*it);

                Eigen::Vector3d e_norm(norm.x, norm.y, norm.z);
                e_norm = e_norm;

                ROS_INFO_STREAM_ONCE("e_norm - " << e_norm(0) << " " << e_norm(1) << " " << e_norm(2));

                double dot = e_norm.dot(to_origin);
                double angle = dot / (e_norm.norm() * to_origin.norm());

                //if (angle * 57.2957795 > 90)
                    //ROS_INFO_STREAM(angle * 57.2957795);

                if (dot < 0.2)
                {                    
                    output->points.push_back(pt);
                    output_normals->points.push_back(norm);
                }
            }

            ROS_INFO_STREAM("i - " << output->points.size());

            output->header = input->header;
            output->width = output->points.size();
            output->height = 1;
            output_normals->header = normals->header;
            output_normals->width = output_normals->points.size();
            output_normals->height = 1;
        }
        else
        {
            ROS_INFO("No normals.");
            output = input;
            output_normals = normals;
        }        

        return true;
    }
}

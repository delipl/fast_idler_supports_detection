#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/impl/implicit_shape_model.hpp>

int main(int argc, char** argv) {
    if (argc < 5 || argc % 2 == 0)  // needs at least one training cloud with class id, plus testing cloud with class id
                                    // (plus name of executable)
        return (-1);

    unsigned int number_of_training_clouds = (argc - 3) / 2;



    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >::Ptr fpfh(
        new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> >);
    fpfh->setRadiusSearch(10.0);
    pcl::Feature<pcl::PointXYZ, pcl::Histogram<153> >::Ptr feature_estimator(fpfh);

    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal> ism;
    ism.setFeatureEstimator(feature_estimator);
    // ism.setTrainingClouds(training_clouds);
    // ism.setTrainingNormals(training_normals);
    // ism.setTrainingClasses(training_classes);
    ism.setSamplingSize(0.001);
    pcl::ism::ImplicitShapeModelEstimation<153, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model(
        new pcl::features::ISMModel);
    // ism.trainISM(model);
    // // return 0;

    std::string file("trained_ism_model.txt");
    // model->saveModelToFile(file);
    model->loadModelFromfile(file);

    unsigned int testing_class = static_cast<unsigned int>(strtol(argv[argc - 1], 0, 10));
    pcl::PointCloud<pcl::PointXYZ>::Ptr testing_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[argc - 2], *testing_cloud) == -1) return (-1);

    pcl::PointCloud<pcl::Normal>::Ptr testing_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setRadiusSearch(10.0);
    normal_estimator.setInputCloud(testing_cloud);
    normal_estimator.compute(*testing_normals);
    std::cout << "testing cloud size " << testing_cloud->size() << std::endl;

    pcl::features::ISMVoteList<pcl::PointXYZ>::Ptr vote_list =
        ism.findObjects(model, testing_cloud, testing_normals, testing_class);

    std::cout << "sigmas: " << model->sigmas_[testing_class] << std::endl;


    double radius = model->sigmas_[testing_class]* 7.5;
    double sigma = model->sigmas_[testing_class]*4.5;  


    std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
    vote_list->findStrongestPeaks(strongest_peaks, testing_class, radius, sigma);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->height = 0;
    colored_cloud->width = 1;

    pcl::PointXYZRGB point;
    point.r = 255;
    point.g = 255;
    point.b = 255;

    for (std::size_t i_point = 0; i_point < testing_cloud->size(); i_point++) {
        point.x = (*testing_cloud)[i_point].x;
        point.y = (*testing_cloud)[i_point].y;
        point.z = (*testing_cloud)[i_point].z;
        colored_cloud->points.push_back(point);
    }
    colored_cloud->height += testing_cloud->size();

    point.r = 255;
    point.g = 0;
    point.b = 0;
    pcl::visualization::PCLVisualizer viewer("ISM BOXES");

    for (std::size_t i_vote = 0; i_vote < strongest_peaks.size(); i_vote++) {
        point.x = strongest_peaks[i_vote].x;
        point.y = strongest_peaks[i_vote].y;
        point.z = strongest_peaks[i_vote].z;
        std::cout << "Strongest peaks coords:\n\tx: " << point.x << "\n\ty: " << point.y << "\n\tz: " << point.z
                  << std::endl;
        colored_cloud->points.push_back(point);
        viewer.addCube(point.x-0.02, point.x+0.02, point.y-0.5, point.y+0.5, point.z-0.10, point.z+0.10, 1.0, 1.0, 1.0, std::to_string(i_vote));
    }

    colored_cloud->points.push_back(point);
    colored_cloud->height += strongest_peaks.size();

    viewer.addPointCloud(colored_cloud);
    while (!viewer.wasStopped()) {
        viewer.spin();
    }

    return (0);
}